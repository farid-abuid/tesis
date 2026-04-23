"""Single logger for one or multiple arms; writes one CSV and optionally plots on run end."""

import csv
import os
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from exo_control_msgs.msg import JointControlTelemetry
from lifecycle_msgs.msg import State, TransitionEvent
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import Trigger


# Control modes that imply a JointControlTelemetry publisher and q_des/dq_des references.
_GRAV_COMP_MODES = ('joint_space_gravity_compensation', 'task_space_gravity_compensation')


class DataLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__('exo_data_logger')

        # General params
        self.declare_parameter('log_root', '~/exo_logs')
        self.declare_parameter('use_controller_session', True)
        self.declare_parameter('use_lifecycle_events', True)
        self.declare_parameter('auto_plot', True)
        self.declare_parameter('sample_rate_hz', 200.0)
        self.declare_parameter('control_mode', 'effort')

        # Per-arm topic lists (aligned by index with `arms`).
        self.declare_parameter('arms', ['main'])
        self.declare_parameter('telemetry_topics', [''])
        self.declare_parameter('session_topics', [''])
        self.declare_parameter('lifecycle_transition_topics', [''])
        self.declare_parameter('command_topics', [''])
        self.declare_parameter('joint_state_topics', ['/joint_states'])

        self._log_root = Path(os.path.expanduser(self.get_parameter('log_root').value))
        self._use_controller_session = bool(self.get_parameter('use_controller_session').value)
        self._use_lifecycle_events = bool(self.get_parameter('use_lifecycle_events').value)
        self._auto_plot = bool(self.get_parameter('auto_plot').value)
        self._sample_rate_hz = float(self.get_parameter('sample_rate_hz').value)
        self._control_mode = str(self.get_parameter('control_mode').value).strip() or 'effort'

        arms_raw = [str(a).strip() for a in (self.get_parameter('arms').value or [])]
        self._arms = [a for a in arms_raw if a] or ['main']
        n = len(self._arms)

        def _list(name: str) -> list[str]:
            vals = [str(v).strip() for v in (self.get_parameter(name).value or [])]
            if len(vals) < n:
                vals = vals + [''] * (n - len(vals))
            return vals[:n]

        self._telemetry_topics = _list('telemetry_topics')
        self._session_topics = _list('session_topics')
        self._lifecycle_topics = _list('lifecycle_transition_topics')
        self._command_topics = _list('command_topics')
        self._joint_state_topics = _list('joint_state_topics')

        # Per-arm latest state (joint-aligned arrays come from telemetry/joint_state/commands).
        self._state: dict[str, dict] = {
            arm: {
                'joint_names': [],
                'q': [], 'dq': [], 'tau_meas': [],
                'q_ref': [], 'dq_ref': [],
                'tau_cmd': [], 'tau_ff': [],
                'q_cmd': [], 'dq_cmd': [], 'tau_cmd_in': [],
                'ee_actual': [], 'ee_desired': [],
                'active': False,
            }
            for arm in self._arms
        }

        # Runtime CSV state
        self._recording = False
        self._csv_file = None
        self._csv_writer = None
        self._header: list[str] | None = None
        self._run_dir: Path | None = None
        self._run_start = None
        self._sample_timer = None

        # QoS profiles
        telem_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        session_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        lc_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscriptions per arm (default arguments bind the arm label into each lambda).
        for arm, telem, sess, life, cmd, js in zip(
            self._arms,
            self._telemetry_topics,
            self._session_topics,
            self._lifecycle_topics,
            self._command_topics,
            self._joint_state_topics,
        ):
            if telem:
                self.create_subscription(
                    JointControlTelemetry, telem,
                    lambda m, a=arm: self._on_telemetry(a, m), telem_qos,
                )
            if js:
                self.create_subscription(
                    JointState, js,
                    lambda m, a=arm: self._on_joint_state(a, m), 10,
                )
            if cmd:
                self.create_subscription(
                    Float64MultiArray, cmd,
                    lambda m, a=arm: self._on_command(a, m), 10,
                )
            if sess and self._use_controller_session:
                self.create_subscription(
                    Bool, sess,
                    lambda m, a=arm: self._on_session(a, m), session_qos,
                )
            if life and self._use_controller_session and self._use_lifecycle_events:
                self.create_subscription(
                    TransitionEvent, life,
                    lambda m, a=arm: self._on_lifecycle(a, m), lc_qos,
                )

        self._srv_start = self.create_service(Trigger, '~/start_logging', self._on_start_logging)
        self._srv_stop = self.create_service(Trigger, '~/stop_logging', self._on_stop_logging)

        self.get_logger().info(
            f'exo_data_logger ready (mode={self._control_mode} arms={self._arms} '
            f'telemetry={self._telemetry_topics} joint_state={self._joint_state_topics})'
        )

    # ---- Message callbacks ----

    def _on_joint_state(self, arm: str, msg: JointState) -> None:
        st = self._state[arm]
        if msg.name:
            st['joint_names'] = list(msg.name)
        if len(msg.position) > 0:
            st['q'] = [float(v) for v in msg.position]
        if len(msg.velocity) > 0:
            st['dq'] = [float(v) for v in msg.velocity]
        if len(msg.effort) > 0:
            st['tau_meas'] = [float(v) for v in msg.effort]

    def _on_telemetry(self, arm: str, msg: JointControlTelemetry) -> None:
        st = self._state[arm]
        if not st['joint_names'] and msg.joint_names:
            st['joint_names'] = list(msg.joint_names)
        # Fallback q/dq/tau_meas if no /joint_states available for this arm
        if not st['q'] and len(msg.position) > 0:
            st['q'] = [float(v) for v in msg.position]
        if not st['dq'] and len(msg.velocity) > 0:
            st['dq'] = [float(v) for v in msg.velocity]
        if not st['tau_meas'] and len(msg.effort_measured) > 0:
            st['tau_meas'] = [float(v) for v in msg.effort_measured]
        if len(msg.position_reference) > 0:
            st['q_ref'] = [float(v) for v in msg.position_reference]
        if len(msg.velocity_reference) > 0:
            st['dq_ref'] = [float(v) for v in msg.velocity_reference]
        if len(msg.effort_command) > 0:
            st['tau_cmd'] = [float(v) for v in msg.effort_command]
        if len(msg.effort_feedforward) > 0:
            st['tau_ff'] = [float(v) for v in msg.effort_feedforward]
        if len(msg.operational_position_actual) >= 3:
            st['ee_actual'] = [float(v) for v in msg.operational_position_actual[:3]]
        if len(msg.operational_position_desired) >= 3:
            st['ee_desired'] = [float(v) for v in msg.operational_position_desired[:3]]

    def _on_command(self, arm: str, msg: Float64MultiArray) -> None:
        # Interpret command meaning from control_mode; never fan out to unrelated slots.
        data = [float(v) for v in msg.data]
        st = self._state[arm]
        mode = self._control_mode
        if mode == 'position':
            st['q_cmd'] = data
        elif mode == 'velocity':
            st['dq_cmd'] = data
        else:
            st['tau_cmd_in'] = data

    def _on_session(self, arm: str, msg: Bool) -> None:
        self._set_arm_active(arm, bool(msg.data), source=f'{arm}_session')

    def _on_lifecycle(self, arm: str, msg: TransitionEvent) -> None:
        gid = msg.goal_state.id
        if gid not in (
            State.PRIMARY_STATE_UNCONFIGURED,
            State.PRIMARY_STATE_INACTIVE,
            State.PRIMARY_STATE_ACTIVE,
            State.PRIMARY_STATE_FINALIZED,
        ):
            return
        active = gid == State.PRIMARY_STATE_ACTIVE
        self._set_arm_active(arm, active, source=f'{arm}_lifecycle')

    def _set_arm_active(self, arm: str, active: bool, *, source: str) -> None:
        st = self._state[arm]
        if st['active'] == active:
            return
        st['active'] = active
        if active and not self._recording:
            self._begin_run(source=source)
        elif not active and self._recording:
            if not any(s['active'] for s in self._state.values()):
                self._end_run(source=source)

    # ---- Header / row construction ----

    def _expected_joint_count(self, arm: str) -> int:
        return len(self._state[arm]['joint_names']) or 3

    def _ensure_joint_names(self, arm: str) -> list[str]:
        names = self._state[arm]['joint_names']
        if names:
            return names
        # Fallback generic names (3 joints per arm on this platform).
        prefix = '' if arm == 'main' else f'{arm}_'
        names = [f'{prefix}joint_{i + 1}' for i in range(3)]
        self._state[arm]['joint_names'] = names
        return names

    def _build_header(self) -> list[str]:
        mode = self._control_mode
        header = ['time_sec']
        for arm in self._arms:
            for j in self._ensure_joint_names(arm):
                header += [f'{j}_q', f'{j}_dq', f'{j}_tau_meas']
                if mode == 'position':
                    header.append(f'{j}_q_cmd')
                elif mode == 'velocity':
                    header.append(f'{j}_dq_cmd')
                elif mode == 'effort':
                    header.append(f'{j}_tau_cmd_in')
                elif mode in _GRAV_COMP_MODES:
                    header += [f'{j}_q_des', f'{j}_dq_des', f'{j}_tau_cmd', f'{j}_tau_ff']
            if mode == 'task_space_gravity_compensation':
                p = '' if arm == 'main' else f'{arm}_'
                header += [
                    f'{p}ee_x', f'{p}ee_y', f'{p}ee_z',
                    f'{p}ee_des_x', f'{p}ee_des_y', f'{p}ee_des_z',
                ]
        return header

    def _build_row(self, t_sec: float) -> list:
        mode = self._control_mode

        def _at(lst: list, i: int):
            if lst and i < len(lst):
                v = lst[i]
                return v if v == v else ''  # NaN -> empty cell
            return ''

        row: list = [f'{t_sec:.6f}']
        for arm in self._arms:
            st = self._state[arm]
            names = st['joint_names']
            n = len(names)
            for i in range(n):
                row += [_at(st['q'], i), _at(st['dq'], i), _at(st['tau_meas'], i)]
                if mode == 'position':
                    row.append(_at(st['q_cmd'], i))
                elif mode == 'velocity':
                    row.append(_at(st['dq_cmd'], i))
                elif mode == 'effort':
                    row.append(_at(st['tau_cmd_in'], i))
                elif mode in _GRAV_COMP_MODES:
                    row += [_at(st['q_ref'], i), _at(st['dq_ref'], i),
                            _at(st['tau_cmd'], i), _at(st['tau_ff'], i)]
            if mode == 'task_space_gravity_compensation':
                ee = st['ee_actual']
                ed = st['ee_desired']
                row += [_at(ee, 0), _at(ee, 1), _at(ee, 2),
                        _at(ed, 0), _at(ed, 1), _at(ed, 2)]
        return row

    # ---- Run lifecycle ----

    def _begin_run(self, source: str) -> None:
        self._end_run(source='restart', plot=self._auto_plot)
        self._log_root.mkdir(parents=True, exist_ok=True)
        run_id = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
        self._run_dir = self._log_root / run_id
        self._run_dir.mkdir(parents=True, exist_ok=False)
        csv_path = self._run_dir / 'data.csv'
        self._csv_file = open(csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._header = None
        self._recording = True
        self._run_start = self.get_clock().now()
        period = 1.0 / max(self._sample_rate_hz, 1.0)
        self._sample_timer = self.create_timer(period, self._on_sample)
        self.get_logger().info(f'Logging started ({source}) -> {csv_path}')

    def _on_sample(self) -> None:
        if not self._recording or self._csv_writer is None:
            return
        if self._header is None:
            # Wait up to 2s for joint_state/telemetry to populate joint names per arm.
            elapsed = (self.get_clock().now() - self._run_start).nanoseconds * 1e-9
            missing = [a for a in self._arms if not self._state[a]['joint_names']]
            if missing and elapsed < 2.0:
                return
            if missing:
                self.get_logger().warn(
                    f'joint names missing for {missing}; using generic fallback'
                )
            self._header = self._build_header()
            self._csv_writer.writerow(self._header)
        t = (self.get_clock().now() - self._run_start).nanoseconds * 1e-9
        row = self._build_row(t)
        if len(row) != len(self._header):
            return
        self._csv_writer.writerow(row)

    def _end_run(self, source: str, plot: bool | None = None) -> None:
        if plot is None:
            plot = self._auto_plot
        if not self._recording:
            return
        self._recording = False
        if self._sample_timer is not None:
            try:
                self._sample_timer.cancel()
                self.destroy_timer(self._sample_timer)
            except Exception:
                pass
            self._sample_timer = None
        if self._csv_file is not None:
            self._csv_file.flush()
            self._csv_file.close()
            self._csv_file = None
        self._csv_writer = None
        csv_path = self._run_dir / 'data.csv' if self._run_dir else None
        plot_dir = self._run_dir / 'plots' if self._run_dir else None
        self.get_logger().info(f'Logging stopped ({source}) -> {self._run_dir}')
        if plot and csv_path is not None and csv_path.is_file() and csv_path.stat().st_size > 0:
            try:
                from exo_utils import plot_run
                plot_dir.mkdir(parents=True, exist_ok=True)
                plot_run.generate_plots(str(csv_path), str(plot_dir), show=False)
                self.get_logger().info(f'Plots written to {plot_dir}')
            except Exception as exc:
                self.get_logger().error(f'Plotting failed: {exc}')
        self._header = None

    # ---- Services ----

    def _on_start_logging(self, _req, resp: Trigger.Response) -> Trigger.Response:
        self._begin_run(source='manual_service')
        resp.success = True
        resp.message = str(self._run_dir) if self._run_dir else ''
        return resp

    def _on_stop_logging(self, _req, resp: Trigger.Response) -> Trigger.Response:
        if not self._recording:
            resp.success = False
            resp.message = 'not recording'
            return resp
        self._end_run(source='manual_service')
        resp.success = True
        resp.message = 'stopped'
        return resp


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._end_run(source='shutdown')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
