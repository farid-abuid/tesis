"""Subscribes to JointControlTelemetry; writes CSV and runs plotting on run end."""

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
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class DataLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__('exo_data_logger')
        self.declare_parameter('telemetry_topic', '')
        self.declare_parameter('session_topic', '')
        self.declare_parameter('log_root', '~/exo_logs')
        self.declare_parameter('use_controller_session', True)
        self.declare_parameter('use_lifecycle_events', True)
        self.declare_parameter('lifecycle_transition_topic', '')
        self.declare_parameter('auto_plot', True)
        self.declare_parameter('position_command_topic', '')
        self.declare_parameter('velocity_command_topic', '')
        self.declare_parameter('effort_command_topic', '')
        self.declare_parameter('joint_state_topic', '/joint_states')

        self._telemetry_topic = str(self.get_parameter('telemetry_topic').value).strip()
        self._session_topic = str(self.get_parameter('session_topic').value).strip()
        self._log_root = Path(os.path.expanduser(self.get_parameter('log_root').value))
        self._use_controller_session = self.get_parameter('use_controller_session').value
        self._use_lifecycle_events = self.get_parameter('use_lifecycle_events').value
        self._lifecycle_topic = str(
            self.get_parameter('lifecycle_transition_topic').value
        ).strip()
        self._auto_plot = self.get_parameter('auto_plot').value
        self._position_cmd_topic = str(self.get_parameter('position_command_topic').value).strip()
        self._velocity_cmd_topic = str(self.get_parameter('velocity_command_topic').value).strip()
        self._effort_cmd_topic = str(self.get_parameter('effort_command_topic').value).strip()
        self._joint_state_topic = str(self.get_parameter('joint_state_topic').value).strip()

        if not self._telemetry_topic or not self._session_topic or not self._lifecycle_topic:
            self.get_logger().fatal(
                'telemetry_topic, session_topic, and lifecycle_transition_topic must be set '
                '(gazebo.launch.py passes them from control_mode).'
            )
            raise ValueError('exo_data_logger: missing topic parameter(s)')

        self._recording = False
        self._csv_file = None
        self._csv_writer = None
        self._header: list[str] | None = None
        self._run_dir: Path | None = None
        self._last_session_val: bool | None = None
        self._latest_position_cmd: list[float] | None = None
        self._latest_velocity_cmd: list[float] | None = None
        self._latest_effort_cmd: list[float] | None = None
        self._latest_joint_names: list[str] = []
        self._latest_joint_position: list[float] | None = None
        self._latest_joint_velocity: list[float] | None = None
        self._latest_joint_effort: list[float] | None = None
        self._joint_count_for_command_header: int | None = None
        self._header_source: str | None = None

        telem_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            JointControlTelemetry, self._telemetry_topic, self._on_telemetry, telem_qos
        )
        if self._position_cmd_topic:
            self.create_subscription(
                Float64MultiArray, self._position_cmd_topic, self._on_position_command, 10
            )
        if self._velocity_cmd_topic:
            self.create_subscription(
                Float64MultiArray, self._velocity_cmd_topic, self._on_velocity_command, 10
            )
        if self._effort_cmd_topic:
            self.create_subscription(
                Float64MultiArray, self._effort_cmd_topic, self._on_effort_command, 10
            )
        if self._joint_state_topic:
            self.create_subscription(JointState, self._joint_state_topic, self._on_joint_state, 10)

        session_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Bool, self._session_topic, self._on_session, session_qos)

        if self._use_controller_session and self._use_lifecycle_events:
            lc_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            self.create_subscription(
                TransitionEvent, self._lifecycle_topic, self._on_lifecycle_transition, lc_qos
            )

        self._srv_start = self.create_service(Trigger, '~/start_logging', self._on_start_logging)
        self._srv_stop = self.create_service(Trigger, '~/stop_logging', self._on_stop_logging)
        extra = ''
        if self._use_controller_session and self._use_lifecycle_events:
            extra = f' lifecycle={self._lifecycle_topic}'
        self.get_logger().info(
            f'exo_data_logger ready (telemetry={self._telemetry_topic} '
            f'session={self._session_topic}{extra})'
        )

    def _build_header(self, msg: JointControlTelemetry) -> list[str]:
        # Column naming matches plot_run expectations (q = position, etc.)
        header = ['time_sec']
        for j in msg.joint_names:
            cols = [
                f'{j}_q',
                f'{j}_dq',
                f'{j}_q_des',
                f'{j}_dq_des',
                f'{j}_tau_cmd',
                f'{j}_tau_ff',
            ]
            if len(msg.effort_measured) == len(msg.joint_names):
                cols.append(f'{j}_tau_meas')
            cols += [f'{j}_q_cmd', f'{j}_dq_cmd', f'{j}_tau_cmd_in']
            header += cols
        if (
            len(msg.operational_position_actual) >= 3
            and len(msg.operational_position_desired) >= 3
        ):
            header += ['ee_x', 'ee_y', 'ee_z', 'ee_des_x', 'ee_des_y', 'ee_des_z']
        return header

    def _build_command_only_header(self, count: int) -> list[str]:
        header = ['time_sec']
        for i in range(count):
            joint = self._joint_name_for_index(i)
            header += [
                f'{joint}_q',
                f'{joint}_dq',
                f'{joint}_tau_meas',
                f'{joint}_q_cmd',
                f'{joint}_dq_cmd',
                f'{joint}_tau_cmd_in',
            ]
        return header

    def _joint_name_for_index(self, index: int) -> str:
        if index < len(self._latest_joint_names):
            return self._latest_joint_names[index]
        return f'joint_{index + 1}'

    def _row_from_msg(self, msg: JointControlTelemetry) -> list[float]:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        row: list[float] = [t]
        n = len(msg.joint_names)
        include_tau_meas = len(msg.effort_measured) == n
        for i in range(n):
            row += [
                float(msg.position[i]),
                float(msg.velocity[i]),
                float(msg.position_reference[i]),
                float(msg.velocity_reference[i]),
                float(msg.effort_command[i]),
                float(msg.effort_feedforward[i]),
            ]
            if include_tau_meas:
                row.append(float(msg.effort_measured[i]))
            q_cmd = self._latest_position_cmd[i] if self._latest_position_cmd and i < len(self._latest_position_cmd) else float('nan')
            dq_cmd = self._latest_velocity_cmd[i] if self._latest_velocity_cmd and i < len(self._latest_velocity_cmd) else float('nan')
            tau_cmd_in = self._latest_effort_cmd[i] if self._latest_effort_cmd and i < len(self._latest_effort_cmd) else float('nan')
            row += [float(q_cmd), float(dq_cmd), float(tau_cmd_in)]
        if (
            len(msg.operational_position_actual) >= 3
            and len(msg.operational_position_desired) >= 3
        ):
            row += [
                float(msg.operational_position_actual[0]),
                float(msg.operational_position_actual[1]),
                float(msg.operational_position_actual[2]),
                float(msg.operational_position_desired[0]),
                float(msg.operational_position_desired[1]),
                float(msg.operational_position_desired[2]),
            ]
        return row

    def _row_from_commands(self, stamp_sec: float) -> list[float]:
        count = self._joint_count_for_command_header or 0
        row: list[float] = [stamp_sec]
        for i in range(count):
            q = self._latest_joint_position[i] if self._latest_joint_position and i < len(self._latest_joint_position) else float('nan')
            dq = self._latest_joint_velocity[i] if self._latest_joint_velocity and i < len(self._latest_joint_velocity) else float('nan')
            tau_meas = self._latest_joint_effort[i] if self._latest_joint_effort and i < len(self._latest_joint_effort) else float('nan')
            q_cmd = self._latest_position_cmd[i] if self._latest_position_cmd and i < len(self._latest_position_cmd) else float('nan')
            dq_cmd = self._latest_velocity_cmd[i] if self._latest_velocity_cmd and i < len(self._latest_velocity_cmd) else float('nan')
            tau_cmd_in = self._latest_effort_cmd[i] if self._latest_effort_cmd and i < len(self._latest_effort_cmd) else float('nan')
            row += [float(q), float(dq), float(tau_meas), float(q_cmd), float(dq_cmd), float(tau_cmd_in)]
        return row

    def _on_telemetry(self, msg: JointControlTelemetry) -> None:
        if not self._recording or self._csv_writer is None:
            return
        if self._header is None:
            self._header = self._build_header(msg)
            self._header_source = 'telemetry'
            self._csv_writer.writerow(self._header)
        row = self._row_from_msg(msg)
        if len(row) != len(self._header):
            self.get_logger().warn('Telemetry shape changed mid-run; skipping row')
            return
        self._csv_writer.writerow(row)

    def _on_position_command(self, msg: Float64MultiArray) -> None:
        self._latest_position_cmd = [float(v) for v in msg.data]
        self._on_command_stream_update(len(msg.data))

    def _on_velocity_command(self, msg: Float64MultiArray) -> None:
        self._latest_velocity_cmd = [float(v) for v in msg.data]
        self._on_command_stream_update(len(msg.data))

    def _on_effort_command(self, msg: Float64MultiArray) -> None:
        self._latest_effort_cmd = [float(v) for v in msg.data]
        self._on_command_stream_update(len(msg.data))

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_joint_names = list(msg.name)
        if len(msg.position) > 0:
            self._latest_joint_position = [float(v) for v in msg.position]
        if len(msg.velocity) > 0:
            self._latest_joint_velocity = [float(v) for v in msg.velocity]
        if len(msg.effort) > 0:
            self._latest_joint_effort = [float(v) for v in msg.effort]

    def _on_command_stream_update(self, count: int) -> None:
        if count <= 0:
            return
        if self._joint_count_for_command_header is None:
            self._joint_count_for_command_header = count
        if not self._recording or self._csv_writer is None:
            return
        if self._header is None:
            self._header = self._build_command_only_header(self._joint_count_for_command_header)
            self._header_source = 'command'
            self._csv_writer.writerow(self._header)
        if self._header_source == 'telemetry':
            # Telemetry-backed CSVs already include latest command values on telemetry writes.
            return
        stamp_sec = self.get_clock().now().nanoseconds * 1e-9
        row = self._row_from_commands(stamp_sec)
        if len(row) != len(self._header):
            self.get_logger().warn('Command shape changed mid-run; skipping command row')
            return
        self._csv_writer.writerow(row)

    def _on_session(self, msg: Bool) -> None:
        if not self._use_controller_session:
            return
        if self._last_session_val is not None and msg.data == self._last_session_val:
            return
        self._last_session_val = bool(msg.data)
        if msg.data:
            self._begin_run(source='controller_activate', replace=False)
        else:
            self._end_run(source='controller_deactivate')

    def _on_lifecycle_transition(self, msg: TransitionEvent) -> None:
        if not self._use_controller_session or not self._use_lifecycle_events:
            return
        gid = msg.goal_state.id
        primary = {
            State.PRIMARY_STATE_UNCONFIGURED,
            State.PRIMARY_STATE_INACTIVE,
            State.PRIMARY_STATE_ACTIVE,
            State.PRIMARY_STATE_FINALIZED,
        }
        if gid not in primary:
            return
        if gid == State.PRIMARY_STATE_ACTIVE:
            self._begin_run(source='lifecycle_active', replace=False)
        else:
            self._end_run(source='lifecycle_inactive')

    def _begin_run(self, source: str, *, replace: bool = True) -> None:
        if self._recording and not replace:
            return
        self._end_run(source='restart', plot=self._auto_plot)
        self._log_root.mkdir(parents=True, exist_ok=True)
        run_id = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
        self._run_dir = self._log_root / run_id
        self._run_dir.mkdir(parents=True, exist_ok=False)
        csv_path = self._run_dir / 'data.csv'
        self._csv_file = open(csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._header = None
        self._joint_count_for_command_header = None
        self._header_source = None
        self._recording = True
        self.get_logger().info(f'Logging started ({source}) -> {csv_path}')

    def _end_run(self, source: str, plot: bool | None = None) -> None:
        if plot is None:
            plot = self._auto_plot
        if not self._recording:
            return
        self._recording = False
        if self._csv_file is not None:
            self._csv_file.flush()
            self._csv_file.close()
            self._csv_file = None
        self._csv_writer = None
        csv_path = self._run_dir / 'data.csv' if self._run_dir else None
        plot_dir = self._run_dir / 'plots' if self._run_dir else None
        self.get_logger().info(f'Logging stopped ({source})')
        if plot and csv_path is not None and csv_path.is_file() and csv_path.stat().st_size > 0:
            try:
                from exo_utils import plot_run
                plot_dir.mkdir(parents=True, exist_ok=True)
                plot_run.generate_plots(str(csv_path), str(plot_dir))
                self.get_logger().info(f'Plots written to {plot_dir}')
            except Exception as exc:
                self.get_logger().error(f'Plotting failed: {exc}')
        self._header = None
        self._header_source = None

    def _on_start_logging(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._begin_run(source='manual_service')
        response.success = True
        response.message = str(self._run_dir) if self._run_dir else ''
        return response

    def _on_stop_logging(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if not self._recording:
            response.success = False
            response.message = 'not recording'
            return response
        self._end_run(source='manual_service')
        response.success = True
        response.message = 'stopped'
        return response


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
