"""Replays a trajectory CSV. Splits left_/right_ joints onto per-arm command topics."""

import csv
import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration as DurationMsg
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectoryPlayerNode(Node):
    def __init__(self) -> None:
        super().__init__('exo_trajectory_player')
        self.declare_parameter('input_file', '')
        # Single-arm defaults; dual-arm uses the per-side overrides below.
        self.declare_parameter('trajectory_topic', '/reference_trajectory')
        self.declare_parameter('position_command_topic', '/position_controller/commands')
        # Dual-arm overrides. Leave empty to use controller-specific defaults based on CSV prefixes.
        self.declare_parameter('left_trajectory_topic', '/left/reference_trajectory')
        self.declare_parameter('right_trajectory_topic', '/right/reference_trajectory')
        self.declare_parameter('left_position_command_topic',
                               '/left_position_controller/commands')
        self.declare_parameter('right_position_command_topic',
                               '/right_position_controller/commands')
        self.declare_parameter('frequency', 0.0)
        self.declare_parameter('loop', False)

        input_file = self.get_parameter('input_file').value
        if not input_file:
            self.get_logger().fatal('input_file parameter is required')
            raise ValueError('trajectory_player: input_file not set')

        self._input_path = Path(os.path.expanduser(input_file))
        self._freq_override = float(self.get_parameter('frequency').value)
        self._loop = bool(self.get_parameter('loop').value)

        self._joint_names: list[str] = []
        self._rows: list[dict] = []
        self._load_csv()

        if not self._rows:
            self.get_logger().error(f'No data rows in {self._input_path}')
            raise ValueError('trajectory_player: empty CSV')

        # Group joints by arm (prefix-based).
        self._groups: dict[str, list[tuple[int, str]]] = {}
        for i, j in enumerate(self._joint_names):
            if j.startswith('left_'):
                key = 'left'
            elif j.startswith('right_'):
                key = 'right'
            else:
                key = 'main'
            self._groups.setdefault(key, []).append((i, j))

        # Create publishers per group. Single-arm CSVs keep the legacy topic names.
        self._pubs: dict[str, tuple] = {}
        topic_map = {
            'main': (str(self.get_parameter('trajectory_topic').value),
                     str(self.get_parameter('position_command_topic').value)),
            'left': (str(self.get_parameter('left_trajectory_topic').value),
                     str(self.get_parameter('left_position_command_topic').value)),
            'right': (str(self.get_parameter('right_trajectory_topic').value),
                      str(self.get_parameter('right_position_command_topic').value)),
        }
        for key in self._groups:
            traj_topic, pos_topic = topic_map[key]
            traj_pub = self.create_publisher(JointTrajectory, traj_topic, 10)
            pos_pub = self.create_publisher(Float64MultiArray, pos_topic, 10)
            self._pubs[key] = (traj_pub, pos_pub, traj_topic, pos_topic)

        if self._freq_override > 0.0:
            freq = self._freq_override
        else:
            if len(self._rows) >= 4:
                t0 = float(self._rows[2]['time_sec'])
                t1 = float(self._rows[3]['time_sec'])
                dt = t1 - t0
                freq = 1.0 / dt if dt > 0.0 else 1000.0
            else:
                freq = 1000.0

        self._current_idx = 0
        self._timer = self.create_timer(1.0 / freq, self._publish_tick)

        summary = ', '.join(
            f'{k}({len(v)} joints -> {self._pubs[k][3]})'
            for k, v in self._groups.items()
        )
        self.get_logger().info(
            f'Trajectory player: {len(self._rows)} points at {freq:.1f} Hz '
            f'from {self._input_path} [{summary}] (loop={self._loop})')

    def _load_csv(self) -> None:
        with self._input_path.open(newline='') as f:
            reader = csv.DictReader(f)
            if reader.fieldnames is None:
                return
            fields = list(reader.fieldnames)
            seen: list[str] = []
            for col in fields:
                if col.endswith('_pos'):
                    seen.append(col[:-4])
            self._joint_names = seen
            self._rows = list(reader)

    def _publish_tick(self) -> None:
        if self._current_idx >= len(self._rows):
            if self._loop:
                self._current_idx = 0
            else:
                self._timer.cancel()
                self.get_logger().info('Trajectory playback finished')
                return

        row = self._rows[self._current_idx]

        for key, joints in self._groups.items():
            traj_pub, pos_pub, _, _ = self._pubs[key]
            msg = JointTrajectory()
            msg.joint_names = [name for _, name in joints]

            pt = JointTrajectoryPoint()
            pt.positions = []
            pt.velocities = []
            for _, j in joints:
                pt.positions.append(float(row.get(f'{j}_pos', 0.0)))
                pt.velocities.append(float(row.get(f'{j}_vel', 0.0)))
            pt.time_from_start = DurationMsg(sec=0, nanosec=0)
            msg.points = [pt]

            traj_pub.publish(msg)
            pos_msg = Float64MultiArray()
            pos_msg.data = list(pt.positions)
            pos_pub.publish(pos_msg)

        self._current_idx += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryPlayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
