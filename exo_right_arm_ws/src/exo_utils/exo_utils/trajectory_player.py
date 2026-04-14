"""Reads a trajectory CSV and publishes JointTrajectory messages at the recorded frequency."""

import csv
import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration as DurationMsg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectoryPlayerNode(Node):
    def __init__(self) -> None:
        super().__init__('exo_trajectory_player')
        self.declare_parameter('input_file', '')
        self.declare_parameter('trajectory_topic', '/reference_trajectory')
        self.declare_parameter('frequency', 0.0)
        self.declare_parameter('loop', False)

        input_file = self.get_parameter('input_file').value
        if not input_file:
            self.get_logger().fatal('input_file parameter is required')
            raise ValueError('trajectory_player: input_file not set')

        self._input_path = Path(os.path.expanduser(input_file))
        self._topic = self.get_parameter('trajectory_topic').value
        self._freq_override = self.get_parameter('frequency').value
        self._loop = self.get_parameter('loop').value

        self._pub = self.create_publisher(JointTrajectory, self._topic, 10)

        self._joint_names: list[str] = []
        self._rows: list[dict] = []
        self._load_csv()

        if not self._rows:
            self.get_logger().error(f'No data rows in {self._input_path}')
            raise ValueError('trajectory_player: empty CSV')

        # Infer frequency from timestamps if not overridden.
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

        self.get_logger().info(
            f'Trajectory player: {len(self._rows)} points at {freq:.1f} Hz '
            f'from {self._input_path} -> {self._topic} (loop={self._loop})')

    def _load_csv(self) -> None:
        with self._input_path.open(newline='') as f:
            reader = csv.DictReader(f)
            if reader.fieldnames is None:
                return
            fields = list(reader.fieldnames)

            # Discover joints from header columns: <joint>_pos / <joint>_vel
            seen = []
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

        msg = JointTrajectory()
        msg.joint_names = self._joint_names

        pt = JointTrajectoryPoint()
        pt.positions = []
        pt.velocities = []
        for j in self._joint_names:
            pt.positions.append(float(row.get(f'{j}_pos', 0.0)))
            pt.velocities.append(float(row.get(f'{j}_vel', 0.0)))

        pt.time_from_start = DurationMsg(sec=0, nanosec=0)
        msg.points = [pt]

        self._pub.publish(msg)
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
