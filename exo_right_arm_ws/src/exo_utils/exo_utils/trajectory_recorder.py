"""Records joint states from /joint_states at a configurable frequency and saves to CSV."""

import csv
import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class TrajectoryRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__('exo_trajectory_recorder')
        self.declare_parameter('frequency', 1000.0)
        self.declare_parameter('output_file', '~/exo_logs/trajectory.csv')
        self.declare_parameter('joints', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('joint_states_topic', '/joint_states')

        self._freq = self.get_parameter('frequency').value
        self._output_file = Path(os.path.expanduser(
            self.get_parameter('output_file').value))
        self._joint_states_topic = self.get_parameter('joint_states_topic').value

        try:
            self._joints_filter = list(self.get_parameter('joints').value)
        except rclpy.exceptions.ParameterUninitializedException:
            self._joints_filter = []

        self._latest_msg: JointState | None = None
        self._recording = False
        self._csv_file = None
        self._csv_writer = None
        self._joint_names: list[str] = []

        self.create_subscription(
            JointState, self._joint_states_topic, self._on_joint_state, 10)

        self._timer = self.create_timer(1.0 / self._freq, self._sample_tick)
        self._timer.cancel()

        self._srv_start = self.create_service(
            Trigger, '~/start_recording', self._on_start)
        self._srv_stop = self.create_service(
            Trigger, '~/stop_recording', self._on_stop)

        self.get_logger().info(
            f'Trajectory recorder ready (freq={self._freq} Hz, '
            f'output={self._output_file}, topic={self._joint_states_topic})')

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_msg = msg

    def _sample_tick(self) -> None:
        if not self._recording or self._latest_msg is None:
            return

        msg = self._latest_msg
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if not self._joint_names:
            if self._joints_filter:
                self._joint_names = [
                    j for j in self._joints_filter if j in msg.name]
            else:
                self._joint_names = list(msg.name)

            header = ['time_sec']
            for j in self._joint_names:
                header += [f'{j}_pos', f'{j}_vel']
            self._csv_writer.writerow(header)

        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        row = [t]
        for j in self._joint_names:
            idx = name_to_idx.get(j)
            if idx is not None:
                pos = msg.position[idx] if idx < len(msg.position) else 0.0
                vel = msg.velocity[idx] if idx < len(msg.velocity) else 0.0
            else:
                pos = 0.0
                vel = 0.0
            row += [pos, vel]

        self._csv_writer.writerow(row)

    def _start_recording(self) -> str:
        if self._recording:
            return str(self._output_file)

        self._output_file.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = open(self._output_file, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._joint_names = []
        self._recording = True
        self._timer.reset()

        self.get_logger().info(f'Recording started -> {self._output_file}')
        return str(self._output_file)

    def _stop_recording(self) -> None:
        if not self._recording:
            return
        self._recording = False
        self._timer.cancel()
        if self._csv_file is not None:
            self._csv_file.flush()
            self._csv_file.close()
            self._csv_file = None
        self._csv_writer = None
        self.get_logger().info('Recording stopped')

    def _on_start(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        path = self._start_recording()
        resp.success = True
        resp.message = path
        return resp

    def _on_stop(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        if not self._recording:
            resp.success = False
            resp.message = 'not recording'
            return resp
        self._stop_recording()
        resp.success = True
        resp.message = 'stopped'
        return resp


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryRecorderNode()

    # Auto-start recording on launch.
    node._start_recording()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_recording()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
