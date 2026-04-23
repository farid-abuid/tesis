"""Records joint states to CSV. Auto-detects single vs dual arm from live topics."""

import csv
import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


# Canonical broadcaster topics for dual-arm bringups (use_local_topics: true).
_DUAL_TOPICS = (
    '/left_joint_state_broadcaster/joint_states',
    '/right_joint_state_broadcaster/joint_states',
)
_SINGLE_TOPIC = '/joint_states'


class TrajectoryRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__('exo_trajectory_recorder')
        self.declare_parameter('frequency', 1000.0)
        self.declare_parameter('output_file', '~/exo_logs/trajectory.csv')
        self.declare_parameter('joints', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('joint_states_topic', '')
        self.declare_parameter('joint_states_topics', rclpy.Parameter.Type.STRING_ARRAY)

        self._freq = float(self.get_parameter('frequency').value)
        self._output_file = Path(os.path.expanduser(
            self.get_parameter('output_file').value))

        try:
            self._joints_filter = list(self.get_parameter('joints').value)
        except rclpy.exceptions.ParameterUninitializedException:
            self._joints_filter = []

        single_topic = str(self.get_parameter('joint_states_topic').value).strip()
        try:
            explicit_topics = [str(t).strip() for t in
                               (self.get_parameter('joint_states_topics').value or [])
                               if str(t).strip()]
        except rclpy.exceptions.ParameterUninitializedException:
            explicit_topics = []

        # Resolve topic list: explicit > single param > auto-detect.
        if explicit_topics:
            topics = explicit_topics
        elif single_topic:
            topics = [single_topic]
        else:
            topics = self._auto_detect_topics()

        self._topics: list[str] = topics
        self._latest: dict[str, JointState] = {}
        self._recording = False
        self._csv_file = None
        self._csv_writer = None
        self._joint_names: list[str] = []

        for topic in self._topics:
            self.create_subscription(
                JointState, topic,
                lambda m, t=topic: self._on_joint_state(t, m), 10,
            )

        self._timer = self.create_timer(1.0 / self._freq, self._sample_tick)
        self._timer.cancel()

        self._srv_start = self.create_service(
            Trigger, '~/start_recording', self._on_start)
        self._srv_stop = self.create_service(
            Trigger, '~/stop_recording', self._on_stop)

        self.get_logger().info(
            f'Trajectory recorder ready (freq={self._freq} Hz, '
            f'output={self._output_file}, topics={self._topics})')

    def _auto_detect_topics(self) -> list[str]:
        # Give the ROS graph a moment to propagate before querying publishers.
        names_and_types = dict(self.get_topic_names_and_types())
        dual_present = all(t in names_and_types for t in _DUAL_TOPICS)
        if dual_present:
            self.get_logger().info('Auto-detected dual-arm broadcasters')
            return list(_DUAL_TOPICS)
        return [_SINGLE_TOPIC]

    def _on_joint_state(self, topic: str, msg: JointState) -> None:
        self._latest[topic] = msg

    def _sample_tick(self) -> None:
        if not self._recording or not self._latest:
            return

        # Use the newest timestamp across topics as the row time.
        t = 0.0
        for msg in self._latest.values():
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if ts > t:
                t = ts

        if not self._joint_names:
            self._joint_names = self._resolve_joint_names()
            if not self._joint_names:
                return
            header = ['time_sec']
            for j in self._joint_names:
                header += [f'{j}_pos', f'{j}_vel']
            self._csv_writer.writerow(header)

        # Build a combined name -> (pos, vel) lookup from the latest per-topic msgs.
        combined: dict[str, tuple[float, float]] = {}
        for msg in self._latest.values():
            for i, name in enumerate(msg.name):
                pos = msg.position[i] if i < len(msg.position) else 0.0
                vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
                combined[name] = (float(pos), float(vel))

        row = [t]
        for j in self._joint_names:
            pos, vel = combined.get(j, (0.0, 0.0))
            row += [pos, vel]
        self._csv_writer.writerow(row)

    def _resolve_joint_names(self) -> list[str]:
        # Union of joint names across all cached messages, preserving topic order.
        seen: set[str] = set()
        ordered: list[str] = []
        for topic in self._topics:
            msg = self._latest.get(topic)
            if msg is None:
                continue
            for name in msg.name:
                if name not in seen:
                    seen.add(name)
                    ordered.append(name)
        if self._joints_filter:
            return [j for j in self._joints_filter if j in seen]
        return ordered

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
