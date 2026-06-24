"""ROS 2 side: launches the read-only exo stack and records joint states.

The stack is spawned as `ros2 launch exo_bringup exo.launch.py arms:=<side>
control_mode:=read_only ...` (motors idle, encoders streaming at 500 Hz) and
stopped with SIGINT so the launch system shuts controllers down cleanly.
"""
import ctypes
import datetime
import signal
import subprocess
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

import config


def _set_pdeathsig():
    """Make the child receive SIGINT if the GUI process dies (Linux only).

    Without this, a crashed/killed GUI leaves the ros2 launch subprocess
    behind; several of those then fight over the Teensy serial port and the
    encoders freeze (observed in practice — three stale stacks at once)."""
    PR_SET_PDEATHSIG = 1
    ctypes.CDLL("libc.so.6", use_errno=True).prctl(PR_SET_PDEATHSIG, signal.SIGINT)


def external_stack_running():
    """True if an exo stack is already running outside this app."""
    return subprocess.run(
        ["pgrep", "-f", "exo.launch.py|ros2_control_node"],
        stdout=subprocess.DEVNULL,
    ).returncode == 0


class StackLauncher:
    """Manages the `ros2 launch` subprocess for the read-only stack."""

    def __init__(self):
        self.proc = None
        self.side = None
        self.log_path = None
        self.attached = False  # using a stack we did not start

    def start(self, side: str):
        if self.running():
            raise RuntimeError("stack already running")
        if external_stack_running():
            # Don't spawn a second ros2_control_node onto the same Teensy
            # port; just attach to the stack that is already up.
            self.attached = True
            self.side = side
            return
        self.attached = False
        self.side = side
        self.log_path = f"/tmp/exo_stack_{datetime.datetime.now():%Y%m%d_%H%M%S}.log"
        self._log_file = open(self.log_path, "w")
        self.proc = subprocess.Popen(
            [
                "ros2", "launch", "exo_bringup", "exo.launch.py",
                f"arms:={side}",
                "control_mode:=read_only",
                "enable_rviz:=false",
                "enable_data_logger:=false",
            ],
            stdout=self._log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=_set_pdeathsig,
        )

    def running(self):
        if self.attached:
            return external_stack_running()
        return self.proc is not None and self.proc.poll() is None

    def stop(self, timeout=10.0):
        self.attached = False  # never stop a stack we did not start
        if self.proc is None:
            return
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        self._log_file.close()
        self.proc = None


class JointStateRecorder(Node):
    """Subscribes to the per-arm joint_state_broadcaster topic (500 Hz)."""

    def __init__(self, side: str):
        super().__init__("emg_acq_joint_recorder")
        self.side = side
        self.joint_names = []
        self.last_msg_time = 0.0   # monotonic arrival time of latest message
        self.latest = None         # (positions, velocities, efforts)
        # Rolling (t_monotonic, positions) history for the live plot, filled at
        # the full 500 Hz message rate regardless of recording state.
        self.plot_buf = deque(maxlen=int(5.0 * 500.0))
        self._lock = threading.Lock()
        self._pending = []         # rows drained by the recorder
        self._recording = False
        self._t0 = 0.0
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.sub = self.create_subscription(
            JointState,
            config.JOINT_STATE_TOPIC_FMT.format(side=side),
            self._callback,
            qos,
        )

    def _callback(self, msg: JointState):
        now = time.monotonic()
        self.last_msg_time = now
        if not self.joint_names:
            self.joint_names = list(msg.name)
        pos = list(msg.position)
        vel = list(msg.velocity) if msg.velocity else [float("nan")] * len(pos)
        eff = list(msg.effort) if msg.effort else [float("nan")] * len(pos)
        self.latest = (pos, vel, eff)
        self.plot_buf.append((now, pos))
        if self._recording:
            with self._lock:
                self._pending.append([now - self._t0] + pos + vel + eff)

    def start_recording(self, t0: float):
        with self._lock:
            self._pending = []
        self._t0 = t0
        self._recording = True

    def stop_recording(self):
        self._recording = False

    def drain(self):
        with self._lock:
            out = self._pending
            self._pending = []
        return out

    def alive(self, max_age=0.5):
        """True if a joint_states message arrived within the last max_age seconds."""
        return (time.monotonic() - self.last_msg_time) < max_age


class RosBridge:
    """Owns the rclpy context, a background spin thread, and the recorder node."""

    def __init__(self):
        self.recorder = None
        self._spin_thread = None

    def start(self, side: str):
        if self.recorder is not None:
            self.stop()
        if not rclpy.ok():
            rclpy.init()
        self.recorder = JointStateRecorder(side)
        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self.recorder,), daemon=True
        )
        self._spin_thread.start()

    def stop(self):
        if self.recorder is not None:
            self.recorder.destroy_node()
            self.recorder = None
        if rclpy.ok():
            rclpy.shutdown()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
            self._spin_thread = None
