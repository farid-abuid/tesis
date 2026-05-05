"""Read BNO055 orientation from Teensy SerialUSB1 and broadcast as TF + sensor_msgs/Imu.

Frame layout from Teensy (ImuRpyStatus, packed, 24 bytes):
  float roll1   = orientation.x  (heading/yaw,  degrees)
  float pitch1  = orientation.y  (pitch,         degrees)
  float yaw1    = orientation.z  (roll,           degrees)
  float roll2   = orientation.x  (heading/yaw,  degrees)
  float pitch2  = orientation.y  (pitch,         degrees)
  float yaw2    = orientation.z  (roll,           degrees)

Note: Teensy field naming is misleading — the Adafruit BNO055 getEvent()
orientation struct maps .x→heading, .y→pitch, .z→roll.  We store them here
as (heading, pitch, roll) per arm and use the ZYX Euler convention to build
the quaternion (same convention the BNO055 fusion engine uses internally).

IMU1 → right arm base link, IMU2 → left arm base link.
"""

from __future__ import annotations

import math
import struct
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError:
    print("pyserial not found — install with: pip install pyserial", file=sys.stderr)
    serial = None

try:
    from scipy.spatial.transform import Rotation
except ImportError:
    print("scipy not found — install with: pip install scipy", file=sys.stderr)
    Rotation = None

HEADER1 = 0xAA
HEADER2 = 0x55
IMU_STRUCT = struct.Struct("<ffffff")  # 6 floats, little-endian, 24 bytes
BAUD = 460800


def _rpy_to_quaternion(heading_deg: float, pitch_deg: float, roll_deg: float):
    """Return (w, x, y, z) quaternion from BNO055 ZYX Euler angles (degrees).

    BNO055 convention: heading = rotation about Z (North), pitch about Y, roll about X.
    Rotation.from_euler('ZYX', ...) matches this convention.
    The resulting quaternion represents the rotation FROM sensor frame TO world frame.
    """
    r = Rotation.from_euler("ZYX", [heading_deg, pitch_deg, roll_deg], degrees=True)
    xyzw = r.as_quat()  # scipy returns [x, y, z, w]
    return float(xyzw[3]), float(xyzw[0]), float(xyzw[1]), float(xyzw[2])  # w, x, y, z


def _read_imu_frame(ser) -> tuple[float, ...] | None:
    """Block-read one [0xAA][0x55][24 bytes] frame.  Returns 6 floats or None on error."""
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] != HEADER1:
            continue
        b2 = ser.read(1)
        if not b2 or b2[0] != HEADER2:
            continue
        payload = ser.read(IMU_STRUCT.size)
        if len(payload) != IMU_STRUCT.size:
            return None
        return IMU_STRUCT.unpack(payload)


class TeensyImuNode(Node):
    def __init__(self) -> None:
        super().__init__("exo_imu_node")

        self.declare_parameter("port", "/dev/teensy_imu")
        self.declare_parameter("arms", "dual")
        # Per-arm RPY calibration offsets (degrees) applied AFTER the BNO055 reading.
        # Adjust these to compensate for IMU chip mis-alignment on the mount.
        self.declare_parameter("right_roll_offset", 0.0)
        self.declare_parameter("right_pitch_offset", 0.0)
        self.declare_parameter("right_yaw_offset", 0.0)
        self.declare_parameter("left_roll_offset", 0.0)
        self.declare_parameter("left_pitch_offset", 0.0)
        self.declare_parameter("left_yaw_offset", 0.0)
        # Mount translation offsets (metres) — must match the URDF fixed mount joint values
        # so the dynamic TF includes the correct position component.
        self.declare_parameter("right_mount_x", 0.0)
        self.declare_parameter("right_mount_y", -0.20)
        self.declare_parameter("right_mount_z", 0.0)
        self.declare_parameter("left_mount_x", 0.0)
        self.declare_parameter("left_mount_y", 0.20)
        self.declare_parameter("left_mount_z", 0.0)

        port = str(self.get_parameter("port").value)
        arms = str(self.get_parameter("arms").value)

        self._active_arms: list[str] = []
        if arms in ("right", "dual"):
            self._active_arms.append("right")
        if arms in ("left", "dual"):
            self._active_arms.append("left")

        self._right_offset = (
            float(self.get_parameter("right_roll_offset").value),
            float(self.get_parameter("right_pitch_offset").value),
            float(self.get_parameter("right_yaw_offset").value),
        )
        self._left_offset = (
            float(self.get_parameter("left_roll_offset").value),
            float(self.get_parameter("left_pitch_offset").value),
            float(self.get_parameter("left_yaw_offset").value),
        )
        self._mount_pos = {
            "right": (
                float(self.get_parameter("right_mount_x").value),
                float(self.get_parameter("right_mount_y").value),
                float(self.get_parameter("right_mount_z").value),
            ),
            "left": (
                float(self.get_parameter("left_mount_x").value),
                float(self.get_parameter("left_mount_y").value),
                float(self.get_parameter("left_mount_z").value),
            ),
        }

        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._imu_pubs: dict[str, rclpy.publisher.Publisher] = {}
        for arm in self._active_arms:
            self._imu_pubs[arm] = self.create_publisher(Imu, f"/{arm}/imu", imu_qos)

        self._tf_broadcaster = TransformBroadcaster(self)

        if serial is None or Rotation is None:
            self.get_logger().error(
                "Missing dependencies (pyserial or scipy). Cannot start serial reader."
            )
            return

        try:
            self._ser = serial.Serial(port=port, baudrate=BAUD, timeout=0.5)
            self._ser.reset_input_buffer()
        except Exception as e:
            self.get_logger().error(f"Failed to open {port}: {e}")
            return

        self.get_logger().info(
            f"Opened {port} at {BAUD} baud — active arms: {self._active_arms}"
        )

        self._reader_thread = threading.Thread(
            target=self._reader_loop, daemon=True, name="imu_reader"
        )
        self._reader_thread.start()

    def _apply_offset(
        self, heading: float, pitch: float, roll: float, offset: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        """Compose IMU reading with calibration offset rotation."""
        r_imu = Rotation.from_euler("ZYX", [heading, pitch, roll], degrees=True)
        r_off = Rotation.from_euler("ZYX", [offset[2], offset[1], offset[0]], degrees=True)
        r_combined = r_imu * r_off
        h, p, ro = r_combined.as_euler("ZYX", degrees=True)
        return float(h), float(p), float(ro)

    def _publish_arm(
        self, arm: str, heading: float, pitch: float, roll: float
    ) -> None:
        offset = self._right_offset if arm == "right" else self._left_offset
        heading, pitch, roll = self._apply_offset(heading, pitch, roll, offset)
        w, x, y, z = _rpy_to_quaternion(heading, pitch, roll)

        now = self.get_clock().now().to_msg()
        child_frame = f"{arm}_base_link"
        mx, my, mz = self._mount_pos[arm]

        # Dynamic TF: base_link → <arm>_base_link
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = "base_link"
        tf_msg.child_frame_id = child_frame
        tf_msg.transform.translation.x = mx
        tf_msg.transform.translation.y = my
        tf_msg.transform.translation.z = mz
        tf_msg.transform.rotation.w = w
        tf_msg.transform.rotation.x = x
        tf_msg.transform.rotation.y = y
        tf_msg.transform.rotation.z = z
        self._tf_broadcaster.sendTransform(tf_msg)

        # sensor_msgs/Imu (orientation only; angular_velocity/linear_acceleration unavailable)
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = child_frame
        imu_msg.orientation.w = w
        imu_msg.orientation.x = x
        imu_msg.orientation.y = y
        imu_msg.orientation.z = z
        # covariance = -1 signals "not provided"
        imu_msg.orientation_covariance[0] = 0.01
        imu_msg.orientation_covariance[4] = 0.01
        imu_msg.orientation_covariance[8] = 0.01
        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0
        self._imu_pubs[arm].publish(imu_msg)

    def _reader_loop(self) -> None:
        while rclpy.ok():
            frame = _read_imu_frame(self._ser)
            if frame is None:
                continue
            # Unpack: (heading1, pitch1, roll1, heading2, pitch2, roll2)
            # Teensy stores as (roll1=heading, pitch1=pitch, yaw1=roll) — see module docstring
            h1, p1, r1, h2, p2, r2 = frame

            if "right" in self._active_arms:
                self._publish_arm("right", h1, p1, r1)
            if "left" in self._active_arms:
                self._publish_arm("left", h2, p2, r2)

    def destroy_node(self) -> None:
        if hasattr(self, "_ser") and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = TeensyImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
