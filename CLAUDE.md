# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and source

```bash
cd ~/tesis/exo_right_arm_ws
colcon build --symlink-install
source install/setup.bash
```

If `colcon build` fails with `ModuleNotFoundError` (yaml, catkin_pkg), conda Python is shadowing the system Python:

```bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

Run tests for a single package:
```bash
colcon test --packages-select exo_bringup && colcon test-result --verbose
```

## Architecture overview

This is a **ROS 2 (Jazzy)** project controlling a bilateral upper-limb exoskeleton via the `ros2_control` framework.

### Hardware communication chain

```
ROS controller_manager
       ↓ ros2_control SystemInterface
exo_hardware (teensy_plugin)
       ↓ serial @ 460800 baud (/dev/teensy_motor)
Teensy 4.1
       ↓ CAN @ 1 Mbps
Motors (CAN IDs 0x141–0x146)
  Motors 1–3 → CAN1 (right arm)
  Motors 4–6 → CAN2 (left arm)
```

### Serial protocol (host ↔ Teensy)

Both directions use the same framing: `[0xAA][0x55][cmd_byte][payload][checksum_xor]`.

**Command byte** (host → Teensy): upper nibble = cmd_type, lower nibble = n_motors.
- `cmd_type 1`: torque (Nm), `2`: speed (rad/s), `3`: position (rad), `5`: read-only, `6`: stop, `7`: shutdown.

**Response** (Teensy → host): header + n×`MotorStatus2` structs (`motorID`, `iq_as_Nm`, `speed_rad/s`, `angle_rad`).

The `joint_dir_sign_` array in `exo_hardware` flips sign on both commands and state readings. Default pattern is `[+1, -1, +1]` repeating per arm.

### ROS packages

| Package | Role |
|---|---|
| `exo_hardware` | `ros2_control` SystemInterface plugin (`teensy_plugin`); serial read/write; stiction compensator |
| `exo_description` | URDF/xacro; STL meshes; RViz config. Arms always use prefixed joints (`right_revolute_1..3`, `left_revolute_1..3`) |
| `exo_bringup` | Launch files (`exo.launch.py`, `gazebo.launch.py`, `sliders_rviz.launch.py`); per-arm controller YAMLs; `launch_common.py` shared logic |
| `exo_utils` | Python utilities: data logger, trajectory recorder/player, plot tool, DC motor ID, serial RTT benchmark; C++ kinematics/dynamics headers |
| `exo_control_msgs` | Custom `JointControlTelemetry.msg` used by gravity comp controllers |
| `gravity_compensation_controller` | Gravity compensation controller using Pinocchio for `τ = g(q) + Kp·(q_des–q) + Kd·(dq_des–dq)` |
| `adaptive_slotine_controller` | Slotine-Li adaptive controller `τ = Y·θ̂ − Kd·s`, `θ̂̇ = −Γ·Yᵀ·s`; uses the arm-specific barycentric regressor `exo_Y` in `exo_utils` (right implemented; left is a stub awaiting the left-arm `robot_Y.m`) |
| `joint_state_broadcaster` | Custom fork of the standard broadcaster |

### Arm/joint naming convention

All joints and topics are always prefixed, even in single-arm mode:
- Joints: `right_revolute_1`, `right_revolute_2`, `right_revolute_3`, `left_revolute_*`
- Per-arm broadcaster: `/<arm>_joint_state_broadcaster/joint_states`
- Forward controller commands: `/<arm>_<mode>_controller/commands` (`Float64MultiArray`, order `[1,2,3]`)
- Gravity comp reference: `/<arm>/reference_trajectory` (`trajectory_msgs/JointTrajectory`)
- `joint_state_publisher` merges both arm topics into `/joint_states` for RViz

### Control modes and YAML files

`control_mode` selects `src/exo_bringup/config/controllers_<mode>_<arm>.yaml`.  
Available: `read_only`, `effort`, `velocity`, `position`, `gravity_compensation`.

`launch_common.py` is the single source of truth for which controllers get spawned and which topics the data logger subscribes to. Edit there when adding new modes.

### Gravity compensation controllers

The `GravityCompensationController` plugin (lifecycle name `<arm>_gravity_compensation_controller`) loads a URDF via Pinocchio at startup. Key YAML parameters: `dynamics_urdf_filename`, `kp`, `kd`, `gravity_scale`. It publishes `JointControlTelemetry` and a `Bool` session topic used by the data logger.

### Data logger and trajectory tools

- Logger writes `~/exo_logs/<run_id>/data.csv`; columns are prefixed by arm (`right_revolute_<n>_q`, etc.).
- `exo_trajectory_recorder` auto-detects whichever broadcaster topics are live.
- `exo_trajectory_player` routes CSV columns to the correct per-arm topics based on prefix; plays both sides simultaneously from a dual-arm CSV.

## Detailed CLI reference

See `exo_right_arm_ws/README.md` for the full reference: all launch commands, topic pub/echo examples, dual-arm verification checklist, and troubleshooting.
