# Exo Right Arm CLI Guide

This document focuses on practical CLI usage: build/setup, launch files, utility scripts, and the main topics to publish/inspect.

## Build and source

```bash
cd ~/tesis/exo_right_arm_ws
colcon build --symlink-install
source install/setup.bash
```

Verify packages are visible in the current shell:

```bash
ros2 pkg list | grep -E "exo_bringup|exo_utils|exo_hardware"
ros2 pkg executables exo_utils
```

## Launch files

Launch files are in `src/exo_bringup/launch/`.

### Real hardware

The `arms` argument selects the configuration:
- `arms:=right` (default): right arm only, motors 1-3, unprefixed joints (`revolute_1..3`).
- `arms:=left`: left arm only, motors 4-6, unprefixed joints.
- `arms:=dual`: both arms, motors 1-6, prefixed joints (`right_revolute_*`, `left_revolute_*`).

```bash
ros2 launch exo_bringup exo.launch.py control_mode:=effort enable_data_logger:=true
ros2 launch exo_bringup exo.launch.py arms:=left control_mode:=effort enable_data_logger:=true
```

Two arms (single `controller_manager`, topics under `/left` and `/right`):

```bash
ros2 launch exo_bringup exo.launch.py arms:=dual control_mode:=effort enable_data_logger:=false
```

Optional mount poses (passed to `exo.xacro`) are available in all launch files:
`right_mount_x`, `right_mount_y`, `right_mount_z`, `right_mount_roll`, `right_mount_pitch`, `right_mount_yaw`,
`left_mount_x`, `left_mount_y`, `left_mount_z`, `left_mount_roll`, `left_mount_pitch`, `left_mount_yaw`.

### Sliders + RViz (no ros2_control)

```bash
ros2 launch exo_bringup sliders_rviz.launch.py
ros2 launch exo_bringup sliders_rviz.launch.py arms:=left
ros2 launch exo_bringup sliders_rviz.launch.py arms:=dual
```

### Gazebo simulation

```bash
ros2 launch exo_bringup gazebo.launch.py control_mode:=effort enable_data_logger:=true
ros2 launch exo_bringup gazebo.launch.py arms:=left control_mode:=effort enable_data_logger:=true
```

```bash
ros2 launch exo_bringup gazebo.launch.py arms:=dual control_mode:=effort enable_data_logger:=false
```

### Control modes

`control_mode` maps to controller YAMLs under `src/exo_bringup/config/`:
- `arms:=right` (default) → `controllers_<control_mode>.yaml`
- `arms:=left` → same `controllers_<control_mode>.yaml`, except `joint_space_gravity_compensation` and `task_space_gravity_compensation` use `controllers_<control_mode>_left.yaml` (which loads the left dynamics URDF).
- `arms:=dual` → `controllers_<control_mode>_dual.yaml`.

Available modes:
- `read_only`
- `effort`
- `velocity`
- `position`
- `joint_space_gravity_compensation`
- `task_space_gravity_compensation`

Example read-only launch:

```bash
ros2 launch exo_bringup exo.launch.py control_mode:=read_only enable_data_logger:=false
```

## Main command topics (publish)

### Single arm (`arms:=right` default, or `arms:=left`)

For group controllers, command order is: `[revolute_1, revolute_2, revolute_3]`.

### Position commands

```bash
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.1, 0.0]}"
```

### Velocity commands

```bash
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.2, 0.0]}"
```

### Effort commands

```bash
ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.5, 0.0]}"
```

### Dual arm (`arms:=dual`)

Per-arm command order is still three joints per message. Topics:

- `/left/commands` — `left_revolute_1`, `left_revolute_2`, `left_revolute_3`
- `/right/commands` — `right_revolute_1`, `right_revolute_2`, `right_revolute_3`

```bash
ros2 topic pub /left/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0]}"
ros2 topic pub /right/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0]}"
```

### Reference trajectory (gravity compensation controllers)

Single arm: `/reference_trajectory`. Dual: `/left/reference_trajectory` and `/right/reference_trajectory`.

```bash
ros2 topic pub /reference_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [revolute_1, revolute_2, revolute_3],
  points: [
    {
      positions: [0.0, 0.1, 0.0],
      velocities: [0.0, 0.0, 0.0],
      time_from_start: {sec: 0, nanosec: 0}
    }
  ]
}"
```

Dual-arm example:

```bash
ros2 topic pub /left/reference_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [left_revolute_1, left_revolute_2, left_revolute_3],
  points: [{positions: [0.0, 0.1, 0.0], velocities: [0.0, 0.0, 0.0], time_from_start: {sec: 0, nanosec: 0}}]
}"
```

## Main state/telemetry topics (inspect)

### Joint states

```bash
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
```

Dual arm:

```bash
ros2 topic hz /left_joint_state_broadcaster/joint_states
ros2 topic hz /right_joint_state_broadcaster/joint_states
```

### Controller manager and hardware diagnostics

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

### Gravity compensation telemetry

```bash
ros2 topic echo /joint_space_gravity_compensation_controller/telemetry --once
ros2 topic echo /task_space_gravity_compensation_controller/telemetry --once
```

Dual arm:

```bash
ros2 topic echo /left/joint_space_gravity_compensation_controller/telemetry --once
ros2 topic echo /right/joint_space_gravity_compensation_controller/telemetry --once
ros2 topic echo /left/task_space_gravity_compensation_controller/telemetry --once
ros2 topic echo /right/task_space_gravity_compensation_controller/telemetry --once
```

### Logger session/lifecycle topics

```bash
ros2 topic echo /joint_space_gravity_compensation_controller/logging/session
ros2 topic echo /joint_space_gravity_compensation_controller/transition_event
```

## Dual-arm verification checklist

```bash
cd ~/tesis/exo_right_arm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 1) Launch dual hardware or dual Gazebo
ros2 launch exo_bringup exo.launch.py arms:=dual control_mode:=effort enable_data_logger:=false
# ros2 launch exo_bringup gazebo.launch.py arms:=dual control_mode:=effort enable_data_logger:=false

# 2) Verify controllers
ros2 control list_controllers

# 3) Verify per-arm states
ros2 topic hz /left_joint_state_broadcaster/joint_states
ros2 topic hz /right_joint_state_broadcaster/joint_states

# 4) Send independent commands
ros2 topic pub /left/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.1, 0.0]}"
ros2 topic pub /right/commands std_msgs/msg/Float64MultiArray "{data: [0.0, -0.1, 0.0]}"
```

## Utility scripts (`exo_utils`)

All scripts run with `ros2 run exo_utils <executable>`.

### Data logger

The launch files spawn `exo_data_logger` automatically when `enable_data_logger:=true`.
It writes one CSV per controller activation under `~/exo_logs/<run_id>/data.csv`.

CSV columns follow the active arm configuration:

- `arms:=right` → `right_revolute_<n>_q`, `right_revolute_<n>_dq`, …, `right_ee_x/y/z` (when applicable).
- `arms:=left` → `left_revolute_<n>_*`, `left_ee_*`.
- `arms:=dual` → both sides interleaved (`left_revolute_*`, `right_revolute_*`, `left_ee_*`, `right_ee_*`).
- Legacy logs without an arm tag (unprefixed `revolute_<n>_*`, `ee_*`) are still parsed by the plotter for back-compat.

Run it standalone (rare; usually launched by the bringup):

```bash
ros2 run exo_utils exo_data_logger
```

### Plot logger output

```bash
ros2 run exo_utils exo_plot_run ~/exo_logs/<run_id>/data.csv
# or, equivalently, as a Python module:
python3 -m exo_utils.plot_run ~/exo_logs/<run_id>/data.csv --no-show
```

By default plots land in `~/exo_logs/<run_id>/plots/`. Use `-o <dir>` to override.

Output layout:

- Single-arm runs (`arms:=right` / `arms:=left`): flat `plots/` directory with figures titled `[right] …` or `[left] …`.
- Dual-arm runs: one subdirectory per arm (`plots/left/`, `plots/right/`), each with its own figures.
- Legacy unprefixed CSVs: flat `plots/` with un-tagged titles (back-compat).

Generated figures (only those whose columns are present in the CSV):
`joint_position.png`, `joint_velocity.png`, `joint_torque.png`, `task_space_xyz.png`.

### DC motor identification

```bash
ros2 run exo_utils exo_dc_motor_id --motor-id 1 --port /dev/teensy_motor
```

### Teensy serial RTT benchmark

```bash
ros2 run exo_utils exo_teensy_serial_rtt --port /dev/teensy_motor --n-motors 6 --cmd-type 1 --samples 1000 --pause-ms 2
```

`--cmd-type` mapping:
- `1`: torque
- `2`: speed
- `3`: position

## Trajectory recorder/player

### Record trajectory from `/joint_states`

```bash
ros2 run exo_utils exo_trajectory_recorder --ros-args -p output_file:=~/recorded_trajs/my_traj.csv -p frequency:=1000.0
```

### Play trajectory to `/reference_trajectory`

```bash
ros2 run exo_utils exo_trajectory_player --ros-args -p input_file:=~/recorded_trajs/my_traj.csv
```

Dual arm (left arm topic):

```bash
ros2 run exo_utils exo_trajectory_player --ros-args -p input_file:=~/recorded_trajs/my_traj.csv -p trajectory_topic:=/left/reference_trajectory
```

Record from `/left_joint_state_broadcaster/joint_states` or `/right_joint_state_broadcaster/joint_states` when using `arms:=dual`:

```bash
ros2 run exo_utils exo_trajectory_recorder --ros-args -p joint_states_topic:=/left_joint_state_broadcaster/joint_states -p output_file:=~/recorded_trajs/left.csv
```

Replay a dual-mode CSV onto a single-arm bringup (`arms:=right` or `arms:=left`). Set
`target_arm` to the side you want; the player keeps only those joints, strips the prefix, and
publishes on the unprefixed (single-arm) topics:

```bash
# Bringup with right arm only
ros2 launch exo_bringup exo.launch.py arms:=right control_mode:=joint_space_gravity_compensation

# Replay only the right side of a dual-mode recording
ros2 run exo_utils exo_trajectory_player --ros-args \
  -p input_file:=~/recorded_trajs/dual.csv \
  -p target_arm:=right
```

Loop playback:

```bash
ros2 run exo_utils exo_trajectory_player --ros-args -p input_file:=~/recorded_trajs/my_traj.csv -p loop:=true
```

## Common troubleshooting

### Package not found

```bash
cd ~/tesis/exo_right_arm_ws
source install/setup.bash
ros2 pkg list | grep exo_utils
```

### `ModuleNotFoundError: No module named 'yaml'`

If logger/trajectory tools fail at startup with this error, install the missing runtime dependency:

```bash
sudo apt update
sudo apt install -y python3-yaml
cd ~/tesis/exo_right_arm_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select exo_utils --symlink-install
source install/setup.bash
```

### Controller type not defined

Usually indicates controller YAML was not loaded. Check launch `control_mode` and that:
- `src/exo_bringup/config/controllers_<control_mode>.yaml` exists
- launch output does not show `Parameter file path is not a file`

### No movement in RViz

Check if `/joint_states` is updating:

```bash
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
```
