# Exo CLI Guide

This document focuses on practical CLI usage: build/setup, launch files, utility scripts, and the main topics to publish/inspect.

All arm configurations (`right`, `left`, `dual`) use the same prefixed naming
convention for joints and topics. Single-arm bringups expose `right_*` / `left_*`
joints and `/right/...` / `/left/...` topics, identical to one half of a dual
bringup. Recorded CSVs are therefore portable across modes.

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
- `arms:=right` (default): right arm only, motors 1-3, joints `right_revolute_1..3`.
- `arms:=left`: left arm only, motors 4-6, joints `left_revolute_1..3`.
- `arms:=dual`: both arms, motors 1-6, joints `left_revolute_*` and `right_revolute_*`.

```bash
ros2 launch exo_bringup exo.launch.py control_mode:=effort enable_data_logger:=true
ros2 launch exo_bringup exo.launch.py arms:=left control_mode:=effort enable_data_logger:=true
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
ros2 launch exo_bringup gazebo.launch.py arms:=dual control_mode:=effort enable_data_logger:=false
```

### Control modes

`control_mode` selects one YAML per active arm under `src/exo_bringup/config/`.
Files are named `controllers_<control_mode>_<arm>.yaml`. The launcher loads:

- `arms:=right` → `controllers_<control_mode>_right.yaml`
- `arms:=left` → `controllers_<control_mode>_left.yaml`
- `arms:=dual` → both `_left.yaml` and `_right.yaml` are loaded into the same controller manager

Each per-arm YAML defines its own broadcaster (`<arm>_joint_state_broadcaster`)
and controller (`<arm>_<control_mode>_controller`); only the requested arm's
controllers are spawned.

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

Forward controllers (`effort`, `velocity`, `position`) accept commands at
`/<arm>_<mode>_controller/commands`. The data order is `[revolute_1, revolute_2, revolute_3]`
for that arm.

### Right arm

```bash
ros2 topic pub /right_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.1, 0.0]}"
ros2 topic pub /right_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.2, 0.0]}"
ros2 topic pub /right_effort_controller/commands   std_msgs/msg/Float64MultiArray "{data: [0.0, 0.5, 0.0]}"
```

### Left arm

```bash
ros2 topic pub /left_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.1, 0.0]}"
ros2 topic pub /left_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.2, 0.0]}"
ros2 topic pub /left_effort_controller/commands   std_msgs/msg/Float64MultiArray "{data: [0.0, 0.5, 0.0]}"
```

### Dual arm

Both per-arm command topics are present simultaneously (same names as above),
publish independently to each side.

### Reference trajectory (gravity compensation controllers)

Each arm's gravity-compensation controller listens on `/<arm>/reference_trajectory`.
Joint names in the message must use the prefixed form (`<arm>_revolute_<n>`).

```bash
ros2 topic pub /right/reference_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [right_revolute_1, right_revolute_2, right_revolute_3],
  points: [
    {
      positions: [0.0, 0.1, 0.0],
      velocities: [0.0, 0.0, 0.0],
      time_from_start: {sec: 0, nanosec: 0}
    }
  ]
}"
```

```bash
ros2 topic pub /left/reference_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [left_revolute_1, left_revolute_2, left_revolute_3],
  points: [{positions: [0.0, 0.1, 0.0], velocities: [0.0, 0.0, 0.0], time_from_start: {sec: 0, nanosec: 0}}]
}"
```

## Main state/telemetry topics (inspect)

### Joint states

Each arm publishes on its own broadcaster topic. The `joint_state_publisher`
merges them into `/joint_states` for RViz / `robot_state_publisher`.

```bash
ros2 topic hz /right_joint_state_broadcaster/joint_states
ros2 topic hz /left_joint_state_broadcaster/joint_states
ros2 topic hz /joint_states                # merged stream used by RViz
```

### Controller manager and hardware diagnostics

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

### Gravity compensation telemetry

```bash
ros2 topic echo /right/joint_space_gravity_compensation_controller/telemetry --once
ros2 topic echo /right/task_space_gravity_compensation_controller/telemetry --once
ros2 topic echo /left/joint_space_gravity_compensation_controller/telemetry  --once
ros2 topic echo /left/task_space_gravity_compensation_controller/telemetry   --once
```

### Logger session/lifecycle topics

```bash
ros2 topic echo /right/joint_space_gravity_compensation_controller/logging/session
ros2 topic echo /right_joint_space_gravity_compensation_controller/transition_event
```

## Dual-arm verification checklist

```bash
cd ~/tesis/exo_right_arm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 1) Launch dual hardware or dual Gazebo
ros2 launch exo_bringup exo.launch.py arms:=dual control_mode:=effort enable_data_logger:=false
# ros2 launch exo_bringup gazebo.launch.py arms:=dual control_mode:=effort enable_data_logger:=false

# 2) Verify controllers (expect left_* and right_* variants for joint_state_broadcaster + chosen mode)
ros2 control list_controllers

# 3) Verify per-arm states
ros2 topic hz /left_joint_state_broadcaster/joint_states
ros2 topic hz /right_joint_state_broadcaster/joint_states

# 4) Send independent commands
ros2 topic pub /left_effort_controller/commands  std_msgs/msg/Float64MultiArray "{data: [0.0,  0.1, 0.0]}"
ros2 topic pub /right_effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, -0.1, 0.0]}"
```

## Utility scripts (`exo_utils`)

All scripts run with `ros2 run exo_utils <executable>`.

### Data logger

The launch files spawn `exo_data_logger` automatically when `enable_data_logger:=true`.
It writes one CSV per controller activation under `~/exo_logs/<run_id>/data.csv`.

CSV columns follow the active arm configuration (joint names always prefixed):

- `arms:=right` → `right_revolute_<n>_q`, `right_revolute_<n>_dq`, …, `right_ee_x/y/z` (when applicable).
- `arms:=left` → `left_revolute_<n>_*`, `left_ee_*`.
- `arms:=dual` → both sides interleaved (`left_revolute_*`, `right_revolute_*`, `left_ee_*`, `right_ee_*`).

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

### Record from `<arm>_joint_state_broadcaster/joint_states`

The recorder auto-detects whichever per-arm broadcaster topics are alive on
the graph (`/left_joint_state_broadcaster/joint_states` and/or
`/right_joint_state_broadcaster/joint_states`). The CSV preserves the joint
prefixes verbatim, so a recording from `arms:=right` produces columns named
`right_revolute_<n>_pos` / `_vel`, identical to the right side of a dual recording.

```bash
ros2 run exo_utils exo_trajectory_recorder --ros-args \
  -p output_file:=~/recorded_trajs/my_traj.csv -p frequency:=1000.0
```

To force a specific topic:

```bash
ros2 run exo_utils exo_trajectory_recorder --ros-args \
  -p joint_states_topic:=/left_joint_state_broadcaster/joint_states \
  -p output_file:=~/recorded_trajs/left.csv
```

### Play a trajectory

The player routes joints to per-arm topics based on the prefix in the CSV:
`left_*` → `/left/reference_trajectory` + `/left_position_controller/commands`,
`right_*` → `/right/reference_trajectory` + `/right_position_controller/commands`.
A dual-arm CSV (both prefixes present) drives both sides simultaneously.
Controllers stay side-local: left controllers subscribe only to left topics, and
right controllers subscribe only to right topics.

```bash
ros2 run exo_utils exo_trajectory_player --ros-args \
  -p input_file:=~/recorded_trajs/my_traj.csv
```

The player does not remap arm sides: it always publishes each recorded prefix to
its matching topics. A dual recording publishes to both left and right topics.

Loop playback:

```bash
ros2 run exo_utils exo_trajectory_player --ros-args \
  -p input_file:=~/recorded_trajs/my_traj.csv -p loop:=true
```

## Common troubleshooting

### Package not found

```bash
cd ~/tesis/exo_right_arm_ws
source install/setup.bash
ros2 pkg list | grep exo_utils
```

### `ModuleNotFoundError: No module named 'yaml'` / `catkin_pkg`

If logger/trajectory tools or `colcon build` fail at startup with these errors,
the conda Python is shadowing the system Python. Force the system interpreter:

```bash
export PATH=/usr/bin:$PATH
sudo apt install -y python3-yaml python3-catkin-pkg
cd ~/tesis/exo_right_arm_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

### Controller type not defined

Usually indicates a controller YAML was not loaded. Check launch `control_mode` and that:
- `src/exo_bringup/config/controllers_<control_mode>_<arm>.yaml` exists
- launch output does not show `Parameter file path is not a file`

### No movement in RViz

Check if the merged `/joint_states` is updating:

```bash
ros2 topic hz /joint_states
ros2 topic hz /right_joint_state_broadcaster/joint_states
ros2 topic hz /left_joint_state_broadcaster/joint_states
```
