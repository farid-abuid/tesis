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

```bash
ros2 launch exo_bringup exo.launch.py control_mode:=effort enable_data_logger:=true
```

### Real hardware with RViz

```bash
ros2 launch exo_bringup rviz.launch.py control_mode:=position enable_data_logger:=true
```

### Gazebo simulation

```bash
ros2 launch exo_bringup gazebo.launch.py control_mode:=effort enable_data_logger:=true
```

### Control modes

`control_mode` maps to `controllers_<control_mode>.yaml` in `src/exo_bringup/config/`.

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

For all group controllers, command order is:
`[revolute_1, revolute_2, revolute_3]`

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

### Reference trajectory (gravity compensation controllers)

Topic: `/reference_trajectory` (`trajectory_msgs/msg/JointTrajectory`)

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

## Main state/telemetry topics (inspect)

### Joint states

```bash
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
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

### Logger session/lifecycle topics

```bash
ros2 topic echo /joint_space_gravity_compensation_controller/logging/session
ros2 topic echo /joint_space_gravity_compensation_controller/transition_event
```

## Utility scripts (`exo_utils`)

All scripts run with `ros2 run exo_utils <executable>`.

### Data logger

```bash
ros2 run exo_utils exo_data_logger
```

### Plot logger output

```bash
ros2 run exo_utils exo_plot_run ~/exo_logs/<run_id>/data.csv
```

### DC motor identification

```bash
ros2 run exo_utils exo_dc_motor_id --motor-id 1 --port /dev/teensy_motor
```

### Teensy serial RTT benchmark

```bash
ros2 run exo_utils exo_teensy_serial_rtt --port /dev/teensy_motor --n-motors 3 --cmd-type 1 --samples 1000 --pause-ms 2
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
