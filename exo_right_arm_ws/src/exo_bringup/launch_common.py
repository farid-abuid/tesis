"""Shared launch logic for single vs dual arm (controller YAML path, spawners, data loggers)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import SetLaunchConfiguration
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _controller_yaml_name(context) -> str:
    arms = context.launch_configurations.get("arms", "single")
    mode = context.launch_configurations.get("control_mode", "effort")
    name = f"controllers_{mode}.yaml"
    if arms == "dual":
        name = f"controllers_{mode}_dual.yaml"
    return name


def set_controller_yaml(context):
    name = _controller_yaml_name(context)
    path = os.path.join(get_package_share_directory("exo_bringup"), "config", name)
    return [SetLaunchConfiguration("exo_controller_yaml_path", path)]


def gazebo_sim_flag(context):
    return [SetLaunchConfiguration("exo_in_gazebo", "true")]


def set_gazebo_controller_config_name(context):
    """Filename only, passed into exo.xacro as controller_config (gz_ros2_control)."""
    name = _controller_yaml_name(context)
    return [SetLaunchConfiguration("gz_controller_config_name", name)]


def _controller_mode_to_stem(mode: str) -> str:
    return f"{mode}_controller"


def spawn_controllers(context):
    arms = context.launch_configurations.get("arms", "single")
    mode = context.launch_configurations.get("control_mode", "effort")
    nodes = []
    if arms == "dual":
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "left_joint_state_broadcaster",
                    "--ros-args",
                    "-r",
                    "/left_joint_state_broadcaster/joint_states:=/left/joint_states",
                ],
                output="screen",
            )
        )
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "right_joint_state_broadcaster",
                    "--ros-args",
                    "-r",
                    "/right_joint_state_broadcaster/joint_states:=/right/joint_states",
                ],
                output="screen",
            )
        )
        if mode == "read_only":
            return nodes
        dual_map = {
            "effort": ("left_effort_controller", "right_effort_controller"),
            "velocity": ("left_velocity_controller", "right_velocity_controller"),
            "position": ("left_position_controller", "right_position_controller"),
            "joint_space_gravity_compensation": (
                "left_joint_space_gravity_compensation_controller",
                "right_joint_space_gravity_compensation_controller",
            ),
            "task_space_gravity_compensation": (
                "left_task_space_gravity_compensation_controller",
                "right_task_space_gravity_compensation_controller",
            ),
        }
        left_c, right_c = dual_map[mode]
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    left_c,
                    "--ros-args",
                    "-r",
                    f"/{left_c}/commands:=/left/commands",
                ],
                output="screen",
            )
        )
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    right_c,
                    "--ros-args",
                    "-r",
                    f"/{right_c}/commands:=/right/commands",
                ],
                output="screen",
            )
        )
        return nodes

    nodes.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        )
    )
    if mode != "read_only":
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[_controller_mode_to_stem(mode)],
                output="screen",
            )
        )
    return nodes


def data_loggers(context):
    enable = context.launch_configurations.get("enable_data_logger", "true").lower() == "true"
    if not enable:
        return []
    arms_mode = context.launch_configurations.get("arms", "single")
    mode = context.launch_configurations.get("control_mode", "effort")
    if mode == "read_only":
        return []
    sim_flag = context.launch_configurations.get("exo_in_gazebo", "false").lower() == "true"
    data_yaml = os.path.join(
        get_package_share_directory("exo_bringup"), "config", "data_logger.yaml"
    )

    # Only gravity compensation controllers publish JointControlTelemetry + session Bool.
    publishes_telemetry = mode in (
        "joint_space_gravity_compensation",
        "task_space_gravity_compensation",
    )
    is_forward = mode in ("effort", "position", "velocity")

    if arms_mode == "dual":
        arms = ["left", "right"]
        left_ctrl = f"left_{mode}_controller"
        right_ctrl = f"right_{mode}_controller"
        if publishes_telemetry:
            telemetry_topics = [
                f"/left/{mode}_controller/telemetry",
                f"/right/{mode}_controller/telemetry",
            ]
            session_topics = [
                f"/left/{mode}_controller/logging/session",
                f"/right/{mode}_controller/logging/session",
            ]
        else:
            telemetry_topics = ["", ""]
            session_topics = ["", ""]
        lifecycle_topics = [
            f"/{left_ctrl}/transition_event",
            f"/{right_ctrl}/transition_event",
        ]
        # spawner `--ros-args -r` does NOT propagate to controllers; subscribe to the real topics.
        command_topics = (
            [f"/{left_ctrl}/commands", f"/{right_ctrl}/commands"] if is_forward else ["", ""]
        )
        # Broadcasters use `use_local_topics: true` in dual configs.
        joint_state_topics = [
            "/left_joint_state_broadcaster/joint_states",
            "/right_joint_state_broadcaster/joint_states",
        ]
    else:
        arms = ["main"]
        ctrl = f"{mode}_controller"
        telemetry_topics = [f"/{ctrl}/telemetry"] if publishes_telemetry else [""]
        session_topics = [f"/{ctrl}/logging/session"] if publishes_telemetry else [""]
        lifecycle_topics = [f"/{ctrl}/transition_event"]
        command_topics = [f"/{ctrl}/commands"] if is_forward else [""]
        joint_state_topics = ["/joint_states"]

    extra = {"use_sim_time": True} if sim_flag else {}
    return [
        Node(
            package="exo_utils",
            executable="exo_data_logger",
            name="exo_data_logger",
            parameters=[
                data_yaml,
                {
                    "arms": arms,
                    "control_mode": mode,
                    "telemetry_topics": telemetry_topics,
                    "session_topics": session_topics,
                    "lifecycle_transition_topics": lifecycle_topics,
                    "command_topics": command_topics,
                    "joint_state_topics": joint_state_topics,
                    **extra,
                },
            ],
            output="screen",
        )
    ]


def robot_description_command(hardware: str, include_gazebo_controller_config: bool = False):
    """hardware: 'real' or 'gazebo'."""
    arms = LaunchConfiguration("arms")
    parts = [
        "xacro ",
        PathJoinSubstitution(
            [FindPackageShare("exo_description"), "urdf", "exo.xacro"]
        ),
        f" hardware:={hardware}",
        " arms:=",
        arms,
        " right_mount_x:=",
        LaunchConfiguration("right_mount_x"),
        " right_mount_y:=",
        LaunchConfiguration("right_mount_y"),
        " right_mount_z:=",
        LaunchConfiguration("right_mount_z"),
        " right_mount_roll:=",
        LaunchConfiguration("right_mount_roll"),
        " right_mount_pitch:=",
        LaunchConfiguration("right_mount_pitch"),
        " right_mount_yaw:=",
        LaunchConfiguration("right_mount_yaw"),
        " left_mount_x:=",
        LaunchConfiguration("left_mount_x"),
        " left_mount_y:=",
        LaunchConfiguration("left_mount_y"),
        " left_mount_z:=",
        LaunchConfiguration("left_mount_z"),
        " left_mount_roll:=",
        LaunchConfiguration("left_mount_roll"),
        " left_mount_pitch:=",
        LaunchConfiguration("left_mount_pitch"),
        " left_mount_yaw:=",
        LaunchConfiguration("left_mount_yaw"),
    ]
    if include_gazebo_controller_config:
        parts += [
            " controller_config:=controllers_",
            LaunchConfiguration("control_mode"),
            PythonExpression(
                ["'_dual.yaml' if '", LaunchConfiguration("arms"), "' == 'dual' else '.yaml'"]
            ),
        ]
    return Command(parts)
