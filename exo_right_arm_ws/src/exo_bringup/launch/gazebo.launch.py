from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

import launch_common

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_controller_arg = DeclareLaunchArgument(
        "control_mode",
        default_value="effort",
    )
    declare_enable_data_logger = DeclareLaunchArgument(
        "enable_data_logger",
        default_value="true",
    )
    declare_arms = DeclareLaunchArgument(
        "arms",
        default_value="right",
        description="right or left: one arm (unprefixed joints). dual: left_ and right_ prefixed arms.",
    )
    declare_right_mount_x = DeclareLaunchArgument("right_mount_x", default_value="0")
    declare_right_mount_y = DeclareLaunchArgument("right_mount_y", default_value="-0.20")
    declare_right_mount_z = DeclareLaunchArgument("right_mount_z", default_value="0")
    declare_right_mount_roll = DeclareLaunchArgument("right_mount_roll", default_value="0")
    declare_right_mount_pitch = DeclareLaunchArgument("right_mount_pitch", default_value="0")
    declare_right_mount_yaw = DeclareLaunchArgument("right_mount_yaw", default_value="0")
    declare_left_mount_x = DeclareLaunchArgument("left_mount_x", default_value="0")
    declare_left_mount_y = DeclareLaunchArgument("left_mount_y", default_value="0.20")
    declare_left_mount_z = DeclareLaunchArgument("left_mount_z", default_value="0")
    declare_left_mount_roll = DeclareLaunchArgument("left_mount_roll", default_value="0")
    declare_left_mount_pitch = DeclareLaunchArgument("left_mount_pitch", default_value="0")
    declare_left_mount_yaw = DeclareLaunchArgument("left_mount_yaw", default_value="0")

    robot_description = ParameterValue(
        launch_common.robot_description_command(
            "gazebo", include_gazebo_controller_config=True
        ),
        value_type=str,
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    world_path = PathJoinSubstitution(
        [
            FindPackageShare("exo_bringup"),
            "worlds",
            "empty_custom.sdf",
        ]
    )

    gui_config_path = PathJoinSubstitution(
        [
            FindPackageShare("exo_bringup"),
            "config",
            "gazebo.config",
        ]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gz_args": ["-r ", world_path, " --gui-config ", gui_config_path],
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "exo",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    kill_gz = ExecuteProcess(cmd=["bash", "-c", "pkill -f gz || true"])

    start_sim = TimerAction(
        period=1.0,
        actions=[
            gz_sim,
            clock_bridge,
            robot_state_pub,
            spawn_robot,
            OpaqueFunction(function=launch_common.spawn_controllers),
            OpaqueFunction(function=launch_common.data_loggers),
        ],
    )

    return LaunchDescription(
        [
            declare_controller_arg,
            declare_enable_data_logger,
            declare_arms,
            declare_right_mount_x,
            declare_right_mount_y,
            declare_right_mount_z,
            declare_right_mount_roll,
            declare_right_mount_pitch,
            declare_right_mount_yaw,
            declare_left_mount_x,
            declare_left_mount_y,
            declare_left_mount_z,
            declare_left_mount_roll,
            declare_left_mount_pitch,
            declare_left_mount_yaw,
            OpaqueFunction(function=launch_common.gazebo_sim_flag),
            kill_gz,
            start_sim,
        ]
    )
