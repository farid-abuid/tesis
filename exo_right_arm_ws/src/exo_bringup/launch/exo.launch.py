from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import launch_common


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
    declare_enable_rviz = DeclareLaunchArgument(
        "enable_rviz",
        default_value="true",
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
    enable_rviz = LaunchConfiguration("enable_rviz")

    robot_description = launch_common.robot_description_command("real")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("exo_description"), "rviz", "exo.rviz"]
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )
    joint_state_merger = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[
            {
                "source_list": [
                    "/right_joint_state_broadcaster/joint_states",
                    "/left_joint_state_broadcaster/joint_states",
                ]
            }
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("arms"), "' == 'dual'"])
        ),
        output="screen",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            LaunchConfiguration("exo_controller_yaml_path"),
        ],
        output="screen",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(enable_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            declare_controller_arg,
            declare_enable_data_logger,
            declare_arms,
            declare_enable_rviz,
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
            OpaqueFunction(function=launch_common.set_controller_yaml),
            robot_state_pub,
            joint_state_merger,
            control_node,
            OpaqueFunction(function=launch_common.spawn_controllers),
            OpaqueFunction(function=launch_common.data_loggers),
            rviz_node,
        ]
    )
