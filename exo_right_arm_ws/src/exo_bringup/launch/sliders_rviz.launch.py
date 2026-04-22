from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import launch_common


def generate_launch_description():
    declare_arms = DeclareLaunchArgument(
        "arms",
        default_value="single",
        description="single: one arm (unprefixed joints). dual: left_ and right_ prefixed arms.",
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
        launch_common.robot_description_command("real"),
        value_type=str,
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("exo_description"), "rviz", "exo.rviz"]
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    joint_state_pub_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription(
        [
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
            robot_state_pub,
            joint_state_pub_gui,
            rviz_node,
        ]
    )
