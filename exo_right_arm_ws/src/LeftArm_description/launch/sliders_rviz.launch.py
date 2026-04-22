from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=PathJoinSubstitution(
            [FindPackageShare("LeftArm_description"), "urdf", "LeftArm.xacro"]
        ),
        description="Absolute path to robot xacro file",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=PathJoinSubstitution(
            [FindPackageShare("LeftArm_description"), "launch", "urdf_ros2.rviz"]
        ),
        description="Absolute path to rviz config file",
    )

    robot_description = Command(["xacro ", LaunchConfiguration("model")])

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    return LaunchDescription(
        [
            model_arg,
            rviz_config_arg,
            joint_state_publisher_gui_node,
            robot_state_publisher_node,
            rviz2_node,
        ]
    )
