from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    robot_description = Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("exo_description"),
            "urdf",
            "exo.xacro"
        ])
    ])

    controller_config = PathJoinSubstitution([
        FindPackageShare("exo_bringup"),
        "config",
        "controllers.yaml"
    ])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controller_config
        ],
        output="screen"
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller"],
    )

    return LaunchDescription([
        control_node,
        joint_state_spawner,
        effort_controller_spawner
    ])