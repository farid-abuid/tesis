from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    robot_description = ParameterValue(
    Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("exo_description"),
            "urdf",
            "exo.xacro"
        ]),
        " hardware:=real"
    ]),
    value_type=str
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare("exo_description"),
        "rviz",
        "exo.rviz"
    ])

    controller_config = PathJoinSubstitution([
        FindPackageShare("exo_bringup"),
        "config",
        "controllers.yaml"
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare("exo_description"),
        "launch",
        "urdf.rviz"
    ])

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen"
    )

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
    rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen"
        )

    return LaunchDescription([
        robot_state_pub,
        control_node,
        joint_state_spawner,
        effort_controller_spawner,
        rviz_node
    ])
