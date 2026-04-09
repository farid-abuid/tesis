from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    control_mode = LaunchConfiguration("control_mode")

    declare_controller_arg = DeclareLaunchArgument(
        "control_mode",
        default_value="effort"
    )

    declare_enable_data_logger = DeclareLaunchArgument(
        "enable_data_logger",
        default_value="true"
    )

    enable_data_logger = LaunchConfiguration("enable_data_logger")

    controller_stem = [control_mode, TextSubstitution(text="_controller")]
    telemetry_topic = [
        TextSubstitution(text="/"),
        control_mode,
        TextSubstitution(text="_controller/telemetry"),
    ]
    session_topic = [
        TextSubstitution(text="/"),
        control_mode,
        TextSubstitution(text="_controller/logging/session"),
    ]
    lifecycle_topic = [
        TextSubstitution(text="/"),
        control_mode,
        TextSubstitution(text="_controller/transition_event"),
    ]

    robot_description = Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("exo_description"),
            "urdf",
            "exo.xacro"
        ]),
        " hardware:=real"
    ])

    controller_config = PathJoinSubstitution([
        FindPackageShare("exo_bringup"),
        "config",
        "controllers_",
        control_mode,
        TextSubstitution(text=".yaml"),
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
        arguments=[controller_stem],
    )

    data_logger_node = Node(
        package="exo_bringup",
        executable="exo_data_logger",
        name="exo_data_logger",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("exo_bringup"),
                "config",
                "data_logger.yaml",
            ]),
            {
                "telemetry_topic": telemetry_topic,
                "session_topic": session_topic,
                "lifecycle_transition_topic": lifecycle_topic,
            },
        ],
        condition=IfCondition(enable_data_logger),
        output="screen",
    )

    return LaunchDescription([
        declare_controller_arg,
        declare_enable_data_logger,
        robot_state_pub,
        control_node,
        joint_state_spawner,
        effort_controller_spawner,
        data_logger_node,
    ])