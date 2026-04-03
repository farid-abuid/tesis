from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
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
        " hardware:=gazebo",
        " controller_config:=controllers_",
        control_mode,
        TextSubstitution(text=".yaml"),
    ])

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }],
        output="screen"
    )

    world_path = PathJoinSubstitution([
        FindPackageShare("exo_bringup"),
        "worlds",
        "empty_custom.sdf"
    ])

    gui_config_path = PathJoinSubstitution([
        FindPackageShare("exo_bringup"),
        "config",
        "gazebo.config",
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments={
            "gz_args": ["-r ", world_path, " --gui-config ", gui_config_path],
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "exo_right_arm"
        ],
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
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
                "use_sim_time": True,
                "telemetry_topic": telemetry_topic,
                "session_topic": session_topic,
                "lifecycle_transition_topic": lifecycle_topic,
            },
        ],
        condition=IfCondition(enable_data_logger),
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    )

    kill_gz = ExecuteProcess(
        cmd=["bash", "-c", "pkill -f gz || true"]
    )

    start_sim = TimerAction(
        period=1.0,
        actions=[
            gz_sim,
            clock_bridge,
            robot_state_pub,
            spawn_robot,
            joint_state_spawner,
            effort_controller_spawner,
            data_logger_node,
        ]
    )

    return LaunchDescription([
        declare_controller_arg,
        declare_enable_data_logger,
        kill_gz,
        start_sim,
    ])
