from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.substitutions import PythonExpression

def generate_launch_description():

    control_mode = LaunchConfiguration("control_mode")

    declare_controller_arg = DeclareLaunchArgument(
        "control_mode",
        default_value="effort"
    )

    controller_config = PythonExpression([
        "'controllers_' + '", control_mode, "' + '.yaml'"
    ])

    controller_name = PythonExpression([
        "'", control_mode, "' + '_controller'"
    ])


    robot_description = Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("exo_description"),
            "urdf",
            "exo.xacro"
        ]),
        " hardware:=gazebo",
        " controller_config:=", controller_config
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

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items()
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
        arguments=[controller_name]
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
            declare_controller_arg,
            gz_sim,
            clock_bridge,
            robot_state_pub,
            spawn_robot,
            #joint_state_spawner,
            effort_controller_spawner
        ]
    )

    return LaunchDescription([
        kill_gz,
        start_sim
    ])