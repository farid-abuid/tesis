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
        default_value="dual",
        description="right, left, or dual. All modes use prefixed joints/topics (right_/left_).",
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
    declare_joint_dynamics = DeclareLaunchArgument(
        "joint_dynamics",
        default_value="true",
        description="Enable collision geometry and joint friction/damping in simulation.",
    )
    # Simulation-only mass disturbances (kg). Added to the link's nominal mass in
    # the Gazebo robot description; the pre-built dynamics URDFs used by
    # model-based controllers are not regenerated, so the dynamic model is unaware.
    sim_mass_args = [
        DeclareLaunchArgument(f"{side}_extra_mass_component_{i}", default_value="0.0",
                              description=f"Extra mass [kg] added to {side}_component_{i} in Gazebo only.")
        for side in ("right", "left")
        for i in (1, 2, 3, 4)
    ] + [
        DeclareLaunchArgument(f"{side}_payload_mass", default_value="0.0",
                              description=f"End-effector payload [kg] added to {side}_component_4 in Gazebo only.")
        for side in ("right", "left")
    ] + [
        DeclareLaunchArgument(
            "extra_mass_radius", default_value="0.05",
            description="Sphere radius [m] used for the inertia contribution of every "
                        "extra mass (sim only). Δ_ixx = Δ_iyy = Δ_izz = (2/5)·m·r²."),
        DeclareLaunchArgument(
            "sim_disturbance", default_value="",
            description="YAML preset (stem or path) under config/sim_disturbance/ that "
                        "populates the per-link extra masses, payloads and radius. "
                        "Empty (default) disables all disturbances."),
    ]

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
    # Merge per-arm broadcasters into /joint_states so robot_state_publisher sees them.
    joint_state_merger = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[
            {
                "source_list": [
                    "/right_joint_state_broadcaster/joint_states",
                    "/left_joint_state_broadcaster/joint_states",
                ],
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
            joint_state_merger,
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
            declare_joint_dynamics,
            *sim_mass_args,
            OpaqueFunction(function=launch_common.apply_sim_disturbance),
            OpaqueFunction(function=launch_common.gazebo_sim_flag),
            OpaqueFunction(function=launch_common.set_gazebo_controller_config_names),
            kill_gz,
            start_sim,
        ]
    )
