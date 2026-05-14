"""Shared launch logic for left/right/dual arm bringups.

Topic and joint naming is always prefixed (`left_` / `right_`); single-arm modes spawn
exactly one side, dual loads both per-arm YAMLs and spawns both sides in one
controller_manager.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetLaunchConfiguration
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_GRAV_COMP_MODES = (
    "gravity_compensation",
    "fl",
    "smc",
    "mrac",
    "mpc",
    "admittance",
    "impedance",
    "smc_i"
)


def _active_arms(context) -> list[str]:
    """Return the list of active arms ('left'/'right') for the current launch context."""
    arms = context.launch_configurations.get("arms", "right")
    if arms == "dual":
        return ["left", "right"]
    if arms in ("left", "right"):
        return [arms]
    raise ValueError(f"arms must be 'left', 'right', or 'dual' (got {arms!r})")


def _controller_yaml_filename(arm: str, mode: str) -> str:
    return os.path.join(mode, f"controllers_{arm}.yaml")


def _controller_yaml_paths(context) -> list[str]:
    mode = context.launch_configurations.get("control_mode", "effort")
    cfg_dir = os.path.join(get_package_share_directory("exo_bringup"), "config")
    return [os.path.join(cfg_dir, _controller_yaml_filename(a, mode)) for a in _active_arms(context)]


def set_controller_yaml(context):
    # Pass the YAML paths as a colon-separated string; the launch consumes them per-path.
    paths = _controller_yaml_paths(context)
    return [SetLaunchConfiguration("exo_controller_yaml_paths", os.pathsep.join(paths))]


def gazebo_sim_flag(context):
    return [SetLaunchConfiguration("exo_in_gazebo", "true")]


def apply_sim_disturbance(context):
    """Translate `sim_disturbance:=<spec>` into the per-link launch configs.

    `spec` can be: empty (disabled, no-op), a bare filename / stem looked up in
    `share/exo_bringup/config/sim_disturbance/`, or an absolute path. Any key
    missing from the YAML keeps its launch-arg default.
    """
    spec = context.launch_configurations.get("sim_disturbance", "").strip()
    if not spec:
        return []
    if not os.path.isabs(spec):
        cfg_dir = os.path.join(
            get_package_share_directory("exo_bringup"), "config", "sim_disturbance"
        )
        candidate = spec if spec.endswith(".yaml") else f"{spec}.yaml"
        spec = os.path.join(cfg_dir, candidate)
    with open(spec) as f:
        data = yaml.safe_load(f) or {}
    actions = []
    if "extra_mass_radius" in data:
        actions.append(
            SetLaunchConfiguration("extra_mass_radius", str(data["extra_mass_radius"]))
        )
    for side in ("right", "left"):
        side_block = data.get(side) or {}
        for i in (1, 2, 3, 4):
            key = f"component_{i}"
            if key in side_block:
                actions.append(
                    SetLaunchConfiguration(
                        f"{side}_extra_mass_{key}", str(side_block[key])
                    )
                )
        if "payload" in side_block:
            actions.append(
                SetLaunchConfiguration(f"{side}_payload_mass", str(side_block["payload"]))
            )
    return actions


def set_gazebo_controller_config_names(context):
    """Set per-arm controller_config_<side> launch configs (filename only, used by exo.xacro)."""
    arms = _active_arms(context)
    mode = context.launch_configurations.get("control_mode", "effort")
    left_name = _controller_yaml_filename("left", mode) if "left" in arms else ""
    right_name = _controller_yaml_filename("right", mode) if "right" in arms else ""
    return [
        SetLaunchConfiguration("gz_controller_config_left", left_name),
        SetLaunchConfiguration("gz_controller_config_right", right_name),
    ]


def spawn_controllers(context):
    arms = _active_arms(context)
    mode = context.launch_configurations.get("control_mode", "effort")
    nodes = []
    for arm in arms:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[f"{arm}_joint_state_broadcaster"],
                output="screen",
            )
        )
    if mode == "read_only":
        return nodes
    for arm in arms:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[f"{arm}_{mode}_controller"],
                output="screen",
            )
        )
    return nodes


def data_loggers(context):
    enable = context.launch_configurations.get("enable_data_logger", "true").lower() == "true"
    if not enable:
        return []
    arms = _active_arms(context)
    mode = context.launch_configurations.get("control_mode", "effort")
    if mode == "read_only":
        return []
    sim_flag = context.launch_configurations.get("exo_in_gazebo", "false").lower() == "true"
    data_yaml = os.path.join(
        get_package_share_directory("exo_bringup"), "config", "data_logger.yaml"
    )

    # Only gravity compensation controllers publish JointControlTelemetry + session Bool.
    publishes_telemetry = mode in _GRAV_COMP_MODES
    is_forward = mode in ("effort", "position", "velocity")

    # Build per-arm topic lists with prefixed naming. Order matches `arms`.
    telemetry_topics: list[str] = []
    session_topics: list[str] = []
    lifecycle_topics: list[str] = []
    command_topics: list[str] = []
    joint_state_topics: list[str] = []
    for arm in arms:
        ctrl = f"{arm}_{mode}_controller"
        if publishes_telemetry:
            telemetry_topics.append(f"/{arm}/{mode}_controller/telemetry")
            session_topics.append(f"/{arm}/{mode}_controller/logging/session")
        else:
            telemetry_topics.append("")
            session_topics.append("")
        lifecycle_topics.append(f"/{ctrl}/transition_event")
        command_topics.append(f"/{ctrl}/commands" if is_forward else "")
        joint_state_topics.append(f"/{arm}_joint_state_broadcaster/joint_states")

    extra = {"use_sim_time": True} if sim_flag else {}
    return [
        Node(
            package="exo_utils",
            executable="exo_data_logger",
            name="exo_data_logger",
            parameters=[
                data_yaml,
                {
                    "arms": arms,
                    "control_mode": mode,
                    "telemetry_topics": telemetry_topics,
                    "session_topics": session_topics,
                    "lifecycle_transition_topics": lifecycle_topics,
                    "command_topics": command_topics,
                    "joint_state_topics": joint_state_topics,
                    **extra,
                },
            ],
            output="screen",
        )
    ]


_SIM_MASS_ARGS = (
    "right_extra_mass_component_1",
    "right_extra_mass_component_2",
    "right_extra_mass_component_3",
    "right_extra_mass_component_4",
    "right_payload_mass",
    "left_extra_mass_component_1",
    "left_extra_mass_component_2",
    "left_extra_mass_component_3",
    "left_extra_mass_component_4",
    "left_payload_mass",
    "extra_mass_radius",
)


def robot_description_command(hardware: str, include_gazebo_controller_config: bool = False):
    """hardware: 'real' or 'gazebo'. When hardware == 'gazebo', per-link sim mass
    disturbance args are forwarded so Gazebo's robot sees them; the pre-built
    dynamics URDFs used by model-based controllers are unaffected."""
    arms = LaunchConfiguration("arms")
    parts = [
        "xacro ",
        PathJoinSubstitution(
            [FindPackageShare("exo_description"), "urdf", "exo.xacro"]
        ),
        f" hardware:={hardware}",
        " arms:=",
        arms,
        " right_mount_x:=",
        LaunchConfiguration("right_mount_x"),
        " right_mount_y:=",
        LaunchConfiguration("right_mount_y"),
        " right_mount_z:=",
        LaunchConfiguration("right_mount_z"),
        " right_mount_roll:=",
        LaunchConfiguration("right_mount_roll"),
        " right_mount_pitch:=",
        LaunchConfiguration("right_mount_pitch"),
        " right_mount_yaw:=",
        LaunchConfiguration("right_mount_yaw"),
        " left_mount_x:=",
        LaunchConfiguration("left_mount_x"),
        " left_mount_y:=",
        LaunchConfiguration("left_mount_y"),
        " left_mount_z:=",
        LaunchConfiguration("left_mount_z"),
        " left_mount_roll:=",
        LaunchConfiguration("left_mount_roll"),
        " left_mount_pitch:=",
        LaunchConfiguration("left_mount_pitch"),
        " left_mount_yaw:=",
        LaunchConfiguration("left_mount_yaw"),
        " joint_dynamics:=",
        LaunchConfiguration("joint_dynamics"),
    ]
    if hardware == "gazebo":
        for name in _SIM_MASS_ARGS:
            parts += [f" {name}:=", LaunchConfiguration(name)]
    if include_gazebo_controller_config:
        parts += [
            " controller_config_left:=",
            LaunchConfiguration("gz_controller_config_left"),
            " controller_config_right:=",
            LaunchConfiguration("gz_controller_config_right"),
        ]
    return Command(parts)
