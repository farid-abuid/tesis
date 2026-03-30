#include "gravity_compensation_controller/gravity_compensation_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <unordered_map>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace gravity_compensation_controller
{

controller_interface::CallbackReturn GravityCompensationController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<std::string>("dynamics_backend", std::string("pinocchio"));
  auto_declare<std::string>("urdf_path", std::string(""));
  auto_declare<std::string>("reference_trajectory_topic", std::string("/reference_trajectory"));
  auto_declare<double>("kp", 50.0);
  auto_declare<double>("kd", 5.0);
  auto_declare<double>("gravity_scale", 1.0);

  RCLCPP_INFO(get_node()->get_logger(), "Gravity compensation controller initialized");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_configure(
  const rclcpp_lifecycle::State &)
{
  joint_names_param_ = get_node()->get_parameter("joints").as_string_array();

  if (joint_names_param_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  dynamics_backend_ = get_node()->get_parameter("dynamics_backend").as_string();
  urdf_path_ = get_node()->get_parameter("urdf_path").as_string();
  reference_trajectory_topic_ =
    get_node()->get_parameter("reference_trajectory_topic").as_string();
  kp_ = get_node()->get_parameter("kp").as_double();
  kd_ = get_node()->get_parameter("kd").as_double();
  gravity_scale_ = get_node()->get_parameter("gravity_scale").as_double();

  if (urdf_path_.empty()) {
    try {
      const std::string share = ament_index_cpp::get_package_share_directory("exo_description");
      urdf_path_ = share + "/urdf/exo_dynamics.urdf";
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "urdf_path is empty and could not resolve exo_description share: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  std::string dyn_err;
  dynamics_ = createDynamicsModel(dynamics_backend_, &dyn_err);
  if (!dynamics_) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", dyn_err.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!dynamics_->loadUrdf(urdf_path_, &dyn_err)) {
    RCLCPP_ERROR(get_node()->get_logger(), "URDF load failed: %s", dyn_err.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  TrajectorySetpoint init;
  init.positions.assign(joint_names_param_.size(), 0.0);
  init.velocities.assign(joint_names_param_.size(), 0.0);
  init.valid = true;
  setpoint_buffer_.writeFromNonRT(init);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured: backend=%s urdf=%s reference_topic=%s joints=%zu",
    dynamics_backend_.c_str(), urdf_path_.c_str(), reference_trajectory_topic_.c_str(),
    joint_names_param_.size());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!init_interfaces()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }

  traj_sub_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    reference_trajectory_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(
      &GravityCompensationController::referenceTrajectoryCallback, this,
      std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "Controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  traj_sub_.reset();
  RCLCPP_INFO(get_node()->get_logger(), "Controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
GravityCompensationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : joint_names_param_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }

  return config;
}

controller_interface::InterfaceConfiguration
GravityCompensationController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : joint_names_param_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }

  return config;
}

bool GravityCompensationController::init_interfaces()
{
  if (state_interfaces_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No state interfaces assigned");
    return false;
  }

  std::unordered_map<std::string, hardware_interface::LoanedStateInterface *> pos_map;
  std::unordered_map<std::string, hardware_interface::LoanedStateInterface *> vel_map;
  std::unordered_map<std::string, hardware_interface::LoanedStateInterface *> eff_map;

  for (auto & iface : state_interfaces_) {
    const auto & joint = iface.get_prefix_name();
    const auto & name = iface.get_interface_name();

    if (name == hardware_interface::HW_IF_POSITION) {
      pos_map[joint] = &iface;
    } else if (name == hardware_interface::HW_IF_VELOCITY) {
      vel_map[joint] = &iface;
    } else if (name == hardware_interface::HW_IF_EFFORT) {
      eff_map[joint] = &iface;
    }
  }

  joint_names_.clear();
  pos_interfaces_.clear();
  vel_interfaces_.clear();
  eff_interfaces_.clear();
  cmd_interfaces_.clear();

  for (const auto & joint : joint_names_param_) {
    if (!pos_map.count(joint) || !vel_map.count(joint)) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Missing state interfaces for joint %s", joint.c_str());
      return false;
    }

    joint_names_.push_back(joint);
    pos_interfaces_.push_back(*pos_map[joint]);
    vel_interfaces_.push_back(*vel_map[joint]);

    if (eff_map.count(joint)) {
      eff_interfaces_.push_back(*eff_map[joint]);
    }
  }

  std::unordered_map<std::string, hardware_interface::LoanedCommandInterface *> cmd_map;

  for (auto & cmd : command_interfaces_) {
    cmd_map[cmd.get_prefix_name()] = &cmd;
  }

  for (const auto & joint : joint_names_) {
    if (!cmd_map.count(joint)) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Missing command interface for joint %s", joint.c_str());
      return false;
    }

    cmd_interfaces_.push_back(*cmd_map[joint]);
  }

  if (cmd_interfaces_.size() != joint_names_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Command interface size mismatch");
    return false;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Initialized %zu joints", joint_names_.size());
  return true;
}

void GravityCompensationController::referenceTrajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (!msg || msg->points.empty()) {
    return;
  }

  const trajectory_msgs::msg::JointTrajectoryPoint & pt = msg->points.front();
  TrajectorySetpoint sp;
  sp.positions.resize(joint_names_param_.size());
  sp.velocities.resize(joint_names_param_.size(), 0.0);

  for (size_t j = 0; j < joint_names_param_.size(); ++j) {
    const std::string & name = joint_names_param_[j];
    auto it = std::find(msg->joint_names.begin(), msg->joint_names.end(), name);
    if (it == msg->joint_names.end()) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Trajectory message missing joint '%s'; ignoring message", name.c_str());
      return;
    }
    const size_t k = static_cast<size_t>(std::distance(msg->joint_names.begin(), it));
    if (k < pt.positions.size()) {
      sp.positions[j] = pt.positions[k];
    } else {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Trajectory point positions too short for joint '%s'", name.c_str());
      return;
    }
    if (k < pt.velocities.size()) {
      sp.velocities[j] = pt.velocities[k];
    }
  }

  sp.valid = true;
  setpoint_buffer_.writeFromNonRT(sp);
}

controller_interface::return_type GravityCompensationController::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  TrajectorySetpoint * sp_ptr = setpoint_buffer_.readFromRT();
  if (!sp_ptr->valid || sp_ptr->positions.size() != joint_names_.size()) {
    return controller_interface::return_type::ERROR;
  }

  std::vector<double> q_meas(joint_names_.size());
  std::vector<double> dq_meas(joint_names_.size());

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    auto q_opt = pos_interfaces_[i].get().get_optional();
    auto dq_opt = vel_interfaces_[i].get().get_optional();
    if (!q_opt || !dq_opt) {
      return controller_interface::return_type::ERROR;
    }
    q_meas[i] = *q_opt;
    dq_meas[i] = *dq_opt;
  }

  std::vector<double> tau_g;
  std::string g_err;
  if (!dynamics_->computeGravityTorque(joint_names_, q_meas, tau_g, &g_err)) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Gravity torque failed: %s", g_err.c_str());
    return controller_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const double q_des = sp_ptr->positions[i];
    const double dq_des = sp_ptr->velocities[i];
    const double tau =
      gravity_scale_ * tau_g[i] + kp_ * (q_des - q_meas[i]) + kd_ * (dq_des - dq_meas[i]);

    if (!cmd_interfaces_[i].get().set_value(tau)) {
      return controller_interface::return_type::ERROR;
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace gravity_compensation_controller

PLUGINLIB_EXPORT_CLASS(
  gravity_compensation_controller::GravityCompensationController,
  controller_interface::ControllerInterface)
