#include "gravity_compensation_controller/gravity_compensation_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace gravity_compensation_controller
{

// ================= INIT =================
controller_interface::CallbackReturn GravityCompensationController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {});
  RCLCPP_INFO(get_node()->get_logger(), "Controller initialized");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ================= CONFIGURE =================
controller_interface::CallbackReturn GravityCompensationController::on_configure(
  const rclcpp_lifecycle::State &)
{
  joint_names_param_ =
    get_node()->get_parameter("joints").as_string_array();

  if (joint_names_param_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  //desired_positions_.assign(joint_names_param_.size(), 0.0);
  desired_positions_ = {0.5, 1.57, 1.57};

  RCLCPP_INFO(get_node()->get_logger(), "Controllerget_logger configured");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ================= ACTIVATE =================
controller_interface::CallbackReturn GravityCompensationController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!init_interfaces())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ================= DEACTIVATE =================
controller_interface::CallbackReturn GravityCompensationController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "Controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ================= INTERFACE REQUEST =================

controller_interface::InterfaceConfiguration
GravityCompensationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : joint_names_param_)
  {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }

  return config;
}

controller_interface::InterfaceConfiguration
GravityCompensationController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : joint_names_param_)
  {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }

  return config;
}

// ================= INTERFACE INITIALIZATION =================

bool GravityCompensationController::init_interfaces()
{
  if (state_interfaces_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No state interfaces assigned");
    return false;
  }

  // Debug print
  // for (auto & iface : state_interfaces_) {
  //  RCLCPP_INFO(get_node()->get_logger(), "STATE: %s/%s",
  //    iface.get_prefix_name().c_str(),
  //    iface.get_interface_name().c_str());
  // }

  // Build maps
  std::unordered_map<std::string, hardware_interface::LoanedStateInterface*> pos_map;
  std::unordered_map<std::string, hardware_interface::LoanedStateInterface*> vel_map;
  std::unordered_map<std::string, hardware_interface::LoanedStateInterface*> eff_map;

  for (auto & iface : state_interfaces_)
  {
    const auto & joint = iface.get_prefix_name();
    const auto & name  = iface.get_interface_name();

    if (name == hardware_interface::HW_IF_POSITION)
      pos_map[joint] = &iface;

    else if (name == hardware_interface::HW_IF_VELOCITY)
      vel_map[joint] = &iface;

    else if (name == hardware_interface::HW_IF_EFFORT)
      eff_map[joint] = &iface;
  }

  // Clear containers
  joint_names_.clear();
  pos_interfaces_.clear();
  vel_interfaces_.clear();
  eff_interfaces_.clear();
  cmd_interfaces_.clear();

  // Validate and assign state interfaces
  for (const auto & joint : joint_names_param_)
  {
    if (!pos_map.count(joint) || !vel_map.count(joint)) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Missing state interfaces for joint %s", joint.c_str());
      return false;
    }

    joint_names_.push_back(joint);

    pos_interfaces_.push_back(*pos_map[joint]);
    vel_interfaces_.push_back(*vel_map[joint]);

    if (eff_map.count(joint))
      eff_interfaces_.push_back(*eff_map[joint]);
  }

  // Build command map
  std::unordered_map<std::string, hardware_interface::LoanedCommandInterface*> cmd_map;

  for (auto & cmd : command_interfaces_)
  {
    cmd_map[cmd.get_prefix_name()] = &cmd;

    // RCLCPP_INFO(get_node()->get_logger(), "CMD: %s/%s",
    //   cmd.get_prefix_name().c_str(),
    //   cmd.get_interface_name().c_str());
  }

  // Assign ordered command interfaces
  for (const auto & joint : joint_names_)
  {
    if (!cmd_map.count(joint)) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Missing command interface for joint %s", joint.c_str());
      return false;
    }

    cmd_interfaces_.push_back(*cmd_map[joint]);
  }

  if (cmd_interfaces_.size() != joint_names_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Command interface size mismatch");
    return false;
  }

  RCLCPP_INFO(get_node()->get_logger(),
    "Initialized %ld joints", joint_names_.size());

  return true;
}

// ================= UPDATE =================

controller_interface::return_type GravityCompensationController::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    auto q_opt  = pos_interfaces_[i].get().get_optional();
    auto dq_opt = vel_interfaces_[i].get().get_optional();

    if (!q_opt || !dq_opt)
      return controller_interface::return_type::ERROR;

    double q  = *q_opt;
    double dq = *dq_opt;

    double error  = desired_positions_[i] - q;
    double effort = kp_ * error;
    RCLCPP_INFO(get_node()->get_logger(), "Effort %f", effort);
    
    if (!cmd_interfaces_[i].get().set_value(effort))
      return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

}  // namespace gravity_compensation_controller

PLUGINLIB_EXPORT_CLASS(
  gravity_compensation_controller::GravityCompensationController,
  controller_interface::ControllerInterface
)