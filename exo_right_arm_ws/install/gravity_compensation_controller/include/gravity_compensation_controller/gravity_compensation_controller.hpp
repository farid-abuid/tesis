#pragma once

#include <vector>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gravity_compensation_controller
{

class GravityCompensationController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:

  // --- YOUR INIT FUNCTION ---
  bool init_interfaces();

  // --- PARAMETERS ---
  std::vector<std::string> joint_names_param_;   // from YAML

  // --- INTERNAL STRUCTURE ---
  std::vector<std::string> joint_names_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> pos_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> vel_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> eff_interfaces_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> cmd_interfaces_;

  // --- CONTROL ---
  std::vector<double> desired_positions_;
  double kp_ = 1.0;
};

}  // namespace gravity_compensation_controller