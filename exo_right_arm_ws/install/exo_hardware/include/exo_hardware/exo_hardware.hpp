#pragma once

#include <vector>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/hardware_info.hpp"

#include "rclcpp/rclcpp.hpp"

//#include <serial/serial.h>

namespace exo_hardware
{

class teensy_plugin : public hardware_interface::SystemInterface
{
public:

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::return_type read(
    const rclcpp::Time &,
    const rclcpp::Duration &) override;

  hardware_interface::return_type write(
    const rclcpp::Time &,
    const rclcpp::Duration &) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;  

private:

  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> effort_;

  std::vector<double> effort_command_;

  std::vector<uint8_t> motor_ids_;

  int serial_fd_;

};

}