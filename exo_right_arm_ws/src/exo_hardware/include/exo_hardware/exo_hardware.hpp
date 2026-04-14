#pragma once

#include <vector>
#include <string>
#include <chrono>
#include <cstdint>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/hardware_info.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

//#include <serial/serial.h>

namespace exo_hardware
{

class teensy_plugin : public hardware_interface::SystemInterface
{
public:

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time &,
    const rclcpp::Duration &) override;

  hardware_interface::return_type write(
    const rclcpp::Time &,
    const rclcpp::Duration &) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;  

private:

  // Sends a serial frame with the given cmd_type (e.g. 6=stop, 7=shutdown).
  void sendSpecialCommand(uint8_t cmd_type);

  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> effort_;

  std::vector<double> position_command_;
  std::vector<double> velocity_command_;
  std::vector<double> effort_command_;
  std::vector<double> last_position_command_;
  std::vector<double> last_velocity_command_;
  std::vector<double> last_effort_command_;
  uint8_t active_cmd_type_ = 5; // 5 = read-only until a controller writes commands
  bool effort_stiction_comp_enabled_ = false;
  std::vector<double> effort_stiction_tau_per_joint_;
  std::vector<double> effort_stiction_slope_per_joint_;

  std::vector<uint8_t> motor_ids_;

  int serial_fd_;

  // Runtime timing stats for control/update diagnostics.
  std::chrono::steady_clock::time_point last_write_tp_{};
  std::chrono::steady_clock::time_point last_read_tp_{};
  std::chrono::steady_clock::time_point stats_window_start_tp_{};
  double write_dt_sum_s_ = 0.0;
  double write_dt_sq_sum_s2_ = 0.0;
  double write_dt_max_s_ = 0.0;
  double read_dt_sum_s_ = 0.0;
  double read_dt_sq_sum_s2_ = 0.0;
  double read_dt_max_s_ = 0.0;
  uint64_t write_cycles_ = 0;
  uint64_t read_cycles_ = 0;
  uint64_t fresh_frames_ = 0;
  uint64_t stale_reads_ = 0;

};

}