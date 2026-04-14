#include "exo_hardware/exo_hardware.hpp"
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <iostream>
#include <limits>
#include <cmath>
#include <algorithm>
#include <sstream>

#define HEADER1 0xAA
#define HEADER2 0x55

namespace exo_hardware
{

struct __attribute__((packed)) MotorStatus2
{
    uint8_t  motorID;
    float effort;
    float speed;
    float angle;
};

// Hardware joint directions (index 0..5): joints 1 and 3 are reversed.
static const int8_t kJointDirection[] = {-1, 1, -1};

static uint8_t computeChecksum(uint8_t *data, int len)
{
    uint8_t cs = 0;

    for(int i=0;i<len;i++)
        cs ^= data[i];

    return cs;
}


hardware_interface::CallbackReturn teensy_plugin::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if(hardware_interface::SystemInterface::on_init(info) !=
     hardware_interface::CallbackReturn::SUCCESS)
  {
      return hardware_interface::CallbackReturn::ERROR;
  }

  size_t n = info.joints.size();

  position_.resize(n);
  velocity_.resize(n);
  effort_.resize(n);
  position_command_.resize(n, std::numeric_limits<double>::quiet_NaN());
  velocity_command_.resize(n, std::numeric_limits<double>::quiet_NaN());
  effort_command_.resize(n, 0.0);
  last_position_command_ = position_command_;
  last_velocity_command_ = velocity_command_;
  last_effort_command_ = effort_command_;
  motor_ids_.resize(n);

  for(size_t i=0;i<n;i++)
      motor_ids_[i] = i+1; // ALWAYS 1,2,3,4,5,6

  // Optional effort-only low-level stiction compensator:
  // tau_out = tau_in + tau_s * tanh(tau_in / slope).
  const auto get_hw_param = [&](const std::string & key, const std::string & fallback) {
    const auto it = info.hardware_parameters.find(key);
    if (it == info.hardware_parameters.end()) {
      return fallback;
    }
    return it->second;
  };
  effort_stiction_comp_enabled_ =
    (get_hw_param("effort_stiction_comp_enabled", "false") == "true");
  const auto parse_csv_doubles = [](const std::string & csv) {
    std::vector<double> vals;
    std::stringstream ss(csv);
    std::string item;
    while (std::getline(ss, item, ',')) {
      if (item.empty()) {
        continue;
      }
      vals.push_back(std::stod(item));
    }
    return vals;
  };
  effort_stiction_tau_per_joint_.assign(
    n, std::stod(get_hw_param("effort_stiction_tau", "0.0")));
  effort_stiction_slope_per_joint_.assign(
    n, std::stod(get_hw_param("effort_stiction_slope", "0.05")));
  const auto tau_values = parse_csv_doubles(get_hw_param("effort_stiction_tau_values", ""));
  if (!tau_values.empty()) {
    for (size_t i = 0; i < n && i < tau_values.size(); ++i) {
      effort_stiction_tau_per_joint_[i] = tau_values[i];
    }
  }
  const auto slope_values = parse_csv_doubles(get_hw_param("effort_stiction_slope_values", ""));
  if (!slope_values.empty()) {
    for (size_t i = 0; i < n && i < slope_values.size(); ++i) {
      effort_stiction_slope_per_joint_[i] = slope_values[i];
    }
  }
  for (size_t i = 0; i < n; ++i) {
    if (effort_stiction_slope_per_joint_[i] <= 0.0) {
      effort_stiction_slope_per_joint_[i] = 0.05;
    }
  }
  RCLCPP_INFO(
    rclcpp::get_logger("exo_hw"),
    "Effort stiction compensation: enabled=%s",
    effort_stiction_comp_enabled_ ? "true" : "false");

    serial_fd_ = ::open("/dev/teensy_motor", O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("exo_hw"), "Failed to open serial port");
    return hardware_interface::CallbackReturn::ERROR;
    }

    struct termios tty;
    tcgetattr(serial_fd_, &tty);

    cfsetispeed(&tty, B460800);
    cfsetospeed(&tty, B460800);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;

    tcsetattr(serial_fd_, TCSANOW, &tty);

    // Initialize timing diagnostics.
    const auto now_tp = std::chrono::steady_clock::now();
    last_write_tp_ = now_tp;
    last_read_tp_ = now_tp;
    stats_window_start_tp_ = now_tp;

  return hardware_interface::CallbackReturn::SUCCESS;
}

void teensy_plugin::sendSpecialCommand(uint8_t special_cmd_type)
{
  const uint8_t n = motor_ids_.size();

  uint8_t buffer[128];
  int idx = 0;

  buffer[idx++] = HEADER1;
  buffer[idx++] = HEADER2;

  uint8_t cmd = (special_cmd_type << 4) | n;
  buffer[idx++] = cmd;

  for (size_t i = 0; i < n; i++)
  {
    uint8_t id = motor_ids_[i];
    float val = 0.0f;
    memcpy(&buffer[idx], &id, 1);
    idx += 1;
    memcpy(&buffer[idx], &val, 4);
    idx += 4;
  }

  uint8_t cs = computeChecksum(&buffer[2], idx - 2);
  buffer[idx++] = cs;

  ::write(serial_fd_, buffer, idx);
}

hardware_interface::CallbackReturn teensy_plugin::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("exo_hw"), "Deactivating: stopping all motors");
  sendSpecialCommand(6);
  active_cmd_type_ = 5;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn teensy_plugin::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("exo_hw"), "Shutting down: shutdown all motors");
  sendSpecialCommand(7);
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
teensy_plugin::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &position_[i]));

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &velocity_[i]));

        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_EFFORT,
                &effort_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
teensy_plugin::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &position_command_[i]));

        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &velocity_command_[i]));

        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name,
                hardware_interface::HW_IF_EFFORT,
                &effort_command_[i]));
    }

    return command_interfaces;
}


hardware_interface::return_type teensy_plugin::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  const auto now_tp = std::chrono::steady_clock::now();
  const double write_dt_s = std::chrono::duration<double>(now_tp - last_write_tp_).count();
  if (write_cycles_ > 0) {
    write_dt_sum_s_ += write_dt_s;
    write_dt_sq_sum_s2_ += write_dt_s * write_dt_s;
    write_dt_max_s_ = std::max(write_dt_max_s_, write_dt_s);
  }
  write_cycles_++;
  last_write_tp_ = now_tp;

  const auto has_changed = [](const std::vector<double> & now, const std::vector<double> & prev) {
    if (now.size() != prev.size()) {
      return true;
    }
    constexpr double eps = 1e-9;
    for (size_t i = 0; i < now.size(); ++i) {
      const bool now_finite = std::isfinite(now[i]);
      const bool prev_finite = std::isfinite(prev[i]);
      if (now_finite != prev_finite) {
        return true;
      }
      if (now_finite && std::fabs(now[i] - prev[i]) > eps) {
        return true;
      }
    }
    return false;
  };

  const bool position_updated = has_changed(position_command_, last_position_command_);
  const bool velocity_updated = has_changed(velocity_command_, last_velocity_command_);
  const bool effort_updated = has_changed(effort_command_, last_effort_command_);

  // Read-only only when command interfaces are still uninitialized (NaN).
  const auto all_nan = [](const std::vector<double> & v) {
    for (const auto & val : v) {
      if (std::isfinite(val)) {
        return false;
      }
    }
    return true;
  };

  const bool no_active_commands =
    all_nan(position_command_) &&
    all_nan(velocity_command_) &&
    all_nan(effort_command_);

  if (no_active_commands) {
    active_cmd_type_ = 5;
  } else if (position_updated) {
    active_cmd_type_ = 3;
  } else if (velocity_updated) {
    active_cmd_type_ = 2;
  } else if (effort_updated) {
    active_cmd_type_ = 1;
  }

  const uint8_t cmd_type = active_cmd_type_; // 1 torque, 2 speed, 3 position
  const std::vector<double> * active_commands = &effort_command_;
  if (cmd_type == 3) {
    active_commands = &position_command_;
  } else if (cmd_type == 2) {
    active_commands = &velocity_command_;
  }

  const uint8_t n = effort_command_.size();

  uint8_t buffer[128];
  int idx = 0;

  buffer[idx++] = HEADER1;
  buffer[idx++] = HEADER2;

  uint8_t cmd = (cmd_type << 4) | n;
  buffer[idx++] = cmd;

  for(size_t i=0;i<n;i++)
  {
      uint8_t id = motor_ids_[i];
      float effort = static_cast<float>((*active_commands)[i]);
      if (!std::isfinite(effort))
      {
        effort = 0.0f;
      }
      if (cmd_type == 1 && effort_stiction_comp_enabled_)
      {
        const double tau_s = effort_stiction_tau_per_joint_[i];
        const double slope = effort_stiction_slope_per_joint_[i];
        const double e = static_cast<double>(effort);
        const double compensated =
          e + tau_s * std::tanh(e / slope);
        effort = static_cast<float>(compensated);
      }
      if (i < (sizeof(kJointDirection) / sizeof(kJointDirection[0])))
      {
        effort *= static_cast<float>(kJointDirection[i]);
      }

      memcpy(&buffer[idx], &id, 1);
      idx += 1;

      memcpy(&buffer[idx], &effort, 4);
      idx += 4;
  }

  uint8_t cs = computeChecksum(&buffer[2], idx-2);
  buffer[idx++] = cs;

  ::write(serial_fd_, buffer, idx);
  last_position_command_ = position_command_;
  last_velocity_command_ = velocity_command_;
  last_effort_command_ = effort_command_;

  return hardware_interface::return_type::OK;
}



hardware_interface::return_type teensy_plugin::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  const auto now_tp = std::chrono::steady_clock::now();
  const double read_dt_s = std::chrono::duration<double>(now_tp - last_read_tp_).count();
  if (read_cycles_ > 0) {
    read_dt_sum_s_ += read_dt_s;
    read_dt_sq_sum_s2_ += read_dt_s * read_dt_s;
    read_dt_max_s_ = std::max(read_dt_max_s_, read_dt_s);
  }
  read_cycles_++;
  last_read_tp_ = now_tp;

  const size_t n = position_.size();

  uint8_t header[2];
  int nread = ::read(serial_fd_, header, 2);
  bool frame_ok = (nread == 2 && header[0] == HEADER1 && header[1] == HEADER2);
  if (!frame_ok) {
    stale_reads_++;
  }

  if (frame_ok) {
    for(size_t i=0;i<n;i++)
    {
        MotorStatus2 status;

        const int payload_read = ::read(serial_fd_, &status, sizeof(MotorStatus2));
        if (payload_read != (int)sizeof(MotorStatus2)) {
          frame_ok = false;
          stale_reads_++;
          break;
        }

        int idx = status.motorID - 1;

        if(idx < 0 || idx >= (int)n)
            continue;

        const float sign =
            (idx < (int)(sizeof(kJointDirection) / sizeof(kJointDirection[0])))
                ? static_cast<float>(kJointDirection[idx])
                : 1.0f;

        position_[idx] =
            sign * status.angle;

        velocity_[idx] =
            sign * status.speed;

        effort_[idx] =
            sign * status.effort;
        //std::cout << " effort=" << status.effort << std::endl;
    }
  }

  if (frame_ok) {
    fresh_frames_++;
  }

  const double window_s = std::chrono::duration<double>(now_tp - stats_window_start_tp_).count();
  if (window_s >= 1.0) {
    const double write_samples = (write_cycles_ > 1) ? static_cast<double>(write_cycles_ - 1) : 0.0;
    const double read_samples = (read_cycles_ > 1) ? static_cast<double>(read_cycles_ - 1) : 0.0;
    const double write_mean_s = (write_samples > 0.0) ? (write_dt_sum_s_ / write_samples) : 0.0;
    const double read_mean_s = (read_samples > 0.0) ? (read_dt_sum_s_ / read_samples) : 0.0;
    const double write_var_s2 = (write_samples > 0.0) ? (write_dt_sq_sum_s2_ / write_samples - write_mean_s * write_mean_s) : 0.0;
    const double read_var_s2 = (read_samples > 0.0) ? (read_dt_sq_sum_s2_ / read_samples - read_mean_s * read_mean_s) : 0.0;
    const double write_jitter_ms = std::sqrt(std::max(0.0, write_var_s2)) * 1000.0;
    const double read_jitter_ms = std::sqrt(std::max(0.0, read_var_s2)) * 1000.0;
    const double write_rate_hz = write_cycles_ / window_s;
    const double read_rate_hz = read_cycles_ / window_s;
    const double fresh_rate_hz = fresh_frames_ / window_s;
    const double stale_pct = (read_cycles_ > 0) ? (100.0 * static_cast<double>(stale_reads_) / static_cast<double>(read_cycles_)) : 0.0;

    RCLCPP_INFO(
      rclcpp::get_logger("exo_hw"),
      "HW stats: write=%.1fHz (jitter=%.3fms, max_dt=%.3fms) read=%.1fHz (jitter=%.3fms, max_dt=%.3fms) fresh=%.1fHz stale=%.1f%%",
      write_rate_hz, write_jitter_ms, write_dt_max_s_ * 1000.0,
      read_rate_hz, read_jitter_ms, read_dt_max_s_ * 1000.0,
      fresh_rate_hz, stale_pct);

    stats_window_start_tp_ = now_tp;
    write_dt_sum_s_ = 0.0;
    write_dt_sq_sum_s2_ = 0.0;
    write_dt_max_s_ = 0.0;
    read_dt_sum_s_ = 0.0;
    read_dt_sq_sum_s2_ = 0.0;
    read_dt_max_s_ = 0.0;
    write_cycles_ = 0;
    read_cycles_ = 0;
    fresh_frames_ = 0;
    stale_reads_ = 0;
  }

  return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  exo_hardware::teensy_plugin,
  hardware_interface::SystemInterface)