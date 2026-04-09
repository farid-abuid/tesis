#include "exo_hardware/exo_hardware.hpp"
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <iostream>
#include <limits>
#include <cmath>

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

    serial_fd_ = ::open("/dev/teensy_motor", O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("exo_hw"), "Failed to open serial port");
    return hardware_interface::CallbackReturn::ERROR;
    }

    struct termios tty;
    tcgetattr(serial_fd_, &tty);

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;

    tcsetattr(serial_fd_, TCSANOW, &tty);

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

  // Controllers switch mode by writing to their claimed command interface.
  if (position_updated) {
    active_cmd_type_ = 9;
  } else if (velocity_updated) {
    active_cmd_type_ = 2;
  } else if (effort_updated) {
    active_cmd_type_ = 1;
  }

  const uint8_t cmd_type = active_cmd_type_; // 1 torque, 2 speed, 9 angle
  const std::vector<double> * active_commands = &effort_command_;
  if (cmd_type == 9) {
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
  const size_t n = position_.size();

  const int expected =
      2 + n*sizeof(MotorStatus2);

  uint8_t header[2];
  int nread = ::read(serial_fd_, header, 2);
  if (nread != 2) return hardware_interface::return_type::OK;

  for(size_t i=0;i<n;i++)
  {
      MotorStatus2 status;

      ::read(serial_fd_, &status, sizeof(MotorStatus2));

      int idx = status.motorID - 1;

      if(idx < 0 || idx >= (int)n)
          continue;

      position_[idx] =
          status.angle;

      velocity_[idx] =
          status.speed;

      effort_[idx] =
          status.effort;
      //std::cout << " effort=" << status.effort << std::endl;
  }

  return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  exo_hardware::teensy_plugin,
  hardware_interface::SystemInterface)