#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "exo_control_msgs/msg/joint_control_telemetry.hpp"
#include "gravity_compensation_controller/dynamics_backend.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace gravity_compensation_controller
{

/// Joint-space PD + gravity feedforward using joint trajectory setpoints
/// (position/velocity references in joint coordinates).
class JointSpaceGravityCompensationController : public controller_interface::ControllerInterface
{
public:
  ~JointSpaceGravityCompensationController() override = default;

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
  bool init_interfaces();

  void referenceTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

  struct TrajectorySetpoint
  {
    std::vector<double> positions;
    std::vector<double> velocities;
    bool valid{false};
  };

  std::vector<std::string> joint_names_param_;

  std::vector<std::string> joint_names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> pos_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> vel_interfaces_;
  std::vector<hardware_interface::LoanedStateInterface *> effort_state_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> cmd_interfaces_;

  std::string dynamics_backend_;
  std::string urdf_path_;
  std::string reference_trajectory_topic_;
  double kp_{50.0};
  double kd_{5.0};
  double gravity_scale_{1.0};

  std::unique_ptr<DynamicsModel> dynamics_;
  realtime_tools::RealtimeBuffer<TrajectorySetpoint> setpoint_buffer_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;

  bool publish_telemetry_{true};
  std::string telemetry_topic_;
  std::string logging_session_topic_;
  rclcpp::Publisher<exo_control_msgs::msg::JointControlTelemetry>::SharedPtr telemetry_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr logging_session_pub_;
};

}  // namespace gravity_compensation_controller
