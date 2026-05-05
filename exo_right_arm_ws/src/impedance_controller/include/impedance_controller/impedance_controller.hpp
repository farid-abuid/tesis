#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "controller_interface/controller_interface.hpp"
#include "exo_control_msgs/msg/joint_control_telemetry.hpp"
#include "exo_utils/dynamics/dynamics_backend.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace impedance_controller
{

/// Joint-space impedance control.
///
/// Shapes the apparent mechanical impedance at the joint level:
///   Md * ë = -Kp*(q - q_des) - Kd*(q̇ - q̇_des) + τ_ext
///
/// Implemented via model-based feedforward so the full nonlinear dynamics are cancelled:
///   τ = M(q)*(q̈_des + Md⁻¹*(-Kp*e - Kd*ė + τ_ext)) + C(q,q̇)*q̇ + g(q)
///
/// τ_ext is read from an optional Float64MultiArray topic (n joint torques, Nm).
class ImpedanceController : public controller_interface::ControllerInterface
{
public:
  ~ImpedanceController() override = default;

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
    std::vector<double> accelerations;
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

  double m_desired_{1.0};   // desired inertia Md (scalar, kg·m²) — shapes apparent inertia
  double kp_imp_{100.0};    // impedance stiffness K (N·m/rad)
  double kd_imp_{20.0};     // impedance damping D (N·m·s/rad)

  // External torque input
  std::string ext_torque_topic_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ext_torque_sub_;
  realtime_tools::RealtimeBuffer<std::vector<double>> ext_torque_buf_;

  std::unique_ptr<exo_utils::dynamics::DynamicsModel> dynamics_;
  realtime_tools::RealtimeBuffer<TrajectorySetpoint> setpoint_buffer_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;

  bool publish_telemetry_{true};
  std::string telemetry_topic_;
  std::string logging_session_topic_;
  rclcpp::Publisher<exo_control_msgs::msg::JointControlTelemetry>::SharedPtr telemetry_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr logging_session_pub_;

  std::string imu_topic_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  realtime_tools::RealtimeBuffer<std::array<double, 4>> imu_orientation_buf_;
};

}  // namespace impedance_controller
