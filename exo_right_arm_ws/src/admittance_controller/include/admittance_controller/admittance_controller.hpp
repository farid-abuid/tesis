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

namespace admittance_controller
{

/// Joint-space admittance control.
///
/// Virtual compliance model:  Md*q̈_v + Bd*q̇_v + Kd*q_v = τ_ext
/// Modified setpoint:         q_cmd = q_des + q_v,  q̇_cmd = q̇_des + q̇_v
/// Inner control (FL):        τ = M(q)*(q̈_cmd + Kp*e + Kd_inner*ė) + C(q,q̇)*q̇ + g(q)
///
/// τ_ext is read from an optional Float64MultiArray topic (n joint torques, Nm).
/// When no message has arrived, τ_ext = 0 (pure trajectory tracking).
class AdmittanceController : public controller_interface::ControllerInterface
{
public:
  ~AdmittanceController() override = default;

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

  // Inner FL servo gains
  double kp_{50.0};
  double kd_{5.0};

  // Virtual compliance parameters (per joint, scalar)
  double md_{1.0};      // desired inertia (kg·m²)
  double bd_{10.0};     // desired damping (N·m·s/rad)
  double kd_adm_{0.0};  // desired stiffness (N·m/rad); 0 = pure admittance

  // Virtual compliance state (updated each cycle in update())
  Eigen::VectorXd q_v_;   // virtual displacement
  Eigen::VectorXd dq_v_;  // virtual velocity

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

}  // namespace admittance_controller
