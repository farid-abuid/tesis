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
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace mrac_controller
{

/// Model reference adaptive control (Slotine-Li).
///
/// Reference model: q̈_m = ω²*(q_des - q_m) - 2ζω*q̇_m
/// Composite error:  s = (q̇ - q̇_r) where q̇_r = q̇_m - Λ*(q - q_m)
/// Control law:      τ = M(q)*q̈_r + C(q,q̇)*q̇_r + ĝ(q,θ) - Kv*s
/// Adaptation:       θ̇ = -Γ * Ŷ(q,q̇,q̇_r,q̈_r)ᵀ * s  (simplified: gravity scale per joint)
class MracController : public controller_interface::ControllerInterface
{
public:
  ~MracController() override = default;

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

  double omega_m_{5.0};   // reference model natural frequency (rad/s)
  double zeta_m_{0.9};    // reference model damping ratio
  double lambda_{5.0};    // composite error slope (Λ, per joint scalar)
  double kv_{20.0};       // velocity error feedback gain
  double gamma_{0.5};     // adaptation gain

  // Reference model state (updated each cycle in update())
  Eigen::VectorXd q_m_;    // reference model position
  Eigen::VectorXd dq_m_;   // reference model velocity
  // Adaptive parameter: per-joint gravity scale estimate (θ ∈ Rⁿ, init = 1)
  Eigen::VectorXd theta_;

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

}  // namespace mrac_controller
