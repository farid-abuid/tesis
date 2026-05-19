#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "controller_interface/controller_interface.hpp"
#include "exo_control_msgs/msg/joint_control_telemetry.hpp"
#include "exo_utils/dynamics/regressor.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace adaptive_slotine_controller
{

/// Slotine-Li adaptive controller (3-DOF arm, regressor form).
///
///   e = q - q_d ,  ė = q̇ - q̇_d
///   q̇_r = q̇_d - Λ·e ,  q̈_r = q̈_d - Λ·ė ,  s = q̇ - q̇_r
///   τ   = Y(q,q̇,q̇_r,q̈_r)·θ̂ - Kd·s          (saturated at ±τ_max)
///   θ̂̇  = -Γ · Yᵀ · s                        (forward-Euler integrated)
///
/// Y is the arm-specific barycentric regressor from exo_utils. θ̂ is seeded from
/// per-link physical parameters (mass / COM / inertia-about-COM) via
/// physicalToBarycentric().
class AdaptiveSlotineController : public controller_interface::ControllerInterface
{
public:
  ~AdaptiveSlotineController() override = default;

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
  static constexpr int kNumJoints = 3;
  static constexpr int kNumParams = 30;  // 10 barycentric params x 3 links

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

  std::string arm_;                       // "right" / "left" -> selects regressor
  std::string reference_trajectory_topic_;

  double gravity_{9.81};
  // Per-joint Λ and Kd. -Kd·s is NOT inertia-scaled, so the discrete stability
  // bound (Kd ≲ 2·I_eff/Δt) differs per joint; the light distal joint needs a
  // much smaller Kd than the proximal ones. Scalar YAML broadcasts to all 3.
  Eigen::Vector3d lambda_{Eigen::Vector3d::Constant(5.0)};
  Eigen::Vector3d kd_{Eigen::Vector3d::Constant(20.0)};
  double tau_max_{10.0};                  // per-joint torque saturation

  // First-order low-pass on the measured velocity the controller consumes
  // (suppresses the Nyquist-frequency limit cycle). 0 cutoff = disabled.
  double velocity_lpf_cutoff_{0.0};       // [Hz]
  Eigen::Vector3d dq_filt_{Eigen::Vector3d::Zero()};
  bool dq_filt_init_{false};

  // Robust adaptation (bounds θ̂ against drift in unidentifiable directions):
  //   θ̂̇ = -Γ ( Yᵀs·[‖s‖≥δ] + σ·(θ̂ - θ̂₀) ),  then clamp θ̂ ∈ [θ̂_lo, θ̂_hi]
  double leakage_sigma_{0.1};             // σ-modification gain toward the prior
  double s_deadband_{0.02};               // freeze the data term when ‖s‖ < δ
  bool projection_enabled_{true};         // clamp θ̂ to a box around θ̂₀
  double projection_rel_{0.5};            // box half-width = max(rel·|θ̂₀|, abs)
  double projection_abs_{0.05};

  Eigen::Matrix<double, kNumParams, 1> gamma_diag_;   // Γ diagonal (repmat per link)
  Eigen::Matrix<double, kNumParams, 1> theta_hat0_;   // seed parameter estimate
  Eigen::Matrix<double, kNumParams, 1> theta_hat_;    // online estimate
  Eigen::Matrix<double, kNumParams, 1> theta_lo_;     // projection box lower bound
  Eigen::Matrix<double, kNumParams, 1> theta_hi_;     // projection box upper bound

  realtime_tools::RealtimeBuffer<TrajectorySetpoint> setpoint_buffer_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;

  bool publish_telemetry_{true};
  std::string telemetry_topic_;
  std::string logging_session_topic_;
  rclcpp::Publisher<exo_control_msgs::msg::JointControlTelemetry>::SharedPtr telemetry_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr logging_session_pub_;
};

}  // namespace adaptive_slotine_controller
