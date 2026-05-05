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

namespace mpc_controller
{

/// Model predictive control with feedback-linearization inner loop.
///
/// Inner loop cancels M, C, g so the error dynamics become a double integrator:
///   ë = u  (u is the acceleration correction)
///
/// Unconstrained finite-horizon MPC is solved analytically at configure time,
/// yielding a constant state-feedback gain K_mpc ∈ Rⁿˣ²ⁿ.
///
/// Control law: τ = M(q)*(q̈_des - K_mpc*[e; ė]) + C(q,q̇)*q̇ + g(q)
class MpcController : public controller_interface::ControllerInterface
{
public:
  ~MpcController() override = default;

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

  int horizon_{10};         // prediction horizon N
  double dt_{0.01};         // prediction timestep (s)
  double q_cost_{100.0};    // position tracking weight (diagonal of Q)
  double dq_cost_{1.0};     // velocity tracking weight (diagonal of Q)
  double u_cost_{0.01};     // control effort weight (diagonal of R)

  // Precomputed MPC gain: u_0 = -K_mpc * [e; ė]  (R^n × R^2n)
  Eigen::MatrixXd K_mpc_;

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

}  // namespace mpc_controller
