#include "mrac_controller/mrac_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <functional>
#include <unordered_map>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace mrac_controller
{

//==================================================================================================
// Slotine-Li MRAC control law
//
// Reference model (per joint, decoupled):
//   q̈_m = ω²*(q_des - q_m) - 2ζω*q̇_m
//
// Composite error (sliding variable):
//   e   = q - q_m,   ė = q̇ - q̇_m
//   q̇_r = q̇_m - Λ*e          (reference velocity)
//   q̈_r = q̈_m - Λ*ė          (reference acceleration)
//   s   = ė + Λ*e = q̇ - q̇_r  (composite error)
//
// Adaptive control law:
//   τ = M(q)*q̈_r + C(q,q̇_r)*q̇_r + θ⊙g(q) - Kv*s
//   where θ ∈ Rⁿ is the per-joint gravity scale (adapted online)
//
// Stable adaptation (Lyapunov-based):
//   θ̇ = -Γ * (g ⊙ s)
//
// The mutable state (q_m, dq_m, theta) is passed by reference so the law
// is a pure top-level function that the class calls with its members.
//==================================================================================================
namespace
{

bool controlLaw(
  exo_utils::dynamics::DynamicsModel & dyn,
  const std::vector<std::string> & names,
  double omega_m,
  double zeta_m,
  double lambda,
  double kv,
  double gamma,
  double dt,
  const std::vector<double> & q_des,
  const std::vector<double> & q,
  const std::vector<double> & dq,
  Eigen::VectorXd & q_m,     // in/out: reference model position
  Eigen::VectorXd & dq_m,    // in/out: reference model velocity
  Eigen::VectorXd & theta,   // in/out: adaptive gravity scale (init = 1)
  Eigen::VectorXd & tau,
  std::vector<double> & tau_g_raw,  // raw gravity torques (for telemetry)
  std::string * error)
{
  const Eigen::Index n = static_cast<Eigen::Index>(names.size());

  Eigen::Map<const Eigen::VectorXd> q_des_v(q_des.data(), n);
  Eigen::Map<const Eigen::VectorXd> q_v(q.data(), n);
  Eigen::Map<const Eigen::VectorXd> dq_v(dq.data(), n);

  // Reference model dynamics
  const Eigen::VectorXd ddq_m =
    omega_m * omega_m * (q_des_v - q_m) - 2.0 * zeta_m * omega_m * dq_m;

  // Composite error
  const Eigen::VectorXd e  = q_v - q_m;
  const Eigen::VectorXd de = dq_v - dq_m;
  const Eigen::VectorXd s  = de + lambda * e;           // sliding variable

  // Reference trajectories for the computed-torque law
  const Eigen::VectorXd dq_r  = dq_m - lambda * e;     // reference velocity
  const Eigen::VectorXd ddq_r = ddq_m - lambda * de;   // reference acceleration

  // Dynamics about reference trajectory
  Eigen::MatrixXd M;
  Eigen::MatrixXd C;
  std::vector<double> dq_r_std(dq_r.data(), dq_r.data() + n);

  if (!dyn.computeMassMatrix(names, q, M, error)) return false;
  if (!dyn.computeCoriolisMatrix(names, q, dq_r_std, C, error)) return false;
  if (!dyn.computeGravityTorque(names, q, tau_g_raw, error)) return false;

  Eigen::Map<const Eigen::VectorXd> g_v(tau_g_raw.data(), n);

  // Adaptive gravity: ĝ = θ ⊙ g(q)
  const Eigen::VectorXd g_hat = theta.cwiseProduct(g_v);

  // Control torque: τ = M*q̈_r + C*q̇_r + ĝ - Kv*s
  tau = M * ddq_r + C * dq_r + g_hat - kv * s;

  // Stable adaptation law: θ̇ = -Γ * (g ⊙ s)
  theta -= dt * gamma * g_v.cwiseProduct(s);

  // Integrate reference model (forward Euler)
  dq_m += dt * ddq_m;
  q_m  += dt * dq_m;

  return true;
}

}  // namespace
//==================================================================================================

controller_interface::CallbackReturn MracController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<std::string>("dynamics_backend", std::string("pinocchio"));
  auto_declare<std::string>("urdf_path", std::string(""));
  auto_declare<std::string>("dynamics_urdf_filename", std::string("exo_dynamics_right.urdf"));
  auto_declare<std::string>("reference_trajectory_topic", std::string("/reference_trajectory"));
  auto_declare<double>("omega_m", 5.0);
  auto_declare<double>("zeta_m", 0.9);
  auto_declare<double>("lambda", 5.0);
  auto_declare<double>("kv", 20.0);
  auto_declare<double>("gamma", 0.5);
  auto_declare<bool>("publish_telemetry", true);
  auto_declare<std::string>("telemetry_topic", std::string("telemetry"));
  auto_declare<std::string>("logging_session_topic", std::string("logging/session"));
  auto_declare<std::string>("imu_topic", std::string(""));

  RCLCPP_INFO(get_node()->get_logger(), "MRAC controller initialized");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MracController::on_configure(
  const rclcpp_lifecycle::State &)
{
  joint_names_param_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_param_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  dynamics_backend_ = get_node()->get_parameter("dynamics_backend").as_string();
  urdf_path_ = get_node()->get_parameter("urdf_path").as_string();
  const std::string dynamics_urdf_filename =
    get_node()->get_parameter("dynamics_urdf_filename").as_string();
  reference_trajectory_topic_ =
    get_node()->get_parameter("reference_trajectory_topic").as_string();
  omega_m_ = get_node()->get_parameter("omega_m").as_double();
  zeta_m_  = get_node()->get_parameter("zeta_m").as_double();
  lambda_  = get_node()->get_parameter("lambda").as_double();
  kv_      = get_node()->get_parameter("kv").as_double();
  gamma_   = get_node()->get_parameter("gamma").as_double();
  publish_telemetry_ = get_node()->get_parameter("publish_telemetry").as_bool();
  telemetry_topic_ = get_node()->get_parameter("telemetry_topic").as_string();
  logging_session_topic_ = get_node()->get_parameter("logging_session_topic").as_string();
  imu_topic_ = get_node()->get_parameter("imu_topic").as_string();

  if (publish_telemetry_) {
    const auto telem_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    telemetry_pub_ = get_node()->create_publisher<exo_control_msgs::msg::JointControlTelemetry>(
      telemetry_topic_, telem_qos);
    const auto session_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    logging_session_pub_ = get_node()->create_publisher<std_msgs::msg::Bool>(
      logging_session_topic_, session_qos);
  }

  if (urdf_path_.empty()) {
    try {
      const std::string share = ament_index_cpp::get_package_share_directory("exo_description");
      urdf_path_ = share + "/urdf/" + dynamics_urdf_filename;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "urdf_path is empty and could not resolve exo_description share: %s", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  std::string dyn_err;
  dynamics_ = exo_utils::dynamics::createDynamicsModel(dynamics_backend_, &dyn_err);
  if (!dynamics_) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", dyn_err.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!dynamics_->loadUrdf(urdf_path_, &dyn_err)) {
    RCLCPP_ERROR(get_node()->get_logger(), "URDF load failed: %s", dyn_err.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  TrajectorySetpoint init;
  init.positions.assign(joint_names_param_.size(), 0.0);
  init.velocities.assign(joint_names_param_.size(), 0.0);
  init.valid = true;
  setpoint_buffer_.writeFromNonRT(init);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured (MRAC): backend=%s omega_m=%.2f zeta_m=%.2f lambda=%.2f kv=%.2f gamma=%.3f",
    dynamics_backend_.c_str(), omega_m_, zeta_m_, lambda_, kv_, gamma_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MracController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!init_interfaces()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Seed reference model from current joint positions to avoid initial transient
  const size_t n = joint_names_.size();
  q_m_   = Eigen::VectorXd::Zero(n);
  dq_m_  = Eigen::VectorXd::Zero(n);
  theta_ = Eigen::VectorXd::Ones(n);
  for (size_t i = 0; i < n; ++i) {
    auto q_opt = pos_interfaces_[i].get().get_optional();
    if (q_opt) q_m_(static_cast<Eigen::Index>(i)) = *q_opt;
  }

  traj_sub_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    reference_trajectory_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&MracController::referenceTrajectoryCallback, this, std::placeholders::_1));

  imu_orientation_buf_.writeFromNonRT({1.0, 0.0, 0.0, 0.0});
  if (!imu_topic_.empty()) {
    imu_sub_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::array<double, 4> q{
          msg->orientation.w, msg->orientation.x,
          msg->orientation.y, msg->orientation.z};
        imu_orientation_buf_.writeFromNonRT(q);
      });
  }

  if (logging_session_pub_) {
    std_msgs::msg::Bool msg;
    msg.data = true;
    logging_session_pub_->publish(msg);
  }

  RCLCPP_INFO(get_node()->get_logger(), "MRAC controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MracController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (logging_session_pub_) {
    std_msgs::msg::Bool msg;
    msg.data = false;
    logging_session_pub_->publish(msg);
  }
  traj_sub_.reset();
  imu_sub_.reset();
  RCLCPP_INFO(get_node()->get_logger(), "MRAC controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MracController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_param_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
MracController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_param_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

bool MracController::init_interfaces()
{
  if (state_interfaces_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No state interfaces assigned");
    return false;
  }

  std::unordered_map<std::string, hardware_interface::LoanedStateInterface *> pos_map;
  std::unordered_map<std::string, hardware_interface::LoanedStateInterface *> vel_map;
  std::unordered_map<std::string, hardware_interface::LoanedStateInterface *> eff_map;

  for (auto & iface : state_interfaces_) {
    const auto & joint = iface.get_prefix_name();
    const auto & name = iface.get_interface_name();
    if (name == hardware_interface::HW_IF_POSITION) pos_map[joint] = &iface;
    else if (name == hardware_interface::HW_IF_VELOCITY) vel_map[joint] = &iface;
    else if (name == hardware_interface::HW_IF_EFFORT) eff_map[joint] = &iface;
  }

  joint_names_.clear();
  pos_interfaces_.clear();
  vel_interfaces_.clear();
  effort_state_.clear();
  cmd_interfaces_.clear();

  for (const auto & joint : joint_names_param_) {
    if (!pos_map.count(joint) || !vel_map.count(joint)) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Missing state interfaces for joint %s", joint.c_str());
      return false;
    }
    joint_names_.push_back(joint);
    pos_interfaces_.push_back(*pos_map[joint]);
    vel_interfaces_.push_back(*vel_map[joint]);
    effort_state_.push_back(eff_map.count(joint) ? eff_map[joint] : nullptr);
  }

  std::unordered_map<std::string, hardware_interface::LoanedCommandInterface *> cmd_map;
  for (auto & cmd : command_interfaces_) cmd_map[cmd.get_prefix_name()] = &cmd;
  for (const auto & joint : joint_names_) {
    if (!cmd_map.count(joint)) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Missing command interface for joint %s", joint.c_str());
      return false;
    }
    cmd_interfaces_.push_back(*cmd_map[joint]);
  }

  RCLCPP_INFO(get_node()->get_logger(), "Initialized %zu joints", joint_names_.size());
  return true;
}

void MracController::referenceTrajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (!msg || msg->points.empty()) return;

  const trajectory_msgs::msg::JointTrajectoryPoint & pt = msg->points.front();
  TrajectorySetpoint sp;
  sp.positions.resize(joint_names_param_.size());
  sp.velocities.resize(joint_names_param_.size(), 0.0);

  for (size_t j = 0; j < joint_names_param_.size(); ++j) {
    const std::string & name = joint_names_param_[j];
    auto it = std::find(msg->joint_names.begin(), msg->joint_names.end(), name);
    if (it == msg->joint_names.end()) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Trajectory message missing joint '%s'; ignoring", name.c_str());
      return;
    }
    const size_t k = static_cast<size_t>(std::distance(msg->joint_names.begin(), it));
    if (k < pt.positions.size()) sp.positions[j] = pt.positions[k]; else return;
    if (k < pt.velocities.size()) sp.velocities[j] = pt.velocities[k];
  }

  sp.valid = true;
  setpoint_buffer_.writeFromNonRT(sp);
}

controller_interface::return_type MracController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  TrajectorySetpoint * sp_ptr = setpoint_buffer_.readFromRT();
  if (!sp_ptr->valid || sp_ptr->positions.size() != joint_names_.size()) {
    return controller_interface::return_type::ERROR;
  }

  std::vector<double> q_meas(joint_names_.size());
  std::vector<double> dq_meas(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    auto q_opt = pos_interfaces_[i].get().get_optional();
    auto dq_opt = vel_interfaces_[i].get().get_optional();
    if (!q_opt || !dq_opt) return controller_interface::return_type::ERROR;
    q_meas[i] = *q_opt;
    dq_meas[i] = *dq_opt;
  }

  //===============================================================================================

  if (imu_sub_) {
    const std::array<double, 4> * q = imu_orientation_buf_.readFromRT();
    if (q) {
      const Eigen::Quaterniond quat((*q)[0], (*q)[1], (*q)[2], (*q)[3]);
      const Eigen::Vector3d g_arm =
        quat.toRotationMatrix().transpose() * Eigen::Vector3d(0.0, 0.0, -9.81);
      std::string err;
      dynamics_->setGravity(g_arm, &err);
    }
  }

  const double dt = period.seconds();
  std::vector<double> tau_g_raw;
  std::string ctrl_err;
  Eigen::VectorXd tau_vec;
  if (!controlLaw(
      *dynamics_, joint_names_,
      omega_m_, zeta_m_, lambda_, kv_, gamma_, dt,
      sp_ptr->positions, q_meas, dq_meas,
      q_m_, dq_m_, theta_,
      tau_vec, tau_g_raw, &ctrl_err))
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "MRAC control law failed: %s", ctrl_err.c_str());
    return controller_interface::return_type::ERROR;
  }

  //===============================================================================================

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (!cmd_interfaces_[i].get().set_value(tau_vec(static_cast<Eigen::Index>(i)))) {
      return controller_interface::return_type::ERROR;
    }
  }

  if (telemetry_pub_) {
    exo_control_msgs::msg::JointControlTelemetry msg;
    msg.header.stamp = time;
    msg.header.frame_id = "base_link";
    msg.joint_names = joint_names_;
    const size_t n = joint_names_.size();
    msg.position.resize(n);
    msg.velocity.resize(n);
    msg.position_reference.resize(n);
    msg.velocity_reference.resize(n);
    msg.effort_command.resize(n);
    msg.effort_feedforward.resize(n);
    bool any_effort = false;
    for (auto * es : effort_state_) { if (es) { any_effort = true; break; } }
    if (any_effort) msg.effort_measured.resize(n);
    for (size_t i = 0; i < n; ++i) {
      const Eigen::Index ii = static_cast<Eigen::Index>(i);
      msg.position[i] = q_meas[i];
      msg.velocity[i] = dq_meas[i];
      msg.position_reference[i] = q_m_(ii);    // reference model output
      msg.velocity_reference[i] = dq_m_(ii);
      msg.effort_feedforward[i] = theta_(ii) * tau_g_raw[i];  // adaptive gravity
      msg.effort_command[i] = tau_vec(ii);
      if (any_effort && effort_state_[i]) {
        auto te = effort_state_[i]->get_optional();
        msg.effort_measured[i] = te ? *te : 0.0;
      }
    }
    telemetry_pub_->publish(msg);
  }

  return controller_interface::return_type::OK;
}

}  // namespace mrac_controller

PLUGINLIB_EXPORT_CLASS(
  mrac_controller::MracController, controller_interface::ControllerInterface)
