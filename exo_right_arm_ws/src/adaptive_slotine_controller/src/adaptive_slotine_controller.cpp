#include "adaptive_slotine_controller/adaptive_slotine_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <unordered_map>

namespace adaptive_slotine_controller
{

//==================================================================================================
// Slotine-Li adaptive control + robust adaptation law
//   e = q - q_d ,  ė = q̇ - q̇_d
//   q̇_r = q̇_d - Λ*e ,  q̈_r = q̈_d - Λ*ė ,  s = q̇ - q̇_r   (= ė + Λ*e)
//   τ   = Y(q,q̇,q̇_r,q̈_r)*θ̂ - Kd*s                       (saturated at ±τ_max)
//
// Robust adaptation (the consistency check showed 12/30 params are structurally
// unidentifiable for a 3R arm — pure -ΓYᵀs drifts there without excitation):
//   θ̂̇ = -Γ ( Yᵀs·[‖s‖≥δ]  +  σ·(θ̂ - θ̂₀) )      (σ-modification toward prior)
//   θ̂ ← clamp(θ̂, θ̂_lo, θ̂_hi)                     (parameter projection)
//
// θ̂ is passed by reference (in/out) so the law is a pure top-level function
// the class calls with its member state.
//==================================================================================================
namespace
{

void controlLaw(
  const std::string & arm,
  const Eigen::Vector3d & lambda,
  const Eigen::Vector3d & kd,
  double tau_max,
  double gravity,
  double dt,
  const exo_utils::dynamics::BarycentricVector & gamma_diag,
  const exo_utils::dynamics::BarycentricVector & theta_hat0,
  double sigma,
  double s_deadband,
  bool projection_enabled,
  const exo_utils::dynamics::BarycentricVector & theta_lo,
  const exo_utils::dynamics::BarycentricVector & theta_hi,
  const Eigen::Vector3d & q_d,
  const Eigen::Vector3d & dq_d,
  const Eigen::Vector3d & ddq_d,
  const Eigen::Vector3d & q,
  const Eigen::Vector3d & dq,
  exo_utils::dynamics::BarycentricVector & theta_hat,  // in/out: adaptive estimate
  Eigen::Vector3d & tau,
  Eigen::Vector3d & tau_model,
  Eigen::Vector3d & s_out)
{
  const Eigen::Vector3d e = q - q_d;
  const Eigen::Vector3d de = dq - dq_d;
  const Eigen::Vector3d qd_r = dq_d - lambda.cwiseProduct(e);
  const Eigen::Vector3d qdd_r = ddq_d - lambda.cwiseProduct(de);
  const Eigen::Vector3d s = dq - qd_r;  // = de + lambda*e

  const exo_utils::dynamics::RegressorMatrix Y =
    exo_utils::dynamics::exo_Y(arm, q, dq, qd_r, qdd_r, gravity);

  tau_model = Y * theta_hat;
  tau = tau_model - kd.cwiseProduct(s);
  tau = tau.cwiseMax(-tau_max).cwiseMin(tau_max);

  s_out = s;

  // Data-driven term, gated by a deadband so sensor noise at small ‖s‖ doesn't
  // drive the (unobservable) parameter directions.
  exo_utils::dynamics::BarycentricVector grad =
    exo_utils::dynamics::BarycentricVector::Zero();
  if (s.norm() >= s_deadband) {
    grad = Y.transpose() * s;
  }
  // σ-modification: always-on pull back toward the physical prior θ̂₀ keeps the
  // estimate bounded even with zero persistent excitation.
  grad += sigma * (theta_hat - theta_hat0);

  theta_hat -= dt * gamma_diag.cwiseProduct(grad);

  // Parameter projection: hard clamp to a box around the prior.
  if (projection_enabled) {
    theta_hat = theta_hat.cwiseMax(theta_lo).cwiseMin(theta_hi);
  }
}

// Read an n-element double parameter into an Eigen vector (errors if size wrong).
template<typename NodeT>
bool getVecParam(
  const NodeT & node, const std::string & name, int n,
  Eigen::VectorXd & out, std::string * error)
{
  const std::vector<double> v = node->get_parameter(name).as_double_array();
  if (static_cast<int>(v.size()) != n) {
    *error = "parameter '" + name + "' must have " + std::to_string(n) + " elements (got " +
      std::to_string(v.size()) + ")";
    return false;
  }
  out = Eigen::Map<const Eigen::VectorXd>(v.data(), n);
  return true;
}
}  // namespace
//==================================================================================================

controller_interface::CallbackReturn AdaptiveSlotineController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<std::string>("arm", std::string("right"));
  auto_declare<std::string>("reference_trajectory_topic", std::string("/reference_trajectory"));
  auto_declare<double>("gravity", 9.81);
  // Scalar lambda/kd broadcast to all 3 joints; the *_per_joint arrays (3
  // elements) override when non-empty (use a small kd for the light joint 3).
  auto_declare<double>("lambda", 5.0);
  auto_declare<double>("kd", 20.0);
  auto_declare<std::vector<double>>("lambda_per_joint", {});
  auto_declare<std::vector<double>>("kd_per_joint", {});
  auto_declare<double>("tau_max", 10.0);
  // First-order low-pass cutoff [Hz] on the measured velocity (0 = disabled).
  auto_declare<double>("velocity_lpf_cutoff", 0.0);
  // Robust adaptation knobs.
  auto_declare<double>("leakage_sigma", 0.1);
  auto_declare<double>("s_deadband", 0.02);
  auto_declare<bool>("projection_enabled", true);
  auto_declare<double>("projection_rel", 0.5);
  auto_declare<double>("projection_abs", 0.05);
  // Per-link adaptation gains (10 entries, repeated for each of the 3 links).
  auto_declare<std::vector<double>>(
    "gamma_per_link", {1.0, 1.0, 1.0, 1.0, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3});
  // Direct barycentric seed (30 = 10 params x 3 links). If non-empty it
  // overrides the physical-parameter seed below — use the URDF-consistent θ*
  // from `ros2 run exo_utils exo_regressor_check`.
  auto_declare<std::vector<double>>("theta0", {});
  // Initial physical parameters (one entry per link; 3 links).
  auto_declare<std::vector<double>>("link_mass", {0.0, 0.0, 0.0});
  auto_declare<std::vector<double>>("com_x", {0.0, 0.0, 0.0});
  auto_declare<std::vector<double>>("com_y", {0.0, 0.0, 0.0});
  auto_declare<std::vector<double>>("com_z", {0.0, 0.0, 0.0});
  auto_declare<std::vector<double>>("inertia_ixx", {0.0, 0.0, 0.0});
  auto_declare<std::vector<double>>("inertia_iyy", {0.0, 0.0, 0.0});
  auto_declare<std::vector<double>>("inertia_izz", {0.0, 0.0, 0.0});
  auto_declare<std::vector<double>>("inertia_ixy", {0.0, 0.0, 0.0});
  auto_declare<std::vector<double>>("inertia_ixz", {0.0, 0.0, 0.0});
  auto_declare<std::vector<double>>("inertia_iyz", {0.0, 0.0, 0.0});
  auto_declare<bool>("publish_telemetry", true);
  auto_declare<std::string>("telemetry_topic", std::string("telemetry"));
  auto_declare<std::string>("logging_session_topic", std::string("logging/session"));

  RCLCPP_INFO(get_node()->get_logger(), "Adaptive Slotine-Li controller initialized");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdaptiveSlotineController::on_configure(
  const rclcpp_lifecycle::State &)
{
  joint_names_param_ = get_node()->get_parameter("joints").as_string_array();
  if (static_cast<int>(joint_names_param_.size()) != kNumJoints) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'joints' must list exactly %d joints (got %zu)", kNumJoints, joint_names_param_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  arm_ = get_node()->get_parameter("arm").as_string();
  if (arm_ != "right" && arm_ != "left") {
    RCLCPP_ERROR(get_node()->get_logger(), "'arm' must be 'right' or 'left' (got '%s')", arm_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (arm_ == "left" && !exo_utils::dynamics::exoLeftYImplemented()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Left-arm regressor not yet implemented (see exo_left_Y in exo_utils/src/dynamics/regressor.cpp)");
    return controller_interface::CallbackReturn::ERROR;
  }

  reference_trajectory_topic_ = get_node()->get_parameter("reference_trajectory_topic").as_string();
  gravity_ = get_node()->get_parameter("gravity").as_double();
  {
    const double lam = get_node()->get_parameter("lambda").as_double();
    const double kd = get_node()->get_parameter("kd").as_double();
    lambda_ = Eigen::Vector3d::Constant(lam);
    kd_ = Eigen::Vector3d::Constant(kd);
    const auto lam_pj = get_node()->get_parameter("lambda_per_joint").as_double_array();
    const auto kd_pj = get_node()->get_parameter("kd_per_joint").as_double_array();
    if (!lam_pj.empty()) {
      if (lam_pj.size() != 3) {
        RCLCPP_ERROR(get_node()->get_logger(), "'lambda_per_joint' must have 3 elements");
        return controller_interface::CallbackReturn::ERROR;
      }
      lambda_ = Eigen::Vector3d(lam_pj[0], lam_pj[1], lam_pj[2]);
    }
    if (!kd_pj.empty()) {
      if (kd_pj.size() != 3) {
        RCLCPP_ERROR(get_node()->get_logger(), "'kd_per_joint' must have 3 elements");
        return controller_interface::CallbackReturn::ERROR;
      }
      kd_ = Eigen::Vector3d(kd_pj[0], kd_pj[1], kd_pj[2]);
    }
  }
  tau_max_ = get_node()->get_parameter("tau_max").as_double();
  velocity_lpf_cutoff_ = get_node()->get_parameter("velocity_lpf_cutoff").as_double();
  dq_filt_init_ = false;
  leakage_sigma_ = get_node()->get_parameter("leakage_sigma").as_double();
  s_deadband_ = get_node()->get_parameter("s_deadband").as_double();
  projection_enabled_ = get_node()->get_parameter("projection_enabled").as_bool();
  projection_rel_ = get_node()->get_parameter("projection_rel").as_double();
  projection_abs_ = get_node()->get_parameter("projection_abs").as_double();
  publish_telemetry_ = get_node()->get_parameter("publish_telemetry").as_bool();
  telemetry_topic_ = get_node()->get_parameter("telemetry_topic").as_string();
  logging_session_topic_ = get_node()->get_parameter("logging_session_topic").as_string();

  std::string perr;

  // Γ diagonal: repmat(gamma_per_link, 1, n_links).
  Eigen::VectorXd gpl;
  if (!getVecParam(get_node(), "gamma_per_link", 10, gpl, &perr)) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", perr.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  for (int i = 0; i < kNumJoints; ++i) {
    gamma_diag_.segment<10>(10 * i) = gpl;
  }

  // Seed θ̂ from per-link physical parameters.
  Eigen::VectorXd m, xc, yc, zc, ixx, iyy, izz, ixy, ixz, iyz;
  if (!getVecParam(get_node(), "link_mass", kNumJoints, m, &perr) ||
    !getVecParam(get_node(), "com_x", kNumJoints, xc, &perr) ||
    !getVecParam(get_node(), "com_y", kNumJoints, yc, &perr) ||
    !getVecParam(get_node(), "com_z", kNumJoints, zc, &perr) ||
    !getVecParam(get_node(), "inertia_ixx", kNumJoints, ixx, &perr) ||
    !getVecParam(get_node(), "inertia_iyy", kNumJoints, iyy, &perr) ||
    !getVecParam(get_node(), "inertia_izz", kNumJoints, izz, &perr) ||
    !getVecParam(get_node(), "inertia_ixy", kNumJoints, ixy, &perr) ||
    !getVecParam(get_node(), "inertia_ixz", kNumJoints, ixz, &perr) ||
    !getVecParam(get_node(), "inertia_iyz", kNumJoints, iyz, &perr))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", perr.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  const std::vector<double> theta0_param =
    get_node()->get_parameter("theta0").as_double_array();
  if (theta0_param.empty()) {
    theta_hat0_ = exo_utils::dynamics::physicalToBarycentric(
      m, xc, yc, zc, ixx, iyy, izz, ixy, ixz, iyz);
    RCLCPP_INFO(get_node()->get_logger(), "Seeded θ̂₀ from physical parameters");
  } else if (static_cast<int>(theta0_param.size()) == kNumParams) {
    theta_hat0_ = Eigen::Map<const exo_utils::dynamics::BarycentricVector>(
      theta0_param.data());
    RCLCPP_INFO(get_node()->get_logger(), "Seeded θ̂₀ from direct 'theta0' vector");
  } else {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'theta0' must have %d elements (got %zu)", kNumParams, theta0_param.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  theta_hat_ = theta_hat0_;

  // Projection box: half-width = max(rel·|θ̂₀|, abs) per parameter.
  const exo_utils::dynamics::BarycentricVector half =
    (projection_rel_ * theta_hat0_.cwiseAbs()).cwiseMax(projection_abs_);
  theta_lo_ = theta_hat0_ - half;
  theta_hi_ = theta_hat0_ + half;

  if (publish_telemetry_) {
    const auto telem_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    telemetry_pub_ = get_node()->create_publisher<exo_control_msgs::msg::JointControlTelemetry>(
      telemetry_topic_, telem_qos);
    const auto session_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    logging_session_pub_ = get_node()->create_publisher<std_msgs::msg::Bool>(
      logging_session_topic_, session_qos);
  }

  setpoint_buffer_.writeFromNonRT(TrajectorySetpoint{});

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured (adaptive Slotine-Li): arm=%s lambda=[%.2f %.2f %.2f] "
    "kd=[%.2f %.2f %.2f] tau_max=%.2f g=%.2f vel_lpf=%.1fHz "
    "| robust: sigma=%.3f deadband=%.3f projection=%s(rel=%.2f abs=%.3f)",
    arm_.c_str(), lambda_(0), lambda_(1), lambda_(2), kd_(0), kd_(1), kd_(2),
    tau_max_, gravity_, velocity_lpf_cutoff_,
    leakage_sigma_, s_deadband_, projection_enabled_ ? "on" : "off",
    projection_rel_, projection_abs_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdaptiveSlotineController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!init_interfaces()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Reset the parameter estimate to its seed on every (re)activation.
  theta_hat_ = theta_hat0_;

  traj_sub_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    reference_trajectory_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&AdaptiveSlotineController::referenceTrajectoryCallback, this, std::placeholders::_1));

  if (logging_session_pub_) {
    std_msgs::msg::Bool msg;
    msg.data = true;
    logging_session_pub_->publish(msg);
  }

  RCLCPP_INFO(get_node()->get_logger(), "Adaptive Slotine-Li controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdaptiveSlotineController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (logging_session_pub_) {
    std_msgs::msg::Bool msg;
    msg.data = false;
    logging_session_pub_->publish(msg);
  }
  traj_sub_.reset();
  RCLCPP_INFO(get_node()->get_logger(), "Adaptive Slotine-Li controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
AdaptiveSlotineController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_param_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
AdaptiveSlotineController::state_interface_configuration() const
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

bool AdaptiveSlotineController::init_interfaces()
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

void AdaptiveSlotineController::referenceTrajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (!msg || msg->points.empty()) return;

  const trajectory_msgs::msg::JointTrajectoryPoint & pt = msg->points.front();
  TrajectorySetpoint sp;
  sp.positions.resize(joint_names_param_.size());
  sp.velocities.resize(joint_names_param_.size(), 0.0);
  sp.accelerations.resize(joint_names_param_.size(), 0.0);

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
    if (k < pt.accelerations.size()) sp.accelerations[j] = pt.accelerations[k];
  }

  sp.valid = true;
  setpoint_buffer_.writeFromNonRT(sp);
}

controller_interface::return_type AdaptiveSlotineController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  Eigen::Vector3d q, dq;
  for (int i = 0; i < kNumJoints; ++i) {
    auto q_opt = pos_interfaces_[static_cast<size_t>(i)].get().get_optional();
    auto dq_opt = vel_interfaces_[static_cast<size_t>(i)].get().get_optional();
    if (!q_opt || !dq_opt) return controller_interface::return_type::ERROR;
    q(i) = *q_opt;
    dq(i) = *dq_opt;
  }

  // First-order low-pass on measured velocity (suppresses the Nyquist limit
  // cycle of the un-inertia-scaled -Kd·s term on the light distal joint).
  if (velocity_lpf_cutoff_ > 0.0) {
    if (!dq_filt_init_) {
      dq_filt_ = dq;
      dq_filt_init_ = true;
    } else {
      const double dt = period.seconds();
      const double tau_c = 1.0 / (2.0 * M_PI * velocity_lpf_cutoff_);
      const double a = dt / (tau_c + dt);  // discrete first-order, a∈(0,1]
      dq_filt_ += a * (dq - dq_filt_);
    }
    dq = dq_filt_;
  }

  TrajectorySetpoint * sp_ptr = setpoint_buffer_.readFromRT();
  TrajectorySetpoint hold_sp;
  if (!sp_ptr->valid || static_cast<int>(sp_ptr->positions.size()) != kNumJoints) {
    // No reference trajectory yet — hold the current measured pose.
    hold_sp.positions.assign(q.data(), q.data() + kNumJoints);
    hold_sp.velocities.assign(kNumJoints, 0.0);
    hold_sp.accelerations.assign(kNumJoints, 0.0);
    hold_sp.valid = true;
    sp_ptr = &hold_sp;
  }

  const Eigen::Vector3d q_d = Eigen::Map<const Eigen::Vector3d>(sp_ptr->positions.data());
  const Eigen::Vector3d dq_d = Eigen::Map<const Eigen::Vector3d>(sp_ptr->velocities.data());
  const Eigen::Vector3d ddq_d = Eigen::Map<const Eigen::Vector3d>(sp_ptr->accelerations.data());

  //===============================================================================================

  Eigen::Vector3d tau;
  Eigen::Vector3d tau_model;
  Eigen::Vector3d s_vec;
  controlLaw(
    arm_, lambda_, kd_, tau_max_, gravity_, period.seconds(), gamma_diag_,
    theta_hat0_, leakage_sigma_, s_deadband_, projection_enabled_,
    theta_lo_, theta_hi_,
    q_d, dq_d, ddq_d, q, dq, theta_hat_, tau, tau_model, s_vec);

  //===============================================================================================

  for (int i = 0; i < kNumJoints; ++i) {
    if (!cmd_interfaces_[static_cast<size_t>(i)].get().set_value(tau(i))) {
      return controller_interface::return_type::ERROR;
    }
  }

  if (telemetry_pub_) {
    exo_control_msgs::msg::JointControlTelemetry msg;
    msg.header.stamp = time;
    msg.header.frame_id = "base_link";
    msg.joint_names = joint_names_;
    const size_t n = kNumJoints;
    msg.position.resize(n);
    msg.velocity.resize(n);
    msg.position_reference.resize(n);
    msg.velocity_reference.resize(n);
    msg.effort_command.resize(n);
    msg.effort_feedforward.resize(n);
    bool any_effort = false;
    for (auto * es : effort_state_) { if (es) { any_effort = true; break; } }
    if (any_effort) msg.effort_measured.resize(n);
    msg.sliding_surface.resize(n);
    msg.sliding_lambda = lambda_(0);  // joint-1 value (telemetry is scalar)
    msg.adaptive_parameters.assign(theta_hat_.data(), theta_hat_.data() + kNumParams);
    msg.adaptive_parameters_initial.assign(
      theta_hat0_.data(), theta_hat0_.data() + kNumParams);
    for (int i = 0; i < kNumJoints; ++i) {
      const size_t ui = static_cast<size_t>(i);
      msg.position[ui] = q(i);
      msg.velocity[ui] = dq(i);
      msg.position_reference[ui] = q_d(i);
      msg.velocity_reference[ui] = dq_d(i);
      msg.sliding_surface[ui] = s_vec(i);
      msg.effort_feedforward[ui] = tau_model(i);  // Y·θ̂ (model torque)
      msg.effort_command[ui] = tau(i);
      if (any_effort && effort_state_[ui]) {
        auto te = effort_state_[ui]->get_optional();
        msg.effort_measured[ui] = te ? *te : 0.0;
      }
    }
    telemetry_pub_->publish(msg);
  }

  return controller_interface::return_type::OK;
}

}  // namespace adaptive_slotine_controller

PLUGINLIB_EXPORT_CLASS(
  adaptive_slotine_controller::AdaptiveSlotineController, controller_interface::ControllerInterface)
