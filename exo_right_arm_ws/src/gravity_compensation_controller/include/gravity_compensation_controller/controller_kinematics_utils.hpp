#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

namespace gravity_compensation_controller
{

/// Placeholder forward kinematics (implement with Pinocchio / analytic IK later).
bool forwardKinematicsEndEffector(
  const std::string & end_effector_frame,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & q,
  Eigen::Vector3d & translation_out,
  std::string * error = nullptr);

/// Placeholder inverse kinematics (implement later).
bool inverseKinematicsEndEffector(
  const std::string & end_effector_frame,
  const std::vector<std::string> & joint_names,
  const Eigen::Vector3d & target_translation,
  const std::vector<double> & q_seed,
  std::vector<double> & q_out,
  std::string * error = nullptr);

}  // namespace gravity_compensation_controller
