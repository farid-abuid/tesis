#include "gravity_compensation_controller/controller_kinematics_utils.hpp"

namespace gravity_compensation_controller
{

bool forwardKinematicsEndEffector(
  const std::string & /* end_effector_frame */,
  const std::vector<std::string> & /* joint_names */,
  const std::vector<double> & /* q */,
  Eigen::Vector3d & /* translation_out */,
  std::string * error)
{
  if (error) {
    *error = "forwardKinematicsEndEffector is not implemented yet";
  }
  return false;
}

bool inverseKinematicsEndEffector(
  const std::string & /* end_effector_frame */,
  const std::vector<std::string> & /* joint_names */,
  const Eigen::Vector3d & /* target_translation */,
  const std::vector<double> & /* q_seed */,
  std::vector<double> & /* q_out */,
  std::string * error)
{
  if (error) {
    *error = "inverseKinematicsEndEffector is not implemented yet";
  }
  return false;
}

}  // namespace gravity_compensation_controller
