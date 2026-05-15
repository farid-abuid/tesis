#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace exo_utils
{
namespace kinematics
{

/// One row of a standard (proximal) Denavit–Hartenberg table.
///
/// The associated link transform for joint i is:
///     T_i = Rz(theta + theta_offset) * Tz(d) * Tx(a) * Rx(alpha)
struct DhRow
{
  double a;             // link length [m]      (translation along X_i)
  double alpha;         // link twist  [rad]    (rotation    around X_i)
  double d;             // link offset [m]      (translation along Z_{i-1})
  double theta_offset;  // joint angle offset [rad] (added to q_i around Z_{i-1})
};

/// Compute the end-effector translation (in the arm's base_link frame) from the
/// joint positions `q`, using the DH table baked in for the requested arm.
///
/// `end_effector_frame` selects which DH table to use; any string starting with
/// "right" maps to the right arm, "left" to the left arm.
///
/// `joint_names` is optional. If empty, `q` is assumed to be in DH order
/// (q[0] -> joint 1, ...). If non-empty, it's used to look up
/// `<arm>_revolute_<i>` so callers can pass a JointState slice in any order.
bool forwardKinematicsEndEffector(
  const std::string & end_effector_frame,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & q,
  Eigen::Vector3d & translation_out,
  std::string * error = nullptr);

/// Same inputs as forwardKinematicsEndEffector but returns the full chain of
/// per-segment transforms (parent_T_child), in order:
///   segments_out[0]      : <arm>_base_link -> DH frame 0   (base offset)
///   segments_out[1..n]   : DH frame i-1   -> DH frame i    (per-joint DH transform)
///   segments_out[n+1]    : DH frame n     -> end-effector  (tool offset)
/// So segments_out.size() == n_joints + 2 (currently 5 for the 3-DOF arm).
bool forwardKinematicsChain(
  const std::string & end_effector_frame,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & q,
  std::vector<Eigen::Isometry3d> & segments_out,
  std::string * error = nullptr);

/// Placeholder inverse kinematics (implement later).
bool inverseKinematicsEndEffector(
  const std::string & end_effector_frame,
  const std::vector<std::string> & joint_names,
  const Eigen::Vector3d & target_translation,
  const std::vector<double> & q_seed,
  std::vector<double> & q_out,
  std::string * error = nullptr);

}  // namespace kinematics
}  // namespace exo_utils
