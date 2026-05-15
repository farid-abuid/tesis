#include "exo_utils/kinematics/kinematics_utils.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>

namespace exo_utils
{
namespace kinematics
{

// ============================================================================
//  DH parameters — substitute your values here.
// ============================================================================
//
// Convention: standard (proximal) Denavit–Hartenberg. Per joint i,
//
//     T_i = Rz(q_i + theta_offset_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
//
// The end-effector pose is then
//
//     T_ee = T_base * T_1 * T_2 * T_3 * T_tool
//
// expressed in the arm's `<arm>_base_link` frame.
//
// `kBaseOffset_*` is the rigid offset from `<arm>_base_link` (the URDF link the
// arm is mounted on) to DH frame 0. `kToolOffset_*` is an optional rigid
// offset appended after the last DH joint (e.g., end-effector tip).
//
// Both offsets are given as XYZ + extrinsic RPY (roll → pitch → yaw, applied as
// R = Rz(yaw) * Ry(pitch) * Rx(roll) — same convention as URDF <origin rpy>).
//
// ----------------------------------------------------------------------------

struct RigidOffset
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

// ---- RIGHT ARM -------------------------------------------------------------
// right_revolute_1 origin in right_base_link (URDF: exo_arm_right.xacro)
constexpr RigidOffset kBaseOffset_right{
  /* x     */ -0.094353,
  /* y     */ -0.00061,
  /* z     */ -0.063136,
  /* roll  */ 1.5707963,
  /* pitch */ 0.0,
  /* yaw   */ -1.5707963,
};

constexpr std::array<DhRow, 3> kDh_right = {{
  // { a [m], alpha [rad], d [m], theta_offset [rad] }
  /* joint 1 */ {0.0, 1.5708, -0.1027, 1.5708},
  /* joint 2 */ {0.2833, 0.0, 0.0, 3.1416},
  /* joint 3 */ {0.1674, 0.0, 0.1139, 0.0},
}};

constexpr RigidOffset kToolOffset_right{
  /* x     */ 0.0,
  /* y     */ 0.0,
  /* z     */ 0.0,
  /* roll  */ 0.0,
  /* pitch */ 0.0,
  /* yaw   */ 0.0,
};

// ---- LEFT ARM --------------------------------------------------------------
// left_revolute_1 origin in left_base_link (URDF: exo_arm_left.xacro)
// Mirror of the right-arm DH: kBaseOffset_left rpy flips roll & yaw sign, and
// kDh_left flips alpha & theta_offset signs (joint axes are y-mirrored).
constexpr RigidOffset kBaseOffset_left{
  /* x     */ -0.094353,
  /* y     */ 0.00061,
  /* z     */ -0.063136,
  /* roll  */ 1.5707963,
  /* pitch */ 0.0,
  /* yaw   */ 1.5707963,
};

constexpr std::array<DhRow, 3> kDh_left = {{
  // { a [m], alpha [rad], d [m], theta_offset [rad] }
  /* joint 1 */ {0.0, 1.5708, 0.1027, -1.5708},
  /* joint 2 */ {0.2833, 0.0, 0.0, 0},
  /* joint 3 */ {0.1674, 0.0, -0.1139, 0.0},
}};

constexpr RigidOffset kToolOffset_left{
  /* x     */ 0.0,
  /* y     */ 0.0,
  /* z     */ 0.0,
  /* roll  */ 0.0,
  /* pitch */ 0.0,
  /* yaw   */ 0.0,
};

// ============================================================================
//  Implementation
// ============================================================================

namespace
{

Eigen::Isometry3d toIsometry(const RigidOffset & o)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() << o.x, o.y, o.z;
  T.linear() = (Eigen::AngleAxisd(o.yaw,   Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(o.pitch, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(o.roll,  Eigen::Vector3d::UnitX())).toRotationMatrix();
  return T;
}

Eigen::Isometry3d dhTransform(const DhRow & row, double q)
{
  const double theta = q + row.theta_offset;
  const double ct = std::cos(theta);
  const double st = std::sin(theta);
  const double ca = std::cos(row.alpha);
  const double sa = std::sin(row.alpha);
  Eigen::Matrix4d M;
  M << ct, -st * ca,  st * sa, row.a * ct,
       st,  ct * ca, -ct * sa, row.a * st,
       0.0, sa,       ca,      row.d,
       0.0, 0.0,      0.0,     1.0;
  Eigen::Isometry3d T;
  T.matrix() = M;
  return T;
}

bool startsWith(const std::string & s, const std::string & p)
{
  return s.size() >= p.size() && std::equal(p.begin(), p.end(), s.begin());
}

bool selectArm(const std::string & end_effector_frame,
               const std::array<DhRow, 3> ** dh,
               const RigidOffset ** base,
               const RigidOffset ** tool,
               std::string * arm_prefix)
{
  if (startsWith(end_effector_frame, "right")) {
    *dh = &kDh_right;
    *base = &kBaseOffset_right;
    *tool = &kToolOffset_right;
    *arm_prefix = "right_";
    return true;
  }
  if (startsWith(end_effector_frame, "left")) {
    *dh = &kDh_left;
    *base = &kBaseOffset_left;
    *tool = &kToolOffset_left;
    *arm_prefix = "left_";
    return true;
  }
  return false;
}

}  // namespace

bool forwardKinematicsChain(
  const std::string & end_effector_frame,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & q,
  std::vector<Eigen::Isometry3d> & segments_out,
  std::string * error)
{
  const std::array<DhRow, 3> * dh = nullptr;
  const RigidOffset * base = nullptr;
  const RigidOffset * tool = nullptr;
  std::string prefix;
  if (!selectArm(end_effector_frame, &dh, &base, &tool, &prefix)) {
    if (error) {
      *error = "unknown end_effector_frame '" + end_effector_frame +
               "' (expected to start with 'right' or 'left')";
    }
    return false;
  }

  std::array<double, 3> q_dh{0.0, 0.0, 0.0};
  if (joint_names.empty()) {
    if (q.size() != dh->size()) {
      if (error) {
        *error = "q has " + std::to_string(q.size()) +
                 " entries, expected " + std::to_string(dh->size());
      }
      return false;
    }
    for (size_t i = 0; i < dh->size(); ++i) {
      q_dh[i] = q[i];
    }
  } else {
    if (q.size() != joint_names.size()) {
      if (error) {
        *error = "q.size() (" + std::to_string(q.size()) +
                 ") != joint_names.size() (" + std::to_string(joint_names.size()) + ")";
      }
      return false;
    }
    for (size_t i = 0; i < dh->size(); ++i) {
      const std::string target = prefix + "revolute_" + std::to_string(i + 1);
      auto it = std::find(joint_names.begin(), joint_names.end(), target);
      if (it == joint_names.end()) {
        if (error) {
          *error = "joint_names is missing '" + target + "'";
        }
        return false;
      }
      q_dh[i] = q[static_cast<size_t>(std::distance(joint_names.begin(), it))];
    }
  }

  segments_out.clear();
  segments_out.reserve(dh->size() + 2);
  segments_out.push_back(toIsometry(*base));
  for (size_t i = 0; i < dh->size(); ++i) {
    segments_out.push_back(dhTransform((*dh)[i], q_dh[i]));
  }
  segments_out.push_back(toIsometry(*tool));
  return true;
}

bool forwardKinematicsEndEffector(
  const std::string & end_effector_frame,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & q,
  Eigen::Vector3d & translation_out,
  std::string * error)
{
  std::vector<Eigen::Isometry3d> segments;
  if (!forwardKinematicsChain(end_effector_frame, joint_names, q, segments, error)) {
    return false;
  }
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  for (const auto & s : segments) {
    T = T * s;
  }
  translation_out = T.translation();
  return true;
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

}  // namespace kinematics
}  // namespace exo_utils
