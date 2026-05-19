#pragma once

#include <Eigen/Core>

#include <string>

// Slotine-Li adaptive-control regressor for the 3-DOF exoskeleton arms.
//
// The dynamics  tau = M(q) qddr + C(q,qd) qdr + g(q)  are exactly linear in a
// 10-parameter barycentric set per link (30 total for n = 3):
//
//   theta_i = [ m_i; mx_i; my_i; mz_i; Jxx_i; Jyy_i; Jzz_i; Jxy_i; Jxz_i; Jyz_i ]
//     mx,my,mz = first moments of mass (= m * c)
//     Jxx..Jyz = second moments of mass ABOUT THE LINK-FRAME ORIGIN
//
// so that  tau = Y(q, qd, qdr, qddr, g) * theta.
//
// Y is arm-specific because each arm has its own DH parameters and base
// transform (baked numerically into the symbolic export from
// euler_lagrange_sym.m / build_regressor.m). exo_right_Y is generated from the
// right-arm robot_Y.m; exo_left_Y must be generated the same way from the
// left-arm DH set (see exoLeftYImplemented()).

namespace exo_utils
{
namespace dynamics
{

/// Regressor matrix: 3 rows (joints) x 30 columns (10 barycentric params x 3 links).
using RegressorMatrix = Eigen::Matrix<double, 3, 30>;
/// Barycentric parameter vector: 30 entries (10 per link, links 1..3).
using BarycentricVector = Eigen::Matrix<double, 30, 1>;

/// Right-arm Slotine-Li regressor Y(q, qd, qd_r, qdd_r, g).
/// Ported verbatim from MATLAB robot_Y.m (right-arm DH + base transform).
RegressorMatrix exo_right_Y(
  const Eigen::Vector3d & q,
  const Eigen::Vector3d & qd,
  const Eigen::Vector3d & qd_r,
  const Eigen::Vector3d & qdd_r,
  double g);

/// Left-arm Slotine-Li regressor Y(q, qd, qd_r, qdd_r, g).
/// Placeholder until the left-arm robot_Y.m is generated and pasted in
/// (see regressor.cpp). Returns zeros while unimplemented.
RegressorMatrix exo_left_Y(
  const Eigen::Vector3d & q,
  const Eigen::Vector3d & qd,
  const Eigen::Vector3d & qd_r,
  const Eigen::Vector3d & qdd_r,
  double g);

/// True once a real left-arm regressor body has been pasted into exo_left_Y.
bool exoLeftYImplemented();

/// Dispatch by arm name ("right" / "left").
RegressorMatrix exo_Y(
  const std::string & arm,
  const Eigen::Vector3d & q,
  const Eigen::Vector3d & qd,
  const Eigen::Vector3d & qd_r,
  const Eigen::Vector3d & qdd_r,
  double g);

/// Convert physical inertial parameters (mass, COM in the link frame, inertia
/// tensor about the COM in frame-i axes, diagonal + products) to the
/// 30-element barycentric vector expected by exo_*_Y. Mirrors the MATLAB
/// physical_to_barycentric() used by sim_adaptive_slotine_li.m.
///
/// Each argument is a 3-vector (one entry per link). Inertia arguments use the
/// usual moments/products of inertia convention (I_xy = -int xy dm).
BarycentricVector physicalToBarycentric(
  const Eigen::Vector3d & m,
  const Eigen::Vector3d & xc,
  const Eigen::Vector3d & yc,
  const Eigen::Vector3d & zc,
  const Eigen::Vector3d & Ixx_c,
  const Eigen::Vector3d & Iyy_c,
  const Eigen::Vector3d & Izz_c,
  const Eigen::Vector3d & Ixy_c,
  const Eigen::Vector3d & Ixz_c,
  const Eigen::Vector3d & Iyz_c);

}  // namespace dynamics
}  // namespace exo_utils
