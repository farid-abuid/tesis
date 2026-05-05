#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

namespace exo_utils
{
namespace dynamics
{

/// Rigid-body dynamics (Pinocchio / optional RBDL): load URDF and map joint order to gravity torques.
class DynamicsModel
{
public:
  virtual ~DynamicsModel() = default;

  virtual bool loadUrdf(const std::string & urdf_path, std::string * error) = 0;

  /// Update the gravity vector used by the dynamics model (expressed in the model's base frame).
  /// Call before computeGravityTorque() each cycle when the base orientation is known from an IMU.
  virtual bool setGravity(const Eigen::Vector3d & g, std::string * error) = 0;

  /// \param joint_names Controller joint order (must exist in the URDF model).
  /// \param q Joint positions (same order and size as joint_names).
  /// \param tau_out Generalized gravity torque for each joint (same order).
  virtual bool computeGravityTorque(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & q,
    std::vector<double> & tau_out,
    std::string * error) = 0;

  /// \param joint_names Controller joint order (must exist in the URDF model).
  /// \param q Joint positions (same order and size as joint_names).
  /// \param M_out n×n joint-space inertia (mass) matrix M(q).
  virtual bool computeMassMatrix(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & q,
    Eigen::MatrixXd & M_out,
    std::string * error) = 0;

  /// \param joint_names Controller joint order (must exist in the URDF model).
  /// \param q Joint positions.
  /// \param dq Joint velocities (same order and size as joint_names).
  /// \param C_out n×n Coriolis/centripetal matrix C(q, dq) such that C*dq gives Coriolis torques.
  virtual bool computeCoriolisMatrix(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & q,
    const std::vector<double> & dq,
    Eigen::MatrixXd & C_out,
    std::string * error) = 0;
};

/// \param backend "pinocchio" or "rbdl" (RBDL must have been found at build time).
std::unique_ptr<DynamicsModel> createDynamicsModel(
  const std::string & backend,
  std::string * error);

}  // namespace dynamics
}  // namespace exo_utils
