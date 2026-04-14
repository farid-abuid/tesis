#pragma once

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

  /// \param joint_names Controller joint order (must exist in the URDF model).
  /// \param q Measured joint positions (same order and size as joint_names).
  /// \param tau_out Generalized gravity torque for each joint (same order).
  virtual bool computeGravityTorque(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & q,
    std::vector<double> & tau_out,
    std::string * error) = 0;
};

/// \param backend "pinocchio" or "rbdl" (RBDL must have been found at build time).
std::unique_ptr<DynamicsModel> createDynamicsModel(
  const std::string & backend,
  std::string * error);

}  // namespace dynamics
}  // namespace exo_utils
