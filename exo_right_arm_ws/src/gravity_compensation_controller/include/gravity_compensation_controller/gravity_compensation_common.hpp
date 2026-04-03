#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <vector>

namespace gravity_compensation_controller
{

inline Eigen::VectorXd computeGravityPdEffort(
  double gravity_scale,
  double kp,
  double kd,
  const std::vector<double> & tau_g,
  const std::vector<double> & q_des,
  const std::vector<double> & dq_des,
  const std::vector<double> & q_meas,
  const std::vector<double> & dq_meas)
{
  const size_t n = tau_g.size();
  Eigen::VectorXd tau(static_cast<Eigen::Index>(n));
  for (size_t i = 0; i < n; ++i) {
    const double tau_ff = gravity_scale * tau_g[i];
    tau(static_cast<Eigen::Index>(i)) =
      tau_ff + kp * (q_des[i] - q_meas[i]) + kd * (dq_des[i] - dq_meas[i]);
  }
  return tau;
}

}  // namespace gravity_compensation_controller
