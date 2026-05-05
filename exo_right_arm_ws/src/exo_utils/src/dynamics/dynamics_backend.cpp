#include "exo_utils/dynamics/dynamics_backend.hpp"

#include <algorithm>
#include <cctype>
#include <limits>
#include <unordered_map>

#ifdef EXO_UTILS_HAVE_PINOCCHIO
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#endif

#ifdef EXO_UTILS_HAVE_RBDL
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/rbdl.h>
#include <urdfdom/urdf_parser/urdf_parser.h>
#endif

namespace exo_utils
{
namespace dynamics
{

#ifdef EXO_UTILS_HAVE_PINOCCHIO

class PinocchioDynamics final : public DynamicsModel
{
public:
  bool loadUrdf(const std::string & urdf_path, std::string * error) override
  {
    try {
      pinocchio::urdf::buildModel(urdf_path, model_, false, false);
      data_ = std::make_unique<pinocchio::Data>(model_);
    } catch (const std::exception & e) {
      if (error) {
        *error = std::string("Pinocchio URDF load failed: ") + e.what();
      }
      return false;
    }
    return true;
  }

  bool setGravity(const Eigen::Vector3d & g, std::string * error) override
  {
    if (!data_) {
      if (error) *error = "Pinocchio model not loaded";
      return false;
    }
    model_.gravity.linear() = g;
    return true;
  }

  bool computeGravityTorque(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & q,
    std::vector<double> & tau_out,
    std::string * error) override
  {
    if (!data_) {
      if (error) *error = "Pinocchio model not loaded";
      return false;
    }
    Eigen::VectorXd q_full, v_full;
    std::vector<int> iv;
    if (!buildJointState_(joint_names, q, nullptr, q_full, v_full, iv, error)) return false;

    Eigen::VectorXd a = Eigen::VectorXd::Zero(model_.nv);
    pinocchio::rnea(model_, *data_, q_full, v_full, a);

    tau_out.resize(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i) {
      tau_out[i] = data_->tau[iv[i]];
    }
    return true;
  }

  bool computeMassMatrix(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & q,
    Eigen::MatrixXd & M_out,
    std::string * error) override
  {
    if (!data_) {
      if (error) *error = "Pinocchio model not loaded";
      return false;
    }
    Eigen::VectorXd q_full, v_full;
    std::vector<int> iv;
    if (!buildJointState_(joint_names, q, nullptr, q_full, v_full, iv, error)) return false;

    pinocchio::crba(model_, *data_, q_full);
    // crba fills only the upper triangle; symmetrize before indexing
    data_->M.triangularView<Eigen::StrictlyLower>() =
      data_->M.transpose().triangularView<Eigen::StrictlyLower>();

    const int n = static_cast<int>(joint_names.size());
    M_out.resize(n, n);
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
        M_out(i, j) = data_->M(iv[i], iv[j]);
      }
    }
    return true;
  }

  bool computeCoriolisMatrix(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & q,
    const std::vector<double> & dq,
    Eigen::MatrixXd & C_out,
    std::string * error) override
  {
    if (!data_) {
      if (error) *error = "Pinocchio model not loaded";
      return false;
    }
    Eigen::VectorXd q_full, v_full;
    std::vector<int> iv;
    if (!buildJointState_(joint_names, q, &dq, q_full, v_full, iv, error)) return false;

    // computeCoriolisMatrix is the dedicated algorithm guaranteed to populate data_->C
    pinocchio::computeCoriolisMatrix(model_, *data_, q_full, v_full);

    const int n = static_cast<int>(joint_names.size());
    C_out.resize(n, n);
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
        C_out(i, j) = data_->C(iv[i], iv[j]);
      }
    }
    return true;
  }

private:
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;

  bool buildJointState_(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & q,
    const std::vector<double> * dq,
    Eigen::VectorXd & q_full,
    Eigen::VectorXd & v_full,
    std::vector<int> & iv_indices,
    std::string * error)
  {
    if (joint_names.size() != q.size()) {
      if (error) *error = "joint_names and q size mismatch";
      return false;
    }
    if (dq && dq->size() != joint_names.size()) {
      if (error) *error = "joint_names and dq size mismatch";
      return false;
    }
    q_full = pinocchio::neutral(model_);
    v_full = Eigen::VectorXd::Zero(model_.nv);
    iv_indices.resize(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i) {
      if (!model_.existJointName(joint_names[i])) {
        if (error) *error = "Unknown joint in Pinocchio model: " + joint_names[i];
        return false;
      }
      const pinocchio::JointIndex jid = model_.getJointId(joint_names[i]);
      if (model_.nqs[jid] != 1 || model_.nvs[jid] != 1) {
        if (error) *error = "Joint " + joint_names[i] + " must be 1-DoF";
        return false;
      }
      q_full[model_.idx_qs[jid]] = q[i];
      iv_indices[i] = model_.idx_vs[jid];
      if (dq) v_full[iv_indices[i]] = (*dq)[i];
    }
    return true;
  }
};

#endif  // EXO_UTILS_HAVE_PINOCCHIO

#ifdef EXO_UTILS_HAVE_RBDL

class RbdlDynamics final : public DynamicsModel
{
public:
  bool loadUrdf(const std::string & urdf_path, std::string * error) override
  {
    model_ = std::make_unique<RigidBodyDynamics::Model>();
    if (!RigidBodyDynamics::Addons::URDFReadFromFile(
        urdf_path.c_str(), model_.get(), false, false))
    {
      if (error) *error = "RBDL URDFReadFromFile failed";
      return false;
    }

    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile(urdf_path);
    if (!urdf_model) {
      if (error) *error = "urdfdom failed to parse URDF for joint name mapping";
      return false;
    }

    joint_q_start_.clear();
    for (const auto & jp : urdf_model->joints_) {
      const std::string & jname = jp.first;
      const urdf::JointSharedPtr & joint = jp.second;
      if (!joint || joint->type == urdf::Joint::FIXED) continue;
      const unsigned int bid = model_->GetBodyId(joint->child_link_name.c_str());
      if (bid == std::numeric_limits<unsigned int>::max() || bid >= model_->mJoints.size()) {
        continue;
      }
      joint_q_start_[jname] = model_->mJoints[bid].q_index;
    }
    return true;
  }

  bool setGravity(const Eigen::Vector3d & g, std::string * error) override
  {
    if (!model_) {
      if (error) *error = "RBDL model not loaded";
      return false;
    }
    model_->gravity = RigidBodyDynamics::Math::Vector3d(g.x(), g.y(), g.z());
    return true;
  }

  bool computeGravityTorque(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & q,
    std::vector<double> & tau_out,
    std::string * error) override
  {
    if (!model_) {
      if (error) *error = "RBDL model not loaded";
      return false;
    }
    RigidBodyDynamics::Math::VectorNd Q, QDot;
    std::vector<unsigned int> iq;
    if (!buildJointState_(joint_names, q, nullptr, Q, QDot, iq, error)) return false;

    RigidBodyDynamics::Math::VectorNd QDDot =
      RigidBodyDynamics::Math::VectorNd::Zero(model_->qdot_size);
    RigidBodyDynamics::Math::VectorNd Tau =
      RigidBodyDynamics::Math::VectorNd::Zero(model_->qdot_size);
    RigidBodyDynamics::InverseDynamics(*model_, Q, QDot, QDDot, Tau);

    tau_out.resize(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i) {
      tau_out[i] = Tau[iq[i]];
    }
    return true;
  }

  bool computeMassMatrix(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & q,
    Eigen::MatrixXd & M_out,
    std::string * error) override
  {
    if (!model_) {
      if (error) *error = "RBDL model not loaded";
      return false;
    }
    RigidBodyDynamics::Math::VectorNd Q, QDot;
    std::vector<unsigned int> iq;
    if (!buildJointState_(joint_names, q, nullptr, Q, QDot, iq, error)) return false;

    RigidBodyDynamics::Math::MatrixNd H =
      RigidBodyDynamics::Math::MatrixNd::Zero(model_->qdot_size, model_->qdot_size);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(*model_, Q, H);

    const int n = static_cast<int>(joint_names.size());
    M_out.resize(n, n);
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
        M_out(i, j) = H(iq[i], iq[j]);
      }
    }
    return true;
  }

  bool computeCoriolisMatrix(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & q,
    const std::vector<double> & dq,
    Eigen::MatrixXd & C_out,
    std::string * error) override
  {
    if (!model_) {
      if (error) *error = "RBDL model not loaded";
      return false;
    }
    RigidBodyDynamics::Math::VectorNd Q, QDot;
    std::vector<unsigned int> iq;
    if (!buildJointState_(joint_names, q, &dq, Q, QDot, iq, error)) return false;

    // g(q) = NonlinearEffects(Q, 0)
    RigidBodyDynamics::Math::VectorNd zero_qdot =
      RigidBodyDynamics::Math::VectorNd::Zero(model_->qdot_size);
    RigidBodyDynamics::Math::VectorNd g_vec =
      RigidBodyDynamics::Math::VectorNd::Zero(model_->qdot_size);
    RigidBodyDynamics::NonlinearEffects(*model_, Q, zero_qdot, g_vec);

    // Column j of C: NonlinearEffects(Q, e_j) - g(q)  where e_j is the j-th unit vector
    const int n = static_cast<int>(joint_names.size());
    C_out.resize(n, n);
    RigidBodyDynamics::Math::VectorNd e_j =
      RigidBodyDynamics::Math::VectorNd::Zero(model_->qdot_size);
    RigidBodyDynamics::Math::VectorNd ne =
      RigidBodyDynamics::Math::VectorNd::Zero(model_->qdot_size);
    for (int j = 0; j < n; ++j) {
      e_j.setZero();
      e_j[iq[j]] = 1.0;
      ne.setZero();
      RigidBodyDynamics::NonlinearEffects(*model_, Q, e_j, ne);
      for (int i = 0; i < n; ++i) {
        C_out(i, j) = ne[iq[i]] - g_vec[iq[i]];
      }
    }
    return true;
  }

private:
  std::unique_ptr<RigidBodyDynamics::Model> model_;
  std::unordered_map<std::string, unsigned int> joint_q_start_;

  bool buildJointState_(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & q,
    const std::vector<double> * dq,
    RigidBodyDynamics::Math::VectorNd & Q,
    RigidBodyDynamics::Math::VectorNd & QDot,
    std::vector<unsigned int> & iq_indices,
    std::string * error)
  {
    if (joint_names.size() != q.size()) {
      if (error) *error = "joint_names and q size mismatch";
      return false;
    }
    if (dq && dq->size() != joint_names.size()) {
      if (error) *error = "joint_names and dq size mismatch";
      return false;
    }
    Q = RigidBodyDynamics::Math::VectorNd::Zero(model_->q_size);
    QDot = RigidBodyDynamics::Math::VectorNd::Zero(model_->qdot_size);
    iq_indices.resize(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i) {
      const auto it = joint_q_start_.find(joint_names[i]);
      if (it == joint_q_start_.end()) {
        if (error) *error = "Unknown joint for RBDL model: " + joint_names[i];
        return false;
      }
      iq_indices[i] = it->second;
      Q[it->second] = q[i];
      if (dq) QDot[it->second] = (*dq)[i];
    }
    return true;
  }
};

#endif  // EXO_UTILS_HAVE_RBDL

std::unique_ptr<DynamicsModel> createDynamicsModel(
  const std::string & backend,
  std::string * error)
{
  std::string b = backend;
  std::transform(b.begin(), b.end(), b.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  if (b == "pinocchio" || b == "pin") {
#ifdef EXO_UTILS_HAVE_PINOCCHIO
    return std::make_unique<PinocchioDynamics>();
#else
    if (error) {
      *error = "Pinocchio backend was not enabled at compile time.";
    }
    return nullptr;
#endif
  }

  if (b == "rbdl") {
#ifdef EXO_UTILS_HAVE_RBDL
    return std::make_unique<RbdlDynamics>();
#else
    if (error) {
      *error = "RBDL backend was not enabled at compile time (library not found).";
    }
    return nullptr;
#endif
  }

  if (error) {
    *error = "Unknown dynamics backend '" + backend + "' (use 'pinocchio' or 'rbdl').";
  }
  return nullptr;
}

}  // namespace dynamics
}  // namespace exo_utils
