#include "gravity_compensation_controller/dynamics_backend.hpp"

#include <algorithm>
#include <cctype>
#include <limits>
#include <unordered_map>

#ifdef GRAVITY_COMP_HAVE_PINOCCHIO
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#endif

#ifdef GRAVITY_COMP_HAVE_RBDL
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/rbdl.h>
#include <urdfdom/urdf_parser/urdf_parser.h>
#endif

namespace gravity_compensation_controller
{

#ifdef GRAVITY_COMP_HAVE_PINOCCHIO

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

  bool computeGravityTorque(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & q,
    std::vector<double> & tau_out,
    std::string * error) override
  {
    if (!data_) {
      if (error) {
        *error = "Pinocchio model not loaded";
      }
      return false;
    }
    if (joint_names.size() != q.size()) {
      if (error) {
        *error = "joint_names and q size mismatch";
      }
      return false;
    }

    Eigen::VectorXd q_full = pinocchio::neutral(model_);
    for (size_t i = 0; i < joint_names.size(); ++i) {
      if (!model_.existJointName(joint_names[i])) {
        if (error) {
          *error = "Unknown joint in Pinocchio model: " + joint_names[i];
        }
        return false;
      }
      const pinocchio::JointIndex jid = model_.getJointId(joint_names[i]);
      const int iq = model_.idx_qs[jid];
      const int nqj = model_.nqs[jid];
      const int nvj = model_.nvs[jid];
      if (nqj != 1 || nvj != 1) {
        if (error) {
          *error = "Joint " + joint_names[i] + " must be 1-DoF for this controller";
        }
        return false;
      }
      q_full[iq] = q[i];
    }

    Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model_.nv);
    pinocchio::rnea(model_, *data_, q_full, v, a);

    tau_out.resize(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i) {
      const pinocchio::JointIndex jid = model_.getJointId(joint_names[i]);
      const int iv = model_.idx_vs[jid];
      tau_out[i] = data_->tau[iv];
    }
    return true;
  }

private:
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
};

#endif  // GRAVITY_COMP_HAVE_PINOCCHIO

#ifdef GRAVITY_COMP_HAVE_RBDL

class RbdlDynamics final : public DynamicsModel
{
public:
  bool loadUrdf(const std::string & urdf_path, std::string * error) override
  {
    model_ = std::make_unique<RigidBodyDynamics::Model>();
    if (!RigidBodyDynamics::Addons::URDFReadFromFile(
        urdf_path.c_str(), model_.get(), false, false))
    {
      if (error) {
        *error = "RBDL URDFReadFromFile failed";
      }
      return false;
    }

    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile(urdf_path);
    if (!urdf_model) {
      if (error) {
        *error = "urdfdom failed to parse URDF for joint name mapping";
      }
      return false;
    }

    joint_q_start_.clear();
    for (const auto & jp : urdf_model->joints_) {
      const std::string & jname = jp.first;
      const urdf::JointSharedPtr & joint = jp.second;
      if (!joint || joint->type == urdf::Joint::FIXED) {
        continue;
      }
      const unsigned int bid = model_->GetBodyId(joint->child_link_name.c_str());
      if (bid == std::numeric_limits<unsigned int>::max() || bid >= model_->mJoints.size()) {
        continue;
      }
      joint_q_start_[jname] = model_->mJoints[bid].q_index;
    }
    return true;
  }

  bool computeGravityTorque(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & q,
    std::vector<double> & tau_out,
    std::string * error) override
  {
    if (!model_) {
      if (error) {
        *error = "RBDL model not loaded";
      }
      return false;
    }
    if (joint_names.size() != q.size()) {
      if (error) {
        *error = "joint_names and q size mismatch";
      }
      return false;
    }

    RigidBodyDynamics::Math::VectorNd Q =
      RigidBodyDynamics::Math::VectorNd::Zero(model_->q_size);
    RigidBodyDynamics::Math::VectorNd QDot =
      RigidBodyDynamics::Math::VectorNd::Zero(model_->qdot_size);
    RigidBodyDynamics::Math::VectorNd QDDot =
      RigidBodyDynamics::Math::VectorNd::Zero(model_->qdot_size);
    RigidBodyDynamics::Math::VectorNd Tau =
      RigidBodyDynamics::Math::VectorNd::Zero(model_->qdot_size);

    for (size_t i = 0; i < joint_names.size(); ++i) {
      const auto it = joint_q_start_.find(joint_names[i]);
      if (it == joint_q_start_.end()) {
        if (error) {
          *error = "Unknown joint for RBDL model: " + joint_names[i];
        }
        return false;
      }
      Q[it->second] = q[i];
    }

    RigidBodyDynamics::InverseDynamics(*model_, Q, QDot, QDDot, Tau);

    tau_out.resize(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i) {
      const unsigned int iq = joint_q_start_.at(joint_names[i]);
      tau_out[i] = Tau[iq];
    }
    return true;
  }

private:
  std::unique_ptr<RigidBodyDynamics::Model> model_;
  std::unordered_map<std::string, unsigned int> joint_q_start_;
};

#endif  // GRAVITY_COMP_HAVE_RBDL

std::unique_ptr<DynamicsModel> createDynamicsModel(
  const std::string & backend,
  std::string * error)
{
  std::string b = backend;
  std::transform(b.begin(), b.end(), b.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  if (b == "pinocchio" || b == "pin") {
#ifdef GRAVITY_COMP_HAVE_PINOCCHIO
    return std::make_unique<PinocchioDynamics>();
#else
    if (error) {
      *error = "Pinocchio backend was not enabled at compile time.";
    }
    return nullptr;
#endif
  }

  if (b == "rbdl") {
#ifdef GRAVITY_COMP_HAVE_RBDL
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

}  // namespace gravity_compensation_controller
