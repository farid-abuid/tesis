// exo_regressor_check
// ---------------------------------------------------------------------------
// Consistency test for the Slotine-Li barycentric regressor exo_*_Y against
// the rigid-body model Pinocchio builds from the same dynamics URDF that the
// Gazebo plant is generated from.
//
// The linear-in-parameters identity (with qd_r = q̇, qdd_r = q̈) is
//
//     Y(q, q̇, q̇, q̈) · θ   ==   M(q)·q̈ + C(q,q̇)·q̇ + g(q)
//
// For N random samples we stack  A = [Y_1; …; Y_N],  b = [τ_1; …; τ_N]  (τ from
// Pinocchio) and solve the least-squares θ* = A⁺b. If Y structurally spans the
// URDF dynamics the relative residual ‖Aθ*−b‖/‖b‖ is ~machine-zero and θ* is a
// consistent seed for the controller; a large residual (especially localized
// to specific joint rows) means the DH model behind Y disagrees with the URDF.
//
// Usage:
//   ros2 run exo_utils exo_regressor_check [--arm right|left]
//       [--urdf PATH] [--samples N] [--gravity G] [--seed S]
//       [--qmax R] [--qdmax R] [--qddmax R]
// ---------------------------------------------------------------------------

#include "exo_utils/dynamics/dynamics_backend.hpp"
#include "exo_utils/dynamics/regressor.hpp"
#include "exo_utils/kinematics/kinematics_utils.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>
#include <string>
#include <vector>

namespace
{

struct Args
{
  std::string arm = "right";
  std::string urdf;
  int samples = 500;
  double gravity = 9.81;
  unsigned seed = 1;
  double qmax = M_PI;
  double qdmax = 3.0;
  double qddmax = 5.0;
  int sweep = 0;        // 1|2|3: dump gravity g_pino vs Y·θ* sweeping that joint
  int sweep_vel = 0;    // 1|2|3: dump Coriolis (g off) vs Y·θ* sweeping that q̇
  int sweep_steps = 25;
  bool no_gravity = false;  // isolate inertial+Coriolis (drop g from ref and Y)
  bool seed_from_urdf = false;  // emit a physical seed from the URDF inertials
  std::string urdf_base;        // URDF link the DH base offset is referenced to
};

bool parseArgs(int argc, char ** argv, Args & a, std::string * error)
{
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    auto next = [&](double & out) {
        if (i + 1 >= argc) {return false;}
        out = std::atof(argv[++i]);
        return true;
      };
    if (k == "--arm" && i + 1 < argc) {
      a.arm = argv[++i];
    } else if (k == "--urdf" && i + 1 < argc) {
      a.urdf = argv[++i];
    } else if (k == "--samples" && i + 1 < argc) {
      a.samples = std::atoi(argv[++i]);
    } else if (k == "--seed-from-urdf") {
      a.seed_from_urdf = true;
    } else if (k == "--urdf-base" && i + 1 < argc) {
      a.urdf_base = argv[++i];
    } else if (k == "--no-gravity") {
      a.no_gravity = true;
    } else if (k == "--sweep-vel" && i + 1 < argc) {
      a.sweep_vel = std::atoi(argv[++i]);
    } else if (k == "--sweep" && i + 1 < argc) {
      a.sweep = std::atoi(argv[++i]);
    } else if (k == "--sweep-steps" && i + 1 < argc) {
      a.sweep_steps = std::atoi(argv[++i]);
    } else if (k == "--seed" && i + 1 < argc) {
      a.seed = static_cast<unsigned>(std::atoi(argv[++i]));
    } else if (k == "--gravity") {
      if (!next(a.gravity)) {*error = "--gravity needs a value"; return false;}
    } else if (k == "--qmax") {
      if (!next(a.qmax)) {*error = "--qmax needs a value"; return false;}
    } else if (k == "--qdmax") {
      if (!next(a.qdmax)) {*error = "--qdmax needs a value"; return false;}
    } else if (k == "--qddmax") {
      if (!next(a.qddmax)) {*error = "--qddmax needs a value"; return false;}
    } else {
      *error = "unknown / malformed argument: " + k;
      return false;
    }
  }
  if (a.arm != "right" && a.arm != "left") {
    *error = "--arm must be 'right' or 'left'";
    return false;
  }
  if (a.samples < 10) {
    *error = "--samples must be >= 10 (need >> 30 for full identifiability)";
    return false;
  }
  return true;
}

Eigen::Isometry3d toIso(const pinocchio::SE3 & m)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = m.rotation();
  t.translation() = m.translation();
  return t;
}

// Read per-link inertials from the URDF, re-express them in the DH link frame
// the regressor uses, and print a physically-valid YAML seed block.
int seedFromUrdf(const Args & a, const std::vector<std::string> & joints)
{
  pinocchio::Model model;
  try {
    pinocchio::urdf::buildModel(a.urdf, model);
  } catch (const std::exception & e) {
    std::cerr << "[exo_regressor_check] pinocchio buildModel failed: "
              << e.what() << "\n";
    return 1;
  }
  pinocchio::Data data(model);
  const Eigen::VectorXd q0 = pinocchio::neutral(model);
  pinocchio::forwardKinematics(model, data, q0);
  pinocchio::updateFramePlacements(model, data);

  // Frame the DH base offset is referenced to (kinematics_utils: <arm>_base_link).
  pinocchio::SE3 root_T_base = pinocchio::SE3::Identity();
  const std::string base_name =
    a.urdf_base.empty() ? (a.arm + "_base_link") : a.urdf_base;
  if (model.existFrame(base_name)) {
    root_T_base = data.oMf[model.getFrameId(base_name)];
  } else {
    std::cerr << "[exo_regressor_check] WARNING: URDF has no frame '"
              << base_name << "'; assuming the URDF root coincides with the DH "
              << "base (pass --urdf-base if the seed looks wrong)\n";
  }

  // DH frame placements at q = 0, expressed in <arm>_base_link.
  std::vector<Eigen::Isometry3d> seg;
  std::string ferr;
  if (!exo_utils::kinematics::forwardKinematicsChain(
      a.arm + "_ee", {}, {0.0, 0.0, 0.0}, seg, &ferr) || seg.size() < 4)
  {
    std::cerr << "[exo_regressor_check] DH chain failed: " << ferr << "\n";
    return 1;
  }
  std::array<Eigen::Isometry3d, 3> base_T_dh;
  Eigen::Isometry3d acc = seg[0];
  for (int i = 1; i <= 3; ++i) {
    acc = acc * seg[static_cast<size_t>(i)];
    base_T_dh[static_cast<size_t>(i - 1)] = acc;
  }

  Eigen::Vector3d m, xc, yc, zc, ixx, iyy, izz, ixy, ixz, iyz;
  for (int i = 0; i < 3; ++i) {
    if (!model.existJointName(joints[static_cast<size_t>(i)])) {
      std::cerr << "[exo_regressor_check] URDF has no joint '"
                << joints[static_cast<size_t>(i)] << "'\n";
      return 1;
    }
    const pinocchio::JointIndex jid =
      model.getJointId(joints[static_cast<size_t>(i)]);
    const pinocchio::Inertia & Yi = model.inertias[jid];
    const double mass = Yi.mass();
    const Eigen::Vector3d c_j = Yi.lever();              // COM in joint frame
    const Eigen::Matrix3d Ic_j = Yi.inertia().matrix();  // about COM, joint axes

    // Constant transform from the pinocchio joint frame to the DH link frame
    // (both rigidly attached to physical link i), evaluated at q = 0.
    const Eigen::Isometry3d base_T_joint =
      toIso(root_T_base.inverse() * data.oMi[jid]);
    const Eigen::Isometry3d dh_T_joint =
      (base_T_joint.inverse() * base_T_dh[static_cast<size_t>(i)]).inverse();
    const Eigen::Matrix3d R = dh_T_joint.rotation();
    const Eigen::Vector3d c_dh = R * c_j + dh_T_joint.translation();
    const Eigen::Matrix3d Ic_dh = R * Ic_j * R.transpose();  // rotate (about COM)

    m(i) = mass;
    xc(i) = c_dh.x(); yc(i) = c_dh.y(); zc(i) = c_dh.z();
    ixx(i) = Ic_dh(0, 0); iyy(i) = Ic_dh(1, 1); izz(i) = Ic_dh(2, 2);
    ixy(i) = Ic_dh(0, 1); ixz(i) = Ic_dh(0, 2); iyz(i) = Ic_dh(1, 2);
  }

  auto row = [](const char * name, const Eigen::Vector3d & v) {
      std::cout << "    " << name << ": [" << v(0) << ", " << v(1) << ", "
                << v(2) << "]\n";
    };
  std::cout.precision(8);
  std::cout << "[exo_regressor_check] physically-valid seed from " << a.urdf
            << "\n  (mass/COM/inertia re-expressed in the DH link frames; "
            << "leave 'theta0' unset and paste below)\n\n";
  row("link_mass  ", m);
  row("com_x      ", xc);
  row("com_y      ", yc);
  row("com_z      ", zc);
  row("inertia_ixx", ixx);
  row("inertia_iyy", iyy);
  row("inertia_izz", izz);
  row("inertia_ixy", ixy);
  row("inertia_ixz", ixz);
  row("inertia_iyz", iyz);
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  Args args;
  std::string err;
  if (!parseArgs(argc, argv, args, &err)) {
    std::cerr << "[exo_regressor_check] " << err << "\n";
    return 2;
  }

  if (args.arm == "left" && !exo_utils::dynamics::exoLeftYImplemented()) {
    std::cerr << "[exo_regressor_check] left-arm regressor is still a stub "
              << "(see exo_left_Y in exo_utils/src/dynamics/regressor.cpp)\n";
    return 2;
  }

  if (args.urdf.empty()) {
    try {
      const std::string share =
        ament_index_cpp::get_package_share_directory("exo_description");
      args.urdf = share + "/urdf/exo_dynamics_" + args.arm + ".urdf";
    } catch (const std::exception & e) {
      std::cerr << "[exo_regressor_check] could not resolve exo_description "
                << "share dir (" << e.what() << "); pass --urdf PATH\n";
      return 2;
    }
  }

  const std::vector<std::string> joints = {
    args.arm + "_revolute_1", args.arm + "_revolute_2", args.arm + "_revolute_3"};

  if (args.seed_from_urdf) {
    return seedFromUrdf(args, joints);
  }

  auto dyn = exo_utils::dynamics::createDynamicsModel("pinocchio", &err);
  if (!dyn) {
    std::cerr << "[exo_regressor_check] " << err << "\n";
    return 1;
  }
  if (!dyn->loadUrdf(args.urdf, &err)) {
    std::cerr << "[exo_regressor_check] URDF load failed: " << err << "\n";
    return 1;
  }

  std::cout << "[exo_regressor_check] arm=" << args.arm
            << "  urdf=" << args.urdf
            << "  samples=" << args.samples
            << "  gravity=" << args.gravity << "\n";

  const int N = args.samples;
  Eigen::MatrixXd A(3 * N, 30);
  Eigen::VectorXd b(3 * N);

  std::mt19937 rng(args.seed);
  std::uniform_real_distribution<double> dq_pos(-args.qmax, args.qmax);
  std::uniform_real_distribution<double> dq_vel(-args.qdmax, args.qdmax);
  std::uniform_real_distribution<double> dq_acc(-args.qddmax, args.qddmax);

  for (int k = 0; k < N; ++k) {
    Eigen::Vector3d q, qd, qdd;
    for (int j = 0; j < 3; ++j) {
      q(j) = dq_pos(rng);
      qd(j) = dq_vel(rng);
      qdd(j) = dq_acc(rng);
    }
    const std::vector<double> qv(q.data(), q.data() + 3);
    const std::vector<double> qdv(qd.data(), qd.data() + 3);

    Eigen::MatrixXd M, C;
    std::vector<double> gvec;
    if (!dyn->computeMassMatrix(joints, qv, M, &err) ||
      !dyn->computeCoriolisMatrix(joints, qv, qdv, C, &err) ||
      !dyn->computeGravityTorque(joints, qv, gvec, &err))
    {
      std::cerr << "[exo_regressor_check] dynamics eval failed at sample "
                << k << ": " << err << "\n";
      return 1;
    }
    Eigen::Map<const Eigen::Vector3d> g(gvec.data());
    const double gY = args.no_gravity ? 0.0 : args.gravity;
    const Eigen::Vector3d tau =
      M * qdd + C * qd + (args.no_gravity ? Eigen::Vector3d::Zero() : Eigen::Vector3d(g));

    // qd_r = qd, qdd_r = qdd  ->  Y·θ must equal the inverse-dynamics torque.
    const exo_utils::dynamics::RegressorMatrix Y =
      exo_utils::dynamics::exo_Y(args.arm, q, qd, qd, qdd, gY);

    A.block(3 * k, 0, 3, 30) = Y;
    b.segment(3 * k, 3) = tau;
  }

  // Minimum-norm least squares via SVD (handles structurally unidentifiable
  // parameter directions gracefully).
  Eigen::BDCSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd sv = svd.singularValues();
  const double smax = sv(0);
  const double smin = sv(sv.size() - 1);
  svd.setThreshold(1e-9);
  const Eigen::VectorXd theta = svd.solve(b);

  const Eigen::VectorXd r = A * theta - b;
  const double rel = r.norm() / b.norm();

  // Per-joint relative residual (rows j, j+3, j+6, …).
  std::array<double, 3> jr{}, jb{};
  for (int k = 0; k < N; ++k) {
    for (int j = 0; j < 3; ++j) {
      jr[j] += r(3 * k + j) * r(3 * k + j);
      jb[j] += b(3 * k + j) * b(3 * k + j);
    }
  }

  std::cout.setf(std::ios::scientific);
  std::cout.precision(4);
  std::cout << "\n=== Consistency ==="
            << (args.no_gravity ? "  [gravity OFF: inertial + Coriolis only]" : "")
            << "\n";
  std::cout << "relative residual  ||Aθ*-b|| / ||b|| = " << rel << "\n";
  for (int j = 0; j < 3; ++j) {
    const double rj = (jb[j] > 0.0) ? std::sqrt(jr[j] / jb[j]) : 0.0;
    std::cout << "  joint " << (j + 1) << " relative residual = " << rj << "\n";
  }
  std::cout << "singular values: max=" << smax << " min=" << smin
            << " cond=" << (smin > 0.0 ? smax / smin : INFINITY)
            << "  rank=" << svd.rank() << "/30\n";
  const char * verdict =
    (rel < 1e-4) ? "PASS  (Y is consistent with the URDF dynamics)"
    : (rel < 1e-1) ? "MARGINAL  (small structural mismatch or numerical noise)"
    : "FAIL  (Y does NOT match the URDF dynamics)";
  std::cout << "verdict: " << verdict << "\n";

  // Identified barycentric θ* and its physical interpretation per link.
  static const std::array<const char *, 10> lbl = {
    "m", "mx", "my", "mz", "Jxx", "Jyy", "Jzz", "Jxy", "Jxz", "Jyz"};
  std::cout.unsetf(std::ios::scientific);
  std::cout.precision(6);
  std::cout << "\n=== Identified barycentric θ* (only trustworthy if PASS) ===\n";
  for (int link = 0; link < 3; ++link) {
    std::cout << "link " << (link + 1) << ":";
    for (int p = 0; p < 10; ++p) {
      std::cout << "  " << lbl[p] << "=" << theta(10 * link + p);
    }
    std::cout << "\n";
  }

  // Paste-ready direct barycentric seed (use this for the controller's
  // 'theta0' param — valid for prediction even where back-conversion isn't).
  std::cout << "\n=== YAML seed: direct barycentric (paste as 'theta0' if PASS) ===\n";
  std::cout << "    theta0: [";
  for (int p = 0; p < 30; ++p) {
    std::cout << theta(p) << (p + 1 < 30 ? ", " : "");
  }
  std::cout << "]\n";

  // Back-convert to the physical parameters the controller YAML expects.
  Eigen::Vector3d m, xc, yc, zc, ixx, iyy, izz, ixy, ixz, iyz;
  bool physical_ok = true;
  for (int link = 0; link < 3; ++link) {
    const auto th = theta.segment(10 * link, 10);
    const double mm = th(0);
    if (mm <= 1e-9) {physical_ok = false;}
    const double cx = mm > 1e-9 ? th(1) / mm : 0.0;
    const double cy = mm > 1e-9 ? th(2) / mm : 0.0;
    const double cz = mm > 1e-9 ? th(3) / mm : 0.0;
    // Second moments of mass about COM (reverse parallel-axis).
    const double Sxx = th(4) - mm * cx * cx;
    const double Syy = th(5) - mm * cy * cy;
    const double Szz = th(6) - mm * cz * cz;
    const double Sxy = th(7) - mm * cx * cy;
    const double Sxz = th(8) - mm * cx * cz;
    const double Syz = th(9) - mm * cy * cz;
    m(link) = mm;
    xc(link) = cx; yc(link) = cy; zc(link) = cz;
    ixx(link) = Syy + Szz;   // inertia tensor about COM
    iyy(link) = Sxx + Szz;
    izz(link) = Sxx + Syy;
    ixy(link) = -Sxy;
    ixz(link) = -Sxz;
    iyz(link) = -Syz;
  }

  auto row = [](const char * name, const Eigen::Vector3d & v) {
      std::cout << "    " << name << ": [" << v(0) << ", " << v(1) << ", " << v(2) << "]\n";
    };
  std::cout << "\n=== YAML seed (controllers_" << args.arm
            << ".yaml — paste if PASS) ===\n";
  if (!physical_ok) {
    std::cout << "  # WARNING: non-physical θ* (mass <= 0); use raw barycentric "
                 "values above instead\n";
  }
  row("link_mass  ", m);
  row("com_x      ", xc);
  row("com_y      ", yc);
  row("com_z      ", zc);
  row("inertia_ixx", ixx);
  row("inertia_iyy", iyy);
  row("inertia_izz", izz);
  row("inertia_ixy", ixy);
  row("inertia_ixz", ixz);
  row("inertia_iyz", iyz);

  // Optional: sweep one joint (others = 0, q̇ = q̈ = 0) and tabulate the
  // gravity discrepancy g_pino - Y(q,0,0,0)·θ* per joint. The *shape* of the
  // joint-3 error vs the swept angle localizes the structural bug.
  if (args.sweep >= 1 && args.sweep <= 3) {
    const int sj = args.sweep - 1;
    std::cout << "\n=== Gravity sweep: joint " << args.sweep
              << " in [-pi, pi], others = 0  (g_pino vs Y·θ*) ===\n";
    std::cout << "   q" << args.sweep
              << "      g1_pino  g1_pred  d1        g2_pino  g2_pred  d2"
              << "        g3_pino  g3_pred  d3\n";
    std::cout.setf(std::ios::fixed);
    std::cout.precision(5);
    const int M = std::max(2, args.sweep_steps);
    for (int s = 0; s < M; ++s) {
      const double a = -M_PI + 2.0 * M_PI * s / (M - 1);
      Eigen::Vector3d q = Eigen::Vector3d::Zero();
      q(sj) = a;
      const std::vector<double> qv(q.data(), q.data() + 3);
      std::vector<double> gvec;
      if (!dyn->computeGravityTorque(joints, qv, gvec, &err)) {
        std::cerr << "[exo_regressor_check] gravity eval failed: " << err << "\n";
        return 1;
      }
      const Eigen::Vector3d g_pino(gvec[0], gvec[1], gvec[2]);
      const Eigen::Vector3d g_pred =
        exo_utils::dynamics::exo_Y(
        args.arm, q, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), args.gravity) * theta;
      std::cout << (a >= 0 ? "  " : " ") << a;
      for (int j = 0; j < 3; ++j) {
        std::cout << "  " << g_pino(j) << " " << g_pred(j) << " "
                  << (g_pino(j) - g_pred(j));
      }
      std::cout << "\n";
    }
    std::cout.unsetf(std::ios::fixed);
  }

  // Optional: sweep one joint's velocity (q = 0, q̈ = 0, gravity OFF) and
  // tabulate the Coriolis/velocity discrepancy  C·q̇ (Pinocchio) vs
  // Y(0,q̇,q̇,0,g=0)·θ*  per joint. Exposes a joint-3 Coriolis sign/coefficient
  // bug, which acts as negative damping (stable at rest, chatters on motion).
  if (args.sweep_vel >= 1 && args.sweep_vel <= 3) {
    const int sj = args.sweep_vel - 1;
    const std::vector<double> q0v = {0.0, 0.0, 0.0};
    std::cout << "\n=== Velocity sweep: q̇" << args.sweep_vel
              << " in [-qdmax, qdmax], q=0 q̈=0 g=0  (C·q̇ vs Y·θ*) ===\n";
    std::cout << "  qd" << args.sweep_vel
              << "     c1_pino  c1_pred  d1        c2_pino  c2_pred  d2"
              << "        c3_pino  c3_pred  d3\n";
    std::cout.setf(std::ios::fixed);
    std::cout.precision(5);
    const int Ms = std::max(2, args.sweep_steps);
    for (int s = 0; s < Ms; ++s) {
      const double v = -args.qdmax + 2.0 * args.qdmax * s / (Ms - 1);
      Eigen::Vector3d qd = Eigen::Vector3d::Zero();
      qd(sj) = v;
      const std::vector<double> qdv(qd.data(), qd.data() + 3);
      Eigen::MatrixXd C;
      if (!dyn->computeCoriolisMatrix(joints, q0v, qdv, C, &err)) {
        std::cerr << "[exo_regressor_check] Coriolis eval failed: " << err << "\n";
        return 1;
      }
      const Eigen::Vector3d c_pino = C * qd;
      const Eigen::Vector3d c_pred =
        exo_utils::dynamics::exo_Y(
        args.arm, Eigen::Vector3d::Zero(), qd, qd,
        Eigen::Vector3d::Zero(), 0.0) * theta;
      std::cout << (v >= 0 ? "  " : " ") << v;
      for (int j = 0; j < 3; ++j) {
        std::cout << "  " << c_pino(j) << " " << c_pred(j) << " "
                  << (c_pino(j) - c_pred(j));
      }
      std::cout << "\n";
    }
    std::cout.unsetf(std::ios::fixed);
  }

  return (rel < 1e-1) ? 0 : 1;
}
