// Subscribes to /joint_states, computes the DH-based FK chain for each arm
// (using exo_utils::kinematics::forwardKinematicsChain), broadcasts a TF per
// DH frame so each system of reference is visible in RViz, and publishes a
// green sphere Marker anchored at the DH end-effector frame.
//
// Frame naming, per arm side `S`:
//   S_base_link            (URDF, parent of the whole DH chain)
//       -> S_dh_0          (base offset)
//           -> S_dh_1      (after DH joint 1)
//               -> S_dh_2  (after DH joint 2)
//                   -> S_dh_3  (after DH joint 3)
//                       -> S_dh_ee  (after tool offset)

#include "exo_utils/kinematics/kinematics_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace
{

struct ArmSpec
{
  const char * arm;          // "right" / "left"
  const char * base_frame;   // "<arm>_base_link" (URDF)
  int marker_id;
};

constexpr std::array<ArmSpec, 2> kArms{{
  {"right", "right_base_link", 0},
  {"left",  "left_base_link",  1},
}};

geometry_msgs::msg::TransformStamped toTransformStamped(
  const Eigen::Isometry3d & T,
  const rclcpp::Time & stamp,
  const std::string & parent,
  const std::string & child)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = parent;
  msg.child_frame_id = child;
  msg.transform.translation.x = T.translation().x();
  msg.transform.translation.y = T.translation().y();
  msg.transform.translation.z = T.translation().z();
  const Eigen::Quaterniond qd(T.linear());
  msg.transform.rotation.x = qd.x();
  msg.transform.rotation.y = qd.y();
  msg.transform.rotation.z = qd.z();
  msg.transform.rotation.w = qd.w();
  return msg;
}

}  // namespace

class FkMarkerNode : public rclcpp::Node
{
public:
  FkMarkerNode()
  : Node("exo_fk_marker")
  {
    marker_radius_ = declare_parameter<double>("marker_radius", 0.03);
    publish_topic_ = declare_parameter<std::string>("topic", "fk_marker");

    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      publish_topic_, rclcpp::QoS(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { onJointState(*msg); });
  }

private:
  void onJointState(const sensor_msgs::msg::JointState & msg)
  {
    visualization_msgs::msg::MarkerArray array;
    std::vector<geometry_msgs::msg::TransformStamped> tfs;
    const rclcpp::Time stamp = msg.header.stamp.sec != 0 || msg.header.stamp.nanosec != 0
      ? rclcpp::Time(msg.header.stamp)
      : now();

    for (const auto & spec : kArms) {
      if (!hasArmJoints(msg, spec.arm)) {
        continue;
      }
      std::vector<Eigen::Isometry3d> segments;
      std::string err;
      if (!exo_utils::kinematics::forwardKinematicsChain(
            spec.arm, msg.name,
            std::vector<double>(msg.position.begin(), msg.position.end()),
            segments, &err))
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "FK failed for %s arm: %s", spec.arm, err.c_str());
        continue;
      }

      // Build the chain of frame names: base_link -> dh_0 -> dh_1 -> ... -> dh_ee.
      // segments.size() == n_joints + 2, so frame names size is n_joints + 3.
      const size_t n = segments.size();           // joints + 2
      const size_t n_joints = n - 2;
      std::vector<std::string> frames;
      frames.reserve(n + 1);
      frames.emplace_back(spec.base_frame);
      frames.emplace_back(std::string(spec.arm) + "_dh_0");
      for (size_t i = 1; i <= n_joints; ++i) {
        frames.emplace_back(std::string(spec.arm) + "_dh_" + std::to_string(i));
      }
      frames.emplace_back(std::string(spec.arm) + "_dh_ee");

      for (size_t i = 0; i < n; ++i) {
        tfs.push_back(toTransformStamped(segments[i], stamp, frames[i], frames[i + 1]));
      }

      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = frames.back();   // <arm>_dh_ee
      m.ns = std::string("exo_fk_") + spec.arm;
      m.id = spec.marker_id;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = 2.0 * marker_radius_;
      m.color.r = 0.0f;
      m.color.g = 1.0f;
      m.color.b = 0.0f;
      m.color.a = 1.0f;
      array.markers.push_back(std::move(m));
    }
    if (!tfs.empty()) {
      tf_broadcaster_->sendTransform(tfs);
    }
    if (!array.markers.empty()) {
      pub_->publish(array);
    }
  }

  static bool hasArmJoints(const sensor_msgs::msg::JointState & msg, const std::string & arm)
  {
    const std::string j1 = arm + "_revolute_1";
    for (const auto & n : msg.name) {
      if (n == j1) return true;
    }
    return false;
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double marker_radius_{0.03};
  std::string publish_topic_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FkMarkerNode>());
  rclcpp::shutdown();
  return 0;
}
