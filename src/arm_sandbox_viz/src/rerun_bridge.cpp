#include "arm_sandbox_viz/rerun_bridge.hpp"

#include <cstddef>
#include <stdexcept>
#include <string>

namespace arm_sandbox_viz
{
namespace
{
constexpr const char * kTimeline = "sim_time";
// File name hint: tells Rerun to use its URDF loader for the /robot_description contents.
constexpr const char * kRobotDescriptionFileName = "robot_description.urdf";
constexpr const char * kRobotEntityPrefix = "robot";

rerun::Transform3D to_rerun(const geometry_msgs::msg::TransformStamped & msg)
{
  const auto & t = msg.transform.translation;
  const auto & r = msg.transform.rotation;
  return rerun::Transform3D::from_translation_rotation(
           {static_cast<float>(t.x), static_cast<float>(t.y), static_cast<float>(t.z)},
           rerun::Quaternion::from_xyzw(
             static_cast<float>(r.x), static_cast<float>(r.y), static_cast<float>(r.z),
             static_cast<float>(r.w)))
    .with_parent_frame(msg.header.frame_id)
    .with_child_frame(msg.child_frame_id);
}

double to_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return rclcpp::Time(stamp).seconds();
}
}  // namespace

RerunBridge::RerunBridge(const rclcpp::NodeOptions & options)
: Node("rerun_bridge", options),
  recording_(declare_parameter<std::string>("application_id", "arm_sandbox")),
  joint_state_log_period_(
    rclcpp::Duration::from_seconds(declare_parameter<double>("joint_state_log_period_s", 0.0)))
{
  const auto grpc_url = declare_parameter<std::string>("grpc_url", "");
  const auto save_path = declare_parameter<std::string>("save_path", "");
  if (grpc_url.empty() == save_path.empty()) {
    RCLCPP_FATAL(get_logger(), "set exactly one of 'grpc_url' or 'save_path'");
    throw std::invalid_argument("invalid rerun_bridge sink config");
  }
  if (joint_state_log_period_ < rclcpp::Duration(0, 0)) {
    RCLCPP_FATAL(get_logger(), "'joint_state_log_period_s' must be >= 0");
    throw std::invalid_argument("invalid rerun_bridge throttle config");
  }

  const rerun::Error error =
    save_path.empty() ? recording_.connect_grpc(grpc_url) : recording_.save(save_path);
  if (error.is_err()) {
    RCLCPP_FATAL(get_logger(), "cannot open the Rerun sink: %s", error.description.c_str());
    throw std::runtime_error("rerun sink failed");
  }
  RCLCPP_INFO(
    get_logger(), "logging to %s", save_path.empty() ? grpc_url.c_str() : save_path.c_str());

  const auto latched = rclcpp::QoS(1).transient_local().reliable();
  robot_description_sub_ = create_subscription<std_msgs::msg::String>(
    "/robot_description", latched,
    [this](const std_msgs::msg::String & msg) { on_robot_description(msg); });
  tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", rclcpp::QoS(100), [this](const tf2_msgs::msg::TFMessage & msg) { on_tf(msg, false); });
  tf_static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(100).transient_local().reliable(),
    [this](const tf2_msgs::msg::TFMessage & msg) { on_tf(msg, true); });
  joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(10),
    [this](const sensor_msgs::msg::JointState & msg) { on_joint_states(msg); });
}

RerunBridge::~RerunBridge()
{
  // Make sure a recording file is complete when the node stops.
  const rerun::Error error = recording_.flush_blocking();
  if (error.is_err()) {
    RCLCPP_ERROR(get_logger(), "flushing the Rerun recording failed: %s", error.description.c_str());
  }
}

void RerunBridge::on_robot_description(const std_msgs::msg::String & msg) const
{
  const auto * contents = reinterpret_cast<const std::byte *>(msg.data.data());
  const rerun::Error error = recording_.try_log_file_from_contents(
    kRobotDescriptionFileName, contents, msg.data.size(), kRobotEntityPrefix, true);
  if (error.is_err()) {
    RCLCPP_ERROR(get_logger(), "cannot log the robot description: %s", error.description.c_str());
  }
}

void RerunBridge::on_tf(const tf2_msgs::msg::TFMessage & msg, bool is_static) const
{
  for (const auto & transform : msg.transforms) {
    if (is_static) {
      recording_.log_static("ros/tf_static/" + transform.child_frame_id, to_rerun(transform));
      continue;
    }
    recording_.set_time_duration_secs(kTimeline, to_seconds(transform.header.stamp));
    recording_.log("ros/tf/" + transform.child_frame_id, to_rerun(transform));
  }
}

void RerunBridge::on_joint_states(const sensor_msgs::msg::JointState & msg)
{
  const rclcpp::Time stamp(msg.header.stamp);
  // A sim reset can move time backwards; then log right away.
  const bool too_soon = last_joint_state_log_ && stamp >= *last_joint_state_log_ &&
                        stamp - *last_joint_state_log_ < joint_state_log_period_;
  if (too_soon) {
    return;
  }
  last_joint_state_log_ = stamp;

  recording_.set_time_duration_secs(kTimeline, stamp.seconds());
  for (std::size_t i = 0; i < msg.name.size(); ++i) {
    const std::string prefix = "joint_states/" + msg.name[i];
    if (i < msg.position.size()) {
      recording_.log(prefix + "/position", rerun::Scalars(msg.position[i]));
    }
    if (i < msg.velocity.size()) {
      recording_.log(prefix + "/velocity", rerun::Scalars(msg.velocity[i]));
    }
    if (i < msg.effort.size()) {
      recording_.log(prefix + "/effort", rerun::Scalars(msg.effort[i]));
    }
  }
}

}  // namespace arm_sandbox_viz
