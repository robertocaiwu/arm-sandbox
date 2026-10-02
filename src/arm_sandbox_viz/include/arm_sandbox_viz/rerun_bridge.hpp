#pragma once

#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace arm_sandbox_viz
{

/// Forwards standard ROS 2 topics to Rerun (FR-12). Reads no sim internals (FR-14a).
///
/// - /robot_description: logged through Rerun's URDF loader, which names each link's coordinate
///   frame after the link and resolves package:// meshes from the ROS environment.
/// - /tf, /tf_static: each transform becomes a Transform3D between named frames, so the URDF
///   geometry follows the robot. /tf is timed by its header stamp (sim time), /tf_static is static.
/// - /joint_states: position, velocity and effort per joint as time series (throttled).
class RerunBridge : public rclcpp::Node
{
public:
  explicit RerunBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RerunBridge() override;

private:
  void on_robot_description(const std_msgs::msg::String & msg) const;
  void on_tf(const tf2_msgs::msg::TFMessage & msg, bool is_static) const;
  void on_joint_states(const sensor_msgs::msg::JointState & msg);

  rerun::RecordingStream recording_;
  rclcpp::Duration joint_state_log_period_;
  std::optional<rclcpp::Time> last_joint_state_log_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
};

}  // namespace arm_sandbox_viz
