// reach_runner: solves the reach task (milestone M2) with the hand-written IK and the joint
// trajectory controller. Thin ROS wrapper: the math is in arm_sandbox_kinematics, the task logic
// in reach_task.hpp.
//
// Two ways to move (parameter `motion`):
// - joint_trajectory (M2): IK from the current joint positions (null-space pull towards home) ->
//   one FollowJointTrajectory goal to the joint trajectory controller.
// - pose_target (M3): publish the target pose to a task-space controller (OSC or impedance), which
//   does the rest; no IK.
// Either way, a target counts once TF shows the end effector held within the task tolerance.
// Exits 0 if every target was reached, 1 otherwise.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "arm_sandbox_kinematics/ik.hpp"
#include "arm_sandbox_kinematics/kinematic_chain.hpp"
#include "arm_sandbox_tasks/reach_task.hpp"

namespace arm_sandbox_tasks
{
namespace
{
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using Clock = std::chrono::steady_clock;
constexpr auto kPollPeriod = std::chrono::milliseconds(20);
constexpr const char * kJointTrajectory = "joint_trajectory";
constexpr const char * kPoseTarget = "pose_target";

Clock::duration seconds(double value)
{
  return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(value));
}
}  // namespace

class ReachRunner : public rclcpp::Node
{
public:
  ReachRunner()
  : Node("reach_runner"),
    arm_joints_(declare_parameter<std::vector<std::string>>("arm_joints", std::vector<std::string>{})),
    base_frame_(declare_parameter<std::string>("base_frame", "")),
    ee_frame_(declare_parameter<std::string>("ee_frame", "")),
    task_(load_reach_task(declare_parameter<std::string>("task_file", ""))),
    startup_timeout_s_(declare_parameter<double>("startup_timeout_s", 0.0)),
    velocity_scale_(declare_parameter<double>("velocity_scale", 0.0)),
    min_move_duration_s_(declare_parameter<double>("min_move_duration_s", 0.0)),
    settle_timeout_s_(declare_parameter<double>("settle_timeout_s", 0.0)),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    const auto motion = declare_parameter<std::string>("motion", "");
    if (motion != kJointTrajectory && motion != kPoseTarget) {
      RCLCPP_FATAL(get_logger(), "motion must be '%s' or '%s', got '%s'", kJointTrajectory, kPoseTarget, motion.c_str());
      throw std::invalid_argument("invalid motion");
    }
    use_pose_target_ = motion == kPoseTarget;
    const auto home = declare_parameter<std::vector<double>>("home", std::vector<double>{});
    if (arm_joints_.empty() || base_frame_.empty() || ee_frame_.empty() || home.size() != arm_joints_.size()) {
      RCLCPP_FATAL(get_logger(), "robot config missing: arm_joints, base_frame, ee_frame, home");
      throw std::invalid_argument("invalid robot config");
    }
    if (startup_timeout_s_ <= 0.0 || velocity_scale_ <= 0.0 || min_move_duration_s_ <= 0.0 || settle_timeout_s_ <= 0.0) {
      RCLCPP_FATAL(get_logger(), "startup_timeout_s, velocity_scale, min_move_duration_s, settle_timeout_s must be > 0");
      throw std::invalid_argument("invalid reach_runner config");
    }
    home_ = Eigen::Map<const Eigen::VectorXd>(home.data(), static_cast<Eigen::Index>(home.size()));

    ik_options_.max_iterations = static_cast<int>(declare_parameter<int>("ik.max_iterations", 0));
    ik_options_.position_tolerance = declare_parameter<double>("ik.position_tolerance", 0.0);
    ik_options_.orientation_tolerance = declare_parameter<double>("ik.orientation_tolerance", 0.0);
    ik_options_.max_damping = declare_parameter<double>("ik.max_damping", 0.0);
    ik_options_.manipulability_threshold = declare_parameter<double>("ik.manipulability_threshold", 0.0);
    ik_options_.max_step = declare_parameter<double>("ik.max_step", 0.0);
    ik_options_.null_space_gain = declare_parameter<double>("ik.null_space_gain", 0.0);
    ik_options_.null_space_target = home_;

    robot_description_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).transient_local().reliable(),
      [this](const std_msgs::msg::String & msg) { robot_description_ = msg.data; });
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState & msg) { latest_joint_state_ = msg; });
    // Both controllers publish their state only while active, so the first message means "ready".
    const auto state_topic = declare_parameter<std::string>("controller_state_topic", "");
    if (use_pose_target_) {
      pose_error_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        state_topic, rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::TwistStamped &) { controller_active_ = true; });
      target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        declare_parameter<std::string>("pose_target_topic", ""), rclcpp::SystemDefaultsQoS());
    } else {
      jtc_state_sub_ = create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        state_topic, rclcpp::SensorDataQoS(),
        [this](const control_msgs::msg::JointTrajectoryControllerState &) { controller_active_ = true; });
      arm_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, declare_parameter<std::string>("arm_action", ""));
    }
  }

  /// Runs every target; true if all were reached.
  bool run()
  {
    if (!wait_until([this] { return robot_description_.has_value(); }, "/robot_description") ||
        !wait_until([this] { return controller_active_; }, "the arm controller to become active") ||
        !wait_until([this] { return current_positions().has_value(); }, "/joint_states with all arm joints")) {
      return false;
    }
    if (use_pose_target_ ? !wait_until([this] { return target_pub_->get_subscription_count() > 0; }, "the controller's target subscription")
                         : !arm_client_->wait_for_action_server(seconds(startup_timeout_s_))) {
      return false;
    }

    const arm_sandbox_kinematics::KinematicChain chain(*robot_description_, base_frame_, ee_frame_);
    if (chain.joint_names() != arm_joints_) {
      RCLCPP_FATAL(get_logger(), "the URDF chain %s -> %s doesn't match robot.yaml arm_joints", base_frame_.c_str(), ee_frame_.c_str());
      return false;
    }

    int reached = 0;
    for (std::size_t i = 0; i < task_.targets.size(); ++i) {
      const bool ok = reach(chain, task_.targets[i], i);
      reached += ok ? 1 : 0;
    }
    RCLCPP_INFO(get_logger(), "task '%s': %d/%zu targets reached", task_.name.c_str(), reached, task_.targets.size());
    return reached == static_cast<int>(task_.targets.size());
  }

private:
  bool reach(const arm_sandbox_kinematics::KinematicChain & chain, const Eigen::Isometry3d & target, std::size_t index)
  {
    const auto start_time = Clock::now();
    const auto deadline = start_time + seconds(task_.time_limit_s);
    if (use_pose_target_) {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = base_frame_;
      msg.header.stamp = now();
      msg.pose = tf2::toMsg(target);
      target_pub_->publish(msg);
      // The controller moves the arm itself, so it may use the whole time limit to get there.
      return hold_within_tolerance(target, index, deadline, start_time, "pose target");
    }

    const Eigen::VectorXd start = *current_positions();
    const auto ik = arm_sandbox_kinematics::solve_ik(chain, target, start, ik_options_);
    if (!ik.converged) {
      RCLCPP_ERROR(get_logger(), "target %zu: IK failed (%.4f m, %.4f rad after %d iterations)",
                   index, ik.position_error, ik.orientation_error, ik.iterations);
      return false;
    }

    const double duration = move_duration(start, ik.q, chain.velocity_limits(), velocity_scale_, min_move_duration_s_);
    if (!move_to(ik.q, duration, deadline)) {
      RCLCPP_ERROR(get_logger(), "target %zu: trajectory failed", index);
      return false;
    }

    // JTC's goal tolerance is per joint, so the pose keeps settling after it succeeds.
    char how[96];
    std::snprintf(how, sizeof(how), "IK %d iterations, move %.2f s", ik.iterations, duration);
    return hold_within_tolerance(
      target, index, std::min(deadline, Clock::now() + seconds(settle_timeout_s_) + seconds(task_.hold_s)), start_time, how);
  }

  /// Waits until TF shows the end effector within the task tolerance for hold_s of sim time.
  bool hold_within_tolerance(
    const Eigen::Isometry3d & target, std::size_t index, Clock::time_point deadline, Clock::time_point start_time,
    const std::string & how)
  {
    PoseError error;
    std::optional<rclcpp::Time> within_since;
    while (true) {
      const std::optional<Eigen::Isometry3d> ee = ee_pose();
      if (ee) {
        error = pose_error(target, *ee);
        if (!within(error, task_.tolerance)) {
          within_since.reset();
        } else if (!within_since) {
          within_since = now();
        } else if ((now() - *within_since).seconds() >= task_.hold_s) {
          const double elapsed = std::chrono::duration<double>(Clock::now() - start_time).count();
          RCLCPP_INFO(get_logger(), "target %zu reached: %.4f m, %.4f rad in %.2f s (%s)",
                      index, error.position, error.orientation, elapsed, how.c_str());
          return true;
        }
      }
      if (Clock::now() > deadline) {
        RCLCPP_ERROR(get_logger(), "target %zu: not held within tolerance (%.4f m, %.4f rad)", index, error.position, error.orientation);
        return false;
      }
      spin_for(kPollPeriod);
    }
  }

  bool move_to(const Eigen::VectorXd & q, double duration_s, Clock::time_point deadline)
  {
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = arm_joints_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(q.data(), q.data() + q.size());
    point.velocities.assign(static_cast<std::size_t>(q.size()), 0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(duration_s);
    goal.trajectory.points.push_back(point);

    auto goal_future = arm_client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_future, deadline - Clock::now()) !=
        rclcpp::FutureReturnCode::SUCCESS || !goal_future.get()) {
      return false;
    }
    auto result_future = arm_client_->async_get_result(goal_future.get());
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future, deadline - Clock::now()) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return false;
    }
    const auto result = result_future.get();
    return result.code == rclcpp_action::ResultCode::SUCCEEDED &&
           result.result->error_code == FollowJointTrajectory::Result::SUCCESSFUL;
  }

  /// Arm joint positions in arm_joints_ order, if the latest /joint_states has all of them.
  std::optional<Eigen::VectorXd> current_positions() const
  {
    if (!latest_joint_state_) {
      return std::nullopt;
    }
    Eigen::VectorXd q(static_cast<Eigen::Index>(arm_joints_.size()));
    for (std::size_t i = 0; i < arm_joints_.size(); ++i) {
      const auto & names = latest_joint_state_->name;
      const auto it = std::find(names.begin(), names.end(), arm_joints_[i]);
      if (it == names.end() || static_cast<std::size_t>(it - names.begin()) >= latest_joint_state_->position.size()) {
        return std::nullopt;
      }
      q[static_cast<Eigen::Index>(i)] = latest_joint_state_->position[static_cast<std::size_t>(it - names.begin())];
    }
    return q;
  }

  std::optional<Eigen::Isometry3d> ee_pose() const
  {
    try {
      return tf2::transformToEigen(tf_buffer_.lookupTransform(base_frame_, ee_frame_, tf2::TimePointZero));
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  template <typename Predicate>
  bool wait_until(Predicate ready, const std::string & what)
  {
    const auto deadline = Clock::now() + seconds(startup_timeout_s_);
    while (!ready()) {
      if (Clock::now() > deadline) {
        RCLCPP_FATAL(get_logger(), "timed out waiting for %s", what.c_str());
        return false;
      }
      spin_for(kPollPeriod);
    }
    return true;
  }

  void spin_for(std::chrono::milliseconds period)
  {
    rclcpp::spin_some(shared_from_this());
    std::this_thread::sleep_for(period);
  }

  std::vector<std::string> arm_joints_;
  std::string base_frame_;
  std::string ee_frame_;
  ReachTask task_;
  double startup_timeout_s_;
  double velocity_scale_;
  double min_move_duration_s_;
  double settle_timeout_s_;
  Eigen::VectorXd home_;
  arm_sandbox_kinematics::IkOptions ik_options_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;  ///< runs its own node and thread
  std::optional<std::string> robot_description_;
  std::optional<sensor_msgs::msg::JointState> latest_joint_state_;
  bool controller_active_ = false;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  bool use_pose_target_ = false;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr jtc_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr pose_error_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_client_;
};

}  // namespace arm_sandbox_tasks

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  bool all_reached = false;
  {
    const auto runner = std::make_shared<arm_sandbox_tasks::ReachRunner>();
    all_reached = runner->run();
  }
  rclcpp::shutdown();
  return all_reached ? 0 : 1;
}
