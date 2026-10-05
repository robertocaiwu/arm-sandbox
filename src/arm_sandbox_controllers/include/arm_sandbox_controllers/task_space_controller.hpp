#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include "arm_sandbox_controllers/robot_dynamics.hpp"
#include "arm_sandbox_controllers/task_space_control.hpp"
#include "arm_sandbox_kinematics/kinematic_chain.hpp"

namespace arm_sandbox_controllers
{

/// ros2_control wrapper shared by the task-space controllers: a thin layer around a
/// `TaskSpaceControl` law from the ROS-free core.
///
/// - Commands: `<joint>/effort`. State: `<joint>/position`, `<joint>/velocity`.
/// - Input: `~/target_pose` (geometry_msgs/PoseStamped, in `base_frame`). On activation the
///   controller holds the current pose until a target arrives.
/// - Output: `~/pose_error` (geometry_msgs/TwistStamped: linear = position error in m, angular =
///   orientation error as angle * axis in rad), published only while active.
/// - Parameters: robot (`joints`, `base_frame`, `ee_frame`, `robot_description`,
///   `rest_configuration`, `compensate_gravity`; injected by sim.launch.py from robot.yaml),
///   model (`armature`), shared settings, and the law's gains (see `declare_gains`).
class TaskSpaceController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  /// Declare the law's gain parameters (called from on_init).
  virtual void declare_gains() = 0;
  /// Build the law from the declared gains. May throw std::invalid_argument on bad gains.
  virtual std::unique_ptr<TaskSpaceControl> make_law(
    const arm_sandbox_kinematics::KinematicChain & chain, RobotDynamics & dynamics, const TaskSpaceSettings & settings) = 0;

private:
  void read_state();
  void on_target(const geometry_msgs::msg::PoseStamped & msg);

  std::vector<std::string> joints_;
  std::string base_frame_;
  std::unique_ptr<arm_sandbox_kinematics::KinematicChain> chain_;
  std::unique_ptr<RobotDynamics> dynamics_;
  std::unique_ptr<TaskSpaceControl> law_;

  Eigen::VectorXd q_;
  Eigen::VectorXd qd_;
  Eigen::VectorXd tau_;
  realtime_tools::RealtimeBuffer<Eigen::Isometry3d> target_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>> error_publisher_;
  rclcpp::Duration publish_period_{0, 0};
  rclcpp::Time last_publish_time_;
};

/// Operational-space controller (plugin `arm_sandbox_controllers/OscController`).
class OscController : public TaskSpaceController
{
protected:
  void declare_gains() override;
  std::unique_ptr<TaskSpaceControl> make_law(
    const arm_sandbox_kinematics::KinematicChain & chain, RobotDynamics & dynamics, const TaskSpaceSettings & settings) override;
};

/// Cartesian impedance controller (plugin `arm_sandbox_controllers/CartesianImpedanceController`).
class CartesianImpedanceController : public TaskSpaceController
{
protected:
  void declare_gains() override;
  std::unique_ptr<TaskSpaceControl> make_law(
    const arm_sandbox_kinematics::KinematicChain & chain, RobotDynamics & dynamics, const TaskSpaceSettings & settings) override;
};

}  // namespace arm_sandbox_controllers
