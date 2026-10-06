#include "arm_sandbox_controllers/task_space_controller.hpp"

#include <stdexcept>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace arm_sandbox_controllers
{
using controller_interface::CallbackReturn;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;

CallbackReturn TaskSpaceController::on_init()
{
  // Robot: injected by sim.launch.py from robot.yaml and the URDF (empty here = must be set).
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<std::string>("base_frame", "");
  auto_declare<std::string>("ee_frame", "");
  auto_declare<std::string>("robot_description", "");
  auto_declare<std::vector<double>>("rest_configuration", {});
  auto_declare<bool>("compensate_gravity", true);
  // Model and shared settings: per-robot controllers YAML. Zero defaults fail validation.
  auto_declare<std::vector<double>>("armature", {});
  auto_declare<double>("max_position_error", 0.0);
  auto_declare<double>("max_orientation_error", 0.0);
  auto_declare<double>("nullspace_stiffness", 0.0);
  auto_declare<double>("nullspace_damping", 0.0);
  auto_declare<double>("singularity_damping", 0.0);
  auto_declare<double>("state_publish_rate", 0.0);
  declare_gains();
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration TaskSpaceController::command_interface_configuration() const
{
  InterfaceConfiguration config{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

InterfaceConfiguration TaskSpaceController::state_interface_configuration() const
{
  // Order matters: read_state() expects all positions, then all velocities.
  InterfaceConfiguration config{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

CallbackReturn TaskSpaceController::on_configure(const rclcpp_lifecycle::State &)
{
  const auto node = get_node();
  joints_ = node->get_parameter("joints").as_string_array();
  base_frame_ = node->get_parameter("base_frame").as_string();
  const auto ee_frame = node->get_parameter("ee_frame").as_string();
  const auto urdf = node->get_parameter("robot_description").as_string();
  const auto rest = node->get_parameter("rest_configuration").as_double_array();
  const auto armature = node->get_parameter("armature").as_double_array();
  const double publish_rate = node->get_parameter("state_publish_rate").as_double();
  if (joints_.empty() || base_frame_.empty() || ee_frame.empty() || urdf.empty()) {
    RCLCPP_FATAL(node->get_logger(), "missing robot parameters: joints, base_frame, ee_frame, robot_description");
    return CallbackReturn::ERROR;
  }
  if (publish_rate <= 0.0) {
    RCLCPP_FATAL(node->get_logger(), "state_publish_rate must be > 0");
    return CallbackReturn::ERROR;
  }

  TaskSpaceSettings settings;
  settings.max_position_error = node->get_parameter("max_position_error").as_double();
  settings.max_orientation_error = node->get_parameter("max_orientation_error").as_double();
  settings.rest_configuration = Eigen::Map<const Eigen::VectorXd>(rest.data(), static_cast<Eigen::Index>(rest.size()));
  settings.nullspace_stiffness = node->get_parameter("nullspace_stiffness").as_double();
  settings.nullspace_damping = node->get_parameter("nullspace_damping").as_double();
  settings.singularity_damping = node->get_parameter("singularity_damping").as_double();
  settings.compensate_gravity = node->get_parameter("compensate_gravity").as_bool();

  try {
    chain_ = std::make_unique<arm_sandbox_kinematics::KinematicChain>(urdf, base_frame_, ee_frame);
    if (chain_->joint_names() != joints_) {
      throw std::invalid_argument("the chain " + base_frame_ + " -> " + ee_frame + " doesn't match `joints`");
    }
    dynamics_ = std::make_unique<RobotDynamics>(
      urdf, joints_, Eigen::Map<const Eigen::VectorXd>(armature.data(), static_cast<Eigen::Index>(armature.size())));
    law_ = make_law(*chain_, *dynamics_, settings);
  } catch (const std::invalid_argument & error) {
    RCLCPP_FATAL(node->get_logger(), "invalid configuration: %s", error.what());
    return CallbackReturn::ERROR;
  }

  const auto n = static_cast<Eigen::Index>(joints_.size());
  q_ = Eigen::VectorXd::Zero(n);
  qd_ = Eigen::VectorXd::Zero(n);
  tau_ = Eigen::VectorXd::Zero(n);

  target_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/target_pose", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::PoseStamped & msg) { on_target(msg); });
  error_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(
    node->create_publisher<geometry_msgs::msg::TwistStamped>("~/pose_error", rclcpp::SystemDefaultsQoS()));
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate);
  RCLCPP_INFO(node->get_logger(), "configured for %zu joints, %s -> %s, gravity compensation %s",
              joints_.size(), base_frame_.c_str(), ee_frame.c_str(), settings.compensate_gravity ? "on" : "off");
  return CallbackReturn::SUCCESS;
}

void TaskSpaceController::on_target(const geometry_msgs::msg::PoseStamped & msg)
{
  if (!msg.header.frame_id.empty() && msg.header.frame_id != base_frame_) {
    RCLCPP_WARN(get_node()->get_logger(), "ignoring target in frame '%s' (expected '%s')",
                msg.header.frame_id.c_str(), base_frame_.c_str());
    return;
  }
  Eigen::Isometry3d target;
  tf2::fromMsg(msg.pose, target);
  target_.writeFromNonRT(target);
}

void TaskSpaceController::read_state()
{
  const std::size_t n = joints_.size();
  for (std::size_t i = 0; i < n; ++i) {
    q_[static_cast<Eigen::Index>(i)] = state_interfaces_[i].get_value();
    qd_[static_cast<Eigen::Index>(i)] = state_interfaces_[n + i].get_value();
  }
}

CallbackReturn TaskSpaceController::on_activate(const rclcpp_lifecycle::State &)
{
  // Hold where the arm is now until a target arrives.
  read_state();
  target_.writeFromNonRT(chain_->fk(q_));
  last_publish_time_ = get_node()->now();
  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & command : command_interfaces_) {
    command.set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type TaskSpaceController::update(const rclcpp::Time & time, const rclcpp::Duration &)
{
  read_state();
  law_->compute(q_, qd_, *target_.readFromRT(), tau_);
  for (std::size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_interfaces_[i].set_value(tau_[static_cast<Eigen::Index>(i)]);
  }

  if (time - last_publish_time_ >= publish_period_ && error_publisher_->trylock()) {
    last_publish_time_ = time;
    auto & msg = error_publisher_->msg_;
    msg.header.stamp = time;
    msg.header.frame_id = base_frame_;
    const Vector6d & error = law_->pose_error();
    msg.twist.linear.x = error[0];
    msg.twist.linear.y = error[1];
    msg.twist.linear.z = error[2];
    msg.twist.angular.x = error[3];
    msg.twist.angular.y = error[4];
    msg.twist.angular.z = error[5];
    error_publisher_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

void OscController::declare_gains()
{
  auto_declare<double>("position_stiffness", 0.0);
  auto_declare<double>("orientation_stiffness", 0.0);
  auto_declare<double>("damping_ratio", 0.0);
}

std::unique_ptr<TaskSpaceControl> OscController::make_law(
  const arm_sandbox_kinematics::KinematicChain & chain, RobotDynamics & dynamics, const TaskSpaceSettings & settings)
{
  OperationalSpaceGains gains;
  gains.position_stiffness = get_node()->get_parameter("position_stiffness").as_double();
  gains.orientation_stiffness = get_node()->get_parameter("orientation_stiffness").as_double();
  gains.damping_ratio = get_node()->get_parameter("damping_ratio").as_double();
  return std::make_unique<OperationalSpaceControl>(chain, dynamics, settings, gains);
}

void CartesianImpedanceController::declare_gains()
{
  auto_declare<double>("translational_stiffness", 0.0);
  auto_declare<double>("rotational_stiffness", 0.0);
  auto_declare<double>("damping_ratio", 0.0);
}

std::unique_ptr<TaskSpaceControl> CartesianImpedanceController::make_law(
  const arm_sandbox_kinematics::KinematicChain & chain, RobotDynamics & dynamics, const TaskSpaceSettings & settings)
{
  ImpedanceGains gains;
  gains.translational_stiffness = get_node()->get_parameter("translational_stiffness").as_double();
  gains.rotational_stiffness = get_node()->get_parameter("rotational_stiffness").as_double();
  gains.damping_ratio = get_node()->get_parameter("damping_ratio").as_double();
  return std::make_unique<CartesianImpedanceControl>(chain, dynamics, settings, gains);
}

}  // namespace arm_sandbox_controllers

PLUGINLIB_EXPORT_CLASS(arm_sandbox_controllers::OscController, controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(arm_sandbox_controllers::CartesianImpedanceController, controller_interface::ControllerInterface)
