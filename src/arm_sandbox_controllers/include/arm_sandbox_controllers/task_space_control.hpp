#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include "arm_sandbox_controllers/robot_dynamics.hpp"
#include "arm_sandbox_kinematics/kinematic_chain.hpp"

namespace arm_sandbox_controllers
{

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

/// Settings shared by both task-space controllers. All come from the controller YAML.
struct TaskSpaceSettings
{
  /// The pose error fed to the law is clamped to these norms, so a far-away target gives a bounded
  /// pull instead of a huge force (the arm then moves at a bounded rate towards it).
  double max_position_error = 0.0;     ///< m, > 0
  double max_orientation_error = 0.0;  ///< rad, > 0
  /// Null-space task: a joint-space spring towards `rest_configuration` (e.g. home), projected so
  /// it doesn't disturb the end-effector. Keeps the redundant 7th degree of freedom from drifting.
  Eigen::VectorXd rest_configuration;
  double nullspace_stiffness = 0.0;  ///< N m / rad, >= 0
  double nullspace_damping = 0.0;    ///< N m s / rad, >= 0
  /// Regularizes the task-space inertia (J M^-1 J^T + eps I)^-1 near singularities, > 0.
  double singularity_damping = 0.0;
  /// Add g(q). False when the robot (or the sim with gravcomp:=true) already compensates gravity.
  bool compensate_gravity = true;
};

/// Shared part of the task-space controllers: tau = J^T F + N^T tau_0 + C q' (+ g), clamped to the
/// effort limits. F is the task-space force computed by the derived law; tau_0 is the null-space
/// task; N^T = I - J^T Jbar^T projects it with the dynamically consistent inverse
/// Jbar = M^-1 J^T Lambda, Lambda = (J M^-1 J^T)^-1. ROS-free. `compute` never allocates.
class TaskSpaceControl
{
public:
  virtual ~TaskSpaceControl() = default;

  /// Torques that drive the end effector towards `target` (pose in the chain's base frame).
  /// `tau` must have num_joints() entries. Throws std::invalid_argument on wrong sizes.
  void compute(const Eigen::VectorXd & q, const Eigen::VectorXd & qd, const Eigen::Isometry3d & target, Eigen::VectorXd & tau);

  /// Clamped pose error [position; orientation] from the last `compute`.
  const Vector6d & pose_error() const { return error_; }
  /// End-effector pose from the last `compute`.
  const Eigen::Isometry3d & ee_pose() const { return pose_; }
  std::size_t num_joints() const { return chain_.num_joints(); }

protected:
  /// Throws std::invalid_argument if the settings are invalid or don't match the chain/dynamics.
  TaskSpaceControl(const arm_sandbox_kinematics::KinematicChain & chain, RobotDynamics & dynamics, TaskSpaceSettings settings);

  /// The law: task-space force F from the pose error and the end-effector twist J q'.
  /// `task_inertia` is Lambda. Must not allocate.
  virtual void task_force(const Vector6d & error, const Vector6d & twist, const Matrix6d & task_inertia, Vector6d & force) const = 0;

private:
  const arm_sandbox_kinematics::KinematicChain & chain_;
  RobotDynamics & dynamics_;
  TaskSpaceSettings settings_;

  // Preallocated workspace (sized in the constructor).
  Eigen::Isometry3d pose_;
  arm_sandbox_kinematics::Jacobian jacobian_;
  Vector6d error_;
  Vector6d twist_;
  Vector6d force_;
  Eigen::LDLT<Eigen::MatrixXd> mass_ldlt_;
  Eigen::MatrixXd minv_jt_;          // M^-1 J^T, n x 6
  Matrix6d task_inertia_;            // Lambda
  Eigen::LDLT<Matrix6d> lambda_inverse_ldlt_;
  Eigen::MatrixXd jbar_;             // M^-1 J^T Lambda, n x 6
  Eigen::VectorXd tau_null_;
  Vector6d jbar_t_tau_null_;         // Jbar^T tau_0
  Eigen::VectorXd projected_null_;
};

/// Operational-space control (Khatib): F = Lambda (Kp e - Kd x'), with Kd = 2 zeta sqrt(Kp).
/// Lambda makes the end effector behave like a unit mass in every direction, so the gains give
/// the same, decoupled response everywhere in the workspace: stiff, accurate pose tracking.
struct OperationalSpaceGains
{
  double position_stiffness = 0.0;     ///< 1/s^2, > 0
  double orientation_stiffness = 0.0;  ///< 1/s^2, > 0
  double damping_ratio = 0.0;          ///< > 0, 1 = critical
};

class OperationalSpaceControl : public TaskSpaceControl
{
public:
  OperationalSpaceControl(
    const arm_sandbox_kinematics::KinematicChain & chain, RobotDynamics & dynamics, TaskSpaceSettings settings,
    OperationalSpaceGains gains);

private:
  void task_force(const Vector6d & error, const Vector6d & twist, const Matrix6d & task_inertia, Vector6d & force) const override;

  Vector6d stiffness_;
  Vector6d damping_;
};

/// Cartesian impedance control: F = K e - D x', a spring-damper between the end effector and the
/// target, D = 2 zeta sqrt(K). Pushing the end effector by F_ext displaces it by about K^-1 F_ext:
/// compliant contact, e.g. pulling a drawer whose path doesn't match the target exactly.
struct ImpedanceGains
{
  double translational_stiffness = 0.0;  ///< N/m, > 0
  double rotational_stiffness = 0.0;     ///< N m / rad, > 0
  double damping_ratio = 0.0;            ///< > 0
};

class CartesianImpedanceControl : public TaskSpaceControl
{
public:
  CartesianImpedanceControl(
    const arm_sandbox_kinematics::KinematicChain & chain, RobotDynamics & dynamics, TaskSpaceSettings settings,
    ImpedanceGains gains);

private:
  void task_force(const Vector6d & error, const Vector6d & twist, const Matrix6d & task_inertia, Vector6d & force) const override;

  Vector6d stiffness_;
  Vector6d damping_;
};

}  // namespace arm_sandbox_controllers
