#include "arm_sandbox_controllers/task_space_control.hpp"

#include <cmath>
#include <stdexcept>

namespace arm_sandbox_controllers
{
namespace
{
/// Scales `vector` down to `max_norm` if it is longer (keeps its direction).
template <typename Vector>
void clamp_norm(Vector && vector, double max_norm)
{
  const double norm = vector.norm();
  if (norm > max_norm) {
    vector *= max_norm / norm;
  }
}

Vector6d critical_damping(const Vector6d & stiffness, double damping_ratio)
{
  return 2.0 * damping_ratio * stiffness.cwiseSqrt();
}
}  // namespace

TaskSpaceControl::TaskSpaceControl(
  const arm_sandbox_kinematics::KinematicChain & chain, RobotDynamics & dynamics, TaskSpaceSettings settings)
: chain_(chain), dynamics_(dynamics), settings_(std::move(settings))
{
  const auto n = static_cast<Eigen::Index>(chain_.num_joints());
  if (dynamics_.num_joints() != chain_.num_joints()) {
    throw std::invalid_argument("TaskSpaceControl: kinematics and dynamics have different joint counts");
  }
  if (settings_.rest_configuration.size() != n) {
    throw std::invalid_argument("TaskSpaceControl: rest_configuration needs one value per joint");
  }
  if (settings_.max_position_error <= 0.0 || settings_.max_orientation_error <= 0.0 ||
      settings_.singularity_damping <= 0.0 || settings_.nullspace_stiffness < 0.0 || settings_.nullspace_damping < 0.0) {
    throw std::invalid_argument(
      "TaskSpaceControl: max errors and singularity_damping must be > 0, null-space gains >= 0");
  }
  jacobian_.resize(6, n);
  mass_ldlt_ = Eigen::LDLT<Eigen::MatrixXd>(n);
  minv_jt_.resize(n, 6);
  jbar_.resize(n, 6);
  tau_null_.resize(n);
  projected_null_.resize(n);
}

void TaskSpaceControl::compute(
  const Eigen::VectorXd & q, const Eigen::VectorXd & qd, const Eigen::Isometry3d & target, Eigen::VectorXd & tau)
{
  const auto n = static_cast<Eigen::Index>(chain_.num_joints());
  if (q.size() != n || qd.size() != n || tau.size() != n) {
    throw std::invalid_argument("TaskSpaceControl::compute: q, qd and tau need one value per joint");
  }

  // Kinematics (hand-written, arm_sandbox_kinematics) and dynamics (Pinocchio).
  chain_.fk(q, pose_);
  chain_.jacobian(q, jacobian_);
  dynamics_.update(q, qd);

  // Pose error [p* - p; angle * axis of R* R^T], clamped; end-effector twist J q'.
  error_.head<3>() = target.translation() - pose_.translation();
  const Eigen::AngleAxisd rotation_error(target.linear() * pose_.linear().transpose());
  error_.tail<3>() = rotation_error.angle() * rotation_error.axis();
  clamp_norm(error_.head<3>(), settings_.max_position_error);
  clamp_norm(error_.tail<3>(), settings_.max_orientation_error);
  twist_.noalias() = jacobian_.lazyProduct(qd);

  // Task-space inertia Lambda = (J M^-1 J^T + eps I)^-1.
  mass_ldlt_.compute(dynamics_.mass_matrix());
  minv_jt_ = jacobian_.transpose();
  mass_ldlt_.solveInPlace(minv_jt_);
  Matrix6d lambda_inverse = jacobian_.lazyProduct(minv_jt_);
  lambda_inverse.diagonal().array() += settings_.singularity_damping;
  lambda_inverse_ldlt_.compute(lambda_inverse);
  task_inertia_ = lambda_inverse_ldlt_.solve(Matrix6d::Identity());

  task_force(error_, twist_, task_inertia_, force_);

  // Null-space task, projected with N^T = I - J^T Jbar^T.
  tau_null_ = settings_.nullspace_stiffness * (settings_.rest_configuration - q) - settings_.nullspace_damping * qd;
  jbar_.noalias() = minv_jt_.lazyProduct(task_inertia_);
  // Two steps through a fixed-size 6-vector: a nested product would evaluate into a heap temporary.
  jbar_t_tau_null_.noalias() = jbar_.transpose() * tau_null_;
  projected_null_ = tau_null_;
  projected_null_.noalias() -= jacobian_.transpose() * jbar_t_tau_null_;

  // tau = J^T F + N^T tau_0 + C q' (+ g).
  tau.noalias() = jacobian_.transpose().lazyProduct(force_);
  tau += projected_null_ + dynamics_.nonlinear_effects();
  if (!settings_.compensate_gravity) {
    tau -= dynamics_.gravity();
  }
  tau = tau.cwiseMax(-dynamics_.effort_limits()).cwiseMin(dynamics_.effort_limits());
}

OperationalSpaceControl::OperationalSpaceControl(
  const arm_sandbox_kinematics::KinematicChain & chain, RobotDynamics & dynamics, TaskSpaceSettings settings,
  OperationalSpaceGains gains)
: TaskSpaceControl(chain, dynamics, std::move(settings))
{
  if (gains.position_stiffness <= 0.0 || gains.orientation_stiffness <= 0.0 || gains.damping_ratio <= 0.0) {
    throw std::invalid_argument("OperationalSpaceControl: stiffnesses and damping_ratio must be > 0");
  }
  stiffness_ << Eigen::Vector3d::Constant(gains.position_stiffness), Eigen::Vector3d::Constant(gains.orientation_stiffness);
  damping_ = critical_damping(stiffness_, gains.damping_ratio);
}

void OperationalSpaceControl::task_force(
  const Vector6d & error, const Vector6d & twist, const Matrix6d & task_inertia, Vector6d & force) const
{
  // A desired acceleration of a unit mass, scaled by the real task-space inertia.
  const Vector6d acceleration = stiffness_.cwiseProduct(error) - damping_.cwiseProduct(twist);
  force.noalias() = task_inertia * acceleration;
}

CartesianImpedanceControl::CartesianImpedanceControl(
  const arm_sandbox_kinematics::KinematicChain & chain, RobotDynamics & dynamics, TaskSpaceSettings settings,
  ImpedanceGains gains)
: TaskSpaceControl(chain, dynamics, std::move(settings))
{
  if (gains.translational_stiffness <= 0.0 || gains.rotational_stiffness <= 0.0 || gains.damping_ratio <= 0.0) {
    throw std::invalid_argument("CartesianImpedanceControl: stiffnesses and damping_ratio must be > 0");
  }
  stiffness_ << Eigen::Vector3d::Constant(gains.translational_stiffness), Eigen::Vector3d::Constant(gains.rotational_stiffness);
  damping_ = critical_damping(stiffness_, gains.damping_ratio);
}

void CartesianImpedanceControl::task_force(
  const Vector6d & error, const Vector6d & twist, const Matrix6d & /*task_inertia*/, Vector6d & force) const
{
  // A spring-damper: no inertia shaping, so contact forces stay those of the chosen stiffness.
  force = stiffness_.cwiseProduct(error) - damping_.cwiseProduct(twist);
}

}  // namespace arm_sandbox_controllers
