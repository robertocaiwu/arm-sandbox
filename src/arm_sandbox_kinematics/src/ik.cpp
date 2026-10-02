#include "arm_sandbox_kinematics/ik.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/SVD>

namespace arm_sandbox_kinematics
{
namespace
{
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

void validate(const KinematicChain & chain, const Eigen::VectorXd & q_seed, const IkOptions & options)
{
  if (static_cast<std::size_t>(q_seed.size()) != chain.num_joints()) {
    throw std::invalid_argument("solve_ik: q_seed has the wrong size");
  }
  if (options.max_iterations <= 0 || options.position_tolerance <= 0.0 ||
      options.orientation_tolerance <= 0.0 || options.max_damping <= 0.0 ||
      options.manipulability_threshold <= 0.0 || options.max_step <= 0.0) {
    throw std::invalid_argument(
      "solve_ik: max_iterations, tolerances, max_damping, manipulability_threshold and max_step "
      "must be > 0");
  }
  if (options.null_space_gain < 0.0 || options.null_space_gain > 1.0) {
    throw std::invalid_argument("solve_ik: null_space_gain must be in [0, 1]");
  }
  if (options.null_space_target &&
      static_cast<std::size_t>(options.null_space_target->size()) != chain.num_joints()) {
    throw std::invalid_argument("solve_ik: null_space_target has the wrong size");
  }
}

/// [position error; orientation error as angle * axis], both in the base frame.
Vector6d pose_error(const Eigen::Isometry3d & target, const Eigen::Isometry3d & current)
{
  const Eigen::AngleAxisd rotation_error(target.linear() * current.linear().transpose());
  Vector6d error;
  error.head<3>() = target.translation() - current.translation();
  error.tail<3>() = rotation_error.angle() * rotation_error.axis();
  return error;
}

/// Squared damping: zero away from singularities, rising to max_damping^2 at manipulability 0.
double damping_squared(const Jacobian & j, const IkOptions & options)
{
  const double manipulability = std::sqrt(std::max(0.0, (j * j.transpose()).determinant()));
  if (manipulability >= options.manipulability_threshold) {
    return 0.0;
  }
  const double ratio = manipulability / options.manipulability_threshold;
  return options.max_damping * options.max_damping * (1.0 - ratio * ratio);
}

/// Projector onto the null space of J: joint motions that don't move the tip. Built from the
/// SVD so it is exact and bounded even near singularities.
Eigen::MatrixXd null_space_projector(const Jacobian & j)
{
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(j, Eigen::ComputeFullV);
  const Eigen::Index rank = svd.rank();
  const Eigen::MatrixXd range = svd.matrixV().leftCols(rank);
  return Eigen::MatrixXd::Identity(j.cols(), j.cols()) - range * range.transpose();
}

/// One damped-least-squares step for pose error `error`, plus the optional null-space task.
/// Damped pseudo-inverse J^T (J J^T + lambda^2 I)^-1, solved rather than inverted.
Eigen::VectorXd damped_step(
  const Jacobian & j, const Vector6d & error, const IkOptions & options, const Eigen::VectorXd & q)
{
  const Matrix6d jjt_damped = j * j.transpose() + damping_squared(j, options) * Matrix6d::Identity();
  Eigen::VectorXd step = j.transpose() * jjt_damped.ldlt().solve(error);
  if (options.null_space_target) {
    step += options.null_space_gain * null_space_projector(j) * (*options.null_space_target - q);
  }
  return step;
}

Eigen::VectorXd clamp_to_limits(const KinematicChain & chain, const Eigen::VectorXd & q)
{
  return q.cwiseMax(chain.lower_limits()).cwiseMin(chain.upper_limits());
}
}  // namespace

IkResult solve_ik(
  const KinematicChain & chain, const Eigen::Isometry3d & target, const Eigen::VectorXd & q_seed,
  const IkOptions & options)
{
  validate(chain, q_seed, options);

  IkResult result;
  result.q = clamp_to_limits(chain, q_seed);
  for (result.iterations = 0;; ++result.iterations) {
    const Vector6d error = pose_error(target, chain.fk(result.q));
    result.position_error = error.head<3>().norm();
    result.orientation_error = error.tail<3>().norm();
    result.converged = result.position_error < options.position_tolerance &&
                       result.orientation_error < options.orientation_tolerance;
    if (result.converged || result.iterations == options.max_iterations) {
      return result;
    }

    // Joint clamping: a joint sitting at a limit whose step points out of range can't help, and
    // the clamp below would throw its share of the step away. Drop its Jacobian column and solve
    // again, so the other joints take over. At most one pass per joint.
    Jacobian j = chain.jacobian(result.q);
    std::vector<bool> locked(chain.num_joints(), false);
    Eigen::VectorXd step;
    for (std::size_t pass = 0; pass <= chain.num_joints(); ++pass) {
      step = damped_step(j, error, options, result.q);
      for (std::size_t i = 0; i < locked.size(); ++i) {
        if (locked[i]) {
          step[static_cast<Eigen::Index>(i)] = 0.0;
        }
      }
      bool newly_locked = false;
      for (std::size_t i = 0; i < locked.size(); ++i) {
        const auto k = static_cast<Eigen::Index>(i);
        const bool pushes_below = result.q[k] <= chain.lower_limits()[k] && step[k] < 0.0;
        const bool pushes_above = result.q[k] >= chain.upper_limits()[k] && step[k] > 0.0;
        if (!locked[i] && (pushes_below || pushes_above)) {
          locked[i] = true;
          j.col(k).setZero();
          newly_locked = true;
        }
      }
      if (!newly_locked) {
        break;
      }
    }

    const double largest = step.cwiseAbs().maxCoeff();
    if (largest > options.max_step) {
      step *= options.max_step / largest;
    }
    result.q = clamp_to_limits(chain, result.q + step);
  }
}

}  // namespace arm_sandbox_kinematics
