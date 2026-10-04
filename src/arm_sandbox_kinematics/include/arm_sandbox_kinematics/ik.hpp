#pragma once

#include <optional>

#include <Eigen/Geometry>

#include "arm_sandbox_kinematics/kinematic_chain.hpp"

namespace arm_sandbox_kinematics
{

/// Settings for `solve_ik`. No defaults on purpose: callers load them from config
/// (CLAUDE.md: no magic numbers). `solve_ik` throws std::invalid_argument if one is invalid.
struct IkOptions
{
  int max_iterations = 0;
  double position_tolerance = 0.0;     ///< m
  double orientation_tolerance = 0.0;  ///< rad
  /// Damping lambda used at a singularity (manipulability 0), > 0. Scaled down as the arm moves
  /// away from singularities, and zero once manipulability exceeds `manipulability_threshold`.
  double max_damping = 0.0;
  double manipulability_threshold = 0.0;  ///< > 0; see KinematicChain::manipulability
  double max_step = 0.0;               ///< largest joint change per iteration (rad or m), > 0
  /// Optional secondary task in the null space of the pose task: pull the joints towards this
  /// configuration (e.g. home) without disturbing the tip pose. Uses `null_space_gain`.
  std::optional<Eigen::VectorXd> null_space_target;
  double null_space_gain = 0.0;        ///< in [0, 1]
};

struct IkResult
{
  Eigen::VectorXd q;               ///< best configuration found, always within joint limits
  bool converged = false;          ///< both errors below their tolerances
  double position_error = 0.0;     ///< m, at `q`
  double orientation_error = 0.0;  ///< rad, at `q`
  int iterations = 0;
};

/// Damped-least-squares inverse kinematics: find q with fk(q) == target, starting at `q_seed`.
///
/// Each iteration takes the pose error e = [p* - p; angle-axis(R* R^T)] and the step
/// dq = J^T (J J^T + lambda^2 I)^-1 e. Far from singularities lambda = 0 (Gauss-Newton, fast);
/// as manipulability w drops below the threshold w0, lambda^2 = max_damping^2 (1 - (w/w0)^2)
/// keeps the step bounded where the plain pseudo-inverse would explode.
/// With a null-space target, the step also gets gain * N (q0 - q), where N = I - V_r V_r^T
/// (from the SVD of J) moves the joints without moving the tip.
/// Steps are limited to `max_step` and the result is clamped to the joint limits.
IkResult solve_ik(
  const KinematicChain & chain, const Eigen::Isometry3d & target, const Eigen::VectorXd & q_seed,
  const IkOptions & options);

}  // namespace arm_sandbox_kinematics
