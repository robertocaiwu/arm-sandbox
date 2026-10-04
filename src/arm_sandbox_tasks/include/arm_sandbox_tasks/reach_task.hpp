#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace arm_sandbox_tasks
{

/// How close the end effector must get to a target pose.
struct PoseTolerance
{
  double position = 0.0;     ///< m
  double orientation = 0.0;  ///< rad
};

/// The reach task (REQUIREMENTS FR-15, task 1): move the end effector to each target in turn.
/// Loaded from `config/tasks/reach.yaml`. ROS-free, so the eval runner and the Gym env can use it.
struct ReachTask
{
  std::string name;
  PoseTolerance tolerance;
  double hold_s = 0.0;                            ///< must stay within tolerance this long, > 0
  double time_limit_s = 0.0;                      ///< per target
  std::vector<Eigen::Isometry3d> targets;         ///< in the robot's base frame
};

/// Parse and validate a reach task file. Throws std::invalid_argument with the reason (and the
/// file name) if a field is missing or invalid, e.g. an unsupported `success.type`.
ReachTask load_reach_task(const std::string & path);

/// Pose from a position and fixed-axis roll/pitch/yaw (rotate about x, then y, then z), the
/// URDF convention.
Eigen::Isometry3d pose_from_xyz_rpy(const Eigen::Vector3d & position, const Eigen::Vector3d & rpy);

struct PoseError
{
  double position = 0.0;     ///< m, distance between the origins
  double orientation = 0.0;  ///< rad, angle of the rotation from `actual` to `target`
};

PoseError pose_error(const Eigen::Isometry3d & target, const Eigen::Isometry3d & actual);

bool within(const PoseError & error, const PoseTolerance & tolerance);

/// Duration for a straight joint-space move so that no joint exceeds `velocity_scale` times its
/// velocity limit (average speed), but never shorter than `min_duration_s`.
double move_duration(
  const Eigen::VectorXd & from, const Eigen::VectorXd & to, const Eigen::VectorXd & velocity_limits,
  double velocity_scale, double min_duration_s);

}  // namespace arm_sandbox_tasks
