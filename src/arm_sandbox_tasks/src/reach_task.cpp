#include "arm_sandbox_tasks/reach_task.hpp"

#include <algorithm>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace arm_sandbox_tasks
{
namespace
{
constexpr const char * kSupportedSuccessType = "ee_at_pose";

Eigen::Vector3d vector3(const YAML::Node & node, const std::string & what)
{
  if (!node || !node.IsSequence() || node.size() != 3) {
    throw std::invalid_argument(what + " must be a list of 3 numbers");
  }
  return Eigen::Vector3d(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
}

double positive(const YAML::Node & node, const std::string & what)
{
  if (!node) {
    throw std::invalid_argument(what + " is missing");
  }
  const double value = node.as<double>();
  if (value <= 0.0) {
    throw std::invalid_argument(what + " must be > 0");
  }
  return value;
}
}  // namespace

ReachTask load_reach_task(const std::string & path)
{
  YAML::Node yaml;
  try {
    yaml = YAML::LoadFile(path);
  } catch (const YAML::Exception & error) {
    throw std::invalid_argument("reach task " + path + ": " + error.what());
  }

  try {
    ReachTask task;
    task.name = yaml["name"] ? yaml["name"].as<std::string>() : "";
    if (task.name.empty()) {
      throw std::invalid_argument("name is missing");
    }
    const YAML::Node success = yaml["success"];
    if (!success || !success["type"] || success["type"].as<std::string>() != kSupportedSuccessType) {
      throw std::invalid_argument(std::string("success.type must be '") + kSupportedSuccessType + "'");
    }
    task.tolerance.position = positive(success["position_tolerance"], "success.position_tolerance");
    task.tolerance.orientation = positive(success["orientation_tolerance"], "success.orientation_tolerance");
    task.hold_s = positive(success["hold_s"], "success.hold_s");
    task.time_limit_s = positive(yaml["time_limit_s"], "time_limit_s");

    const YAML::Node targets = yaml["targets"];
    if (!targets || !targets.IsSequence() || targets.size() == 0) {
      throw std::invalid_argument("targets must be a non-empty list");
    }
    for (std::size_t i = 0; i < targets.size(); ++i) {
      const std::string where = "targets[" + std::to_string(i) + "]";
      task.targets.push_back(pose_from_xyz_rpy(
        vector3(targets[i]["position"], where + ".position"), vector3(targets[i]["rpy"], where + ".rpy")));
    }
    return task;
  } catch (const YAML::Exception & error) {
    throw std::invalid_argument("reach task " + path + ": " + error.what());
  } catch (const std::invalid_argument & error) {
    throw std::invalid_argument("reach task " + path + ": " + error.what());
  }
}

Eigen::Isometry3d pose_from_xyz_rpy(const Eigen::Vector3d & position, const Eigen::Vector3d & rpy)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
                    .toRotationMatrix();
  pose.translation() = position;
  return pose;
}

PoseError pose_error(const Eigen::Isometry3d & target, const Eigen::Isometry3d & actual)
{
  PoseError error;
  error.position = (target.translation() - actual.translation()).norm();
  error.orientation = Eigen::AngleAxisd(target.linear() * actual.linear().transpose()).angle();
  return error;
}

bool within(const PoseError & error, const PoseTolerance & tolerance)
{
  return error.position <= tolerance.position && error.orientation <= tolerance.orientation;
}

double move_duration(
  const Eigen::VectorXd & from, const Eigen::VectorXd & to, const Eigen::VectorXd & velocity_limits,
  double velocity_scale, double min_duration_s)
{
  if (from.size() != to.size() || from.size() != velocity_limits.size()) {
    throw std::invalid_argument("move_duration: vectors must have the same size");
  }
  if (velocity_scale <= 0.0 || (velocity_limits.array() <= 0.0).any()) {
    throw std::invalid_argument("move_duration: velocity_scale and velocity limits must be > 0");
  }
  const double slowest =
    ((to - from).cwiseAbs().array() / (velocity_scale * velocity_limits.array())).maxCoeff();
  return std::max(slowest, min_duration_s);
}

}  // namespace arm_sandbox_tasks
