#include "arm_sandbox_kinematics/kinematic_chain.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <urdf_parser/urdf_parser.h>

namespace arm_sandbox_kinematics
{
namespace
{
Eigen::Isometry3d to_isometry(const urdf::Pose & pose)
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
  pose.rotation.getQuaternion(x, y, z, w);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = Eigen::Quaterniond(w, x, y, z).normalized().toRotationMatrix();
  transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  return transform;
}
}  // namespace

KinematicChain::KinematicChain(
  const std::string & urdf_xml, const std::string & base_link, const std::string & tip_link)
{
  const urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(urdf_xml);
  if (!model) {
    throw std::invalid_argument("KinematicChain: the URDF does not parse");
  }
  if (!model->getLink(base_link)) {
    throw std::invalid_argument("KinematicChain: base link '" + base_link + "' not in the URDF");
  }
  if (!model->getLink(tip_link)) {
    throw std::invalid_argument("KinematicChain: tip link '" + tip_link + "' not in the URDF");
  }

  // Walk from the tip up to the base, then reverse: URDF links only know their parent.
  std::vector<urdf::JointConstSharedPtr> path;
  for (urdf::LinkConstSharedPtr link = model->getLink(tip_link); link->name != base_link;
       link = model->getLink(link->parent_joint->parent_link_name)) {
    if (!link->parent_joint) {
      throw std::invalid_argument(
        "KinematicChain: '" + tip_link + "' is not below '" + base_link + "'");
    }
    path.push_back(link->parent_joint);
  }
  std::reverse(path.begin(), path.end());

  // Fixed joints are folded into the next moving joint's origin (or the tip offset).
  Eigen::Isometry3d pending = Eigen::Isometry3d::Identity();
  std::vector<double> lower;
  std::vector<double> upper;
  std::vector<double> velocity;
  for (const auto & joint : path) {
    pending = pending * to_isometry(joint->parent_to_joint_origin_transform);
    if (joint->type == urdf::Joint::FIXED) {
      continue;
    }
    if (joint->mimic) {
      throw std::invalid_argument("KinematicChain: mimic joint '" + joint->name + "' in the chain");
    }

    JointType type = JointType::kRevolute;
    double low = -std::numeric_limits<double>::infinity();
    double high = std::numeric_limits<double>::infinity();
    switch (joint->type) {
      case urdf::Joint::CONTINUOUS:
        break;
      case urdf::Joint::REVOLUTE:
        low = joint->limits->lower;
        high = joint->limits->upper;
        break;
      case urdf::Joint::PRISMATIC:
        type = JointType::kPrismatic;
        low = joint->limits->lower;
        high = joint->limits->upper;
        break;
      default:
        throw std::invalid_argument(
          "KinematicChain: joint '" + joint->name + "' has an unsupported type");
    }

    const Eigen::Vector3d axis(joint->axis.x, joint->axis.y, joint->axis.z);
    joints_.push_back(Joint{pending, axis.normalized(), type});
    joint_names_.push_back(joint->name);
    lower.push_back(low);
    upper.push_back(high);
    velocity.push_back(joint->limits ? joint->limits->velocity : std::numeric_limits<double>::infinity());
    pending = Eigen::Isometry3d::Identity();
  }
  tip_offset_ = pending;

  lower_limits_ = Eigen::Map<const Eigen::VectorXd>(lower.data(), static_cast<Eigen::Index>(lower.size()));
  upper_limits_ = Eigen::Map<const Eigen::VectorXd>(upper.data(), static_cast<Eigen::Index>(upper.size()));
  velocity_limits_ =
    Eigen::Map<const Eigen::VectorXd>(velocity.data(), static_cast<Eigen::Index>(velocity.size()));
}

void KinematicChain::check_size(const Eigen::VectorXd & q) const
{
  if (static_cast<std::size_t>(q.size()) != joints_.size()) {
    throw std::invalid_argument(
      "KinematicChain: expected " + std::to_string(joints_.size()) + " joint values, got " +
      std::to_string(q.size()));
  }
}

Eigen::Isometry3d KinematicChain::fk(const Eigen::VectorXd & q) const
{
  check_size(q);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const Joint & joint = joints_[i];
    const double value = q[static_cast<Eigen::Index>(i)];
    transform = transform * joint.origin;
    if (joint.type == JointType::kRevolute) {
      transform.rotate(Eigen::AngleAxisd(value, joint.axis));
    } else {
      transform.translate(value * joint.axis);
    }
  }
  return transform * tip_offset_;
}

Jacobian KinematicChain::jacobian(const Eigen::VectorXd & q) const
{
  check_size(q);
  // Joint axes and origins in the base frame, then the tip position.
  std::vector<Eigen::Vector3d> axes(joints_.size());
  std::vector<Eigen::Vector3d> origins(joints_.size());
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const Joint & joint = joints_[i];
    const double value = q[static_cast<Eigen::Index>(i)];
    transform = transform * joint.origin;
    axes[i] = transform.linear() * joint.axis;
    origins[i] = transform.translation();
    if (joint.type == JointType::kRevolute) {
      transform.rotate(Eigen::AngleAxisd(value, joint.axis));
    } else {
      transform.translate(value * joint.axis);
    }
  }
  const Eigen::Vector3d tip = (transform * tip_offset_).translation();

  Jacobian jacobian(6, static_cast<Eigen::Index>(joints_.size()));
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto column = static_cast<Eigen::Index>(i);
    if (joints_[i].type == JointType::kRevolute) {
      // A rotation about axis z through point p moves the tip with z x (tip - p).
      jacobian.block<3, 1>(0, column) = axes[i].cross(tip - origins[i]);
      jacobian.block<3, 1>(3, column) = axes[i];
    } else {
      jacobian.block<3, 1>(0, column) = axes[i];
      jacobian.block<3, 1>(3, column) = Eigen::Vector3d::Zero();
    }
  }
  return jacobian;
}

double KinematicChain::manipulability(const Eigen::VectorXd & q) const
{
  const Jacobian j = jacobian(q);
  return std::sqrt(std::max(0.0, (j * j.transpose()).determinant()));
}

}  // namespace arm_sandbox_kinematics
