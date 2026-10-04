#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace arm_sandbox_kinematics
{

/// 6 x n geometric Jacobian. Rows 0-2: linear velocity of the tip origin, rows 3-5: angular
/// velocity, both expressed in the base frame.
using Jacobian = Eigen::Matrix<double, 6, Eigen::Dynamic>;

/// Serial kinematic chain from `base_link` to `tip_link`, built from a URDF.
///
/// Forward kinematics is the product of each joint's fixed origin transform and its motion
/// (rotation about / translation along the joint axis). Fixed joints are folded into the
/// origins, so the chain's variables are only its revolute, continuous and prismatic joints,
/// in order from base to tip. ROS-free: needs only Eigen and urdfdom.
class KinematicChain
{
public:
  /// Throws std::invalid_argument if the URDF doesn't parse, a link is missing, `tip_link`
  /// isn't below `base_link`, or the chain contains a joint type other than revolute,
  /// continuous, prismatic or fixed (or a mimic joint).
  KinematicChain(const std::string & urdf_xml, const std::string & base_link, const std::string & tip_link);

  std::size_t num_joints() const { return joints_.size(); }
  /// Moving joints, from base to tip. `q` vectors use this order.
  const std::vector<std::string> & joint_names() const { return joint_names_; }
  /// URDF position limits. Continuous joints get -inf/+inf.
  const Eigen::VectorXd & lower_limits() const { return lower_limits_; }
  const Eigen::VectorXd & upper_limits() const { return upper_limits_; }
  /// URDF velocity limits (rad/s or m/s).
  const Eigen::VectorXd & velocity_limits() const { return velocity_limits_; }

  /// Pose of the tip frame in the base frame. Throws std::invalid_argument on a wrong-size `q`.
  Eigen::Isometry3d fk(const Eigen::VectorXd & q) const;

  /// Geometric Jacobian of the tip origin in the base frame (see `Jacobian`).
  Jacobian jacobian(const Eigen::VectorXd & q) const;

  /// Yoshikawa's manipulability, sqrt(det(J J^T)). Zero at a singularity.
  double manipulability(const Eigen::VectorXd & q) const;

private:
  enum class JointType { kRevolute, kPrismatic };

  struct Joint
  {
    Eigen::Isometry3d origin;  ///< parent link -> joint frame, including preceding fixed joints
    Eigen::Vector3d axis;      ///< unit axis in the joint frame
    JointType type;
  };

  void check_size(const Eigen::VectorXd & q) const;

  std::vector<Joint> joints_;
  Eigen::Isometry3d tip_offset_;  ///< fixed joints after the last moving joint
  std::vector<std::string> joint_names_;
  Eigen::VectorXd lower_limits_;
  Eigen::VectorXd upper_limits_;
  Eigen::VectorXd velocity_limits_;
};

}  // namespace arm_sandbox_kinematics
