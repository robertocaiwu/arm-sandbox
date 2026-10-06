// RobotDynamics: the Pinocchio model reduced to the arm joints, with armature on M's diagonal.
// (That it matches MuJoCo's dynamics is checked in arm_sandbox_bringup/test/test_dynamics_model.py.)

#include <gtest/gtest.h>

#include "arm_sandbox_controllers/robot_dynamics.hpp"
#include "control_fixture.hpp"

namespace arm_sandbox_controllers::test
{
namespace
{
constexpr double kArmature = 0.1;
constexpr double kTolerance = 1e-12;

TEST(RobotDynamics, ReducedToTheArmJoints)
{
  const RobotConfig config = robot_config();
  const auto n = static_cast<Eigen::Index>(config.arm_joints.size());
  RobotDynamics dynamics(robot_urdf(), config.arm_joints, Eigen::VectorXd::Constant(n, kArmature));
  EXPECT_EQ(dynamics.num_joints(), config.arm_joints.size());
  EXPECT_TRUE((dynamics.effort_limits().array() > 0.0).all());
}

TEST(RobotDynamics, ArmatureIsAddedToTheMassMatrixDiagonal)
{
  const RobotConfig config = robot_config();
  const auto n = static_cast<Eigen::Index>(config.arm_joints.size());
  RobotDynamics with_armature(robot_urdf(), config.arm_joints, Eigen::VectorXd::Constant(n, kArmature));
  RobotDynamics without_armature(robot_urdf(), config.arm_joints, Eigen::VectorXd::Zero(n));
  const Eigen::VectorXd qd = Eigen::VectorXd::Zero(n);
  with_armature.update(config.home, qd);
  without_armature.update(config.home, qd);

  const Eigen::MatrixXd & mass = with_armature.mass_matrix();
  EXPECT_TRUE(mass.isApprox(mass.transpose(), kTolerance));
  EXPECT_GT(mass.ldlt().vectorD().minCoeff(), 0.0);  // positive definite
  const Eigen::MatrixXd difference = mass - without_armature.mass_matrix();
  EXPECT_TRUE(difference.isApprox(Eigen::MatrixXd(Eigen::VectorXd::Constant(n, kArmature).asDiagonal()), kTolerance));
  // At rest, C q' + g is just g.
  EXPECT_TRUE(with_armature.nonlinear_effects().isApprox(with_armature.gravity(), kTolerance));
}

TEST(RobotDynamics, RejectsBadInput)
{
  const RobotConfig config = robot_config();
  const auto n = static_cast<Eigen::Index>(config.arm_joints.size());
  const Eigen::VectorXd armature = Eigen::VectorXd::Constant(n, kArmature);
  EXPECT_THROW(RobotDynamics("<robot", config.arm_joints, armature), std::invalid_argument);
  EXPECT_THROW(RobotDynamics(robot_urdf(), {"no_such_joint"}, armature), std::invalid_argument);
  std::vector<std::string> reversed(config.arm_joints.rbegin(), config.arm_joints.rend());
  EXPECT_THROW(RobotDynamics(robot_urdf(), reversed, armature), std::invalid_argument);
  EXPECT_THROW(RobotDynamics(robot_urdf(), config.arm_joints, Eigen::VectorXd::Zero(2)), std::invalid_argument);
}

}  // namespace
}  // namespace arm_sandbox_controllers::test
