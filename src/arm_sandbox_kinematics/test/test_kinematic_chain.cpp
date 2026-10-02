// KinematicChain (FK, Jacobian, manipulability) against Pinocchio on the same URDF, and the
// Jacobian against finite differences of our own FK.

#include <gtest/gtest.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "arm_sandbox_kinematics/kinematic_chain.hpp"
#include "robot_fixture.hpp"

namespace arm_sandbox_kinematics::test
{
namespace
{
constexpr int kNumRandomConfigurations = 100;
constexpr unsigned kSeed = 42;
constexpr double kExactTolerance = 1e-9;      // same model, same math: only rounding differs
constexpr double kFiniteDifferenceStep = 1e-6;
constexpr double kFiniteDifferenceTolerance = 1e-6;
constexpr double kSingularManipulability = 1e-9;

class KinematicChainTest : public ::testing::Test
{
protected:
  KinematicChainTest()
  : config_(robot_config()), chain_(robot_urdf(), config_.base_frame, config_.ee_frame)
  {
    pinocchio::urdf::buildModelFromXML(robot_urdf(), model_);
    data_ = pinocchio::Data(model_);
    ee_frame_id_ = model_.getFrameId(config_.ee_frame);
  }

  /// Our chain's q embedded in Pinocchio's full configuration (other joints, e.g. fingers, at 0).
  Eigen::VectorXd to_pinocchio(const Eigen::VectorXd & q) const
  {
    Eigen::VectorXd full = pinocchio::neutral(model_);
    for (std::size_t i = 0; i < chain_.num_joints(); ++i) {
      const auto joint_id = model_.getJointId(chain_.joint_names()[i]);
      full[model_.joints[joint_id].idx_q()] = q[static_cast<Eigen::Index>(i)];
    }
    return full;
  }

  RobotConfig config_;
  KinematicChain chain_;
  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex ee_frame_id_ = 0;
};

TEST_F(KinematicChainTest, JointsAndLimitsMatchRobotConfig)
{
  EXPECT_EQ(chain_.joint_names(), config_.arm_joints);
  for (std::size_t i = 0; i < chain_.num_joints(); ++i) {
    const auto index = static_cast<Eigen::Index>(i);
    EXPECT_LT(chain_.lower_limits()[index], chain_.upper_limits()[index]);
    EXPECT_GT(chain_.velocity_limits()[index], 0.0);
  }
}

TEST_F(KinematicChainTest, ForwardKinematicsMatchesPinocchio)
{
  RandomConfigurations random(chain_, kSeed);
  for (int sample = 0; sample < kNumRandomConfigurations; ++sample) {
    const Eigen::VectorXd q = random.next();
    pinocchio::framesForwardKinematics(model_, data_, to_pinocchio(q));
    const Eigen::Isometry3d pose = chain_.fk(q);
    const pinocchio::SE3 & expected = data_.oMf[ee_frame_id_];
    EXPECT_TRUE(pose.translation().isApprox(expected.translation(), kExactTolerance)) << "sample " << sample;
    EXPECT_TRUE(pose.linear().isApprox(expected.rotation(), kExactTolerance)) << "sample " << sample;
  }
}

TEST_F(KinematicChainTest, JacobianMatchesPinocchio)
{
  RandomConfigurations random(chain_, kSeed);
  for (int sample = 0; sample < kNumRandomConfigurations; ++sample) {
    const Eigen::VectorXd q = random.next();
    // LOCAL_WORLD_ALIGNED: velocity of the frame origin, in base axes, [linear; angular].
    pinocchio::Data::Matrix6x full(6, model_.nv);
    full.setZero();
    pinocchio::computeFrameJacobian(model_, data_, to_pinocchio(q), ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, full);
    Jacobian expected(6, static_cast<Eigen::Index>(chain_.num_joints()));
    for (std::size_t i = 0; i < chain_.num_joints(); ++i) {
      const auto joint_id = model_.getJointId(chain_.joint_names()[i]);
      expected.col(static_cast<Eigen::Index>(i)) = full.col(model_.joints[joint_id].idx_v());
    }
    EXPECT_TRUE(chain_.jacobian(q).isApprox(expected, kExactTolerance)) << "sample " << sample;
  }
}

TEST_F(KinematicChainTest, JacobianMatchesFiniteDifferences)
{
  RandomConfigurations random(chain_, kSeed);
  for (int sample = 0; sample < kNumRandomConfigurations; ++sample) {
    const Eigen::VectorXd q = random.next();
    const Jacobian jacobian = chain_.jacobian(q);
    for (Eigen::Index i = 0; i < q.size(); ++i) {
      Eigen::VectorXd plus = q;
      Eigen::VectorXd minus = q;
      plus[i] += kFiniteDifferenceStep;
      minus[i] -= kFiniteDifferenceStep;
      const Eigen::Isometry3d pose_plus = chain_.fk(plus);
      const Eigen::Isometry3d pose_minus = chain_.fk(minus);
      const Eigen::Vector3d linear =
        (pose_plus.translation() - pose_minus.translation()) / (2.0 * kFiniteDifferenceStep);
      const Eigen::AngleAxisd rotation(pose_plus.linear() * pose_minus.linear().transpose());
      const Eigen::Vector3d angular = rotation.angle() * rotation.axis() / (2.0 * kFiniteDifferenceStep);
      EXPECT_LT((jacobian.block<3, 1>(0, i) - linear).norm(), kFiniteDifferenceTolerance) << "joint " << i;
      EXPECT_LT((jacobian.block<3, 1>(3, i) - angular).norm(), kFiniteDifferenceTolerance) << "joint " << i;
    }
  }
}

TEST_F(KinematicChainTest, ManipulabilityIsZeroWhenStretchedOut)
{
  // All joints at zero: the arm points straight up and several joint axes line up, so the
  // Jacobian loses rank. (The pose is outside joint 4's limits; FK doesn't care.)
  const Eigen::VectorXd stretched = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(chain_.num_joints()));
  EXPECT_LT(chain_.manipulability(stretched), kSingularManipulability);
  EXPECT_GT(chain_.manipulability(config_.home), kSingularManipulability);
}

TEST_F(KinematicChainTest, RejectsBadInput)
{
  const std::string urdf = robot_urdf();
  EXPECT_THROW(KinematicChain("<robot", config_.base_frame, config_.ee_frame), std::invalid_argument);
  EXPECT_THROW(KinematicChain(urdf, "no_such_link", config_.ee_frame), std::invalid_argument);
  EXPECT_THROW(KinematicChain(urdf, config_.base_frame, "no_such_link"), std::invalid_argument);
  EXPECT_THROW(KinematicChain(urdf, config_.ee_frame, config_.base_frame), std::invalid_argument);
  EXPECT_THROW(chain_.fk(Eigen::VectorXd::Zero(2)), std::invalid_argument);
  EXPECT_THROW(chain_.jacobian(Eigen::VectorXd::Zero(2)), std::invalid_argument);
}

}  // namespace
}  // namespace arm_sandbox_kinematics::test
