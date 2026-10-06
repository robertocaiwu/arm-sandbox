// The allocation-free overloads used by the real-time controllers (Plan 04): same results as the
// allocating versions, and no heap allocation. Built with EIGEN_RUNTIME_NO_MALLOC and without
// NDEBUG, so Eigen aborts on any allocation while it is disallowed.

#include <gtest/gtest.h>

#include "arm_sandbox_kinematics/kinematic_chain.hpp"
#include "robot_fixture.hpp"

namespace arm_sandbox_kinematics::test
{
namespace
{
constexpr int kNumRandomConfigurations = 20;
constexpr unsigned kSeed = 3;

class RealtimeTest : public ::testing::Test
{
protected:
  RealtimeTest() : config_(robot_config()), chain_(robot_urdf(), config_.base_frame, config_.ee_frame) {}

  RobotConfig config_;
  KinematicChain chain_;
};

TEST_F(RealtimeTest, GuardCatchesAllocations)
{
  // Positive control: if this didn't abort, the no-allocation test below would prove nothing.
  EXPECT_DEATH(
    {
      Eigen::internal::set_is_malloc_allowed(false);
      Eigen::VectorXd allocates(16);
      allocates.setZero();
    },
    "");
}

TEST_F(RealtimeTest, OverloadsMatchAndDoNotAllocate)
{
  RandomConfigurations random(chain_, kSeed);
  Eigen::Isometry3d pose;
  Jacobian jacobian(6, static_cast<Eigen::Index>(chain_.num_joints()));
  for (int sample = 0; sample < kNumRandomConfigurations; ++sample) {
    const Eigen::VectorXd q = random.next();
    Eigen::internal::set_is_malloc_allowed(false);
    chain_.fk(q, pose);
    chain_.jacobian(q, jacobian);
    Eigen::internal::set_is_malloc_allowed(true);
    EXPECT_TRUE(pose.isApprox(chain_.fk(q)));
    EXPECT_TRUE(jacobian.isApprox(chain_.jacobian(q)));
  }
}

TEST_F(RealtimeTest, RejectsWrongSizeOutput)
{
  Jacobian wrong(6, 2);
  EXPECT_THROW(chain_.jacobian(config_.home, wrong), std::invalid_argument);
}

}  // namespace
}  // namespace arm_sandbox_kinematics::test
