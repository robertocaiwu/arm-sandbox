// Damped-least-squares IK: FK(IK(FK(q))) round trips, joint limits, the null-space task, and
// failure behavior. Targets come from FK of random configurations, so they are reachable.

#include <gtest/gtest.h>

#include <cstdio>

#include "arm_sandbox_kinematics/ik.hpp"
#include "arm_sandbox_kinematics/kinematic_chain.hpp"
#include "robot_fixture.hpp"

namespace arm_sandbox_kinematics::test
{
namespace
{
constexpr int kNumTargets = 200;
constexpr unsigned kSeed = 7;
constexpr double kSeedPerturbationRad = 0.3;
// Seeding at home is harder: targets anywhere in the workspace, some needing a different arm
// configuration (elbow flipped, base turned around) that a local solver can't reach from home.
// Measured 82.5% (165/200) when this test was written; this floor catches regressions.
constexpr double kMinConvergenceFromHome = 0.80;
constexpr double kUnreachableDistanceM = 2.0;

IkOptions test_options()
{
  IkOptions options;
  options.max_iterations = 500;
  options.position_tolerance = 1e-4;
  options.orientation_tolerance = 1e-3;
  options.max_damping = 0.05;
  options.manipulability_threshold = 0.01;
  options.max_step = 0.2;
  return options;
}

class IkTest : public ::testing::Test
{
protected:
  IkTest() : config_(robot_config()), chain_(robot_urdf(), config_.base_frame, config_.ee_frame) {}

  void expect_valid(const IkResult & result, const Eigen::Isometry3d & target, const IkOptions & options) const
  {
    EXPECT_TRUE(result.converged);
    const Eigen::Isometry3d reached = chain_.fk(result.q);
    EXPECT_LT((reached.translation() - target.translation()).norm(), options.position_tolerance);
    EXPECT_LT(Eigen::AngleAxisd(reached.linear() * target.linear().transpose()).angle(), options.orientation_tolerance);
  }

  bool within_limits(const Eigen::VectorXd & q) const
  {
    return (q.array() >= chain_.lower_limits().array()).all() && (q.array() <= chain_.upper_limits().array()).all();
  }

  RobotConfig config_;
  KinematicChain chain_;
};

TEST_F(IkTest, RoundTripFromNearbySeed)
{
  const IkOptions options = test_options();
  RandomConfigurations random(chain_, kSeed);
  std::uniform_real_distribution<double> noise(-kSeedPerturbationRad, kSeedPerturbationRad);
  for (int sample = 0; sample < kNumTargets; ++sample) {
    const Eigen::VectorXd q_true = random.next();
    const Eigen::Isometry3d target = chain_.fk(q_true);
    Eigen::VectorXd seed = q_true;
    for (Eigen::Index i = 0; i < seed.size(); ++i) {
      seed[i] += noise(random.generator());
    }
    const IkResult result = solve_ik(chain_, target, seed, options);
    SCOPED_TRACE("sample " + std::to_string(sample));
    expect_valid(result, target, options);
    EXPECT_TRUE(within_limits(result.q));
  }
}

TEST_F(IkTest, MostTargetsConvergeFromHome)
{
  const IkOptions options = test_options();
  RandomConfigurations random(chain_, kSeed);
  int converged = 0;
  for (int sample = 0; sample < kNumTargets; ++sample) {
    const Eigen::Isometry3d target = chain_.fk(random.next());
    const IkResult result = solve_ik(chain_, target, config_.home, options);
    EXPECT_TRUE(within_limits(result.q));
    if (result.converged) {
      ++converged;
      expect_valid(result, target, options);
    }
  }
  const double rate = static_cast<double>(converged) / kNumTargets;
  std::printf("IK from home: %d/%d targets converged (%.1f%%)\n", converged, kNumTargets, 100.0 * rate);
  EXPECT_GE(rate, kMinConvergenceFromHome);
}

TEST_F(IkTest, NullSpaceTaskKeepsSolutionsNearerHome)
{
  IkOptions plain = test_options();
  IkOptions near_home = test_options();
  near_home.null_space_target = config_.home;
  near_home.null_space_gain = 0.1;

  RandomConfigurations random(chain_, kSeed);
  double plain_distance = 0.0;
  double near_home_distance = 0.0;
  int compared = 0;
  for (int sample = 0; sample < kNumTargets; ++sample) {
    const Eigen::Isometry3d target = chain_.fk(random.next());
    const IkResult a = solve_ik(chain_, target, config_.home, plain);
    const IkResult b = solve_ik(chain_, target, config_.home, near_home);
    if (!a.converged || !b.converged) {
      continue;
    }
    plain_distance += (a.q - config_.home).norm();
    near_home_distance += (b.q - config_.home).norm();
    ++compared;
  }
  ASSERT_GT(compared, 0);
  std::printf("mean distance to home: plain %.3f, with null-space task %.3f (%d targets)\n",
              plain_distance / compared, near_home_distance / compared, compared);
  EXPECT_LT(near_home_distance, plain_distance);
}

TEST_F(IkTest, UnreachableTargetFailsInsideLimits)
{
  const IkOptions options = test_options();
  Eigen::Isometry3d target = chain_.fk(config_.home);
  target.translation().x() += kUnreachableDistanceM;
  const IkResult result = solve_ik(chain_, target, config_.home, options);
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.iterations, options.max_iterations);
  EXPECT_GT(result.position_error, options.position_tolerance);
  EXPECT_TRUE(within_limits(result.q));
}

TEST_F(IkTest, RejectsInvalidOptions)
{
  const Eigen::Isometry3d target = chain_.fk(config_.home);
  EXPECT_THROW(solve_ik(chain_, target, config_.home, IkOptions{}), std::invalid_argument);
  EXPECT_THROW(solve_ik(chain_, target, Eigen::VectorXd::Zero(2), test_options()), std::invalid_argument);
  IkOptions bad_gain = test_options();
  bad_gain.null_space_gain = 2.0;
  EXPECT_THROW(solve_ik(chain_, target, config_.home, bad_gain), std::invalid_argument);
  IkOptions bad_target = test_options();
  bad_target.null_space_target = Eigen::VectorXd::Zero(2);
  EXPECT_THROW(solve_ik(chain_, target, config_.home, bad_target), std::invalid_argument);
}

}  // namespace
}  // namespace arm_sandbox_kinematics::test
