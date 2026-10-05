// `compute` of both control laws never allocates. Built with EIGEN_RUNTIME_NO_MALLOC and without
// NDEBUG, with the control-core sources compiled into this test, so Eigen aborts on any heap
// allocation while it is disallowed. (KinematicChain's overloads have the same check in
// arm_sandbox_kinematics.)

#include <gtest/gtest.h>

#include "arm_sandbox_controllers/task_space_control.hpp"
#include "control_fixture.hpp"

namespace arm_sandbox_controllers::test
{
namespace
{
constexpr int kNumCycles = 100;

TaskSpaceSettings settings(const Eigen::VectorXd & rest)
{
  TaskSpaceSettings s;
  s.max_position_error = 0.1;
  s.max_orientation_error = 0.5;
  s.rest_configuration = rest;
  s.nullspace_stiffness = 10.0;
  s.nullspace_damping = 2.0;
  s.singularity_damping = 1e-3;
  return s;
}

TEST(Realtime, GuardCatchesAllocations)
{
  EXPECT_DEATH(
    {
      Eigen::internal::set_is_malloc_allowed(false);
      Eigen::VectorXd allocates(16);
      allocates.setZero();
    },
    "");
}

TEST(Realtime, ComputeDoesNotAllocate)
{
  const RobotConfig config = robot_config();
  const auto n = static_cast<Eigen::Index>(config.arm_joints.size());
  const arm_sandbox_kinematics::KinematicChain chain(robot_urdf(), config.base_frame, config.ee_frame);
  RobotDynamics dynamics(robot_urdf(), config.arm_joints, Eigen::VectorXd::Constant(n, 0.1));
  OperationalSpaceControl osc(chain, dynamics, settings(config.home), {400.0, 400.0, 1.0});
  CartesianImpedanceControl impedance(chain, dynamics, settings(config.home), {500.0, 50.0, 1.0});

  Eigen::Isometry3d target = chain.fk(config.home);
  target.translation().x() += 0.05;
  const Eigen::VectorXd qd = Eigen::VectorXd::Constant(n, 0.1);
  Eigen::VectorXd tau(n);
  for (int cycle = 0; cycle < kNumCycles; ++cycle) {
    Eigen::internal::set_is_malloc_allowed(false);
    osc.compute(config.home, qd, target, tau);
    impedance.compute(config.home, qd, target, tau);
    Eigen::internal::set_is_malloc_allowed(true);
  }
  EXPECT_TRUE(tau.allFinite());
}

}  // namespace
}  // namespace arm_sandbox_controllers::test
