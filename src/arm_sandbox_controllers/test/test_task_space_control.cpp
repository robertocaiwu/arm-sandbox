// The task-space control laws in closed loop with a simulated arm (Pinocchio forward dynamics at
// the controller rate): they reach targets, the impedance law behaves like a spring, gravity
// compensation switches, torques stay within limits, and `compute` never allocates.

#include <gtest/gtest.h>

#include <memory>

#include "arm_sandbox_controllers/task_space_control.hpp"
#include "arm_sandbox_kinematics/kinematic_chain.hpp"
#include "control_fixture.hpp"

namespace arm_sandbox_controllers::test
{
namespace
{
constexpr double kDt = 0.001;  // 1 kHz, as in the sim
constexpr double kArmature = 0.1;
constexpr double kSettleTimeS = 3.0;
constexpr double kPositionToleranceM = 1e-3;
constexpr double kOrientationToleranceRad = 1e-2;
constexpr double kPushForceN = 10.0;
constexpr double kSpringTolerance = 0.05;  // 5 % of the expected deflection

TaskSpaceSettings test_settings(const Eigen::VectorXd & rest)
{
  TaskSpaceSettings settings;
  settings.max_position_error = 0.1;
  settings.max_orientation_error = 0.5;
  settings.rest_configuration = rest;
  settings.nullspace_stiffness = 10.0;
  settings.nullspace_damping = 2.0;
  settings.singularity_damping = 1e-3;
  settings.compensate_gravity = true;
  return settings;
}

OperationalSpaceGains osc_gains() { return {400.0, 400.0, 1.0}; }
ImpedanceGains impedance_gains() { return {500.0, 50.0, 1.0}; }

class TaskSpaceControlTest : public ::testing::Test
{
protected:
  TaskSpaceControlTest()
  : config_(robot_config()),
    n_(static_cast<Eigen::Index>(config_.arm_joints.size())),
    armature_(Eigen::VectorXd::Constant(n_, kArmature)),
    chain_(robot_urdf(), config_.base_frame, config_.ee_frame),
    dynamics_(robot_urdf(), config_.arm_joints, armature_)
  {
  }

  /// Target offset from the home pose by a few cm and a rotation about the world z axis.
  Eigen::Isometry3d offset_target() const
  {
    Eigen::Isometry3d target = chain_.fk(config_.home);
    target.translation() += Eigen::Vector3d(0.05, -0.05, 0.05);
    target.linear() = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix() * target.linear();
    return target;
  }

  /// Runs `control` against the plant for `seconds`, with an optional external force at the tip.
  Plant simulate(TaskSpaceControl & control, const Eigen::Isometry3d & target, double seconds,
                 const Eigen::Vector3d & external_force = Eigen::Vector3d::Zero())
  {
    Plant plant(robot_urdf(), config_.arm_joints, armature_, config_.home);
    Eigen::VectorXd tau(n_);
    for (int step = 0; step < static_cast<int>(seconds / kDt); ++step) {
      control.compute(plant.q, plant.qd, target, tau);
      const arm_sandbox_kinematics::Jacobian j = chain_.jacobian(plant.q);
      plant.step(tau + j.topRows<3>().transpose() * external_force, kDt);
    }
    return plant;
  }

  void expect_at(const Plant & plant, const Eigen::Isometry3d & target) const
  {
    const Eigen::Isometry3d reached = chain_.fk(plant.q);
    EXPECT_LT((reached.translation() - target.translation()).norm(), kPositionToleranceM);
    EXPECT_LT(Eigen::AngleAxisd(reached.linear() * target.linear().transpose()).angle(), kOrientationToleranceRad);
  }

  RobotConfig config_;
  Eigen::Index n_;
  Eigen::VectorXd armature_;
  arm_sandbox_kinematics::KinematicChain chain_;
  RobotDynamics dynamics_;
};

TEST_F(TaskSpaceControlTest, OperationalSpaceReachesTarget)
{
  OperationalSpaceControl control(chain_, dynamics_, test_settings(config_.home), osc_gains());
  const Eigen::Isometry3d target = offset_target();
  expect_at(simulate(control, target, kSettleTimeS), target);
}

TEST_F(TaskSpaceControlTest, ImpedanceReachesTarget)
{
  CartesianImpedanceControl control(chain_, dynamics_, test_settings(config_.home), impedance_gains());
  const Eigen::Isometry3d target = offset_target();
  expect_at(simulate(control, target, kSettleTimeS), target);
}

TEST_F(TaskSpaceControlTest, ImpedanceDeflectsLikeASpring)
{
  // A steady push F on the end effector settles where the spring balances it: x = F / k.
  const ImpedanceGains gains = impedance_gains();
  CartesianImpedanceControl control(chain_, dynamics_, test_settings(config_.home), gains);
  const Eigen::Isometry3d target = chain_.fk(config_.home);
  const Eigen::Vector3d push(kPushForceN, 0.0, 0.0);
  const Plant plant = simulate(control, target, kSettleTimeS, push);
  const Eigen::Vector3d deflection = chain_.fk(plant.q).translation() - target.translation();
  const double expected = kPushForceN / gains.translational_stiffness;
  EXPECT_NEAR(deflection.x(), expected, kSpringTolerance * expected);
  EXPECT_LT(deflection.tail<2>().norm(), kSpringTolerance * expected);
}

TEST_F(TaskSpaceControlTest, GravityCompensationIsSwitchable)
{
  // At the target and at rest, the only torque left is gravity, if the controller compensates it.
  const Eigen::Isometry3d target = chain_.fk(config_.home);
  const Eigen::VectorXd qd = Eigen::VectorXd::Zero(n_);
  Eigen::VectorXd tau(n_);
  TaskSpaceSettings settings = test_settings(config_.home);

  CartesianImpedanceControl compensating(chain_, dynamics_, settings, impedance_gains());
  compensating.compute(config_.home, qd, target, tau);
  dynamics_.update(config_.home, qd);
  EXPECT_TRUE(tau.isApprox(dynamics_.gravity(), 1e-9));

  settings.compensate_gravity = false;
  CartesianImpedanceControl not_compensating(chain_, dynamics_, settings, impedance_gains());
  not_compensating.compute(config_.home, qd, target, tau);
  EXPECT_LT(tau.norm(), 1e-9);
}

TEST_F(TaskSpaceControlTest, TorquesStayWithinEffortLimits)
{
  OperationalSpaceGains stiff = osc_gains();
  stiff.position_stiffness = 1e6;
  stiff.orientation_stiffness = 1e6;
  OperationalSpaceControl control(chain_, dynamics_, test_settings(config_.home), stiff);
  Eigen::VectorXd tau(n_);
  control.compute(config_.home, Eigen::VectorXd::Zero(n_), offset_target(), tau);
  EXPECT_TRUE((tau.cwiseAbs().array() <= dynamics_.effort_limits().array()).all());
  EXPECT_TRUE((tau.cwiseAbs().array() == dynamics_.effort_limits().array()).any());  // it did saturate
}

TEST_F(TaskSpaceControlTest, RejectsInvalidSettings)
{
  TaskSpaceSettings wrong_rest = test_settings(Eigen::VectorXd::Zero(2));
  EXPECT_THROW(OperationalSpaceControl(chain_, dynamics_, wrong_rest, osc_gains()), std::invalid_argument);
  TaskSpaceSettings no_clamp = test_settings(config_.home);
  no_clamp.max_position_error = 0.0;
  EXPECT_THROW(CartesianImpedanceControl(chain_, dynamics_, no_clamp, impedance_gains()), std::invalid_argument);
  EXPECT_THROW(OperationalSpaceControl(chain_, dynamics_, test_settings(config_.home), OperationalSpaceGains{}), std::invalid_argument);
  EXPECT_THROW(CartesianImpedanceControl(chain_, dynamics_, test_settings(config_.home), ImpedanceGains{}), std::invalid_argument);

  OperationalSpaceControl control(chain_, dynamics_, test_settings(config_.home), osc_gains());
  Eigen::VectorXd wrong_tau(2);
  EXPECT_THROW(control.compute(config_.home, Eigen::VectorXd::Zero(n_), offset_target(), wrong_tau), std::invalid_argument);
}

}  // namespace
}  // namespace arm_sandbox_controllers::test
