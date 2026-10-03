// ROS-free reach task logic: task file parsing and validation, pose errors, move timing.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

#include "arm_sandbox_tasks/reach_task.hpp"

namespace arm_sandbox_tasks
{
namespace
{
constexpr double kTolerance = 1e-12;

/// Writes `contents` to a temporary task file and returns its path.
std::string write_task(const std::string & contents)
{
  const std::string path = ::testing::TempDir() + "reach_task_test.yaml";
  std::ofstream(path) << contents;
  return path;
}

const char * const kValidTask = R"(
name: reach
success: {type: ee_at_pose, position_tolerance: 0.005, orientation_tolerance: 0.035, hold_s: 0.5}
time_limit_s: 30.0
targets:
  - {position: [0.4, 0.1, 0.3], rpy: [3.14159265, 0.0, 0.5]}
)";

TEST(ReachTask, ShippedTaskFileLoads)
{
  const ReachTask task = load_reach_task(REACH_TASK_FILE);
  EXPECT_EQ(task.name, "reach");
  EXPECT_FALSE(task.targets.empty());
  EXPECT_GT(task.tolerance.position, 0.0);
  EXPECT_GT(task.time_limit_s, 0.0);
}

TEST(ReachTask, ParsesTargetPose)
{
  const ReachTask task = load_reach_task(write_task(kValidTask));
  ASSERT_EQ(task.targets.size(), 1u);
  EXPECT_TRUE(task.targets[0].translation().isApprox(Eigen::Vector3d(0.4, 0.1, 0.3)));
  EXPECT_DOUBLE_EQ(task.tolerance.position, 0.005);
  EXPECT_DOUBLE_EQ(task.tolerance.orientation, 0.035);
  EXPECT_DOUBLE_EQ(task.hold_s, 0.5);
}

TEST(ReachTask, RejectsInvalidFiles)
{
  EXPECT_THROW(load_reach_task("/no/such/file.yaml"), std::invalid_argument);
  std::string wrong_type = kValidTask;
  wrong_type.replace(wrong_type.find("ee_at_pose"), 10, "object_in_region");
  EXPECT_THROW(load_reach_task(write_task(wrong_type)), std::invalid_argument);
  std::string bad_position = kValidTask;
  bad_position.replace(bad_position.find("[0.4, 0.1, 0.3]"), 15, "[0.4, 0.1]");
  EXPECT_THROW(load_reach_task(write_task(bad_position)), std::invalid_argument);
  std::string zero_tolerance = kValidTask;
  zero_tolerance.replace(zero_tolerance.find("0.005"), 5, "0.0");
  EXPECT_THROW(load_reach_task(write_task(zero_tolerance)), std::invalid_argument);
  EXPECT_THROW(load_reach_task(write_task("name: reach\n")), std::invalid_argument);
}

TEST(ReachTask, RpyFollowsUrdfConvention)
{
  // Fixed axes x, then y, then z: R = Rz(yaw) Ry(pitch) Rx(roll).
  const Eigen::Isometry3d down = pose_from_xyz_rpy(Eigen::Vector3d::Zero(), Eigen::Vector3d(M_PI, 0.0, 0.0));
  EXPECT_TRUE((down.linear() * Eigen::Vector3d::UnitZ()).isApprox(-Eigen::Vector3d::UnitZ()));
  const Eigen::Isometry3d yawed = pose_from_xyz_rpy(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, M_PI / 2));
  EXPECT_TRUE((yawed.linear() * Eigen::Vector3d::UnitX()).isApprox(Eigen::Vector3d::UnitY()));
  // Two axes at once, so the order matters: Rz(90) Rx(90) maps y to +z; the reversed order
  // Rx(90) Rz(90) would map it to -x. Single-axis cases can't tell the two apart.
  const Eigen::Isometry3d rolled_and_yawed =
    pose_from_xyz_rpy(Eigen::Vector3d::Zero(), Eigen::Vector3d(M_PI / 2, 0.0, M_PI / 2));
  EXPECT_TRUE((rolled_and_yawed.linear() * Eigen::Vector3d::UnitY()).isApprox(Eigen::Vector3d::UnitZ()));
}

TEST(ReachTask, PoseErrorAndTolerance)
{
  const Eigen::Isometry3d target = pose_from_xyz_rpy(Eigen::Vector3d(0.4, 0.0, 0.3), Eigen::Vector3d(M_PI, 0.0, 0.0));
  Eigen::Isometry3d actual = target;
  actual.translation().x() += 0.003;
  actual.rotate(Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitZ()));
  const PoseError error = pose_error(target, actual);
  EXPECT_NEAR(error.position, 0.003, kTolerance);
  EXPECT_NEAR(error.orientation, 0.02, kTolerance);
  EXPECT_TRUE(within(error, PoseTolerance{0.005, 0.035}));
  EXPECT_FALSE(within(error, PoseTolerance{0.002, 0.035}));
  EXPECT_FALSE(within(error, PoseTolerance{0.005, 0.01}));
}

TEST(ReachTask, MoveDurationFollowsTheSlowestJoint)
{
  const Eigen::VectorXd from = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd to(2);
  to << 1.0, 0.5;
  Eigen::VectorXd limits(2);
  limits << 2.0, 0.5;
  // Joint 2: 0.5 rad at 0.5 * 0.5 rad/s = 2 s; joint 1: 1 rad at 1 rad/s = 1 s.
  EXPECT_NEAR(move_duration(from, to, limits, 0.5, 0.1), 2.0, kTolerance);
  EXPECT_NEAR(move_duration(from, from, limits, 0.5, 0.1), 0.1, kTolerance);  // minimum
  EXPECT_THROW(move_duration(from, to, limits, 0.0, 0.1), std::invalid_argument);
  EXPECT_THROW(move_duration(from, Eigen::VectorXd::Zero(3), limits, 0.5, 0.1), std::invalid_argument);
}

}  // namespace
}  // namespace arm_sandbox_tasks
