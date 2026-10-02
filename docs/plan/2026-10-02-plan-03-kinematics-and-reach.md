# Plan 03 — Kinematics (M2) and the Reach Task

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A hand-written, ROS-free C++ kinematics library (FK, geometric Jacobian, damped-least-squares IK with singularity and joint-limit handling, manipulability), checked against Pinocchio, and the reach task solved with it in the running sim. This is milestone M2.

**Architecture:** `arm_sandbox_kinematics` is a plain C++17 library (Eigen + urdfdom, no ROS) that builds a `KinematicChain` from a URDF string, with a free function `solve_ik`. Its GoogleTests compare it against Pinocchio on the real Panda URDF, which CMake generates from the xacro. `arm_sandbox_tasks` adds the reach task: ROS-free task logic (task file, pose error, move timing) plus a thin `reach_runner` node. The node solves each target with the IK, sends the arm controller (JTC) one trajectory goal, and checks the end-effector pose from TF.

**Tech Stack:** C++17, Eigen 3.4, urdfdom 3.0 (`urdf_parser`), Pinocchio 4.0 (test reference only), yaml-cpp, GoogleTest (`ament_cmake_gtest`), `rclcpp` / `rclcpp_action` / `tf2_ros`, `launch_testing`.

**Spec:** [`../specs/2026-10-01-arm-sandbox-design.md`](../specs/2026-10-01-arm-sandbox-design.md), sections "Kinematics" and "Tasks and Executive", and [`../REQUIREMENTS.md`](../REQUIREMENTS.md) (M2, FR-15 task 1, FR-18). Plan 02 ([`2026-10-02-plan-02-sim-bringup-and-viewer.md`](2026-10-02-plan-02-sim-bringup-and-viewer.md)) built the sim this plan drives.

## Global Constraints

- ROS 2 **Humble**, MuJoCo **3.12.0**, `mujoco_ros2_control` **0.1.2** (D7, D13, D14). Nothing new to install: Eigen, urdfdom, Pinocchio, yaml-cpp and GoogleTest are already in the image.
- **Hand-written kinematics, library dynamics** (spec, Decisions). FK, Jacobian and IK are written here; Pinocchio is used only in tests, as the reference (FR-18).
- **Pure library + thin node** (CLAUDE.md). `arm_sandbox_kinematics` and the task logic in `arm_sandbox_tasks` don't include any ROS header. Only `reach_runner.cpp` does.
- **No robot-specific values in code.** Joint names, frames and the home pose come from `robot.yaml`, and the URDF comes from `/robot_description`. Tests read the test robot's `robot.yaml` too.
- **No magic numbers in code.** Solver settings, tolerances, speeds and timeouts live in YAML (`config/reach_runner.yaml`, `config/tasks/reach.yaml`). Tests may define named constants at the top of the test file.
- C++17, `-Wall -Wextra -Wpedantic -Werror`. C++ packages default to `CMAKE_BUILD_TYPE=Release` when none is given (colcon passes none; unoptimized Eigen is about 50× slower here: 0.5 s vs ~25 s for the IK tests).
- All tests are headless. Each `launch_testing` test gets its own `ROS_DOMAIN_ID` (Plan 02 uses 41–44; this plan adds 45).
- User decisions (2026-10-02): the reach runs as a **runner node**, not a `MoveToPose` action (that comes with MoveIt in M4). **Analytic IK is deferred** to a later plan. Both are recorded as D18/D19 in Task 5.

## Where each step runs

All steps run in the dev container (`make shell` or the VS Code dev container), after `source /opt/ros/humble/setup.bash`. After `make test` has built the workspace, also `source install/setup.bash`.

## Before Task 1: branch

Plan 02's work is on `feat/m1-sim-viewer`. Branch from it, or from `master` once it's merged:

```bash
git fetch origin
git switch -c feat/m2-kinematics feat/m1-sim-viewer   # or: origin/master after the Plan 02 PR is merged
```

This plan was executed end to end in a scratch workspace on 2026-10-02 before it was written: `colcon test` gave 63 tests, 0 failures. The numbers quoted below (convergence rates, errors) come from that run.

## File Structure

| Path | Responsibility |
|---|---|
| `src/arm_sandbox_kinematics/include/arm_sandbox_kinematics/kinematic_chain.hpp`, `src/kinematic_chain.cpp` | `KinematicChain`: URDF → chain, FK, geometric Jacobian, manipulability |
| `src/arm_sandbox_kinematics/include/arm_sandbox_kinematics/ik.hpp`, `src/ik.cpp` | `IkOptions`, `IkResult`, `solve_ik` (damped least squares) |
| `src/arm_sandbox_kinematics/test/robot_fixture.hpp` | Test robot URDF + `robot.yaml`, random configurations |
| `src/arm_sandbox_kinematics/test/test_kinematic_chain.cpp`, `test/test_ik.cpp` | GoogleTests, against Pinocchio and finite differences |
| `src/arm_sandbox_tasks/include/arm_sandbox_tasks/reach_task.hpp`, `src/reach_task.cpp` | ROS-free: reach task file, pose error, move duration |
| `src/arm_sandbox_tasks/config/tasks/reach.yaml` | The reach task: targets, success tolerance, hold time, time limit |
| `src/arm_sandbox_tasks/src/reach_runner.cpp`, `config/reach_runner.yaml`, `launch/reach.launch.py` | The ROS runner: IK → JTC → TF check |
| `src/arm_sandbox_tasks/test/test_reach_task.cpp`, `test/test_reach.py` | Task-logic GoogleTest; full-stack launch test |
| `Makefile` | Adds `start_reach` |
| `docs/…`, `CLAUDE.md`, package `README.md`s | Updated in Task 5 |

## Out of scope (later plans)

- Analytic Panda IK (D19), and random-restart / multi-seed IK.
- `MoveToPose` and the other skill actions, `arm_sandbox_interfaces`, the behavior tree, MoveIt 2: M4.
- Dynamics (mass matrix, Coriolis, gravity from Pinocchio) and the custom controllers: M3.
- pybind11 bindings for the Gym environment: Phase B.
- Cartesian straight-line motion. The reach moves in joint space: one JTC point, interpolated by JTC.

---

# Part 1 — Kinematics Library

### Task 1: `KinematicChain` — FK, Jacobian, manipulability, checked against Pinocchio

Theory, short (the package README in Task 5 has the long version):

- **FK** is a product of transforms. For each moving joint i, multiply its fixed origin transform (from the URDF, with any fixed joints before it folded in) by its motion: a rotation of qᵢ about the joint axis (revolute), or a translation of qᵢ along it (prismatic).
- **Geometric Jacobian**, 6×n, in the base frame. For a revolute joint with world axis zᵢ through point pᵢ, the column is [zᵢ × (p_tip − pᵢ); zᵢ]. For a prismatic joint it is [zᵢ; 0].
- **Manipulability**, w = √det(J Jᵀ) (Yoshikawa). It is zero at a singularity.

**Files:**
- Create: `src/arm_sandbox_kinematics/package.xml`, `src/arm_sandbox_kinematics/CMakeLists.txt`
- Create: `src/arm_sandbox_kinematics/include/arm_sandbox_kinematics/kinematic_chain.hpp`, `src/arm_sandbox_kinematics/src/kinematic_chain.cpp`
- Test: `src/arm_sandbox_kinematics/test/robot_fixture.hpp`, `src/arm_sandbox_kinematics/test/test_kinematic_chain.cpp`

**Interfaces:**
- Consumes: `panda/urdf/panda.urdf.xacro` and `panda/config/robot.yaml` from `arm_sandbox_description` (Plan 02), installed under `share/arm_sandbox_description/panda/`.
- Produces (namespace `arm_sandbox_kinematics`, CMake target `arm_sandbox_kinematics::arm_sandbox_kinematics`):
  - `using Jacobian = Eigen::Matrix<double, 6, Eigen::Dynamic>`, with rows [linear; angular].
  - `KinematicChain(const std::string & urdf_xml, const std::string & base_link, const std::string & tip_link)`. It throws `std::invalid_argument` on a bad URDF or link, a tip not below the base, an unsupported joint type, or a mimic joint.
  - Methods: `num_joints()`; `joint_names()`, base to tip; `lower_limits()`, `upper_limits()` and `velocity_limits()` as `Eigen::VectorXd`; `fk(q) -> Eigen::Isometry3d`; `jacobian(q) -> Jacobian`; `manipulability(q) -> double`. All of them throw `std::invalid_argument` on a wrong-size `q`.
  - Test helpers (namespace `arm_sandbox_kinematics::test`): `robot_urdf()`, `robot_config()` returning a `RobotConfig` with `arm_joints`, `base_frame`, `ee_frame` and `home`, and `RandomConfigurations(chain, seed).next()`.

- [ ] **Step 1: Package skeleton**

```bash
mkdir -p src/arm_sandbox_kinematics/{include/arm_sandbox_kinematics,src,test}
```

`src/arm_sandbox_kinematics/package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>arm_sandbox_kinematics</name>
  <version>0.1.0</version>
  <description>ROS-free kinematics for serial arms: FK, geometric Jacobian, damped-least-squares IK, manipulability. Checked against Pinocchio.</description>
  <maintainer email="robertocw_18@hotmail.com">Roberto Cai</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>eigen</depend>
  <depend>urdfdom</depend>

  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>arm_sandbox_description</test_depend>
  <test_depend>pinocchio</test_depend>
  <test_depend>xacro</test_depend>
  <test_depend>yaml_cpp_vendor</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

`src/arm_sandbox_kinematics/CMakeLists.txt`. It already lists `src/ik.cpp` and `test_ik`, which Task 2 adds. To build this task alone, drop `src/ik.cpp` from `add_library` and `test_ik` from the `foreach` list, then put them back in Task 2, Step 1.

```cmake
cmake_minimum_required(VERSION 3.16)
project(arm_sandbox_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -Werror)
# Eigen-heavy math is ~10x slower unoptimized; colcon passes no build type by default.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)

# ROS-free library: only Eigen and urdfdom (design spec "Kinematics").
add_library(${PROJECT_NAME} SHARED src/kinematic_chain.cpp src/ik.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(${PROJECT_NAME} PRIVATE ${urdfdom_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen PRIVATE urdfdom::urdf_parser)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 urdfdom)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(arm_sandbox_description REQUIRED)
  find_package(pinocchio REQUIRED)
  find_package(yaml_cpp_vendor REQUIRED)
  find_program(XACRO_EXECUTABLE xacro REQUIRED)

  # The tests check the library on a real robot: the URDF is generated from the robot's xacro
  # at build time, and names/limits come from its robot.yaml.
  set(TEST_ROBOT panda)
  set(TEST_ROBOT_DIR "${arm_sandbox_description_DIR}/../${TEST_ROBOT}")
  set(TEST_URDF "${CMAKE_CURRENT_BINARY_DIR}/${TEST_ROBOT}.urdf")
  add_custom_command(
    OUTPUT ${TEST_URDF}
    COMMAND ${XACRO_EXECUTABLE} ${TEST_ROBOT_DIR}/urdf/${TEST_ROBOT}.urdf.xacro -o ${TEST_URDF}
    DEPENDS ${TEST_ROBOT_DIR}/urdf/${TEST_ROBOT}.urdf.xacro
    COMMENT "Generating ${TEST_ROBOT}.urdf for the kinematics tests")
  add_custom_target(test_urdf ALL DEPENDS ${TEST_URDF})

  foreach(test_name test_kinematic_chain test_ik)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    add_dependencies(${test_name} test_urdf)
    target_link_libraries(${test_name} ${PROJECT_NAME} pinocchio::pinocchio yaml-cpp)
    target_compile_definitions(${test_name} PRIVATE
      TEST_URDF_PATH="${TEST_URDF}"
      TEST_ROBOT_CONFIG_PATH="${TEST_ROBOT_DIR}/config/robot.yaml")
  endforeach()
endif()

ament_package()
```

The tests run on the real Panda: CMake generates `panda.urdf` from the xacro at build time, and the names come from `robot.yaml`. That makes them the FR-18 cross-check, not a toy model.

- [ ] **Step 2: Write the test fixture and the failing tests**

`src/arm_sandbox_kinematics/test/robot_fixture.hpp`:

```cpp
#pragma once

// Shared by the kinematics tests: the test robot's URDF (generated by CMake from its xacro) and
// its robot.yaml, plus a seeded generator of random configurations inside the joint limits.

#include <fstream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include "arm_sandbox_kinematics/kinematic_chain.hpp"

namespace arm_sandbox_kinematics::test
{

struct RobotConfig
{
  std::vector<std::string> arm_joints;
  std::string base_frame;
  std::string ee_frame;
  Eigen::VectorXd home;
};

inline std::string read_file(const std::string & path)
{
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("cannot read " + path);
  }
  std::stringstream contents;
  contents << file.rdbuf();
  return contents.str();
}

inline std::string robot_urdf() { return read_file(TEST_URDF_PATH); }

inline RobotConfig robot_config()
{
  const YAML::Node yaml = YAML::LoadFile(TEST_ROBOT_CONFIG_PATH);
  RobotConfig config;
  config.arm_joints = yaml["arm_joints"].as<std::vector<std::string>>();
  config.base_frame = yaml["base_frame"].as<std::string>();
  config.ee_frame = yaml["ee_frame"].as<std::string>();
  const auto home = yaml["home"].as<std::vector<double>>();
  config.home = Eigen::Map<const Eigen::VectorXd>(home.data(), static_cast<Eigen::Index>(home.size()));
  return config;
}

/// Uniformly random configurations inside the chain's joint limits, reproducible per seed.
class RandomConfigurations
{
public:
  RandomConfigurations(const KinematicChain & chain, unsigned seed) : chain_(chain), generator_(seed) {}

  Eigen::VectorXd next()
  {
    Eigen::VectorXd q(static_cast<Eigen::Index>(chain_.num_joints()));
    for (Eigen::Index i = 0; i < q.size(); ++i) {
      std::uniform_real_distribution<double> range(chain_.lower_limits()[i], chain_.upper_limits()[i]);
      q[i] = range(generator_);
    }
    return q;
  }

  std::mt19937 & generator() { return generator_; }

private:
  const KinematicChain & chain_;
  std::mt19937 generator_;
};

}  // namespace arm_sandbox_kinematics::test
```

`src/arm_sandbox_kinematics/test/test_kinematic_chain.cpp`:

```cpp
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
```

What each test proves:
- **`ForwardKinematicsMatchesPinocchio` / `JacobianMatchesPinocchio`:** both models read the same URDF, so they must agree to rounding (1e-9). Pinocchio's `LOCAL_WORLD_ALIGNED` Jacobian is the same quantity as ours: the velocity of the frame origin, in base axes, as [linear; angular].
- **`JacobianMatchesFiniteDifferences`:** checks the Jacobian against our own FK, independently of Pinocchio. That catches a wrong sign or a wrong reference point even if both libraries shared a mistake.
- **`ManipulabilityIsZeroWhenStretchedOut`:** at q = 0 the Panda stands straight up with joints 1, 3, 5 and 7 aligned, so the Jacobian loses rank. Home is well-conditioned.

- [ ] **Step 3: Run the tests to see them fail**

Run: `make test`
Expected: the build of `arm_sandbox_kinematics` FAILS with `fatal error: arm_sandbox_kinematics/kinematic_chain.hpp: No such file or directory`.

- [ ] **Step 4: Write `KinematicChain`**

`src/arm_sandbox_kinematics/include/arm_sandbox_kinematics/kinematic_chain.hpp`:

```cpp
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
```

`src/arm_sandbox_kinematics/src/kinematic_chain.cpp`:

```cpp
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
```

- [ ] **Step 5: Run the tests to see them pass**

Run: `make test`
Expected: no compiler warnings. `colcon test-result --verbose` shows `test_kinematic_chain` with 6 tests passed (`./build/arm_sandbox_kinematics/test_kinematic_chain` runs in well under a second).

If a Pinocchio comparison fails, the message names the sample. Compare our `fk` with Pinocchio's `oMf` for that `q` by hand before changing tolerances: the two must agree to 1e-9.

- [ ] **Step 6: Commit**

```bash
git add src/arm_sandbox_kinematics
git commit -m "feat(kinematics): KinematicChain with FK, Jacobian and manipulability, checked against Pinocchio"
```

### Task 2: Damped-least-squares IK with singularity and joint-limit handling

Each iteration computes the pose error e = [p* − p; θ·axis of R* Rᵀ] and takes the step Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e. Three refinements, each found necessary while writing this plan:

1. **Damping scaled by manipulability** (singularity handling, REQUIREMENTS §2). λ is 0 while w ≥ w₀, which gives fast Gauss–Newton steps; the reach targets converge in 4–7 iterations. As w → 0, λ² rises to `max_damping`² · (1 − (w/w₀)²) and keeps the step bounded where the plain pseudo-inverse explodes. A *fixed* λ = 0.05 made every solve slow to converge.
2. **Exact null-space projector.** The secondary task (pull towards home) uses N = I − V_r V_rᵀ from the SVD of J. Built from the damped pseudo-inverse instead, N isn't a true projector, and the pull towards home leaked into the tip pose.
3. **Joint clamping** (Baerlocher & Boulic). A joint sitting at a limit whose step points out of range has its Jacobian column removed, and the step is solved again, so the other joints take over. Without it, 2 of 200 round trips from a nearby seed stalled against a limit.

**Files:**
- Create: `src/arm_sandbox_kinematics/include/arm_sandbox_kinematics/ik.hpp`, `src/arm_sandbox_kinematics/src/ik.cpp`
- Test: `src/arm_sandbox_kinematics/test/test_ik.cpp`

**Interfaces:**
- Consumes: `KinematicChain` and the test fixture (Task 1).
- Produces:
  - `struct IkOptions { int max_iterations; double position_tolerance; double orientation_tolerance; double max_damping; double manipulability_threshold; double max_step; std::optional<Eigen::VectorXd> null_space_target; double null_space_gain; }`
  - `struct IkResult { Eigen::VectorXd q; bool converged; double position_error; double orientation_error; int iterations; }`
  - `IkResult solve_ik(const KinematicChain & chain, const Eigen::Isometry3d & target, const Eigen::VectorXd & q_seed, const IkOptions & options)`. It throws `std::invalid_argument` on invalid options or a wrong-size seed. `result.q` is always within the joint limits.
  - (The spec's `KinematicChain::ik(...)` became this free function, so the chain stays a pure model. Task 5 updates the spec.)

- [ ] **Step 1: Write the failing tests**

If you trimmed the CMake file in Task 1, put `src/ik.cpp` and `test_ik` back now.

`src/arm_sandbox_kinematics/test/test_ik.cpp`:

```cpp
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
```

`MostTargetsConvergeFromHome` characterizes the solver rather than demanding perfection. A local solver started at home can't reach configurations that need the elbow flipped or the base turned around; it converged on 82.5% of random workspace targets when this plan was written. The reach runner seeds each target from the arm's current pose, which is the easy case (see Task 4).

- [ ] **Step 2: Run them to see them fail**

Run: `make test`
Expected: the build FAILS with `fatal error: arm_sandbox_kinematics/ik.hpp: No such file or directory`.

- [ ] **Step 3: Write `solve_ik`**

`src/arm_sandbox_kinematics/include/arm_sandbox_kinematics/ik.hpp`:

```cpp
#pragma once

#include <optional>

#include <Eigen/Geometry>

#include "arm_sandbox_kinematics/kinematic_chain.hpp"

namespace arm_sandbox_kinematics
{

/// Settings for `solve_ik`. No defaults on purpose: callers load them from config
/// (CLAUDE.md: no magic numbers). `solve_ik` throws std::invalid_argument if one is invalid.
struct IkOptions
{
  int max_iterations = 0;
  double position_tolerance = 0.0;     ///< m
  double orientation_tolerance = 0.0;  ///< rad
  /// Damping lambda used at a singularity (manipulability 0), > 0. Scaled down as the arm moves
  /// away from singularities, and zero once manipulability exceeds `manipulability_threshold`.
  double max_damping = 0.0;
  double manipulability_threshold = 0.0;  ///< > 0; see KinematicChain::manipulability
  double max_step = 0.0;               ///< largest joint change per iteration (rad or m), > 0
  /// Optional secondary task in the null space of the pose task: pull the joints towards this
  /// configuration (e.g. home) without disturbing the tip pose. Uses `null_space_gain`.
  std::optional<Eigen::VectorXd> null_space_target;
  double null_space_gain = 0.0;        ///< in [0, 1]
};

struct IkResult
{
  Eigen::VectorXd q;               ///< best configuration found, always within joint limits
  bool converged = false;          ///< both errors below their tolerances
  double position_error = 0.0;     ///< m, at `q`
  double orientation_error = 0.0;  ///< rad, at `q`
  int iterations = 0;
};

/// Damped-least-squares inverse kinematics: find q with fk(q) == target, starting at `q_seed`.
///
/// Each iteration takes the pose error e = [p* - p; angle-axis(R* R^T)] and the step
/// dq = J^T (J J^T + lambda^2 I)^-1 e. Far from singularities lambda = 0 (Gauss-Newton, fast);
/// as manipulability w drops below the threshold w0, lambda^2 = max_damping^2 (1 - (w/w0)^2)
/// keeps the step bounded where the plain pseudo-inverse would explode.
/// With a null-space target, the step also gets gain * N (q0 - q), where N = I - V_r V_r^T
/// (from the SVD of J) moves the joints without moving the tip.
/// Steps are limited to `max_step` and the result is clamped to the joint limits.
IkResult solve_ik(
  const KinematicChain & chain, const Eigen::Isometry3d & target, const Eigen::VectorXd & q_seed,
  const IkOptions & options);

}  // namespace arm_sandbox_kinematics
```

`src/arm_sandbox_kinematics/src/ik.cpp`:

```cpp
#include "arm_sandbox_kinematics/ik.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/SVD>

namespace arm_sandbox_kinematics
{
namespace
{
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

void validate(const KinematicChain & chain, const Eigen::VectorXd & q_seed, const IkOptions & options)
{
  if (static_cast<std::size_t>(q_seed.size()) != chain.num_joints()) {
    throw std::invalid_argument("solve_ik: q_seed has the wrong size");
  }
  if (options.max_iterations <= 0 || options.position_tolerance <= 0.0 ||
      options.orientation_tolerance <= 0.0 || options.max_damping <= 0.0 ||
      options.manipulability_threshold <= 0.0 || options.max_step <= 0.0) {
    throw std::invalid_argument(
      "solve_ik: max_iterations, tolerances, max_damping, manipulability_threshold and max_step "
      "must be > 0");
  }
  if (options.null_space_gain < 0.0 || options.null_space_gain > 1.0) {
    throw std::invalid_argument("solve_ik: null_space_gain must be in [0, 1]");
  }
  if (options.null_space_target &&
      static_cast<std::size_t>(options.null_space_target->size()) != chain.num_joints()) {
    throw std::invalid_argument("solve_ik: null_space_target has the wrong size");
  }
}

/// [position error; orientation error as angle * axis], both in the base frame.
Vector6d pose_error(const Eigen::Isometry3d & target, const Eigen::Isometry3d & current)
{
  const Eigen::AngleAxisd rotation_error(target.linear() * current.linear().transpose());
  Vector6d error;
  error.head<3>() = target.translation() - current.translation();
  error.tail<3>() = rotation_error.angle() * rotation_error.axis();
  return error;
}

/// Squared damping: zero away from singularities, rising to max_damping^2 at manipulability 0.
double damping_squared(const Jacobian & j, const IkOptions & options)
{
  const double manipulability = std::sqrt(std::max(0.0, (j * j.transpose()).determinant()));
  if (manipulability >= options.manipulability_threshold) {
    return 0.0;
  }
  const double ratio = manipulability / options.manipulability_threshold;
  return options.max_damping * options.max_damping * (1.0 - ratio * ratio);
}

/// Projector onto the null space of J: joint motions that don't move the tip. Built from the
/// SVD so it is exact and bounded even near singularities.
Eigen::MatrixXd null_space_projector(const Jacobian & j)
{
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(j, Eigen::ComputeFullV);
  const Eigen::Index rank = svd.rank();
  const Eigen::MatrixXd range = svd.matrixV().leftCols(rank);
  return Eigen::MatrixXd::Identity(j.cols(), j.cols()) - range * range.transpose();
}

/// One damped-least-squares step for pose error `error`, plus the optional null-space task.
/// Damped pseudo-inverse J^T (J J^T + lambda^2 I)^-1, solved rather than inverted.
Eigen::VectorXd damped_step(
  const Jacobian & j, const Vector6d & error, const IkOptions & options, const Eigen::VectorXd & q)
{
  const Matrix6d jjt_damped = j * j.transpose() + damping_squared(j, options) * Matrix6d::Identity();
  Eigen::VectorXd step = j.transpose() * jjt_damped.ldlt().solve(error);
  if (options.null_space_target) {
    step += options.null_space_gain * null_space_projector(j) * (*options.null_space_target - q);
  }
  return step;
}

Eigen::VectorXd clamp_to_limits(const KinematicChain & chain, const Eigen::VectorXd & q)
{
  return q.cwiseMax(chain.lower_limits()).cwiseMin(chain.upper_limits());
}
}  // namespace

IkResult solve_ik(
  const KinematicChain & chain, const Eigen::Isometry3d & target, const Eigen::VectorXd & q_seed,
  const IkOptions & options)
{
  validate(chain, q_seed, options);

  IkResult result;
  result.q = clamp_to_limits(chain, q_seed);
  for (result.iterations = 0;; ++result.iterations) {
    const Vector6d error = pose_error(target, chain.fk(result.q));
    result.position_error = error.head<3>().norm();
    result.orientation_error = error.tail<3>().norm();
    result.converged = result.position_error < options.position_tolerance &&
                       result.orientation_error < options.orientation_tolerance;
    if (result.converged || result.iterations == options.max_iterations) {
      return result;
    }

    // Joint clamping: a joint sitting at a limit whose step points out of range can't help, and
    // the clamp below would throw its share of the step away. Drop its Jacobian column and solve
    // again, so the other joints take over. At most one pass per joint.
    Jacobian j = chain.jacobian(result.q);
    std::vector<bool> locked(chain.num_joints(), false);
    Eigen::VectorXd step;
    for (std::size_t pass = 0; pass <= chain.num_joints(); ++pass) {
      step = damped_step(j, error, options, result.q);
      for (std::size_t i = 0; i < locked.size(); ++i) {
        if (locked[i]) {
          step[static_cast<Eigen::Index>(i)] = 0.0;
        }
      }
      bool newly_locked = false;
      for (std::size_t i = 0; i < locked.size(); ++i) {
        const auto k = static_cast<Eigen::Index>(i);
        const bool pushes_below = result.q[k] <= chain.lower_limits()[k] && step[k] < 0.0;
        const bool pushes_above = result.q[k] >= chain.upper_limits()[k] && step[k] > 0.0;
        if (!locked[i] && (pushes_below || pushes_above)) {
          locked[i] = true;
          j.col(k).setZero();
          newly_locked = true;
        }
      }
      if (!newly_locked) {
        break;
      }
    }

    const double largest = step.cwiseAbs().maxCoeff();
    if (largest > options.max_step) {
      step *= options.max_step / largest;
    }
    result.q = clamp_to_limits(chain, result.q + step);
  }
}

}  // namespace arm_sandbox_kinematics
```

- [ ] **Step 4: Run the tests to see them pass**

Run: `make test`
Expected: `test_ik` reports 5 tests passed. Run `./build/arm_sandbox_kinematics/test_ik` to see the measured numbers:

```
IK from home: 165/200 targets converged (82.5%)
mean distance to home: plain 3.043, with null-space task 2.906 (164 targets)
```

The whole binary should take about 0.5 s. If it takes tens of seconds, the package was built without optimization: check that `CMAKE_BUILD_TYPE` defaults to `Release` in its `CMakeLists.txt`, then rebuild with `colcon build --packages-select arm_sandbox_kinematics --cmake-clean-cache`.

- [ ] **Step 5: Commit**

```bash
git add src/arm_sandbox_kinematics
git commit -m "feat(kinematics): damped-least-squares IK with manipulability-scaled damping, null-space task and joint clamping"
```

---

# Part 2 — The Reach Task (M2)

### Task 3: Reach task logic (`arm_sandbox_tasks`, ROS-free part)

The task file format follows the spec's task YAML ("Tasks and Executive") and fills in the reach task's success predicate, `ee_at_pose`. A target counts as reached once the end effector stays within tolerance for `hold_s`, the same idea as the spec's `object_in_region` `hold_s`. Without the hold, the first run declared success at 4.5–5.0 mm, while the arm was still settling into the band.

**Files:**
- Create: `src/arm_sandbox_tasks/package.xml`, `src/arm_sandbox_tasks/CMakeLists.txt`
- Create: `src/arm_sandbox_tasks/include/arm_sandbox_tasks/reach_task.hpp`, `src/arm_sandbox_tasks/src/reach_task.cpp`
- Create: `src/arm_sandbox_tasks/config/tasks/reach.yaml`
- Test: `src/arm_sandbox_tasks/test/test_reach_task.cpp`

**Interfaces:**
- Consumes: nothing from earlier tasks (Eigen and yaml-cpp only).
- Produces (namespace `arm_sandbox_tasks`, static library target `reach_task`):
  - `struct PoseTolerance { double position; double orientation; }`
  - `struct ReachTask { std::string name; PoseTolerance tolerance; double hold_s; double time_limit_s; std::vector<Eigen::Isometry3d> targets; }`
  - `ReachTask load_reach_task(const std::string & path)`, which throws `std::invalid_argument` naming the file and the field.
  - `Eigen::Isometry3d pose_from_xyz_rpy(const Eigen::Vector3d & position, const Eigen::Vector3d & rpy)`, using the URDF convention: fixed axes x, then y, then z.
  - `struct PoseError { double position; double orientation; }`; `PoseError pose_error(target, actual)`; `bool within(const PoseError &, const PoseTolerance &)`.
  - `double move_duration(const Eigen::VectorXd & from, const Eigen::VectorXd & to, const Eigen::VectorXd & velocity_limits, double velocity_scale, double min_duration_s)`.

- [ ] **Step 1: Package skeleton and the task file**

```bash
mkdir -p src/arm_sandbox_tasks/{include/arm_sandbox_tasks,src,config/tasks,launch,test}
```

`src/arm_sandbox_tasks/package.xml`. It already lists the ROS dependencies of Task 4's node; they're harmless here.

```xml
<?xml version="1.0"?>
<package format="3">
  <name>arm_sandbox_tasks</name>
  <version>0.1.0</version>
  <description>Task definitions and runners for arm-sandbox. M2: the reach task, solved with the hand-written IK.</description>
  <maintainer email="robertocw_18@hotmail.com">Roberto Cai</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>arm_sandbox_kinematics</depend>
  <depend>control_msgs</depend>
  <depend>eigen</depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_action</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>tf2_eigen</depend>
  <depend>tf2_ros</depend>
  <depend>yaml_cpp_vendor</depend>

  <exec_depend>arm_sandbox_description</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>python3-yaml</exec_depend>

  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>arm_sandbox_bringup</test_depend>
  <test_depend>launch_testing_ament_cmake</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

`src/arm_sandbox_tasks/CMakeLists.txt`. As in Task 1, this is the final file. To build Task 3 alone, leave out the `reach_runner` executable, its `install(TARGETS ...)` line, the `launch` directory in `install(DIRECTORY ...)`, and the `add_launch_test` line, then add them back in Task 4, Step 1.

```cmake
cmake_minimum_required(VERSION 3.16)
project(arm_sandbox_tasks)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -Werror)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(arm_sandbox_kinematics REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# ROS-free task logic (task files, pose errors, move timing), unit-tested on its own.
add_library(reach_task STATIC src/reach_task.cpp)
set_target_properties(reach_task PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(reach_task PUBLIC include)
target_link_libraries(reach_task PUBLIC Eigen3::Eigen yaml-cpp)

# Thin ROS node on top of it.
add_executable(reach_runner src/reach_runner.cpp)
target_link_libraries(reach_runner reach_task arm_sandbox_kinematics::arm_sandbox_kinematics)
ament_target_dependencies(reach_runner control_msgs rclcpp rclcpp_action sensor_msgs std_msgs tf2_eigen tf2_ros)

install(TARGETS reach_runner DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_reach_task test/test_reach_task.cpp)
  target_link_libraries(test_reach_task reach_task)
  target_compile_definitions(test_reach_task PRIVATE
    REACH_TASK_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/tasks/reach.yaml")
  # Full stack: sim + reach_runner. Own ROS domain (see arm_sandbox_bringup/CMakeLists.txt).
  add_launch_test(test/test_reach.py TIMEOUT 300 ENV ROS_DOMAIN_ID=45)
endif()

ament_package()
```

`src/arm_sandbox_tasks/config/tasks/reach.yaml`. The targets sit in front of the robot around the home TCP pose (0.307, 0, 0.487), gripper down, at different heights, ±0.25 m to the sides, some with yaw. The last one is home.

```yaml
# Reach task (REQUIREMENTS FR-15, task 1): move the end effector (robot.yaml `ee_frame`) to each
# target pose in turn. Poses are in the robot's base frame (robot.yaml `base_frame`): position in m,
# rpy in rad, fixed axes x, y, z as in URDF. rpy [pi, 0, 0] points the gripper straight down.
name: reach
success:
  type: ee_at_pose
  position_tolerance: 0.005 # m
  orientation_tolerance: 0.035 # rad (2 deg)
  hold_s: 0.5 # must stay within tolerance this long (sim time)
time_limit_s: 30.0 # per target: the move, plus settling into tolerance
targets:
  - {position: [0.45, 0.0, 0.35], rpy: [3.14159265, 0.0, 0.0]}
  - {position: [0.40, 0.25, 0.30], rpy: [3.14159265, 0.0, 0.5]}
  - {position: [0.40, -0.25, 0.45], rpy: [3.14159265, 0.0, -0.5]}
  - {position: [0.55, 0.10, 0.20], rpy: [3.14159265, 0.0, 0.0]}
  - {position: [0.307, 0.0, 0.487], rpy: [3.14159265, 0.0, 0.0]} # the home pose's TCP
```

- [ ] **Step 2: Write the failing tests**

`src/arm_sandbox_tasks/test/test_reach_task.cpp`:

```cpp
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
```

Run: `make test`
Expected: the build FAILS with `fatal error: arm_sandbox_tasks/reach_task.hpp: No such file or directory`.

- [ ] **Step 3: Write the task logic**

`src/arm_sandbox_tasks/include/arm_sandbox_tasks/reach_task.hpp`:

```cpp
#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace arm_sandbox_tasks
{

/// How close the end effector must get to a target pose.
struct PoseTolerance
{
  double position = 0.0;     ///< m
  double orientation = 0.0;  ///< rad
};

/// The reach task (REQUIREMENTS FR-15, task 1): move the end effector to each target in turn.
/// Loaded from `config/tasks/reach.yaml`. ROS-free, so the eval runner and the Gym env can use it.
struct ReachTask
{
  std::string name;
  PoseTolerance tolerance;
  double hold_s = 0.0;                            ///< must stay within tolerance this long, > 0
  double time_limit_s = 0.0;                      ///< per target
  std::vector<Eigen::Isometry3d> targets;         ///< in the robot's base frame
};

/// Parse and validate a reach task file. Throws std::invalid_argument with the reason (and the
/// file name) if a field is missing or invalid, e.g. an unsupported `success.type`.
ReachTask load_reach_task(const std::string & path);

/// Pose from a position and fixed-axis roll/pitch/yaw (rotate about x, then y, then z), the
/// URDF convention.
Eigen::Isometry3d pose_from_xyz_rpy(const Eigen::Vector3d & position, const Eigen::Vector3d & rpy);

struct PoseError
{
  double position = 0.0;     ///< m, distance between the origins
  double orientation = 0.0;  ///< rad, angle of the rotation from `actual` to `target`
};

PoseError pose_error(const Eigen::Isometry3d & target, const Eigen::Isometry3d & actual);

bool within(const PoseError & error, const PoseTolerance & tolerance);

/// Duration for a straight joint-space move so that no joint exceeds `velocity_scale` times its
/// velocity limit (average speed), but never shorter than `min_duration_s`.
double move_duration(
  const Eigen::VectorXd & from, const Eigen::VectorXd & to, const Eigen::VectorXd & velocity_limits,
  double velocity_scale, double min_duration_s);

}  // namespace arm_sandbox_tasks
```

`src/arm_sandbox_tasks/src/reach_task.cpp`:

```cpp
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
```

- [ ] **Step 4: Run the tests to see them pass**

Run: `make test`
Expected: `test_reach_task` reports 6 tests passed.

- [ ] **Step 5: Commit**

```bash
git add src/arm_sandbox_tasks
git commit -m "feat(tasks): reach task file and ROS-free task logic (pose error, hold, move timing)"
```

### Task 4: `reach_runner` — the reach task solved in the sim (M2 acceptance)

For each target, the runner:
1. solves the IK from the current joint positions, with a null-space pull towards home;
2. sends one `FollowJointTrajectory` goal, timed so that no joint exceeds `velocity_scale` × its URDF velocity limit;
3. waits until TF (`base_frame` → `ee_frame`, from `robot_state_publisher`) shows the end effector holding within tolerance for `hold_s` of sim time.

It exits 0 only if every target was reached.

Measured while writing this plan, as the error at the end of the 0.5 s hold, repeatable to about 0.1 mm across runs:
- **gravcomp off (the default):** 1.2–4.1 mm. JTC's integral term is still carrying the gravity load.
- **gravcomp on:** 0.1–0.3 mm.

The 5 mm tolerance passes both. The gap is the motivation for explicit gravity compensation in M3.

**Files:**
- Create: `src/arm_sandbox_tasks/src/reach_runner.cpp`, `src/arm_sandbox_tasks/config/reach_runner.yaml`, `src/arm_sandbox_tasks/launch/reach.launch.py`
- Test: `src/arm_sandbox_tasks/test/test_reach.py`
- Modify: `Makefile`, `CLAUDE.md`, `docs/PROJECT_STRUCTURE.md` (the `start_reach` target)

**Interfaces:**
- Consumes: `KinematicChain`, `solve_ik`, `IkOptions` (Tasks 1–2); `load_reach_task`, `pose_error`, `within`, `move_duration` (Task 3). From a running sim (`make start_sim`): `/robot_description`, `/joint_states`, `/tf`, `/arm_controller/state`, `/arm_controller/follow_joint_trajectory`.
- Produces:
  - Executable `arm_sandbox_tasks/reach_runner`, node `reach_runner`. Parameters:
    - from `robot.yaml`, set by the launch: `arm_joints`, `base_frame`, `ee_frame`, `home`;
    - set by the launch: `task_file`;
    - from `config/reach_runner.yaml`: `arm_action`, `controller_state_topic`, `startup_timeout_s`, `velocity_scale`, `min_move_duration_s`, `settle_timeout_s`, and `ik.*` (the `IkOptions` fields except `null_space_target`, which is `home`).
  - `ros2 launch arm_sandbox_tasks reach.launch.py robot:=panda task:=reach`. The launch ends when the runner exits.
  - `make start_reach [ARGS="..."]`.

- [ ] **Step 1: Write the failing launch test**

If you trimmed the CMake file in Task 3, put the `reach_runner` lines, `launch` in `install(DIRECTORY ...)`, and the `add_launch_test` line back now.

`src/arm_sandbox_tasks/test/test_reach.py`:

```python
"""M2 acceptance: with the sim running, reach_runner reaches every target of the reach task.

Runs once per gravity-compensation mode (D15), like test_arm_trajectory.py.
"""

import unittest
from pathlib import Path

import launch_testing
import launch_testing.asserts
import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest

# Startup (~10 s) plus every target's move and settling.
REACH_TIMEOUT_S = 180.0


def launch_file(package: str, name: str) -> str:
    return str(Path(get_package_share_directory(package)) / "launch" / name)


@pytest.mark.launch_test
@launch_testing.parametrize("gravcomp", ["false", "true"])
def generate_test_description(gravcomp: str):
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file("arm_sandbox_bringup", "sim.launch.py")),
        launch_arguments={"robot": "panda", "viewer": "false", "rerun": "false", "gravcomp": gravcomp}.items(),
    )
    reach = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file("arm_sandbox_tasks", "reach.launch.py")),
        launch_arguments={"robot": "panda", "task": "reach"}.items(),
    )
    return LaunchDescription([sim, reach, ReadyToTest()])


class TestReach(unittest.TestCase):
    def test_reach_runner_finishes(self, proc_info) -> None:
        proc_info.assertWaitForShutdown(process="reach_runner", timeout=REACH_TIMEOUT_S)


@launch_testing.post_shutdown_test()
class TestReachResult(unittest.TestCase):
    def test_every_target_reached(self, proc_info, proc_output) -> None:
        # reach_runner exits 0 only if all targets were reached.
        launch_testing.asserts.assertExitCodes(proc_info, process="reach_runner")
        # ROS loggers write to stderr.
        launch_testing.asserts.assertInStderr(proc_output, "targets reached", "reach_runner")
```

Run: `make test`
Expected: FAIL. The build stops on the missing `src/reach_runner.cpp` (`Cannot find source file`).

- [ ] **Step 2: Write the runner config and the launch file**

`src/arm_sandbox_tasks/config/reach_runner.yaml`:

```yaml
# reach_runner: solves each reach target with arm_sandbox_kinematics' IK and drives the arm there
# through the joint trajectory controller. Robot names and the home pose come from robot.yaml
# (passed in by reach.launch.py); the task file by its `task_file` parameter.
reach_runner:
  ros__parameters:
    arm_action: /arm_controller/follow_joint_trajectory
    # JTC publishes its state only while active, so the first message means "ready".
    controller_state_topic: /arm_controller/state
    startup_timeout_s: 60.0
    velocity_scale: 0.3 # fraction of each joint's URDF velocity limit (average speed of a move)
    min_move_duration_s: 1.0
    # After JTC reports success (its goal tolerance is per joint), the end effector gets up to this
    # long to settle into the task's pose tolerance and hold it for the task's `hold_s`.
    settle_timeout_s: 3.0
    ik:
      max_iterations: 500
      position_tolerance: 1.0e-4 # m; much tighter than the task tolerance
      orientation_tolerance: 1.0e-3 # rad
      max_damping: 0.05
      manipulability_threshold: 0.01
      max_step: 0.2 # rad per iteration
      null_space_gain: 0.1 # pull the redundant joint towards robot.yaml home
```

`src/arm_sandbox_tasks/launch/reach.launch.py`:

```python
"""Run the reach task (milestone M2) against a running sim (`make start_sim`).

    ros2 launch arm_sandbox_tasks reach.launch.py robot:=panda task:=reach

reach_runner gets the robot's names and home pose from its robot.yaml, the task from
config/tasks/<task>.yaml, and its solver/motion settings from config/reach_runner.yaml.
It exits 0 if every target was reached; the launch then ends.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_CONFIG_KEYS = ("arm_joints", "base_frame", "ee_frame", "home")


def require_file(path: Path, what: str) -> Path:
    if not path.is_file():
        raise FileNotFoundError(f"{what} not found: {path}")
    return path


def launch_setup(context: LaunchContext) -> list:
    robot = LaunchConfiguration("robot").perform(context)
    task = LaunchConfiguration("task").perform(context)
    tasks_dir = Path(get_package_share_directory("arm_sandbox_tasks"))
    robot_config_file = Path(get_package_share_directory("arm_sandbox_description")) / robot / "config" / "robot.yaml"
    robot_config = yaml.safe_load(require_file(robot_config_file, "robot config").read_text())

    reach_runner = Node(
        package="arm_sandbox_tasks",
        executable="reach_runner",
        name="reach_runner",
        parameters=[
            str(require_file(tasks_dir / "config" / "reach_runner.yaml", "reach_runner config")),
            {key: robot_config[key] for key in ROBOT_CONFIG_KEYS},
            {
                "task_file": str(require_file(tasks_dir / "config" / "tasks" / f"{task}.yaml", "task file")),
                "use_sim_time": True,
            },
        ],
        output="screen",
    )
    return [
        reach_runner,
        RegisterEventHandler(OnProcessExit(target_action=reach_runner, on_exit=[EmitEvent(event=Shutdown())])),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="panda", description="Robot folder in arm_sandbox_description"),
            DeclareLaunchArgument("task", default_value="reach", description="Task file in config/tasks/ (without .yaml)"),
            OpaqueFunction(function=launch_setup),
        ]
    )
```

- [ ] **Step 3: Write the node**

`src/arm_sandbox_tasks/src/reach_runner.cpp`:

```cpp
// reach_runner: solves the reach task (milestone M2) with the hand-written IK and the joint
// trajectory controller. Thin ROS wrapper: the math is in arm_sandbox_kinematics, the task logic
// in reach_task.hpp.
//
// For each target: IK from the current joint positions (null-space pull towards home) -> one
// FollowJointTrajectory goal -> wait until TF shows the end effector within the task tolerance.
// Exits 0 if every target was reached, 1 otherwise.

#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "arm_sandbox_kinematics/ik.hpp"
#include "arm_sandbox_kinematics/kinematic_chain.hpp"
#include "arm_sandbox_tasks/reach_task.hpp"

namespace arm_sandbox_tasks
{
namespace
{
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using Clock = std::chrono::steady_clock;
constexpr auto kPollPeriod = std::chrono::milliseconds(20);

Clock::duration seconds(double value)
{
  return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(value));
}
}  // namespace

class ReachRunner : public rclcpp::Node
{
public:
  ReachRunner()
  : Node("reach_runner"),
    arm_joints_(declare_parameter<std::vector<std::string>>("arm_joints", std::vector<std::string>{})),
    base_frame_(declare_parameter<std::string>("base_frame", "")),
    ee_frame_(declare_parameter<std::string>("ee_frame", "")),
    task_(load_reach_task(declare_parameter<std::string>("task_file", ""))),
    startup_timeout_s_(declare_parameter<double>("startup_timeout_s", 0.0)),
    velocity_scale_(declare_parameter<double>("velocity_scale", 0.0)),
    min_move_duration_s_(declare_parameter<double>("min_move_duration_s", 0.0)),
    settle_timeout_s_(declare_parameter<double>("settle_timeout_s", 0.0)),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    const auto home = declare_parameter<std::vector<double>>("home", std::vector<double>{});
    if (arm_joints_.empty() || base_frame_.empty() || ee_frame_.empty() || home.size() != arm_joints_.size()) {
      RCLCPP_FATAL(get_logger(), "robot config missing: arm_joints, base_frame, ee_frame, home");
      throw std::invalid_argument("invalid robot config");
    }
    if (startup_timeout_s_ <= 0.0 || velocity_scale_ <= 0.0 || min_move_duration_s_ <= 0.0 || settle_timeout_s_ <= 0.0) {
      RCLCPP_FATAL(get_logger(), "startup_timeout_s, velocity_scale, min_move_duration_s, settle_timeout_s must be > 0");
      throw std::invalid_argument("invalid reach_runner config");
    }
    home_ = Eigen::Map<const Eigen::VectorXd>(home.data(), static_cast<Eigen::Index>(home.size()));

    ik_options_.max_iterations = static_cast<int>(declare_parameter<int>("ik.max_iterations", 0));
    ik_options_.position_tolerance = declare_parameter<double>("ik.position_tolerance", 0.0);
    ik_options_.orientation_tolerance = declare_parameter<double>("ik.orientation_tolerance", 0.0);
    ik_options_.max_damping = declare_parameter<double>("ik.max_damping", 0.0);
    ik_options_.manipulability_threshold = declare_parameter<double>("ik.manipulability_threshold", 0.0);
    ik_options_.max_step = declare_parameter<double>("ik.max_step", 0.0);
    ik_options_.null_space_gain = declare_parameter<double>("ik.null_space_gain", 0.0);
    ik_options_.null_space_target = home_;

    robot_description_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).transient_local().reliable(),
      [this](const std_msgs::msg::String & msg) { robot_description_ = msg.data; });
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState & msg) { latest_joint_state_ = msg; });
    controller_state_sub_ = create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      declare_parameter<std::string>("controller_state_topic", ""), rclcpp::SensorDataQoS(),
      [this](const control_msgs::msg::JointTrajectoryControllerState &) { controller_active_ = true; });
    arm_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, declare_parameter<std::string>("arm_action", ""));
  }

  /// Runs every target; true if all were reached.
  bool run()
  {
    if (!wait_until([this] { return robot_description_.has_value(); }, "/robot_description") ||
        !wait_until([this] { return controller_active_; }, "the arm controller to become active") ||
        !wait_until([this] { return current_positions().has_value(); }, "/joint_states with all arm joints") ||
        !arm_client_->wait_for_action_server(seconds(startup_timeout_s_))) {
      return false;
    }

    const arm_sandbox_kinematics::KinematicChain chain(*robot_description_, base_frame_, ee_frame_);
    if (chain.joint_names() != arm_joints_) {
      RCLCPP_FATAL(get_logger(), "the URDF chain %s -> %s doesn't match robot.yaml arm_joints", base_frame_.c_str(), ee_frame_.c_str());
      return false;
    }

    int reached = 0;
    for (std::size_t i = 0; i < task_.targets.size(); ++i) {
      const bool ok = reach(chain, task_.targets[i], i);
      reached += ok ? 1 : 0;
    }
    RCLCPP_INFO(get_logger(), "task '%s': %d/%zu targets reached", task_.name.c_str(), reached, task_.targets.size());
    return reached == static_cast<int>(task_.targets.size());
  }

private:
  bool reach(const arm_sandbox_kinematics::KinematicChain & chain, const Eigen::Isometry3d & target, std::size_t index)
  {
    const auto deadline = Clock::now() + seconds(task_.time_limit_s);
    const Eigen::VectorXd start = *current_positions();
    const auto ik = arm_sandbox_kinematics::solve_ik(chain, target, start, ik_options_);
    if (!ik.converged) {
      RCLCPP_ERROR(get_logger(), "target %zu: IK failed (%.4f m, %.4f rad after %d iterations)",
                   index, ik.position_error, ik.orientation_error, ik.iterations);
      return false;
    }

    const double duration = move_duration(start, ik.q, chain.velocity_limits(), velocity_scale_, min_move_duration_s_);
    if (!move_to(ik.q, duration, deadline)) {
      RCLCPP_ERROR(get_logger(), "target %zu: trajectory failed", index);
      return false;
    }

    // JTC's goal tolerance is per joint, so the pose keeps settling after it succeeds. The target
    // counts as reached once the end effector stays within tolerance for hold_s of sim time.
    const auto settle_deadline = std::min(deadline, Clock::now() + seconds(settle_timeout_s_) + seconds(task_.hold_s));
    PoseError error;
    std::optional<rclcpp::Time> within_since;
    while (true) {
      const std::optional<Eigen::Isometry3d> ee = ee_pose();
      if (ee) {
        error = pose_error(target, *ee);
        if (!within(error, task_.tolerance)) {
          within_since.reset();
        } else if (!within_since) {
          within_since = now();
        } else if ((now() - *within_since).seconds() >= task_.hold_s) {
          RCLCPP_INFO(get_logger(), "target %zu reached: %.4f m, %.4f rad (IK %d iterations, move %.2f s)",
                      index, error.position, error.orientation, ik.iterations, duration);
          return true;
        }
      }
      if (Clock::now() > settle_deadline) {
        RCLCPP_ERROR(get_logger(), "target %zu: not held within tolerance (%.4f m, %.4f rad)", index, error.position, error.orientation);
        return false;
      }
      spin_for(kPollPeriod);
    }
  }

  bool move_to(const Eigen::VectorXd & q, double duration_s, Clock::time_point deadline)
  {
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = arm_joints_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(q.data(), q.data() + q.size());
    point.velocities.assign(static_cast<std::size_t>(q.size()), 0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(duration_s);
    goal.trajectory.points.push_back(point);

    auto goal_future = arm_client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_future, deadline - Clock::now()) !=
        rclcpp::FutureReturnCode::SUCCESS || !goal_future.get()) {
      return false;
    }
    auto result_future = arm_client_->async_get_result(goal_future.get());
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future, deadline - Clock::now()) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return false;
    }
    const auto result = result_future.get();
    return result.code == rclcpp_action::ResultCode::SUCCEEDED &&
           result.result->error_code == FollowJointTrajectory::Result::SUCCESSFUL;
  }

  /// Arm joint positions in arm_joints_ order, if the latest /joint_states has all of them.
  std::optional<Eigen::VectorXd> current_positions() const
  {
    if (!latest_joint_state_) {
      return std::nullopt;
    }
    Eigen::VectorXd q(static_cast<Eigen::Index>(arm_joints_.size()));
    for (std::size_t i = 0; i < arm_joints_.size(); ++i) {
      const auto & names = latest_joint_state_->name;
      const auto it = std::find(names.begin(), names.end(), arm_joints_[i]);
      if (it == names.end() || static_cast<std::size_t>(it - names.begin()) >= latest_joint_state_->position.size()) {
        return std::nullopt;
      }
      q[static_cast<Eigen::Index>(i)] = latest_joint_state_->position[static_cast<std::size_t>(it - names.begin())];
    }
    return q;
  }

  std::optional<Eigen::Isometry3d> ee_pose() const
  {
    try {
      return tf2::transformToEigen(tf_buffer_.lookupTransform(base_frame_, ee_frame_, tf2::TimePointZero));
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  template <typename Predicate>
  bool wait_until(Predicate ready, const std::string & what)
  {
    const auto deadline = Clock::now() + seconds(startup_timeout_s_);
    while (!ready()) {
      if (Clock::now() > deadline) {
        RCLCPP_FATAL(get_logger(), "timed out waiting for %s", what.c_str());
        return false;
      }
      spin_for(kPollPeriod);
    }
    return true;
  }

  void spin_for(std::chrono::milliseconds period)
  {
    rclcpp::spin_some(shared_from_this());
    std::this_thread::sleep_for(period);
  }

  std::vector<std::string> arm_joints_;
  std::string base_frame_;
  std::string ee_frame_;
  ReachTask task_;
  double startup_timeout_s_;
  double velocity_scale_;
  double min_move_duration_s_;
  double settle_timeout_s_;
  Eigen::VectorXd home_;
  arm_sandbox_kinematics::IkOptions ik_options_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;  ///< runs its own node and thread
  std::optional<std::string> robot_description_;
  std::optional<sensor_msgs::msg::JointState> latest_joint_state_;
  bool controller_active_ = false;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr controller_state_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_client_;
};

}  // namespace arm_sandbox_tasks

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  bool all_reached = false;
  {
    const auto runner = std::make_shared<arm_sandbox_tasks::ReachRunner>();
    all_reached = runner->run();
  }
  rclcpp::shutdown();
  return all_reached ? 0 : 1;
}
```

Three details that bit while writing this plan:
- **Vector parameter defaults.** `declare_parameter<std::vector<...>>(name, {})` doesn't compile: `{}` is ambiguous with `ParameterDescriptor`. Spell out `std::vector<...>{}`.
- **Clock types.** Mixing `steady_clock` time points with `duration<double>` doesn't compile; the `seconds()` helper returns `Clock::duration`.
- **Where the logs go.** `RCLCPP_*` logs go to stderr, so the launch test uses `assertInStderr`.

- [ ] **Step 4: Run it by hand**

```bash
make test                      # builds; test_reach should now pass too
make start_sim ARGS="viewer:=false rerun:=false" &   # or in a second terminal
sleep 15
source install/setup.bash && ros2 launch arm_sandbox_tasks reach.launch.py
```

Expected output: one line per target, then the summary, then a clean exit:

```
target 0 reached: 0.0038 m, 0.0096 rad (IK 27 iterations, move 3.24 s)
...
task 'reach': 5/5 targets reached
[INFO] [reach_runner-1]: process has finished cleanly
```

(The numbers vary with where the arm starts, within about 0.1 mm.) Stop the sim with `kill %1` or Ctrl+C.

If a target fails with `IK failed`, the target is out of reach from the current pose. Check its position against the workspace before touching the solver. If it fails with `not held within tolerance`, the arm got there but didn't settle in time: run with `gravcomp:=true` to see whether gravity is the cause.

- [ ] **Step 5: Run the tests to see them pass**

Run: `make test`
Expected: 0 failures. `test_reach` runs once per gravity-compensation mode, with 2 tests each (the runner finishes; every target reached). `test_reach_task`, `test_kinematic_chain` and `test_ik` pass as before.

- [ ] **Step 6: `make start_reach`**

In the `Makefile`, add `start_reach` to `.PHONY` and to the container-targets comment (`# Container targets (in dev container): smoke, test, viewer, start_sim, start_reach`). Then append:

```make
# Container: run the reach task against a running sim (`make start_sim` in another terminal).
# ARGS are launch arguments, e.g. make start_reach ARGS="task:=reach"
start_reach:
	$(ROS_SETUP) && source install/setup.bash && ros2 launch arm_sandbox_tasks reach.launch.py $(ARGS)
```

In `CLAUDE.md` ("Inside the dev container"), after the `make start_sim` line, add:

```markdown
- `make start_reach` - run the reach task (M2) against a running sim: IK → joint trajectory → pose check
```

In `docs/PROJECT_STRUCTURE.md`, add a Commands-table row after `make start_sim`:

```markdown
| `make start_reach` | container | Run the reach task against a running sim (`ARGS="task:=reach"`) |
```

- [ ] **Step 7: Watch it once** [needs a display]

```bash
make start_sim      # terminal 1: native viewer + Rerun
make start_reach    # terminal 2
```

Expected: the arm visits the five targets with the gripper pointing down, in both viewers, and terminal 2 ends with `5/5 targets reached`. This run is the M2 demo video.

- [ ] **Step 8: Commit**

```bash
git add src/arm_sandbox_tasks Makefile CLAUDE.md docs/PROJECT_STRUCTURE.md
git commit -m "feat(tasks): reach_runner solves the reach task with the hand-written IK; make start_reach"
```

---

# Part 3 — Docs

### Task 5: Record the decisions, update the docs, add READMEs, log the session

**Files:**
- Modify: `docs/REQUIREMENTS.md` (§10), `docs/specs/2026-10-01-arm-sandbox-design.md` (Kinematics, Tasks and Executive), `docs/PROJECT_STRUCTURE.md`, `CLAUDE.md`, `docs/MEMORY.md`
- Create: `src/arm_sandbox_kinematics/README.md`, `src/arm_sandbox_tasks/README.md`

**Interfaces:**
- Consumes: everything above.
- Produces: docs that match the code. Plan 04 (M3, control) is written from them.

- [ ] **Step 1: Decisions log.** In `docs/REQUIREMENTS.md` §10, append after D17:

```markdown
| D18 | M2's reach task runs as a `reach_runner` node (IK → one JTC goal → TF check), not a `MoveToPose` action | Smallest proof of M2; skill actions come with MoveIt in M4, which would replace an IK-only action server anyway |
| D19 | Analytic IK deferred; M2 ships damped-least-squares IK (manipulability-scaled damping, SVD null-space task, joint clamping) | The spec specifies numerical IK; closed-form Panda IK (q7 as free parameter) is robot-specific and becomes a stretch item |
```

and in the status line (line 3), insert `, D18–D19 added 2026-10-02 (Plan 03)` right after `(Plan 02)`, keeping the rest of the line.

- [ ] **Step 2: Design spec**

In `docs/specs/2026-10-01-arm-sandbox-design.md`, section "Kinematics (`arm_sandbox_kinematics`)", replace the bullet list (from `- \`KinematicChain(urdf, base, tip)\`` to the `- Tests:` bullet) with:

```markdown
- `KinematicChain(urdf, base, tip)`: fixed joints folded into the next moving joint's origin; revolute, continuous and prismatic joints supported
- `fk(q) → Isometry3d`
- `jacobian(q) → Matrix<6,n>` (geometric, base frame, rows [linear; angular])
- `solve_ik(chain, target, q_seed, opts) → {q, converged, errors, iterations}`: a free function, so the chain stays a pure model. Damped least squares with damping scaled by manipulability (zero away from singularities), an optional null-space task (stay near home) through the exact SVD projector, joint clamping at limits (D19)
- `manipulability(q)` (Yoshikawa, √det(J Jᵀ))
- Tests: FK and Jacobian vs Pinocchio at random configurations (1e-9), Jacobian vs finite differences, FK∘IK round trips, null-space effect, failure behavior. They run on the real Panda URDF, generated from the xacro at build time.
```

In section "Tasks and Executive (`arm_sandbox_tasks`)", after the task YAML example, add:

```markdown
- The reach task (M2) is `config/tasks/reach.yaml`: `targets` (position + rpy in the base frame) and `success: {type: ee_at_pose, position_tolerance, orientation_tolerance, hold_s}`. `reach_runner` (D18) solves each target with `solve_ik`, sends one JTC goal, and checks the pose from TF. Measured error after the hold: ≤ 4.1 mm without gravity compensation, ≤ 0.3 mm with it.
```

- [ ] **Step 3: `docs/PROJECT_STRUCTURE.md` and `CLAUDE.md`**

- `PROJECT_STRUCTURE.md`:
  - Replace the status note with `> **Status: M0–M2 done (Plans 01–03).** These exist: the dev environment, \`tests/env/\`, \`arm_sandbox_description\`, \`arm_sandbox_sim\` (scene composition only), \`arm_sandbox_bringup\`, \`arm_sandbox_viz\`, \`arm_sandbox_kinematics\`, and \`arm_sandbox_tasks\` (reach task only). Everything else is the target layout from \`docs/REQUIREMENTS.md\` (§6). Update it as packages are created.`
  - Packages table, `arm_sandbox_tasks` row, Purpose column: `Task configs and runners. Now: the reach task (\`reach_runner\`); later BehaviorTree.CPP executive and skills`.
- `CLAUDE.md` status line: `> **Status:** M0–M2 done (Plans 01–03): dev environment, \`arm_sandbox_description\`, \`arm_sandbox_sim\` (scene composition), \`arm_sandbox_bringup\` (\`make start_sim\`), \`arm_sandbox_viz\` (Rerun bridge), \`arm_sandbox_kinematics\` (FK/Jacobian/IK, checked against Pinocchio), \`arm_sandbox_tasks\` (reach task, \`make start_reach\`). Next: M3 (control). Every other package and command below is still the **plan** from \`docs/REQUIREMENTS.md\`. Update this file as they become real.`

- [ ] **Step 4: Package READMEs** (short, theory first, NFR-6)

`src/arm_sandbox_kinematics/README.md`:

```markdown
# arm_sandbox_kinematics

Hand-written kinematics for serial arms, in plain C++17 + Eigen (no ROS). Built from a URDF, so it
works for any arm. Checked against Pinocchio in the tests (FR-18).

## Forward kinematics

T(q) = Π_i O_i · M_i(q_i) · T_tip. O_i is joint i's fixed origin (fixed joints before it folded in),
M_i rotates by q_i about the joint axis (revolute) or translates along it (prismatic).

## Geometric Jacobian

Maps joint velocities to the tip's twist in the base frame, [v; ω] = J(q) q̇. Revolute column:
[z_i × (p_tip − p_i); z_i], prismatic: [z_i; 0], with z_i and p_i the joint's world axis and origin.
Manipulability w = √det(J Jᵀ) measures how far the arm is from a singularity (w = 0).

## Inverse kinematics (damped least squares)

Iterate Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e on the pose error e = [p* − p; θ·axis(R* Rᵀ)].
- λ = 0 away from singularities (Gauss–Newton, a few iterations); below the manipulability
  threshold w₀, λ² = λ_max² (1 − (w/w₀)²) keeps steps bounded where the pseudo-inverse explodes.
- A 7-DoF arm has one redundant degree of freedom: the null-space term k · N (q_home − q), with
  N = I − V_r V_rᵀ from the SVD of J, moves the joints towards home without moving the tip.
- Joint clamping: a joint at its limit that the step pushes further out is dropped from J, and the
  step is solved again.

A local solver started far away can miss solutions that need a different arm configuration
(elbow flipped); seeding from the current pose, as the reach runner does, avoids that.
```

`src/arm_sandbox_tasks/README.md`:

```markdown
# arm_sandbox_tasks

Task definitions and the programs that solve them. Task files live in `config/tasks/`; the task logic
(`reach_task.hpp`) is ROS-free so the eval runner and the Gym environment can reuse it.

## Reach (M2)

    make start_sim      # terminal 1
    make start_reach    # terminal 2: IK → joint trajectory → pose check, exits 0 if all targets reached

`reach_runner` solves each target in `config/tasks/reach.yaml` with `arm_sandbox_kinematics`, starting
from the current joint positions with a null-space pull towards home. It sends the joint trajectory
controller one goal, timed so no joint exceeds `velocity_scale` × its velocity limit, and counts the
target as reached once TF shows the end effector within tolerance for `hold_s`.

Without simulator gravity compensation the controller's integral term carries the arm's weight, so
the end effector settles within a few millimetres (≤ 4.1 mm after 0.5 s); with `gravcomp:=true` it
is within 0.3 mm. Explicit gravity compensation in the controller is M3.
```

- [ ] **Step 5: Log the session**

Append a new section at the end of `docs/MEMORY.md`, after the last session. Don't change earlier sections.

```markdown
---

## Session — <date you finish Plan 03>

**Plan:** [Plan 03 — Kinematics (M2) and the Reach Task](plan/2026-10-02-plan-03-kinematics-and-reach.md). Branch `feat/m2-kinematics`.

### Changes

- <one bullet per task commit, with its hash: what it added and what its tests prove>

### Learned

- <anything that surprised you while executing; keep the plan's own findings out unless they changed>

### State at end of session

<milestone status, test count from `make test`, what's next>
```

- [ ] **Step 6: Check, then commit**

```bash
grep -n "D18\|D19" docs/REQUIREMENTS.md docs/specs/2026-10-01-arm-sandbox-design.md   # Expected: hits in both
grep -n "start_reach" CLAUDE.md docs/PROJECT_STRUCTURE.md                             # Expected: hits in both
make test   # Expected: 0 failures
git add CLAUDE.md docs src/arm_sandbox_kinematics/README.md src/arm_sandbox_tasks/README.md
git commit -m "docs: record D18-D19, document kinematics and the reach task, add READMEs"
```

---

## Done when

- `make test` passes with no compiler warnings, including `test_kinematic_chain` (FK and Jacobian equal to Pinocchio's to 1e-9), `test_ik`, `test_reach_task`, and `test_reach` with gravcomp off and on.
- `make start_sim` + `make start_reach` reaches all 5 targets and exits 0, visible in the native viewer and in Rerun (**M2**).
- D18–D19 recorded; the spec, CLAUDE.md, PROJECT_STRUCTURE.md and the two READMEs match the code; `docs/MEMORY.md` has the session.
- Next: Plan 04 (M3: operational-space / impedance controllers with Pinocchio dynamics, drawer task).
