# Plan 04 — Task-Space Control (M3, part 1): Operational-Space and Impedance Controllers

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Two custom `ros2_control` controllers, operational-space control (OSC) and Cartesian impedance, built on a ROS-free control core (Pinocchio dynamics plus the hand-written kinematics from M2). They are selectable at launch, reach the M2 reach targets more accurately and faster than IK + JTC, and behave like a spring when pushed. This is the first half of M3; Plan 05 adds the drawer task.

**Architecture:** The new package `arm_sandbox_controllers` has two layers:
- **`control_core`, a ROS-free library:**
  - `RobotDynamics`: Pinocchio on the URDF, reduced to the arm joints, plus the actuators' armature. It provides M(q), C q̇ + g and g.
  - `TaskSpaceControl`: computes τ = Jᵀ F + Nᵀ τ₀ + C q̇ (+ g), using `KinematicChain`'s new allocation-free overloads for the Jacobian and FK.
  - Two laws for the task-space force F: OSC, F = Λ (Kp e − Kd ẋ); and impedance, F = K e − D ẋ.
- **Two thin plugins** sharing one base class.

`sim.launch.py` loads every arm controller inactive and activates the one named by `arm_controller:=`. It also injects the robot's URDF, names, rest pose and `compensate_gravity` (the opposite of `gravcomp`) into the task-space controllers. `reach_runner` gains a pose-target mode, so the same reach task compares all three controllers.

**Tech Stack:** C++17, Eigen, Pinocchio 4.0 (`crba`, `nonLinearEffects`, `computeGeneralizedGravity`, `aba`, `buildReducedModel`, `Model::armature`), `ros2_control` (`controller_interface`, `realtime_tools` 2.15 `RealtimeBuffer`/`RealtimePublisher`, pluginlib), GoogleTest, pytest, `launch_testing`.

**Spec:** [`../specs/2026-10-01-arm-sandbox-design.md`](../specs/2026-10-01-arm-sandbox-design.md), sections "Controllers" and "Kinematics", and [`../REQUIREMENTS.md`](../REQUIREMENTS.md) (M3, FR-19, D15). Plans 02–03 built the sim, the bring-up and the kinematics this plan uses.

## Task overview

Click a task to jump to it.

- [Task 1: Allocation-free `fk` and `jacobian` overloads](#task-1-allocation-free-fk-and-jacobian-overloads): Allocation-free `fk`/`jacobian` overloads, proven with Eigen's no-malloc guard.
- [Task 2: `RobotDynamics` — Pinocchio, reduced to the arm, with armature](#task-2-robotdynamics--pinocchio-reduced-to-the-arm-with-armature): `RobotDynamics`: Pinocchio reduced to the arm, with the MJCF's armature.
- [Task 3: The control laws — OSC and Cartesian impedance](#task-3-the-control-laws--osc-and-cartesian-impedance): OSC and Cartesian impedance laws on a shared, allocation-free core; closed-loop tests.
- [Task 4: The `ros2_control` plugins and the launch wiring](#task-4-the-ros2_control-plugins-and-the-launch-wiring): `ros2_control` plugins; `arm_controller:=` and `external_wrench:=` launch arguments; model matches MuJoCo.
- [Task 5: The impedance controller is a spring (compliance test)](#task-5-the-impedance-controller-is-a-spring-compliance-test): Compliance test: a 10 N push deflects the impedance controller by F/k, and it springs back.
- [Task 6: Reach with the task-space controllers, and the comparison](#task-6-reach-with-the-task-space-controllers-and-the-comparison): Reach with OSC and impedance (pose targets); all three controllers compared, both gravcomp modes.
- [Task 7: Record the decisions, update the docs, add READMEs, log the session](#task-7-record-the-decisions-update-the-docs-add-readmes-log-the-session): Docs: decisions D20–D22, controllers README, top-level README, reading list, session log.

## Global Constraints

- ROS 2 **Humble**, MuJoCo **3.12.0**, `mujoco_ros2_control` **0.1.2** (D7, D13, D14). Nothing new to install.
- **Pure library + thin node** (CLAUDE.md): `control_core` includes no ROS header; only `task_space_controller.cpp` does.
- **Real-time safe** (spec "Controllers"): no heap allocation in `update()`. Enforced by tests built with Eigen's `EIGEN_RUNTIME_NO_MALLOC` and without `NDEBUG`.
- **Hand-written kinematics, library dynamics** (spec): the controllers take J and FK from `arm_sandbox_kinematics`, and M, C, g from Pinocchio.
- **Gravity is compensated exactly once** (D15): with `gravcomp:=false` the controllers add g(q); with `gravcomp:=true` they must not. The launch sets `compensate_gravity = not gravcomp`.
- **No robot-specific values in code**, and no magic numbers. Robot names, frames, the URDF and the rest pose come from `robot.yaml` and the URDF via the launch. Gains, armature and limits come from `bringup/config/<robot>_controllers.yaml`.
- C++17, `-Wall -Wextra -Wpedantic -Werror`, and `CMAKE_BUILD_TYPE` defaults to `Release` (Plan 03).
- All tests are headless, and each `launch_testing` test gets its own `ROS_DOMAIN_ID`. Existing tests use 41–45; this plan adds 46.
- User decisions (2026-10-04): M3 is split into **two plans** (this one: controllers; Plan 05: drawer task). Plan 04 builds **OSC + Cartesian impedance**; the joint impedance controller waits for Phase B. The controllers use **our `KinematicChain`**, made allocation-free.

## Where each step runs

All steps run in the dev container after `source /opt/ros/humble/setup.bash`. After `make test` has built the workspace, also `source install/setup.bash`. Manual sim checks use `timeout -s INT …` or Ctrl+C. In a non-interactive script, background jobs ignore SIGINT, so a plain `kill -INT` doesn't stop them.

## Before Task 1: branch

```bash
git switch master && git pull
git switch -c feat/m3-controllers
```

This plan's code was built and run end to end in a scratch workspace on 2026-10-04 before it was written: `colcon test` gave 95 tests, 0 failures. Numbers quoted below come from that run.

## File Structure

| Path | Responsibility |
|---|---|
| `src/arm_sandbox_kinematics/…/kinematic_chain.hpp`, `src/kinematic_chain.cpp` | Add allocation-free `fk(q, pose)` and `jacobian(q, J)` overloads |
| `src/arm_sandbox_kinematics/test/test_realtime.cpp` | The overloads match and never allocate |
| `src/arm_sandbox_controllers/` (new) | Control core, plugins, plugin description, tests |
| `…/include/arm_sandbox_controllers/robot_dynamics.hpp`, `src/robot_dynamics.cpp` | `RobotDynamics`: Pinocchio, reduced to the arm, with armature |
| `…/include/arm_sandbox_controllers/task_space_control.hpp`, `src/task_space_control.cpp` | `TaskSpaceControl` and the two laws |
| `…/include/arm_sandbox_controllers/task_space_controller.hpp`, `src/task_space_controller.cpp`, `arm_sandbox_controllers.xml` | The `ros2_control` plugins `OscController` and `CartesianImpedanceController` |
| `…/test/control_fixture.hpp`, `test_robot_dynamics.cpp`, `test_task_space_control.cpp`, `test_realtime.cpp` | Control-core tests (closed loop on a simulated plant) |
| `src/arm_sandbox_bringup/config/panda_controllers.yaml` | Add both controllers: armature, settings, gains |
| `src/arm_sandbox_bringup/config/sim_plugins.yaml` (new) | The sim's external-wrench plugin (opt-in) |
| `src/arm_sandbox_bringup/launch/sim.launch.py` | `arm_controller`, `external_wrench` arguments; injected controller parameters |
| `src/arm_sandbox_bringup/test/test_dynamics_model.py`, `test_impedance_compliance.py` (new) | Model vs MuJoCo; spring behaviour in the sim |
| `src/arm_sandbox_tasks/src/reach_runner.cpp`, `launch/reach.launch.py`, `config/reach_runner.yaml`, `test/test_reach.py` | Pose-target mode and `controller` argument; the reach test runs all three controllers |
| `docs/…`, `CLAUDE.md`, READMEs | Updated in Task 7 |

## Out of scope (later plans)

- The drawer task, task objects in the scene, and a shared task-runner base: **Plan 05**.
- The joint impedance controller and pybind11 bindings: Phase B.
- Switching controllers at runtime from a task (the behavior tree's job, M4). Here the arm controller is chosen at launch.
- The spec's full controller state topic (target, actual, error). The controllers publish only `~/pose_error` for now, which is enough for readiness checks and tuning. Plotting it in Rerun comes later too; the bridge only logs standard topics so far.

---

# Part 1 — Control Core (ROS-free)

### Task 1: Allocation-free `fk` and `jacobian` overloads

`KinematicChain::jacobian(q)` returns a new matrix and uses per-call `std::vector`s, which is fine for IK but not inside a 1 kHz control loop. The new overloads write into caller-owned outputs. The Jacobian is computed in two passes (first the tip, then each joint's axis and origin), so it needs no per-joint storage. The existing functions now call the overloads, so the logic isn't duplicated.

To *prove* "never allocates", the test compiles the library source with Eigen's allocation guard (`EIGEN_RUNTIME_NO_MALLOC`) on and `NDEBUG` undefined, because Release builds would otherwise silence the guard's assert. A positive-control test checks that the guard really fires.

**Files:**
- Modify: `src/arm_sandbox_kinematics/include/arm_sandbox_kinematics/kinematic_chain.hpp`, `src/arm_sandbox_kinematics/src/kinematic_chain.cpp`, `src/arm_sandbox_kinematics/CMakeLists.txt`
- Test: `src/arm_sandbox_kinematics/test/test_realtime.cpp`

**Interfaces:**
- Produces: `void KinematicChain::fk(const Eigen::VectorXd & q, Eigen::Isometry3d & pose) const` and `void KinematicChain::jacobian(const Eigen::VectorXd & q, Jacobian & jacobian) const`. `jacobian` must be pre-sized 6 × `num_joints()`; otherwise it throws `std::invalid_argument`. Neither allocates.

- [ ] **Step 1: Write the failing test**

`src/arm_sandbox_kinematics/test/test_realtime.cpp`:

```cpp
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
```

Replace `src/arm_sandbox_kinematics/CMakeLists.txt` with the version below. It adds `test_realtime`, which compiles `src/kinematic_chain.cpp` itself, with the guard on.

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

  # Real-time check: compiles the library sources into the test with Eigen's allocation guard on.
  # Release builds define NDEBUG, which would silence the guard's assert, so undefine it here.
  ament_add_gtest(test_realtime test/test_realtime.cpp src/kinematic_chain.cpp)
  add_dependencies(test_realtime test_urdf)
  target_include_directories(test_realtime PRIVATE include ${urdfdom_INCLUDE_DIRS})
  target_link_libraries(test_realtime Eigen3::Eigen urdfdom::urdf_parser yaml-cpp)
  target_compile_definitions(test_realtime PRIVATE EIGEN_RUNTIME_NO_MALLOC
    TEST_URDF_PATH="${TEST_URDF}"
    TEST_ROBOT_CONFIG_PATH="${TEST_ROBOT_DIR}/config/robot.yaml")
  target_compile_options(test_realtime PRIVATE -UNDEBUG)
endif()

ament_package()
```

- [ ] **Step 2: Run it to see it fail**

Run: `make test`
Expected: `test_realtime.cpp` FAILS to compile with `no matching function for call to ‘…KinematicChain::fk(…, Eigen::Isometry3d&)’`.

- [ ] **Step 3: Add the overloads**

Replace `kinematic_chain.hpp` and `kinematic_chain.cpp` with:

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
  /// Same, written into `pose`. Never allocates: safe in a real-time control loop.
  void fk(const Eigen::VectorXd & q, Eigen::Isometry3d & pose) const;

  /// Geometric Jacobian of the tip origin in the base frame (see `Jacobian`).
  Jacobian jacobian(const Eigen::VectorXd & q) const;
  /// Same, written into `jacobian`, which must already be 6 x num_joints(). Never allocates.
  /// Throws std::invalid_argument on a wrong-size `q` or `jacobian`.
  void jacobian(const Eigen::VectorXd & q, Jacobian & jacobian) const;

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
  /// Applies joint i's origin and motion to `transform` (in place, no allocation).
  void advance(std::size_t i, double value, Eigen::Isometry3d & transform) const;

  std::vector<Joint> joints_;
  Eigen::Isometry3d tip_offset_;  ///< fixed joints after the last moving joint
  std::vector<std::string> joint_names_;
  Eigen::VectorXd lower_limits_;
  Eigen::VectorXd upper_limits_;
  Eigen::VectorXd velocity_limits_;
};

}  // namespace arm_sandbox_kinematics
```

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

void KinematicChain::advance(std::size_t i, double value, Eigen::Isometry3d & transform) const
{
  const Joint & joint = joints_[i];
  transform = transform * joint.origin;
  if (joint.type == JointType::kRevolute) {
    transform.rotate(Eigen::AngleAxisd(value, joint.axis));
  } else {
    transform.translate(value * joint.axis);
  }
}

Eigen::Isometry3d KinematicChain::fk(const Eigen::VectorXd & q) const
{
  Eigen::Isometry3d pose;
  fk(q, pose);
  return pose;
}

void KinematicChain::fk(const Eigen::VectorXd & q, Eigen::Isometry3d & pose) const
{
  check_size(q);
  pose = Eigen::Isometry3d::Identity();
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    advance(i, q[static_cast<Eigen::Index>(i)], pose);
  }
  pose = pose * tip_offset_;
}

Jacobian KinematicChain::jacobian(const Eigen::VectorXd & q) const
{
  Jacobian result(6, static_cast<Eigen::Index>(joints_.size()));
  jacobian(q, result);
  return result;
}

void KinematicChain::jacobian(const Eigen::VectorXd & q, Jacobian & jacobian) const
{
  check_size(q);
  if (static_cast<std::size_t>(jacobian.cols()) != joints_.size()) {
    throw std::invalid_argument("KinematicChain: the Jacobian must have one column per joint");
  }
  // Two passes, so no per-joint storage is needed: first the tip position, then each joint's
  // world axis and origin, which give its column directly.
  Eigen::Isometry3d tip_pose;
  fk(q, tip_pose);
  const Eigen::Vector3d tip = tip_pose.translation();

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const Joint & joint = joints_[i];
    const auto column = static_cast<Eigen::Index>(i);
    transform = transform * joint.origin;
    const Eigen::Vector3d axis = transform.linear() * joint.axis;
    if (joint.type == JointType::kRevolute) {
      // A rotation about axis z through point p moves the tip with z x (tip - p).
      jacobian.block<3, 1>(0, column) = axis.cross(tip - transform.translation());
      jacobian.block<3, 1>(3, column) = axis;
      transform.rotate(Eigen::AngleAxisd(q[column], joint.axis));
    } else {
      jacobian.block<3, 1>(0, column) = axis;
      jacobian.block<3, 1>(3, column) = Eigen::Vector3d::Zero();
      transform.translate(q[column] * joint.axis);
    }
  }
}

double KinematicChain::manipulability(const Eigen::VectorXd & q) const
{
  const Jacobian j = jacobian(q);
  return std::sqrt(std::max(0.0, (j * j.transpose()).determinant()));
}

}  // namespace arm_sandbox_kinematics
```

- [ ] **Step 4: Run the tests to see them pass**

Run: `make test`
Expected: `test_realtime` 3 passed; `test_kinematic_chain` (6) and `test_ik` (5) still pass.

- [ ] **Step 5: Check the guard really catches allocations**

Temporarily add `Eigen::VectorXd planted = q * 2.0; (void)planted;` as the second line of `fk(q, pose)`, then run `make test`.
Expected: `test_realtime` aborts with `heap allocation is forbidden (EIGEN_RUNTIME_NO_MALLOC is defined …)`. Remove the planted lines and run `make test` again; everything passes.

- [ ] **Step 6: Commit**

```bash
git add src/arm_sandbox_kinematics
git commit -m "feat(kinematics): allocation-free fk/jacobian overloads for real-time control"
```

### Task 2: `RobotDynamics` — Pinocchio, reduced to the arm, with armature

Joint-space dynamics, M(q) q̈ + C(q, q̇) q̇ + g(q) = τ. Two details matter:
- **Reduction:** the model is reduced to the arm joints. The fingers are locked at zero, and their small mass stays attached to the hand.
- **Armature:** the MJCF gives every Panda joint `armature="0.1"`, a reflected rotor inertia that MuJoCo adds to M's diagonal. Without it, M is off by 0.1 on the diagonal, which is large next to the wrist links' inertia. With it, Pinocchio's M matches MuJoCo's to 3e-8 and g to 3e-9. Task 4's `test_dynamics_model.py` checks that against the sim.

**Files:**
- Create: `src/arm_sandbox_controllers/package.xml`, `src/arm_sandbox_controllers/CMakeLists.txt`
- Create: `src/arm_sandbox_controllers/include/arm_sandbox_controllers/robot_dynamics.hpp`, `src/arm_sandbox_controllers/src/robot_dynamics.cpp`
- Test: `src/arm_sandbox_controllers/test/control_fixture.hpp`, `src/arm_sandbox_controllers/test/test_robot_dynamics.cpp`

**Interfaces:**
- Consumes: the URDF and `robot.yaml` of `arm_sandbox_description` (the tests generate the URDF from the xacro, as in Plan 03).
- Produces (namespace `arm_sandbox_controllers`, CMake target `control_core`):
  - `RobotDynamics(const std::string & urdf_xml, const std::vector<std::string> & arm_joints, const Eigen::VectorXd & armature)`. It throws `std::invalid_argument` on a bad URDF, a missing joint, a joint order different from `arm_joints`, or a wrong armature size.
  - `num_joints()`, `effort_limits()` (from the URDF), and `update(q, qd)`, which never allocates. After `update`: `mass_matrix()` (symmetric), `nonlinear_effects()` (C q̇ + g) and `gravity()`.
  - `forward_dynamics(q, qd, tau) -> q̈` (ABA), used as the simulated plant in tests.
  - Test helpers (namespace `arm_sandbox_controllers::test`): `robot_urdf()`, `robot_config()`, and `Plant(urdf, joints, armature, q0)` with `.step(tau, dt)`, `.q`, `.qd`.

- [ ] **Step 1: Package skeleton**

```bash
mkdir -p src/arm_sandbox_controllers/{include/arm_sandbox_controllers,src,test}
```

`src/arm_sandbox_controllers/package.xml`. It already lists the plugin dependencies of Task 4; they're harmless now.

```xml
<?xml version="1.0"?>
<package format="3">
  <name>arm_sandbox_controllers</name>
  <version>0.1.0</version>
  <description>Task-space controllers for arm-sandbox: a ROS-free control core (operational-space and Cartesian impedance control, Pinocchio dynamics) and its ros2_control plugins.</description>
  <maintainer email="robertocw_18@hotmail.com">Roberto Cai</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>arm_sandbox_kinematics</depend>
  <depend>controller_interface</depend>
  <depend>eigen</depend>
  <depend>geometry_msgs</depend>
  <depend>hardware_interface</depend>
  <depend>pinocchio</depend>
  <depend>pluginlib</depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>realtime_tools</depend>
  <depend>tf2_eigen</depend>

  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>arm_sandbox_description</test_depend>
  <test_depend>xacro</test_depend>
  <test_depend>yaml_cpp_vendor</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

`src/arm_sandbox_controllers/CMakeLists.txt`. This is the final file. Until Task 4, leave out:
- `src/task_space_control.cpp` from `CONTROL_CORE_SOURCES` (Task 3 adds it);
- the `task_space_controllers` library block, its `install(TARGETS task_space_controllers …)` line and `pluginlib_export_plugin_description_file` (Task 4);
- `test_task_space_control` from the `foreach` list, and the whole `test_realtime` block (Task 3).

```cmake
cmake_minimum_required(VERSION 3.16)
project(arm_sandbox_controllers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -Werror)
# Eigen/Pinocchio math is far slower unoptimized; colcon passes no build type by default.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(arm_sandbox_kinematics REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2_eigen REQUIRED)

# ROS-free control core: dynamics + task-space control laws.
set(CONTROL_CORE_SOURCES src/robot_dynamics.cpp src/task_space_control.cpp)
add_library(control_core SHARED ${CONTROL_CORE_SOURCES})
target_include_directories(control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(control_core PUBLIC
  arm_sandbox_kinematics::arm_sandbox_kinematics Eigen3::Eigen pinocchio::pinocchio)

# Thin ros2_control plugins on top of the core.
add_library(task_space_controllers SHARED src/task_space_controller.cpp)
target_link_libraries(task_space_controllers control_core)
ament_target_dependencies(task_space_controllers
  controller_interface geometry_msgs hardware_interface pluginlib rclcpp rclcpp_lifecycle realtime_tools tf2_eigen)
pluginlib_export_plugin_description_file(controller_interface arm_sandbox_controllers.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS task_space_controllers
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS control_core EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(arm_sandbox_kinematics Eigen3 pinocchio)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(arm_sandbox_description REQUIRED)
  find_package(yaml_cpp_vendor REQUIRED)
  find_program(XACRO_EXECUTABLE xacro REQUIRED)

  # Same approach as arm_sandbox_kinematics: test on the real robot's URDF and robot.yaml.
  set(TEST_ROBOT panda)
  set(TEST_ROBOT_DIR "${arm_sandbox_description_DIR}/../${TEST_ROBOT}")
  set(TEST_URDF "${CMAKE_CURRENT_BINARY_DIR}/${TEST_ROBOT}.urdf")
  add_custom_command(
    OUTPUT ${TEST_URDF}
    COMMAND ${XACRO_EXECUTABLE} ${TEST_ROBOT_DIR}/urdf/${TEST_ROBOT}.urdf.xacro -o ${TEST_URDF}
    DEPENDS ${TEST_ROBOT_DIR}/urdf/${TEST_ROBOT}.urdf.xacro
    COMMENT "Generating ${TEST_ROBOT}.urdf for the control tests")
  add_custom_target(test_urdf ALL DEPENDS ${TEST_URDF})
  set(TEST_DEFINITIONS TEST_URDF_PATH="${TEST_URDF}" TEST_ROBOT_CONFIG_PATH="${TEST_ROBOT_DIR}/config/robot.yaml")

  foreach(test_name test_robot_dynamics test_task_space_control)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    add_dependencies(${test_name} test_urdf)
    target_link_libraries(${test_name} control_core yaml-cpp)
    target_compile_definitions(${test_name} PRIVATE ${TEST_DEFINITIONS})
  endforeach()

  # Real-time check: the control-core sources compiled into the test with Eigen's allocation
  # guard on, and NDEBUG undefined so the guard's assert fires.
  ament_add_gtest(test_realtime test/test_realtime.cpp ${CONTROL_CORE_SOURCES})
  add_dependencies(test_realtime test_urdf)
  target_include_directories(test_realtime PRIVATE include)
  target_link_libraries(test_realtime arm_sandbox_kinematics::arm_sandbox_kinematics Eigen3::Eigen pinocchio::pinocchio yaml-cpp)
  target_compile_definitions(test_realtime PRIVATE EIGEN_RUNTIME_NO_MALLOC ${TEST_DEFINITIONS})
  target_compile_options(test_realtime PRIVATE -UNDEBUG)
endif()

ament_package()
```

- [ ] **Step 2: Write the failing tests**

`src/arm_sandbox_controllers/test/control_fixture.hpp`:

```cpp
#pragma once

// Shared by the control-core tests: the test robot's URDF (generated by CMake from its xacro),
// its robot.yaml, and a simulated plant (Pinocchio forward dynamics, semi-implicit Euler).

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include "arm_sandbox_controllers/robot_dynamics.hpp"

namespace arm_sandbox_controllers::test
{

struct RobotConfig
{
  std::vector<std::string> arm_joints;
  std::string base_frame;
  std::string ee_frame;
  Eigen::VectorXd home;
};

inline std::string robot_urdf()
{
  std::ifstream file(TEST_URDF_PATH);
  if (!file) {
    throw std::runtime_error("cannot read " TEST_URDF_PATH);
  }
  std::stringstream contents;
  contents << file.rdbuf();
  return contents.str();
}

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

/// The simulated arm: its own RobotDynamics (gravity always acts on it), stepped with
/// semi-implicit Euler at the controller rate.
class Plant
{
public:
  Plant(const std::string & urdf, const std::vector<std::string> & joints, const Eigen::VectorXd & armature, const Eigen::VectorXd & q0)
  : dynamics_(urdf, joints, armature), q(q0), qd(Eigen::VectorXd::Zero(q0.size()))
  {
  }

  void step(const Eigen::VectorXd & tau, double dt)
  {
    qd += dynamics_.forward_dynamics(q, qd, tau) * dt;
    q += qd * dt;
  }

private:
  RobotDynamics dynamics_;

public:
  Eigen::VectorXd q;
  Eigen::VectorXd qd;
};

}  // namespace arm_sandbox_controllers::test
```

`src/arm_sandbox_controllers/test/test_robot_dynamics.cpp`:

```cpp
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
```

Run: `make test`
Expected: FAIL at CMake configure with `Cannot find source file` (`src/robot_dynamics.cpp` doesn't exist yet).

- [ ] **Step 3: Write `RobotDynamics`**

```cpp
#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace arm_sandbox_controllers
{

/// Joint-space dynamics of the arm, M(q) q'' + C(q, q') q' + g(q) = tau, from Pinocchio on the URDF.
///
/// The model is reduced to `arm_joints` (other joints, e.g. the fingers, are locked at zero) and
/// gets the actuators' `armature` (reflected rotor inertia) added to the diagonal of M, as the
/// MJCF does. ROS-free. After construction, `update` and `forward_dynamics` never allocate.
class RobotDynamics
{
public:
  /// Throws std::invalid_argument if the URDF doesn't parse, an arm joint is missing, the reduced
  /// model's joint order differs from `arm_joints`, or `armature` has the wrong size.
  RobotDynamics(
    const std::string & urdf_xml, const std::vector<std::string> & arm_joints, const Eigen::VectorXd & armature);

  std::size_t num_joints() const { return static_cast<std::size_t>(model_.nv); }
  /// URDF effort limits of the arm joints (N m or N).
  const Eigen::VectorXd & effort_limits() const { return effort_limits_; }

  /// Computes M(q), C(q, q') q' + g(q) and g(q). Read them with the getters below.
  void update(const Eigen::VectorXd & q, const Eigen::VectorXd & qd);
  const Eigen::MatrixXd & mass_matrix() const { return mass_matrix_; }  ///< symmetric
  const Eigen::VectorXd & nonlinear_effects() const { return data_.nle; }  ///< C q' + g
  const Eigen::VectorXd & gravity() const { return data_.g; }

  /// q'' for torques `tau` (articulated-body algorithm). Used as the simulated plant in tests.
  const Eigen::VectorXd & forward_dynamics(const Eigen::VectorXd & q, const Eigen::VectorXd & qd, const Eigen::VectorXd & tau);

private:
  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::MatrixXd mass_matrix_;
  Eigen::VectorXd effort_limits_;
};

}  // namespace arm_sandbox_controllers
```

```cpp
#include "arm_sandbox_controllers/robot_dynamics.hpp"

#include <algorithm>
#include <stdexcept>

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace arm_sandbox_controllers
{

RobotDynamics::RobotDynamics(
  const std::string & urdf_xml, const std::vector<std::string> & arm_joints, const Eigen::VectorXd & armature)
{
  pinocchio::Model full;
  try {
    pinocchio::urdf::buildModelFromXML(urdf_xml, full);
  } catch (const std::exception & error) {
    throw std::invalid_argument(std::string("RobotDynamics: the URDF does not parse: ") + error.what());
  }
  for (const auto & name : arm_joints) {
    if (!full.existJointName(name)) {
      throw std::invalid_argument("RobotDynamics: arm joint '" + name + "' not in the URDF");
    }
  }

  // Lock every joint that isn't an arm joint (e.g. the fingers) at its neutral position.
  std::vector<pinocchio::JointIndex> locked;
  for (pinocchio::JointIndex id = 1; id < static_cast<pinocchio::JointIndex>(full.njoints); ++id) {
    if (std::find(arm_joints.begin(), arm_joints.end(), full.names[id]) == arm_joints.end()) {
      locked.push_back(id);
    }
  }
  model_ = pinocchio::buildReducedModel(full, locked, pinocchio::neutral(full));

  const std::vector<std::string> reduced_names(model_.names.begin() + 1, model_.names.end());
  if (reduced_names != arm_joints) {
    throw std::invalid_argument("RobotDynamics: the URDF's joint order differs from arm_joints");
  }
  if (armature.size() != model_.nv) {
    throw std::invalid_argument("RobotDynamics: armature needs one value per arm joint");
  }
  model_.armature = armature;
  data_ = pinocchio::Data(model_);
  mass_matrix_ = Eigen::MatrixXd::Zero(model_.nv, model_.nv);
  effort_limits_ = model_.effortLimit;
}

void RobotDynamics::update(const Eigen::VectorXd & q, const Eigen::VectorXd & qd)
{
  pinocchio::crba(model_, data_, q);  // fills the upper triangle of data_.M (armature included)
  mass_matrix_.triangularView<Eigen::Upper>() = data_.M.triangularView<Eigen::Upper>();
  mass_matrix_.triangularView<Eigen::StrictlyLower>() = data_.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::nonLinearEffects(model_, data_, q, qd);
  pinocchio::computeGeneralizedGravity(model_, data_, q);
}

const Eigen::VectorXd & RobotDynamics::forward_dynamics(
  const Eigen::VectorXd & q, const Eigen::VectorXd & qd, const Eigen::VectorXd & tau)
{
  return pinocchio::aba(model_, data_, q, qd, tau);
}

}  // namespace arm_sandbox_controllers
```

- [ ] **Step 4: Run the tests to see them pass**

Run: `make test`
Expected: `test_robot_dynamics` 3 passed. (A full first build of this package takes about 2 minutes: Pinocchio's templates are heavy.)

- [ ] **Step 5: Commit**

```bash
git add src/arm_sandbox_controllers
git commit -m "feat(controllers): RobotDynamics from Pinocchio, reduced to the arm, with armature"
```

### Task 3: The control laws — OSC and Cartesian impedance

All three equations work at the end effector, with e = [p* − p; θ·axis(R* Rᵀ)] (clamped) and ẋ = J q̇:

- **Shared part:**
  - Torque: τ = Jᵀ F + Nᵀ τ₀ + C q̇ (+ g if `compensate_gravity`), clamped to the URDF effort limits.
  - Null-space task: τ₀ = k (q_rest − q) − d q̇, projected with Nᵀ = I − Jᵀ J̄ᵀ.
  - J̄ = M⁻¹ Jᵀ Λ, where Λ = (J M⁻¹ Jᵀ + ε I)⁻¹ is the task-space inertia.
- **OSC (Khatib):** F = Λ (Kp e − Kd ẋ), with Kd = 2ζ√Kp. Λ makes the end effector respond like a unit mass in every direction: stiff, decoupled tracking.
- **Cartesian impedance (Hogan):** F = K e − D ẋ, with D = 2ζ√K. A push F_ext settles at e ≈ K⁻¹ F_ext, like a spring.

The tests close the loop on a simulated plant (the same Pinocchio model, ABA at 1 kHz) instead of trusting formulas. Two findings from writing this plan:
- **A real-time bug the guard caught.** The first version computed `Jᵀ (J̄ᵀ τ₀)` as one nested expression. Eigen evaluated the inner product into a heap temporary, and `test_realtime` aborted. Splitting it through a preallocated `Vector6d` fixed it.
- **The spring test has teeth.** With the impedance law accidentally scaled by Λ (OSC-like), the push also deflected the arm sideways, and `ImpedanceDeflectsLikeASpring` failed.

**Files:**
- Create: `src/arm_sandbox_controllers/include/arm_sandbox_controllers/task_space_control.hpp`, `src/arm_sandbox_controllers/src/task_space_control.cpp`
- Test: `src/arm_sandbox_controllers/test/test_task_space_control.cpp`, `src/arm_sandbox_controllers/test/test_realtime.cpp`
- Modify: `src/arm_sandbox_controllers/CMakeLists.txt` (restore `task_space_control.cpp`, `test_task_space_control` and the `test_realtime` block)

**Interfaces:**
- Consumes: `KinematicChain` with the Task 1 overloads; `RobotDynamics` (Task 2).
- Produces:
  - `struct TaskSpaceSettings { double max_position_error, max_orientation_error; Eigen::VectorXd rest_configuration; double nullspace_stiffness, nullspace_damping, singularity_damping; bool compensate_gravity; }`
  - `struct OperationalSpaceGains { double position_stiffness, orientation_stiffness, damping_ratio; }`
  - `struct ImpedanceGains { double translational_stiffness, rotational_stiffness, damping_ratio; }`
  - `class TaskSpaceControl` with `compute(q, qd, target, tau)` (never allocates; throws `std::invalid_argument` on wrong sizes), `pose_error()` (`Vector6d`), `ee_pose()` and `num_joints()`.
  - `OperationalSpaceControl(chain, dynamics, settings, OperationalSpaceGains)` and `CartesianImpedanceControl(chain, dynamics, settings, ImpedanceGains)`. Both throw `std::invalid_argument` on invalid settings or gains.

- [ ] **Step 1: Write the failing tests**

`src/arm_sandbox_controllers/test/test_task_space_control.cpp`:

```cpp
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
```

`src/arm_sandbox_controllers/test/test_realtime.cpp`:

```cpp
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
```

Restore the CMake lines named in the Files list above. Run: `make test`
Expected: FAIL at CMake configure with `Cannot find source file` (`src/task_space_control.cpp`).

- [ ] **Step 2: Write the laws**

```cpp
#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include "arm_sandbox_controllers/robot_dynamics.hpp"
#include "arm_sandbox_kinematics/kinematic_chain.hpp"

namespace arm_sandbox_controllers
{

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

/// Settings shared by both task-space controllers. All come from the controller YAML.
struct TaskSpaceSettings
{
  /// The pose error fed to the law is clamped to these norms, so a far-away target gives a bounded
  /// pull instead of a huge force (the arm then moves at a bounded rate towards it).
  double max_position_error = 0.0;     ///< m, > 0
  double max_orientation_error = 0.0;  ///< rad, > 0
  /// Null-space task: a joint-space spring towards `rest_configuration` (e.g. home), projected so
  /// it doesn't disturb the end-effector. Keeps the redundant 7th degree of freedom from drifting.
  Eigen::VectorXd rest_configuration;
  double nullspace_stiffness = 0.0;  ///< N m / rad, >= 0
  double nullspace_damping = 0.0;    ///< N m s / rad, >= 0
  /// Regularizes the task-space inertia (J M^-1 J^T + eps I)^-1 near singularities, > 0.
  double singularity_damping = 0.0;
  /// Add g(q). False when the robot (or the sim with gravcomp:=true) already compensates gravity.
  bool compensate_gravity = true;
};

/// Shared part of the task-space controllers: tau = J^T F + N^T tau_0 + C q' (+ g), clamped to the
/// effort limits. F is the task-space force computed by the derived law; tau_0 is the null-space
/// task; N^T = I - J^T Jbar^T projects it with the dynamically consistent inverse
/// Jbar = M^-1 J^T Lambda, Lambda = (J M^-1 J^T)^-1. ROS-free. `compute` never allocates.
class TaskSpaceControl
{
public:
  virtual ~TaskSpaceControl() = default;

  /// Torques that drive the end effector towards `target` (pose in the chain's base frame).
  /// `tau` must have num_joints() entries. Throws std::invalid_argument on wrong sizes.
  void compute(const Eigen::VectorXd & q, const Eigen::VectorXd & qd, const Eigen::Isometry3d & target, Eigen::VectorXd & tau);

  /// Clamped pose error [position; orientation] from the last `compute`.
  const Vector6d & pose_error() const { return error_; }
  /// End-effector pose from the last `compute`.
  const Eigen::Isometry3d & ee_pose() const { return pose_; }
  std::size_t num_joints() const { return chain_.num_joints(); }

protected:
  /// Throws std::invalid_argument if the settings are invalid or don't match the chain/dynamics.
  TaskSpaceControl(const arm_sandbox_kinematics::KinematicChain & chain, RobotDynamics & dynamics, TaskSpaceSettings settings);

  /// The law: task-space force F from the pose error and the end-effector twist J q'.
  /// `task_inertia` is Lambda. Must not allocate.
  virtual void task_force(const Vector6d & error, const Vector6d & twist, const Matrix6d & task_inertia, Vector6d & force) const = 0;

private:
  const arm_sandbox_kinematics::KinematicChain & chain_;
  RobotDynamics & dynamics_;
  TaskSpaceSettings settings_;

  // Preallocated workspace (sized in the constructor).
  Eigen::Isometry3d pose_;
  arm_sandbox_kinematics::Jacobian jacobian_;
  Vector6d error_;
  Vector6d twist_;
  Vector6d force_;
  Eigen::LDLT<Eigen::MatrixXd> mass_ldlt_;
  Eigen::MatrixXd minv_jt_;          // M^-1 J^T, n x 6
  Matrix6d task_inertia_;            // Lambda
  Eigen::LDLT<Matrix6d> lambda_inverse_ldlt_;
  Eigen::MatrixXd jbar_;             // M^-1 J^T Lambda, n x 6
  Eigen::VectorXd tau_null_;
  Vector6d jbar_t_tau_null_;         // Jbar^T tau_0
  Eigen::VectorXd projected_null_;
};

/// Operational-space control (Khatib): F = Lambda (Kp e - Kd x'), with Kd = 2 zeta sqrt(Kp).
/// Lambda makes the end effector behave like a unit mass in every direction, so the gains give
/// the same, decoupled response everywhere in the workspace: stiff, accurate pose tracking.
struct OperationalSpaceGains
{
  double position_stiffness = 0.0;     ///< 1/s^2, > 0
  double orientation_stiffness = 0.0;  ///< 1/s^2, > 0
  double damping_ratio = 0.0;          ///< > 0, 1 = critical
};

class OperationalSpaceControl : public TaskSpaceControl
{
public:
  OperationalSpaceControl(
    const arm_sandbox_kinematics::KinematicChain & chain, RobotDynamics & dynamics, TaskSpaceSettings settings,
    OperationalSpaceGains gains);

private:
  void task_force(const Vector6d & error, const Vector6d & twist, const Matrix6d & task_inertia, Vector6d & force) const override;

  Vector6d stiffness_;
  Vector6d damping_;
};

/// Cartesian impedance control: F = K e - D x', a spring-damper between the end effector and the
/// target, D = 2 zeta sqrt(K). Pushing the end effector by F_ext displaces it by about K^-1 F_ext:
/// compliant contact, e.g. pulling a drawer whose path doesn't match the target exactly.
struct ImpedanceGains
{
  double translational_stiffness = 0.0;  ///< N/m, > 0
  double rotational_stiffness = 0.0;     ///< N m / rad, > 0
  double damping_ratio = 0.0;            ///< > 0
};

class CartesianImpedanceControl : public TaskSpaceControl
{
public:
  CartesianImpedanceControl(
    const arm_sandbox_kinematics::KinematicChain & chain, RobotDynamics & dynamics, TaskSpaceSettings settings,
    ImpedanceGains gains);

private:
  void task_force(const Vector6d & error, const Vector6d & twist, const Matrix6d & task_inertia, Vector6d & force) const override;

  Vector6d stiffness_;
  Vector6d damping_;
};

}  // namespace arm_sandbox_controllers
```

```cpp
#include "arm_sandbox_controllers/task_space_control.hpp"

#include <cmath>
#include <stdexcept>

namespace arm_sandbox_controllers
{
namespace
{
/// Scales `vector` down to `max_norm` if it is longer (keeps its direction).
template <typename Vector>
void clamp_norm(Vector && vector, double max_norm)
{
  const double norm = vector.norm();
  if (norm > max_norm) {
    vector *= max_norm / norm;
  }
}

Vector6d critical_damping(const Vector6d & stiffness, double damping_ratio)
{
  return 2.0 * damping_ratio * stiffness.cwiseSqrt();
}
}  // namespace

TaskSpaceControl::TaskSpaceControl(
  const arm_sandbox_kinematics::KinematicChain & chain, RobotDynamics & dynamics, TaskSpaceSettings settings)
: chain_(chain), dynamics_(dynamics), settings_(std::move(settings))
{
  const auto n = static_cast<Eigen::Index>(chain_.num_joints());
  if (dynamics_.num_joints() != chain_.num_joints()) {
    throw std::invalid_argument("TaskSpaceControl: kinematics and dynamics have different joint counts");
  }
  if (settings_.rest_configuration.size() != n) {
    throw std::invalid_argument("TaskSpaceControl: rest_configuration needs one value per joint");
  }
  if (settings_.max_position_error <= 0.0 || settings_.max_orientation_error <= 0.0 ||
      settings_.singularity_damping <= 0.0 || settings_.nullspace_stiffness < 0.0 || settings_.nullspace_damping < 0.0) {
    throw std::invalid_argument(
      "TaskSpaceControl: max errors and singularity_damping must be > 0, null-space gains >= 0");
  }
  jacobian_.resize(6, n);
  mass_ldlt_ = Eigen::LDLT<Eigen::MatrixXd>(n);
  minv_jt_.resize(n, 6);
  jbar_.resize(n, 6);
  tau_null_.resize(n);
  projected_null_.resize(n);
}

void TaskSpaceControl::compute(
  const Eigen::VectorXd & q, const Eigen::VectorXd & qd, const Eigen::Isometry3d & target, Eigen::VectorXd & tau)
{
  const auto n = static_cast<Eigen::Index>(chain_.num_joints());
  if (q.size() != n || qd.size() != n || tau.size() != n) {
    throw std::invalid_argument("TaskSpaceControl::compute: q, qd and tau need one value per joint");
  }

  // Kinematics (hand-written, arm_sandbox_kinematics) and dynamics (Pinocchio).
  chain_.fk(q, pose_);
  chain_.jacobian(q, jacobian_);
  dynamics_.update(q, qd);

  // Pose error [p* - p; angle * axis of R* R^T], clamped; end-effector twist J q'.
  error_.head<3>() = target.translation() - pose_.translation();
  const Eigen::AngleAxisd rotation_error(target.linear() * pose_.linear().transpose());
  error_.tail<3>() = rotation_error.angle() * rotation_error.axis();
  clamp_norm(error_.head<3>(), settings_.max_position_error);
  clamp_norm(error_.tail<3>(), settings_.max_orientation_error);
  twist_.noalias() = jacobian_.lazyProduct(qd);

  // Task-space inertia Lambda = (J M^-1 J^T + eps I)^-1.
  mass_ldlt_.compute(dynamics_.mass_matrix());
  minv_jt_ = jacobian_.transpose();
  mass_ldlt_.solveInPlace(minv_jt_);
  Matrix6d lambda_inverse = jacobian_.lazyProduct(minv_jt_);
  lambda_inverse.diagonal().array() += settings_.singularity_damping;
  lambda_inverse_ldlt_.compute(lambda_inverse);
  task_inertia_ = lambda_inverse_ldlt_.solve(Matrix6d::Identity());

  task_force(error_, twist_, task_inertia_, force_);

  // Null-space task, projected with N^T = I - J^T Jbar^T.
  tau_null_ = settings_.nullspace_stiffness * (settings_.rest_configuration - q) - settings_.nullspace_damping * qd;
  jbar_.noalias() = minv_jt_.lazyProduct(task_inertia_);
  // Two steps through a fixed-size 6-vector: a nested product would evaluate into a heap temporary.
  jbar_t_tau_null_.noalias() = jbar_.transpose() * tau_null_;
  projected_null_ = tau_null_;
  projected_null_.noalias() -= jacobian_.transpose() * jbar_t_tau_null_;

  // tau = J^T F + N^T tau_0 + C q' (+ g).
  tau.noalias() = jacobian_.transpose().lazyProduct(force_);
  tau += projected_null_ + dynamics_.nonlinear_effects();
  if (!settings_.compensate_gravity) {
    tau -= dynamics_.gravity();
  }
  tau = tau.cwiseMax(-dynamics_.effort_limits()).cwiseMin(dynamics_.effort_limits());
}

OperationalSpaceControl::OperationalSpaceControl(
  const arm_sandbox_kinematics::KinematicChain & chain, RobotDynamics & dynamics, TaskSpaceSettings settings,
  OperationalSpaceGains gains)
: TaskSpaceControl(chain, dynamics, std::move(settings))
{
  if (gains.position_stiffness <= 0.0 || gains.orientation_stiffness <= 0.0 || gains.damping_ratio <= 0.0) {
    throw std::invalid_argument("OperationalSpaceControl: stiffnesses and damping_ratio must be > 0");
  }
  stiffness_ << Eigen::Vector3d::Constant(gains.position_stiffness), Eigen::Vector3d::Constant(gains.orientation_stiffness);
  damping_ = critical_damping(stiffness_, gains.damping_ratio);
}

void OperationalSpaceControl::task_force(
  const Vector6d & error, const Vector6d & twist, const Matrix6d & task_inertia, Vector6d & force) const
{
  // A desired acceleration of a unit mass, scaled by the real task-space inertia.
  const Vector6d acceleration = stiffness_.cwiseProduct(error) - damping_.cwiseProduct(twist);
  force.noalias() = task_inertia * acceleration;
}

CartesianImpedanceControl::CartesianImpedanceControl(
  const arm_sandbox_kinematics::KinematicChain & chain, RobotDynamics & dynamics, TaskSpaceSettings settings,
  ImpedanceGains gains)
: TaskSpaceControl(chain, dynamics, std::move(settings))
{
  if (gains.translational_stiffness <= 0.0 || gains.rotational_stiffness <= 0.0 || gains.damping_ratio <= 0.0) {
    throw std::invalid_argument("CartesianImpedanceControl: stiffnesses and damping_ratio must be > 0");
  }
  stiffness_ << Eigen::Vector3d::Constant(gains.translational_stiffness), Eigen::Vector3d::Constant(gains.rotational_stiffness);
  damping_ = critical_damping(stiffness_, gains.damping_ratio);
}

void CartesianImpedanceControl::task_force(
  const Vector6d & error, const Vector6d & twist, const Matrix6d & /*task_inertia*/, Vector6d & force) const
{
  // A spring-damper: no inertia shaping, so contact forces stay those of the chosen stiffness.
  force = stiffness_.cwiseProduct(error) - damping_.cwiseProduct(twist);
}

}  // namespace arm_sandbox_controllers
```

- [ ] **Step 3: Run the tests to see them pass**

Run: `make test`
Expected: `test_task_space_control` 6 passed, `test_realtime` 2 passed, and `test_robot_dynamics` 3 passed.

- [ ] **Step 4: Plant one bug to check the spring test**

In `CartesianImpedanceControl::task_force`, temporarily use the inertia: `force = task_inertia * (stiffness_.cwiseProduct(error) - damping_.cwiseProduct(twist));`.
Expected: `ImpedanceDeflectsLikeASpring` FAILS: the push now also moves the arm sideways by about 2.5 mm, while the test allows 1 mm. Revert, then run `make test`; everything passes.

- [ ] **Step 5: Commit**

```bash
git add src/arm_sandbox_controllers
git commit -m "feat(controllers): operational-space and Cartesian impedance laws on a shared, allocation-free core"
```

---

# Part 2 — Controllers in the Sim

### Task 4: The `ros2_control` plugins and the launch wiring

`TaskSpaceController` is a thin `ControllerInterface`:
- **Configure:** reads its parameters, then builds the chain, the dynamics and the law. Humble controllers can't fetch the URDF themselves (no `get_robot_description()` yet), so it arrives as a parameter.
- **Activate:** holds the current pose until a target arrives.
- **Each update:** reads q and q̇, calls `compute`, and writes efforts.
- **Topics:** takes targets on `~/target_pose` through a `RealtimeBuffer`, and publishes `~/pose_error` through a `RealtimePublisher`. That topic exists only while the controller is active, so its first message also means "ready".

`OscController` and `CartesianImpedanceController` only declare their gains and create their law.

`sim.launch.py` changes:
- **One source for robot facts.** It writes a small parameter file for the task-space controllers with `joints`, `base_frame`, `ee_frame` and `rest_configuration` from `robot.yaml`, `robot_description` from the URDF, and `compensate_gravity = not gravcomp`. That way gravity is compensated exactly once (D20).
- **A YAML pitfall.** PyYAML emits aliases (`&id001`/`*id001`) for shared objects, and `rcl`'s parser rejects them: "Will not support aliasing". The launch writes the file with a no-alias dumper.
- **Controller choice:** all arm controllers load inactive; `activate_at_home` activates the one named by `arm_controller:=` (default `arm_controller`, the JTC) plus the gripper.
- **The wrench plugin is opt-in**, via `external_wrench:=true`. `mujoco_ros2_control` 0.1.2's `ExternalWrenchPlugin` makes `ros2_control_node` segfault on shutdown (exit −11). This was confirmed by toggling only the plugin, so only tests and demos that push on the robot enable it (D22).

**Files:**
- Create: `src/arm_sandbox_controllers/include/arm_sandbox_controllers/task_space_controller.hpp`, `src/arm_sandbox_controllers/src/task_space_controller.cpp`, `src/arm_sandbox_controllers/arm_sandbox_controllers.xml`
- Modify: `src/arm_sandbox_controllers/CMakeLists.txt` (restore the plugin library lines)
- Modify: `src/arm_sandbox_bringup/config/panda_controllers.yaml`, `src/arm_sandbox_bringup/launch/sim.launch.py`, `src/arm_sandbox_bringup/package.xml`, `src/arm_sandbox_bringup/CMakeLists.txt`
- Create: `src/arm_sandbox_bringup/config/sim_plugins.yaml`
- Test: `src/arm_sandbox_bringup/test/test_dynamics_model.py`

**Interfaces:**
- Consumes: `OperationalSpaceControl`, `CartesianImpedanceControl`, `RobotDynamics`, `KinematicChain`.
- Produces:
  - Plugins `arm_sandbox_controllers/OscController` and `arm_sandbox_controllers/CartesianImpedanceController` (`controller_interface::ControllerInterface`), controller names `osc_controller` and `cartesian_impedance_controller`.
  - Topics `/<controller>/target_pose` (`geometry_msgs/PoseStamped`, in `base_frame`) and `/<controller>/pose_error` (`geometry_msgs/TwistStamped`).
  - `sim.launch.py` arguments `arm_controller:=arm_controller|osc_controller|cartesian_impedance_controller` and `external_wrench:=false|true`. With the wrench plugin on, the service is `/external_wrench/apply_wrench` (`mujoco_ros2_control_msgs/srv/ApplyExternalWrench`): request field `wrenches.external_wrenches`, force in the **body** frame (the docs say so; the message comment wrongly says world).

- [ ] **Step 1: Write the failing model test**

`src/arm_sandbox_bringup/test/test_dynamics_model.py`:

```python
"""The task-space controllers' dynamics model matches the simulated robot.

The controllers use Pinocchio on the URDF, reduced to the arm joints, plus the `armature` from the
controllers YAML (arm_sandbox_controllers::RobotDynamics). MuJoCo simulates the MJCF. Their mass
matrix M(q) and gravity torques g(q) must agree, or the controllers act on a wrong model.
"""

from pathlib import Path

import mujoco
import numpy as np
import pinocchio as pin
import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

ROBOT = "panda"
TASK_SPACE_CONTROLLERS = ["osc_controller", "cartesian_impedance_controller"]
NUM_RANDOM_CONFIGURATIONS = 50
RANDOM_SEED = 0
# Same model up to rounding; measured 3e-8 (M) and 3e-9 (g) when this test was written.
MODEL_TOLERANCE = 1e-6

ROBOT_DIR = Path(get_package_share_directory("arm_sandbox_description")) / ROBOT
CONTROLLERS_FILE = Path(get_package_share_directory("arm_sandbox_bringup")) / "config" / f"{ROBOT}_controllers.yaml"


@pytest.fixture(scope="module")
def robot_config() -> dict:
    return yaml.safe_load((ROBOT_DIR / "config" / "robot.yaml").read_text())


@pytest.fixture(scope="module")
def controllers() -> dict:
    return yaml.safe_load(CONTROLLERS_FILE.read_text())


@pytest.fixture(scope="module")
def mj_model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(ROBOT_DIR / "mjcf" / "panda.xml"))


def reduced_model(robot_config: dict, armature: list[float]) -> pin.Model:
    """What RobotDynamics builds: the URDF reduced to the arm joints, plus armature."""
    full = pin.buildModelFromXML(xacro.process_file(str(ROBOT_DIR / "urdf" / f"{ROBOT}.urdf.xacro")).toxml())
    locked = [full.getJointId(name) for name in full.names[1:] if name not in robot_config["arm_joints"]]
    model = pin.buildReducedModel(full, locked, pin.neutral(full))
    model.armature = np.array(armature)
    return model


@pytest.mark.parametrize("controller", TASK_SPACE_CONTROLLERS)
def test_armature_matches_mjcf(controllers: dict, robot_config: dict, mj_model: mujoco.MjModel, controller: str) -> None:
    armature = controllers[controller]["ros__parameters"]["armature"]
    dofs = [mj_model.joint(joint).dofadr[0] for joint in robot_config["arm_joints"]]
    np.testing.assert_allclose(armature, mj_model.dof_armature[dofs])


def test_mass_matrix_and_gravity_match_mujoco(controllers: dict, robot_config: dict, mj_model: mujoco.MjModel) -> None:
    armature = controllers[TASK_SPACE_CONTROLLERS[0]]["ros__parameters"]["armature"]
    model = reduced_model(robot_config, armature)
    data = model.createData()
    mj_data = mujoco.MjData(mj_model)
    joints = robot_config["arm_joints"]
    dofs = [mj_model.joint(joint).dofadr[0] for joint in joints]
    qpos = [mj_model.joint(joint).qposadr[0] for joint in joints]
    rng = np.random.default_rng(RANDOM_SEED)

    for _ in range(NUM_RANDOM_CONFIGURATIONS):
        q = np.array([rng.uniform(*mj_model.joint(joint).range) for joint in joints])
        mj_data.qpos[:] = 0.0  # fingers closed, as in the reduced model (locked at neutral)
        mj_data.qpos[qpos] = q
        mj_data.qvel[:] = 0.0
        mujoco.mj_forward(mj_model, mj_data)
        full_mass = np.zeros((mj_model.nv, mj_model.nv))
        mujoco.mj_fullM(mj_model, mj_data, full_mass)

        mass = pin.crba(model, data, q)
        mass = np.triu(mass) + np.triu(mass, 1).T  # crba fills the upper triangle
        gravity = pin.computeGeneralizedGravity(model, data, q)

        np.testing.assert_allclose(mass, full_mass[np.ix_(dofs, dofs)], atol=MODEL_TOLERANCE)
        # At zero velocity MuJoCo's bias force is gravity alone.
        np.testing.assert_allclose(gravity, mj_data.qfrc_bias[dofs], atol=MODEL_TOLERANCE)
```

Replace `src/arm_sandbox_bringup/CMakeLists.txt` and `src/arm_sandbox_bringup/package.xml`. The new versions register `test_dynamics_model` and `test_impedance_compliance`; the latter is Task 5's, so comment out its `add_launch_test` line until then.

```cmake
cmake_minimum_required(VERSION 3.16)
project(arm_sandbox_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/activate_at_home DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # The task-space controllers' dynamics model (URDF + armature) vs the simulated MJCF.
  ament_add_pytest_test(test_dynamics_model test/test_dynamics_model.py)
  # Each launch test gets its own ROS domain, so sims started by tests of different packages
  # (colcon runs packages in parallel) or a `make sim` in another terminal can't interfere.
  add_launch_test(test/test_sim_bringup.py TIMEOUT 180 ENV ROS_DOMAIN_ID=41)
  add_launch_test(test/test_arm_trajectory.py TIMEOUT 300 ENV ROS_DOMAIN_ID=42)
  add_launch_test(test/test_sim_recording.py TIMEOUT 180 ENV ROS_DOMAIN_ID=44)
  add_launch_test(test/test_impedance_compliance.py TIMEOUT 180 ENV ROS_DOMAIN_ID=46)
endif()

ament_package()
```

```xml
<?xml version="1.0"?>
<package format="3">
  <name>arm_sandbox_bringup</name>
  <version>0.1.0</version>
  <description>Top-level launch files and per-robot controller configs for arm-sandbox.</description>
  <maintainer email="robertocw_18@hotmail.com">Roberto Cai</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>arm_sandbox_controllers</exec_depend>
  <exec_depend>arm_sandbox_description</exec_depend>
  <exec_depend>arm_sandbox_sim</exec_depend>
  <exec_depend>arm_sandbox_viz</exec_depend>
  <exec_depend>builtin_interfaces</exec_depend>
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>controller_manager_msgs</exec_depend>
  <exec_depend>gripper_controllers</exec_depend>
  <exec_depend>joint_state_broadcaster</exec_depend>
  <exec_depend>joint_trajectory_controller</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>mujoco_ros2_control</exec_depend>
  <exec_depend>mujoco_ros2_control_msgs</exec_depend>
  <exec_depend>mujoco_ros2_control_plugins</exec_depend>
  <exec_depend>python3-yaml</exec_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>

  <test_depend>action_msgs</test_depend>
  <test_depend>ament_cmake_pytest</test_depend>
  <test_depend>pinocchio</test_depend>
  <test_depend>control_msgs</test_depend>
  <test_depend>launch_testing_ament_cmake</test_depend>
  <test_depend>launch_testing_ros</test_depend>
  <test_depend>rclpy</test_depend>
  <test_depend>rosgraph_msgs</test_depend>
  <test_depend>python3-numpy</test_depend>
  <test_depend>sensor_msgs</test_depend>
  <test_depend>tf2_ros</test_depend>
  <test_depend>trajectory_msgs</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

Run: `make test`
Expected: `test_dynamics_model` FAILS with `KeyError: 'osc_controller'` (the controllers YAML has no such section yet).

- [ ] **Step 2: Write the plugins**

```cpp
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include "arm_sandbox_controllers/robot_dynamics.hpp"
#include "arm_sandbox_controllers/task_space_control.hpp"
#include "arm_sandbox_kinematics/kinematic_chain.hpp"

namespace arm_sandbox_controllers
{

/// ros2_control wrapper shared by the task-space controllers: a thin layer around a
/// `TaskSpaceControl` law from the ROS-free core.
///
/// - Commands: `<joint>/effort`. State: `<joint>/position`, `<joint>/velocity`.
/// - Input: `~/target_pose` (geometry_msgs/PoseStamped, in `base_frame`). On activation the
///   controller holds the current pose until a target arrives.
/// - Output: `~/pose_error` (geometry_msgs/TwistStamped: linear = position error in m, angular =
///   orientation error as angle * axis in rad), published only while active.
/// - Parameters: robot (`joints`, `base_frame`, `ee_frame`, `robot_description`,
///   `rest_configuration`, `compensate_gravity`; injected by sim.launch.py from robot.yaml),
///   model (`armature`), shared settings, and the law's gains (see `declare_gains`).
class TaskSpaceController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  /// Declare the law's gain parameters (called from on_init).
  virtual void declare_gains() = 0;
  /// Build the law from the declared gains. May throw std::invalid_argument on bad gains.
  virtual std::unique_ptr<TaskSpaceControl> make_law(
    const arm_sandbox_kinematics::KinematicChain & chain, RobotDynamics & dynamics, const TaskSpaceSettings & settings) = 0;

private:
  void read_state();
  void on_target(const geometry_msgs::msg::PoseStamped & msg);

  std::vector<std::string> joints_;
  std::string base_frame_;
  std::unique_ptr<arm_sandbox_kinematics::KinematicChain> chain_;
  std::unique_ptr<RobotDynamics> dynamics_;
  std::unique_ptr<TaskSpaceControl> law_;

  Eigen::VectorXd q_;
  Eigen::VectorXd qd_;
  Eigen::VectorXd tau_;
  realtime_tools::RealtimeBuffer<Eigen::Isometry3d> target_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>> error_publisher_;
  rclcpp::Duration publish_period_{0, 0};
  rclcpp::Time last_publish_time_;
};

/// Operational-space controller (plugin `arm_sandbox_controllers/OscController`).
class OscController : public TaskSpaceController
{
protected:
  void declare_gains() override;
  std::unique_ptr<TaskSpaceControl> make_law(
    const arm_sandbox_kinematics::KinematicChain & chain, RobotDynamics & dynamics, const TaskSpaceSettings & settings) override;
};

/// Cartesian impedance controller (plugin `arm_sandbox_controllers/CartesianImpedanceController`).
class CartesianImpedanceController : public TaskSpaceController
{
protected:
  void declare_gains() override;
  std::unique_ptr<TaskSpaceControl> make_law(
    const arm_sandbox_kinematics::KinematicChain & chain, RobotDynamics & dynamics, const TaskSpaceSettings & settings) override;
};

}  // namespace arm_sandbox_controllers
```

```cpp
#include "arm_sandbox_controllers/task_space_controller.hpp"

#include <stdexcept>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace arm_sandbox_controllers
{
using controller_interface::CallbackReturn;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;

CallbackReturn TaskSpaceController::on_init()
{
  // Robot: injected by sim.launch.py from robot.yaml and the URDF (empty here = must be set).
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<std::string>("base_frame", "");
  auto_declare<std::string>("ee_frame", "");
  auto_declare<std::string>("robot_description", "");
  auto_declare<std::vector<double>>("rest_configuration", {});
  auto_declare<bool>("compensate_gravity", true);
  // Model and shared settings: per-robot controllers YAML. Zero defaults fail validation.
  auto_declare<std::vector<double>>("armature", {});
  auto_declare<double>("max_position_error", 0.0);
  auto_declare<double>("max_orientation_error", 0.0);
  auto_declare<double>("nullspace_stiffness", 0.0);
  auto_declare<double>("nullspace_damping", 0.0);
  auto_declare<double>("singularity_damping", 0.0);
  auto_declare<double>("state_publish_rate", 0.0);
  declare_gains();
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration TaskSpaceController::command_interface_configuration() const
{
  InterfaceConfiguration config{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

InterfaceConfiguration TaskSpaceController::state_interface_configuration() const
{
  // Order matters: read_state() expects all positions, then all velocities.
  InterfaceConfiguration config{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

CallbackReturn TaskSpaceController::on_configure(const rclcpp_lifecycle::State &)
{
  const auto node = get_node();
  joints_ = node->get_parameter("joints").as_string_array();
  base_frame_ = node->get_parameter("base_frame").as_string();
  const auto ee_frame = node->get_parameter("ee_frame").as_string();
  const auto urdf = node->get_parameter("robot_description").as_string();
  const auto rest = node->get_parameter("rest_configuration").as_double_array();
  const auto armature = node->get_parameter("armature").as_double_array();
  const double publish_rate = node->get_parameter("state_publish_rate").as_double();
  if (joints_.empty() || base_frame_.empty() || ee_frame.empty() || urdf.empty()) {
    RCLCPP_FATAL(node->get_logger(), "missing robot parameters: joints, base_frame, ee_frame, robot_description");
    return CallbackReturn::ERROR;
  }
  if (publish_rate <= 0.0) {
    RCLCPP_FATAL(node->get_logger(), "state_publish_rate must be > 0");
    return CallbackReturn::ERROR;
  }

  TaskSpaceSettings settings;
  settings.max_position_error = node->get_parameter("max_position_error").as_double();
  settings.max_orientation_error = node->get_parameter("max_orientation_error").as_double();
  settings.rest_configuration = Eigen::Map<const Eigen::VectorXd>(rest.data(), static_cast<Eigen::Index>(rest.size()));
  settings.nullspace_stiffness = node->get_parameter("nullspace_stiffness").as_double();
  settings.nullspace_damping = node->get_parameter("nullspace_damping").as_double();
  settings.singularity_damping = node->get_parameter("singularity_damping").as_double();
  settings.compensate_gravity = node->get_parameter("compensate_gravity").as_bool();

  try {
    chain_ = std::make_unique<arm_sandbox_kinematics::KinematicChain>(urdf, base_frame_, ee_frame);
    if (chain_->joint_names() != joints_) {
      throw std::invalid_argument("the chain " + base_frame_ + " -> " + ee_frame + " doesn't match `joints`");
    }
    dynamics_ = std::make_unique<RobotDynamics>(
      urdf, joints_, Eigen::Map<const Eigen::VectorXd>(armature.data(), static_cast<Eigen::Index>(armature.size())));
    law_ = make_law(*chain_, *dynamics_, settings);
  } catch (const std::invalid_argument & error) {
    RCLCPP_FATAL(node->get_logger(), "invalid configuration: %s", error.what());
    return CallbackReturn::ERROR;
  }

  const auto n = static_cast<Eigen::Index>(joints_.size());
  q_ = Eigen::VectorXd::Zero(n);
  qd_ = Eigen::VectorXd::Zero(n);
  tau_ = Eigen::VectorXd::Zero(n);

  target_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/target_pose", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::PoseStamped & msg) { on_target(msg); });
  error_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(
    node->create_publisher<geometry_msgs::msg::TwistStamped>("~/pose_error", rclcpp::SystemDefaultsQoS()));
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate);
  RCLCPP_INFO(node->get_logger(), "configured for %zu joints, %s -> %s, gravity compensation %s",
              joints_.size(), base_frame_.c_str(), ee_frame.c_str(), settings.compensate_gravity ? "on" : "off");
  return CallbackReturn::SUCCESS;
}

void TaskSpaceController::on_target(const geometry_msgs::msg::PoseStamped & msg)
{
  if (!msg.header.frame_id.empty() && msg.header.frame_id != base_frame_) {
    RCLCPP_WARN(get_node()->get_logger(), "ignoring target in frame '%s' (expected '%s')",
                msg.header.frame_id.c_str(), base_frame_.c_str());
    return;
  }
  Eigen::Isometry3d target;
  tf2::fromMsg(msg.pose, target);
  target_.writeFromNonRT(target);
}

void TaskSpaceController::read_state()
{
  const std::size_t n = joints_.size();
  for (std::size_t i = 0; i < n; ++i) {
    q_[static_cast<Eigen::Index>(i)] = state_interfaces_[i].get_value();
    qd_[static_cast<Eigen::Index>(i)] = state_interfaces_[n + i].get_value();
  }
}

CallbackReturn TaskSpaceController::on_activate(const rclcpp_lifecycle::State &)
{
  // Hold where the arm is now until a target arrives.
  read_state();
  target_.writeFromNonRT(chain_->fk(q_));
  last_publish_time_ = get_node()->now();
  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & command : command_interfaces_) {
    command.set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type TaskSpaceController::update(const rclcpp::Time & time, const rclcpp::Duration &)
{
  read_state();
  law_->compute(q_, qd_, *target_.readFromRT(), tau_);
  for (std::size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_interfaces_[i].set_value(tau_[static_cast<Eigen::Index>(i)]);
  }

  if (time - last_publish_time_ >= publish_period_ && error_publisher_->trylock()) {
    last_publish_time_ = time;
    auto & msg = error_publisher_->msg_;
    msg.header.stamp = time;
    msg.header.frame_id = base_frame_;
    const Vector6d & error = law_->pose_error();
    msg.twist.linear.x = error[0];
    msg.twist.linear.y = error[1];
    msg.twist.linear.z = error[2];
    msg.twist.angular.x = error[3];
    msg.twist.angular.y = error[4];
    msg.twist.angular.z = error[5];
    error_publisher_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

void OscController::declare_gains()
{
  auto_declare<double>("position_stiffness", 0.0);
  auto_declare<double>("orientation_stiffness", 0.0);
  auto_declare<double>("damping_ratio", 0.0);
}

std::unique_ptr<TaskSpaceControl> OscController::make_law(
  const arm_sandbox_kinematics::KinematicChain & chain, RobotDynamics & dynamics, const TaskSpaceSettings & settings)
{
  OperationalSpaceGains gains;
  gains.position_stiffness = get_node()->get_parameter("position_stiffness").as_double();
  gains.orientation_stiffness = get_node()->get_parameter("orientation_stiffness").as_double();
  gains.damping_ratio = get_node()->get_parameter("damping_ratio").as_double();
  return std::make_unique<OperationalSpaceControl>(chain, dynamics, settings, gains);
}

void CartesianImpedanceController::declare_gains()
{
  auto_declare<double>("translational_stiffness", 0.0);
  auto_declare<double>("rotational_stiffness", 0.0);
  auto_declare<double>("damping_ratio", 0.0);
}

std::unique_ptr<TaskSpaceControl> CartesianImpedanceController::make_law(
  const arm_sandbox_kinematics::KinematicChain & chain, RobotDynamics & dynamics, const TaskSpaceSettings & settings)
{
  ImpedanceGains gains;
  gains.translational_stiffness = get_node()->get_parameter("translational_stiffness").as_double();
  gains.rotational_stiffness = get_node()->get_parameter("rotational_stiffness").as_double();
  gains.damping_ratio = get_node()->get_parameter("damping_ratio").as_double();
  return std::make_unique<CartesianImpedanceControl>(chain, dynamics, settings, gains);
}

}  // namespace arm_sandbox_controllers

PLUGINLIB_EXPORT_CLASS(arm_sandbox_controllers::OscController, controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(arm_sandbox_controllers::CartesianImpedanceController, controller_interface::ControllerInterface)
```

`src/arm_sandbox_controllers/arm_sandbox_controllers.xml`:

```xml
<library path="task_space_controllers">
  <class name="arm_sandbox_controllers/OscController"
         type="arm_sandbox_controllers::OscController"
         base_class_type="controller_interface::ControllerInterface">
    <description>Operational-space controller: tracks a target pose (~/target_pose) with the task-space inertia; effort interface.</description>
  </class>
  <class name="arm_sandbox_controllers/CartesianImpedanceController"
         type="arm_sandbox_controllers::CartesianImpedanceController"
         base_class_type="controller_interface::ControllerInterface">
    <description>Cartesian impedance controller: a spring-damper towards a target pose (~/target_pose); effort interface.</description>
  </class>
</library>
```

Restore the plugin library lines in `src/arm_sandbox_controllers/CMakeLists.txt` (the Task 2 version above is the full file).

- [ ] **Step 3: Configure the controllers and wire the launch**

Replace `src/arm_sandbox_bringup/config/panda_controllers.yaml`:

```yaml
# ros2_control controllers for the Panda (per-robot config, selected by robot:=panda).
controller_manager:
  ros__parameters:
    update_rate: 1000 # Hz; one update per MuJoCo step (panda.xml timestep 0.001 s)
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    gripper_controller:
      type: position_controllers/GripperActionController
    osc_controller:
      type: arm_sandbox_controllers/OscController
    cartesian_impedance_controller:
      type: arm_sandbox_controllers/CartesianImpedanceController

joint_state_broadcaster:
  ros__parameters:
    update_rate: 100 # Hz; /joint_states for RSP, viewers and tests (control uses state interfaces)

# JTC on the effort interface: it runs a PID per joint (design spec "Controllers"). The integral term
# carries the gravity load when MuJoCo doesn't compensate it (gravcomp:=false, D15).
arm_controller:
  ros__parameters:
    joints: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7]
    command_interfaces: [effort]
    state_interfaces: [position, velocity]
    state_publish_rate: 50.0
    action_monitor_rate: 20.0
    constraints:
      stopped_velocity_tolerance: 0.05
      goal_time: 2.0 # s allowed after the trajectory ends to get inside `goal`
      panda_joint1: {trajectory: 0.2, goal: 0.02}
      panda_joint2: {trajectory: 0.2, goal: 0.02}
      panda_joint3: {trajectory: 0.2, goal: 0.02}
      panda_joint4: {trajectory: 0.2, goal: 0.02}
      panda_joint5: {trajectory: 0.2, goal: 0.02}
      panda_joint6: {trajectory: 0.2, goal: 0.02}
      panda_joint7: {trajectory: 0.2, goal: 0.02}
    gains: # i_clamp limits the integral torque (N m), below each joint's torque limit
      panda_joint1: {p: 1000.0, i: 500.0, d: 50.0, i_clamp: 30.0}
      panda_joint2: {p: 1000.0, i: 500.0, d: 50.0, i_clamp: 30.0}
      panda_joint3: {p: 1000.0, i: 500.0, d: 50.0, i_clamp: 30.0}
      panda_joint4: {p: 1000.0, i: 500.0, d: 50.0, i_clamp: 30.0}
      panda_joint5: {p: 300.0, i: 150.0, d: 15.0, i_clamp: 6.0}
      panda_joint6: {p: 200.0, i: 100.0, d: 10.0, i_clamp: 6.0}
      panda_joint7: {p: 100.0, i: 50.0, d: 5.0, i_clamp: 6.0}

gripper_controller:
  ros__parameters:
    joint: panda_finger_joint1 # m per finger, 0 (closed) to 0.04 (open)
    goal_tolerance: 0.002
    max_effort: 100.0
    allow_stalling: true # a grasped object stops the fingers before the goal
    stall_velocity_threshold: 0.001
    stall_timeout: 1.0

# Task-space controllers (arm_sandbox_controllers, M3). sim.launch.py injects the robot's joints,
# frames, URDF, rest pose (home) and compensate_gravity (= not gravcomp); below is the per-robot
# model and tuning. Both share the settings block; only the gains differ.
osc_controller:
  ros__parameters:
    # MJCF <joint armature>: reflected rotor inertia, added to M(q). Without it, M is off by 0.1
    # on the diagonal. test_dynamics_model.py checks it against the MJCF.
    armature: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    max_position_error: 0.1 # m; a far target pulls with at most this error
    max_orientation_error: 0.5 # rad
    nullspace_stiffness: 10.0 # N m/rad, towards the rest pose (home)
    nullspace_damping: 2.0 # N m s/rad
    singularity_damping: 0.001 # regularizes (J M^-1 J^T + eps I)^-1
    state_publish_rate: 50.0 # Hz, ~/pose_error
    position_stiffness: 400.0 # 1/s^2 (unit-mass task space)
    orientation_stiffness: 400.0 # 1/s^2
    damping_ratio: 1.0

cartesian_impedance_controller:
  ros__parameters:
    armature: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    max_position_error: 0.1
    max_orientation_error: 0.5
    nullspace_stiffness: 10.0
    nullspace_damping: 2.0
    singularity_damping: 0.001
    state_publish_rate: 50.0
    translational_stiffness: 500.0 # N/m: a 10 N push moves the end effector 2 cm
    rotational_stiffness: 50.0 # N m/rad
    damping_ratio: 1.0
```

Create `src/arm_sandbox_bringup/config/sim_plugins.yaml`:

```yaml
# mujoco_ros2_control plugins (D13), loaded into the sim's own node when sim.launch.py gets
# external_wrench:=true. Tools for tests and demos, not for controllers: controllers never talk to
# MuJoCo. (Opt-in because this plugin makes ros2_control_node segfault on shutdown in 0.1.2.)
mujoco_ros2_control_node:
  ros__parameters:
    mujoco_plugins:
      # Push on a body: ~/apply_wrench (mujoco_ros2_control_msgs/srv/ApplyExternalWrench).
      external_wrench:
        type: mujoco_ros2_control_plugins/ExternalWrenchPlugin
```

Replace `src/arm_sandbox_bringup/launch/sim.launch.py`:

```python
"""Bring up one robot in MuJoCo behind ros2_control (milestone M1).

    ros2 launch arm_sandbox_bringup sim.launch.py robot:=panda viewer:=true gravcomp:=false rerun:=true

Startup order: with a torque-controlled arm and no gravity compensation, the arm sags from the
moment physics starts, because zero torque is commanded until a controller is active, and JTC holds
whatever pose it sees when it activates. So the arm and gripper controllers are loaded inactive, and
activate_at_home resets the sim to the `home` keyframe and activates them in back-to-back service
calls (a spawner started after the reset would leave ~2 s for the arm to fall).
"""

from pathlib import Path

import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from arm_sandbox_sim.scene import write_composed_scene

# mujoco_ros2_control 0.1.2 (D13) serves its services from this node.
RESET_WORLD_SERVICE = "/mujoco_ros2_control_node/reset_world"
SWITCH_CONTROLLER_SERVICE = "/controller_manager/switch_controller"
# Every robot's MJCF defines this keyframe, equal to `home` in its robot.yaml.
HOME_KEYFRAME = "home"
JOINT_TRAJECTORY_CONTROLLER = "arm_controller"
# Task-space controllers (arm_sandbox_controllers): they get the robot's URDF, names and rest pose
# injected (see task_space_parameters_file). Only one arm controller is active at a time.
TASK_SPACE_CONTROLLERS = ["osc_controller", "cartesian_impedance_controller"]
ARM_CONTROLLERS = [JOINT_TRAJECTORY_CONTROLLER, *TASK_SPACE_CONTROLLERS]
GRIPPER_CONTROLLER = "gripper_controller"
# Startup service calls give up after this long (sim start with meshes takes a few seconds).
STARTUP_SERVICE_TIMEOUT_S = 30.0


def is_true(context: LaunchContext, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).lower() == "true"


def require_file(path: Path, what: str) -> Path:
    if not path.is_file():
        raise FileNotFoundError(f"{what} not found: {path}")
    return path


def spawner(*arguments: str) -> Node:
    return Node(package="controller_manager", executable="spawner", arguments=list(arguments), output="screen")


class _NoAliasDumper(yaml.SafeDumper):
    """rcl's params parser rejects YAML aliases (&id001 / *id001), which PyYAML emits for shared objects."""

    def ignore_aliases(self, data) -> bool:
        return True


def task_space_parameters_file(robot_config: dict, robot_description: str, compensate_gravity: bool) -> Path:
    """Parameters every task-space controller needs from the robot, as a ROS params file.

    Written per launch so robot.yaml and the URDF stay the single source (no copies in the
    controllers YAML). compensate_gravity is the opposite of the sim's gravcomp: gravity must be
    compensated exactly once.
    """
    robot_parameters = {
        "joints": robot_config["arm_joints"],
        "base_frame": robot_config["base_frame"],
        "ee_frame": robot_config["ee_frame"],
        "rest_configuration": [float(value) for value in robot_config["home"]],
        "robot_description": robot_description,
        "compensate_gravity": compensate_gravity,
    }
    contents = {name: {"ros__parameters": robot_parameters} for name in TASK_SPACE_CONTROLLERS}
    path = Path(tempfile.mkdtemp(prefix="arm_sandbox_controllers_")) / "task_space_controllers.yaml"
    path.write_text(yaml.dump(contents, Dumper=_NoAliasDumper))
    return path


def rerun_actions(context: LaunchContext) -> list:
    """The Rerun viewer (own process) and the bridge that feeds it (design spec "Visualization")."""
    if not is_true(context, "rerun"):
        return []
    bridge_config = require_file(
        Path(get_package_share_directory("arm_sandbox_viz")) / "config" / "rerun_bridge.yaml", "Rerun bridge config"
    )
    overrides = {"use_sim_time": True}
    actions = []
    save_path = LaunchConfiguration("rerun_save").perform(context)
    if save_path:
        overrides.update({"save_path": save_path, "grpc_url": ""})
    else:
        # Web viewer on http://localhost:9090/?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9876%2Fproxy, gRPC on 9876 (bridge config `grpc_url`).
        actions.append(ExecuteProcess(cmd=["rerun", "--serve-web"], output="screen"))
    actions.append(
        Node(
            package="arm_sandbox_viz",
            executable="rerun_bridge",
            name="rerun_bridge",
            parameters=[str(bridge_config), overrides],
            output="screen",
        )
    )
    return actions


def launch_setup(context: LaunchContext) -> list:
    robot = LaunchConfiguration("robot").perform(context)
    arm_controller = LaunchConfiguration("arm_controller").perform(context)
    if arm_controller not in ARM_CONTROLLERS:
        raise ValueError(f"arm_controller must be one of {ARM_CONTROLLERS}, got '{arm_controller}'")
    description_dir = Path(get_package_share_directory("arm_sandbox_description")) / robot
    bringup_dir = Path(get_package_share_directory("arm_sandbox_bringup"))

    robot_config = yaml.safe_load(require_file(description_dir / "config" / "robot.yaml", "robot config").read_text())
    controllers_file = require_file(bringup_dir / "config" / f"{robot}_controllers.yaml", "controllers config")
    scene = write_composed_scene(
        require_file(description_dir / "mjcf" / "scene.xml", "MuJoCo scene"),
        robot_root=robot_config["base_frame"],
        gravcomp=is_true(context, "gravcomp"),
    )
    robot_description = xacro.process_file(
        str(require_file(description_dir / "urdf" / f"{robot}.urdf.xacro", "URDF")),
        mappings={"mujoco_model": str(scene), "headless": str(not is_true(context, "viewer")).lower()},
    ).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": ParameterValue(robot_description, value_type=str), "use_sim_time": True}],
        output="screen",
    )
    # Opt-in: mujoco_ros2_control 0.1.2's ExternalWrenchPlugin makes ros2_control_node segfault on
    # shutdown (exit code -11), so only tests and demos that push on the robot enable it.
    sim_plugin_files = (
        [str(require_file(bringup_dir / "config" / "sim_plugins.yaml", "sim plugins config"))]
        if is_true(context, "external_wrench")
        else []
    )
    control_node = Node(
        # mujoco_ros2_control ships its own ros2_control_node (same parameters as upstream's).
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        parameters=[
            {"use_sim_time": True},
            str(controllers_file),
            str(task_space_parameters_file(robot_config, robot_description, not is_true(context, "gravcomp"))),
            *sim_plugin_files,
        ],
        # Humble's controller_manager reads the URDF from ~/robot_description.
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
        on_exit=Shutdown(),
    )
    joint_state_broadcaster = spawner("joint_state_broadcaster")
    # Loaded and configured, but not active: activate_at_home activates them right after the reset.
    arm_and_gripper = spawner(*ARM_CONTROLLERS, GRIPPER_CONTROLLER, "--inactive")
    activate_at_home = Node(
        package="arm_sandbox_bringup",
        executable="activate_at_home",
        parameters=[
            {
                "reset_service": RESET_WORLD_SERVICE,
                "keyframe": HOME_KEYFRAME,
                "switch_service": SWITCH_CONTROLLER_SERVICE,
                "controllers": [arm_controller, GRIPPER_CONTROLLER],
                "timeout_s": STARTUP_SERVICE_TIMEOUT_S,
            }
        ],
        output="screen",
    )
    return rerun_actions(context) + [
        robot_state_publisher,
        control_node,
        joint_state_broadcaster,
        arm_and_gripper,
        RegisterEventHandler(OnProcessExit(target_action=arm_and_gripper, on_exit=[activate_at_home])),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="panda", description="Robot folder in arm_sandbox_description"),
            DeclareLaunchArgument("viewer", default_value="true", description="Open the native MuJoCo viewer"),
            DeclareLaunchArgument(
                "external_wrench",
                default_value="false",
                description="Enable the sim's ~/apply_wrench service (push on a body; for tests and demos)",
            ),
            DeclareLaunchArgument(
                "arm_controller",
                default_value=JOINT_TRAJECTORY_CONTROLLER,
                description=f"Arm controller to activate: one of {ARM_CONTROLLERS}",
            ),
            DeclareLaunchArgument(
                "gravcomp",
                default_value="false",
                description="MuJoCo compensates the arm's gravity (like the real Panda); false: controllers do",
            ),
            DeclareLaunchArgument(
                "rerun", default_value="true", description="Show the sim in the Rerun web viewer (http://localhost:9090/?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9876%2Fproxy)"
            ),
            DeclareLaunchArgument(
                "rerun_save", default_value="", description="Record to this .rrd file instead of starting a viewer"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
```

- [ ] **Step 4: Run the tests to see them pass**

Run: `make test`
Expected: `test_dynamics_model` 3 passed. All earlier tests still pass (`test_sim_bringup`, `test_arm_trajectory` and `test_sim_recording` use the default `arm_controller`).

- [ ] **Step 5: Try the OSC by hand**

```bash
make start_sim ARGS="viewer:=false rerun:=false arm_controller:=osc_controller"   # terminal 1
```

In terminal 2:

```bash
source install/setup.bash
ros2 control list_controllers   # osc_controller active; arm_controller and cartesian_impedance_controller inactive
ros2 topic pub --once /osc_controller/target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: panda_link0}, pose: {position: {x: 0.45, y: 0.1, z: 0.35}, orientation: {x: 1.0, w: 0.0}}}"
ros2 run tf2_ros tf2_echo panda_link0 panda_hand_tcp   # Translation: [0.450, 0.100, 0.350] within about a second
```

The launch output shows `configured for 7 joints, panda_link0 -> panda_hand_tcp, gravity compensation on`, and with `gravcomp:=true` it says `off`. Stop with Ctrl+C. Expected: `process has finished cleanly` for `ros2_control_node` (with `external_wrench:=true` it dies with −11 on shutdown, see above).

- [ ] **Step 6: Commit**

```bash
git add src/arm_sandbox_controllers src/arm_sandbox_bringup
git commit -m "feat(controllers): OSC and Cartesian impedance ros2_control plugins; select with arm_controller:="
```

### Task 5: The impedance controller is a spring (compliance test)

The test pushes the hand with 10 N for 3 s through the sim's wrench plugin. The deflection must equal F/k within 20 %; with k = 500 N/m that's 2 cm. After release the arm must return within 2 mm. Measured by hand while writing this plan: a 2.0 cm deflection along world −y (the force is in the hand frame, whose y axis points along world −y at home), and a full return.

**Files:**
- Test: `src/arm_sandbox_bringup/test/test_impedance_compliance.py`
- Modify: `src/arm_sandbox_bringup/CMakeLists.txt` (un-comment its `add_launch_test` line)

**Interfaces:**
- Consumes: `cartesian_impedance_controller`, `external_wrench:=true`, `/external_wrench/apply_wrench` (Task 4).

- [ ] **Step 1: Write the test**

```python
"""M3: the Cartesian impedance controller behaves like a spring in the running sim.

A steady push F on the hand (the sim's external-wrench plugin) displaces the end effector by about
F / k, k being the controller's translational stiffness; after the push it returns to its target.
"""

import time
import unittest
from pathlib import Path

import numpy as np
import pytest
import rclpy
import tf2_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from controller_manager.test_utils import check_controllers_running
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from mujoco_ros2_control_msgs.msg import ExternalWrench
from mujoco_ros2_control_msgs.srv import ApplyExternalWrench

ROBOT = "panda"
CONTROLLER = "cartesian_impedance_controller"
CONTROLLERS = ["joint_state_broadcaster", CONTROLLER, "gripper_controller"]
WRENCH_SERVICE = "/external_wrench/apply_wrench"
PUSHED_BODY = "panda_hand"
PUSH_FORCE_N = 10.0
PUSH_DURATION_S = 3
STARTUP_TIMEOUT_S = 60.0
SETTLE_TIME_S = 2.0  # the push is measured this long after it starts, the return this long after it ends
DEFLECTION_TOLERANCE = 0.2  # 20 % of F / k
RETURN_TOLERANCE_M = 0.002


def controller_parameters() -> dict:
    path = Path(get_package_share_directory("arm_sandbox_bringup")) / "config" / f"{ROBOT}_controllers.yaml"
    return yaml.safe_load(path.read_text())[CONTROLLER]["ros__parameters"]


def robot_config() -> dict:
    path = Path(get_package_share_directory("arm_sandbox_description")) / ROBOT / "config" / "robot.yaml"
    return yaml.safe_load(path.read_text())


@pytest.mark.launch_test
def generate_test_description() -> LaunchDescription:
    sim_launch = Path(get_package_share_directory("arm_sandbox_bringup")) / "launch" / "sim.launch.py"
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_launch)),
        launch_arguments={
            "robot": ROBOT,
            "viewer": "false",
            "rerun": "false",
            "arm_controller": CONTROLLER,
            "external_wrench": "true",
        }.items(),
    )
    return LaunchDescription([sim, ReadyToTest()])


class TestImpedanceCompliance(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.node = rclpy.create_node("test_impedance_compliance")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def tearDown(self) -> None:
        self.node.destroy_node()

    def spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def ee_position(self) -> np.ndarray:
        config = robot_config()
        transform = self.tf_buffer.lookup_transform(config["base_frame"], config["ee_frame"], rclpy.time.Time())
        t = transform.transform.translation
        return np.array([t.x, t.y, t.z])

    def test_push_deflects_like_a_spring_and_releases(self) -> None:
        check_controllers_running(self.node, CONTROLLERS, timeout=STARTUP_TIMEOUT_S)
        client = self.node.create_client(ApplyExternalWrench, WRENCH_SERVICE)
        self.assertTrue(client.wait_for_service(timeout_sec=STARTUP_TIMEOUT_S), f"{WRENCH_SERVICE} not available")
        self.spin_for(SETTLE_TIME_S)
        rest = self.ee_position()

        push = ExternalWrench()
        push.wrench.header.frame_id = PUSHED_BODY
        push.wrench.wrench.force.y = PUSH_FORCE_N  # in the body frame
        push.duration = Duration(sec=PUSH_DURATION_S)
        request = ApplyExternalWrench.Request()
        request.wrenches.external_wrenches = [push]
        # The service answers only when the push is over, so call it asynchronously.
        future = client.call_async(request)
        self.spin_for(SETTLE_TIME_S)
        pushed = self.ee_position()

        rclpy.spin_until_future_complete(self.node, future, timeout_sec=PUSH_DURATION_S + STARTUP_TIMEOUT_S)
        self.assertTrue(future.result() is not None and future.result().success, "push was not applied")
        self.spin_for(SETTLE_TIME_S)
        released = self.ee_position()

        expected = PUSH_FORCE_N / controller_parameters()["translational_stiffness"]
        deflection = np.linalg.norm(pushed - rest)
        self.assertAlmostEqual(deflection, expected, delta=DEFLECTION_TOLERANCE * expected, msg=f"deflection {deflection:.4f} m")
        self.assertLess(np.linalg.norm(released - rest), RETURN_TOLERANCE_M)
```

- [ ] **Step 2: Run it**

Run: `make test`
Expected: `test_impedance_compliance` passes (1 test, about 10 s). The controllers already exist, so this acceptance test can pass right away.

- [ ] **Step 3: Check it measures the stiffness**

In `test_impedance_compliance.py`, temporarily replace `controller_parameters()["translational_stiffness"]` with `250.0`, so the test expects 4 cm. Run `make test`.
Expected: `test_impedance_compliance` FAILS (`deflection 0.0200 m`). Revert, then run it again; it passes.

- [ ] **Step 4: Commit**

```bash
git add src/arm_sandbox_bringup
git commit -m "test(bringup): the impedance controller deflects by F/k under a push and springs back"
```

### Task 6: Reach with the task-space controllers, and the comparison

`reach_runner` gets `motion: pose_target`: it publishes each reach target to `/<controller>/target_pose` and waits for the same TF hold check as before, with no IK. `reach.launch.py` gets `controller:=`. It derives the motion mode and topics from the controller's name, so `arm_action` and `controller_state_topic` leave `reach_runner.yaml`.

The reach test now runs all three controllers, each with gravcomp off and on. The gravcomp-on cases are essential. With the launch wiring planted wrong (always compensate), the OSC missed by about 2.7 cm and the impedance controller by 15–31 cm, while the gravcomp-off runs still passed.

Results from the scratch run, same five targets, error after the 0.5 s hold:

| Controller | gravcomp off | gravcomp on | Time per target |
|---|---|---|---|
| IK + JTC (M2) | 1.3–4.0 mm | 0.1–0.3 mm | 1.5–3.6 s |
| OSC | ≈ 0.1 mm | ≈ 0.1 mm | 0.9–1.2 s |
| Cartesian impedance (500 N/m) | 0.3–2.7 mm | 0.3–2.6 mm | 1.5–2.2 s |

**Files:**
- Modify: `src/arm_sandbox_tasks/src/reach_runner.cpp`, `src/arm_sandbox_tasks/launch/reach.launch.py`, `src/arm_sandbox_tasks/config/reach_runner.yaml`, `src/arm_sandbox_tasks/test/test_reach.py`, `src/arm_sandbox_tasks/package.xml`, `src/arm_sandbox_tasks/CMakeLists.txt`

**Interfaces:**
- Consumes: `/<controller>/target_pose`, `/<controller>/pose_error` (Task 4).
- Produces:
  - `reach_runner` parameters `motion` (`joint_trajectory` | `pose_target`), `pose_target_topic` and `controller_state_topic`. `arm_action` applies to `joint_trajectory` only.
  - `ros2 launch arm_sandbox_tasks reach.launch.py controller:=…`.
  - The log line becomes `target N reached: E m, R rad in T s (how)`.

- [ ] **Step 1: Write the failing test**

Replace `src/arm_sandbox_tasks/test/test_reach.py`, and in `src/arm_sandbox_tasks/CMakeLists.txt` change the reach test's timeout to 900 s (it now runs six launches):

```python
"""M2/M3 acceptance: with the sim running, reach_runner reaches every target of the reach task.

Runs once per arm controller and gravity-compensation mode (D15): IK + joint trajectory (M2), and
the operational-space and impedance controllers (M3). With gravcomp:=true the task-space
controllers must not add gravity themselves (else they'd miss by centimetres).
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
@launch_testing.parametrize(
    "controller, gravcomp",
    [
        ("arm_controller", "false"),
        ("arm_controller", "true"),
        ("osc_controller", "false"),
        ("osc_controller", "true"),
        ("cartesian_impedance_controller", "false"),
        ("cartesian_impedance_controller", "true"),
    ],
)
def generate_test_description(controller: str, gravcomp: str):
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file("arm_sandbox_bringup", "sim.launch.py")),
        launch_arguments={
            "robot": "panda",
            "viewer": "false",
            "rerun": "false",
            "gravcomp": gravcomp,
            "arm_controller": controller,
        }.items(),
    )
    reach = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file("arm_sandbox_tasks", "reach.launch.py")),
        launch_arguments={"robot": "panda", "task": "reach", "controller": controller}.items(),
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
find_package(geometry_msgs REQUIRED)
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
ament_target_dependencies(reach_runner control_msgs geometry_msgs rclcpp rclcpp_action sensor_msgs std_msgs tf2_eigen tf2_ros)

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
  add_launch_test(test/test_reach.py TIMEOUT 900 ENV ROS_DOMAIN_ID=45)
endif()

ament_package()
```

Run: `make test`
Expected: `test_reach` FAILS for the `osc_controller` and `cartesian_impedance_controller` cases (`reach.launch.py` doesn't know `controller`, so the runner waits for the JTC and times out). The `arm_controller` cases still pass.

- [ ] **Step 2: Add the pose-target mode**

`src/arm_sandbox_tasks/package.xml` (adds `geometry_msgs`):

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
  <depend>geometry_msgs</depend>
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

`src/arm_sandbox_tasks/config/reach_runner.yaml`:

```yaml
# reach_runner: drives the arm to each reach target, either with arm_sandbox_kinematics' IK and the
# joint trajectory controller, or by sending the pose to a task-space controller (OSC, impedance).
# reach.launch.py passes the robot's names and home pose (robot.yaml), the task file, and the
# motion mode and controller topics derived from its `controller` argument.
reach_runner:
  ros__parameters:
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

    ros2 launch arm_sandbox_tasks reach.launch.py robot:=panda task:=reach controller:=arm_controller

`controller` must be the arm controller the sim activated (sim.launch.py arm_controller:=...):
arm_controller (IK + joint trajectory, M2) or a task-space controller (pose target, M3).

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
JOINT_TRAJECTORY_CONTROLLER = "arm_controller"
TASK_SPACE_CONTROLLERS = ("osc_controller", "cartesian_impedance_controller")


def motion_parameters(controller: str) -> dict:
    """How reach_runner moves the arm with `controller`, and which of its topics to use."""
    if controller == JOINT_TRAJECTORY_CONTROLLER:
        return {
            "motion": "joint_trajectory",
            "arm_action": f"/{controller}/follow_joint_trajectory",
            "controller_state_topic": f"/{controller}/state",
        }
    if controller in TASK_SPACE_CONTROLLERS:
        return {
            "motion": "pose_target",
            "pose_target_topic": f"/{controller}/target_pose",
            "controller_state_topic": f"/{controller}/pose_error",
        }
    raise ValueError(f"controller must be one of {(JOINT_TRAJECTORY_CONTROLLER, *TASK_SPACE_CONTROLLERS)}, got '{controller}'")


def require_file(path: Path, what: str) -> Path:
    if not path.is_file():
        raise FileNotFoundError(f"{what} not found: {path}")
    return path


def launch_setup(context: LaunchContext) -> list:
    robot = LaunchConfiguration("robot").perform(context)
    task = LaunchConfiguration("task").perform(context)
    controller = LaunchConfiguration("controller").perform(context)
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
            motion_parameters(controller),
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
            DeclareLaunchArgument(
                "controller",
                default_value=JOINT_TRAJECTORY_CONTROLLER,
                description="Active arm controller: arm_controller, osc_controller or cartesian_impedance_controller",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
```

`src/arm_sandbox_tasks/src/reach_runner.cpp`:

```cpp
// reach_runner: solves the reach task (milestone M2) with the hand-written IK and the joint
// trajectory controller. Thin ROS wrapper: the math is in arm_sandbox_kinematics, the task logic
// in reach_task.hpp.
//
// Two ways to move (parameter `motion`):
// - joint_trajectory (M2): IK from the current joint positions (null-space pull towards home) ->
//   one FollowJointTrajectory goal to the joint trajectory controller.
// - pose_target (M3): publish the target pose to a task-space controller (OSC or impedance), which
//   does the rest; no IK.
// Either way, a target counts once TF shows the end effector held within the task tolerance.
// Exits 0 if every target was reached, 1 otherwise.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
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
constexpr const char * kJointTrajectory = "joint_trajectory";
constexpr const char * kPoseTarget = "pose_target";

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
    const auto motion = declare_parameter<std::string>("motion", "");
    if (motion != kJointTrajectory && motion != kPoseTarget) {
      RCLCPP_FATAL(get_logger(), "motion must be '%s' or '%s', got '%s'", kJointTrajectory, kPoseTarget, motion.c_str());
      throw std::invalid_argument("invalid motion");
    }
    use_pose_target_ = motion == kPoseTarget;
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
    // Both controllers publish their state only while active, so the first message means "ready".
    const auto state_topic = declare_parameter<std::string>("controller_state_topic", "");
    if (use_pose_target_) {
      pose_error_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        state_topic, rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::TwistStamped &) { controller_active_ = true; });
      target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        declare_parameter<std::string>("pose_target_topic", ""), rclcpp::SystemDefaultsQoS());
    } else {
      jtc_state_sub_ = create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        state_topic, rclcpp::SensorDataQoS(),
        [this](const control_msgs::msg::JointTrajectoryControllerState &) { controller_active_ = true; });
      arm_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, declare_parameter<std::string>("arm_action", ""));
    }
  }

  /// Runs every target; true if all were reached.
  bool run()
  {
    if (!wait_until([this] { return robot_description_.has_value(); }, "/robot_description") ||
        !wait_until([this] { return controller_active_; }, "the arm controller to become active") ||
        !wait_until([this] { return current_positions().has_value(); }, "/joint_states with all arm joints")) {
      return false;
    }
    if (use_pose_target_ ? !wait_until([this] { return target_pub_->get_subscription_count() > 0; }, "the controller's target subscription")
                         : !arm_client_->wait_for_action_server(seconds(startup_timeout_s_))) {
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
    const auto start_time = Clock::now();
    const auto deadline = start_time + seconds(task_.time_limit_s);
    if (use_pose_target_) {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = base_frame_;
      msg.header.stamp = now();
      msg.pose = tf2::toMsg(target);
      target_pub_->publish(msg);
      // The controller moves the arm itself, so it may use the whole time limit to get there.
      return hold_within_tolerance(target, index, deadline, start_time, "pose target");
    }

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

    // JTC's goal tolerance is per joint, so the pose keeps settling after it succeeds.
    char how[96];
    std::snprintf(how, sizeof(how), "IK %d iterations, move %.2f s", ik.iterations, duration);
    return hold_within_tolerance(
      target, index, std::min(deadline, Clock::now() + seconds(settle_timeout_s_) + seconds(task_.hold_s)), start_time, how);
  }

  /// Waits until TF shows the end effector within the task tolerance for hold_s of sim time.
  bool hold_within_tolerance(
    const Eigen::Isometry3d & target, std::size_t index, Clock::time_point deadline, Clock::time_point start_time,
    const std::string & how)
  {
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
          const double elapsed = std::chrono::duration<double>(Clock::now() - start_time).count();
          RCLCPP_INFO(get_logger(), "target %zu reached: %.4f m, %.4f rad in %.2f s (%s)",
                      index, error.position, error.orientation, elapsed, how.c_str());
          return true;
        }
      }
      if (Clock::now() > deadline) {
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
  bool use_pose_target_ = false;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr jtc_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr pose_error_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
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

- [ ] **Step 3: Run the tests to see them pass**

Run: `make test`
Expected: 0 failures. `test_reach` runs 6 configurations with 2 tests each (about 2–3 minutes).

- [ ] **Step 4: Compare by hand**

```bash
for c in arm_controller osc_controller cartesian_impedance_controller; do
  timeout -s INT 120 make start_sim ARGS="viewer:=false rerun:=false arm_controller:=$c" > /tmp/sim_$c.log 2>&1 &
  sleep 3; make start_reach ARGS="controller:=$c" 2>&1 | grep -E "reached|targets"; wait
done
```

Expected: 5/5 for each, with errors and times like the table above.

- [ ] **Step 5: Commit**

```bash
git add src/arm_sandbox_tasks
git commit -m "feat(tasks): reach with the task-space controllers (pose targets); test all three controllers"
```

---

# Part 3 — Docs

### Task 7: Record the decisions, update the docs, add READMEs, log the session

**Files:**
- Modify: `docs/REQUIREMENTS.md` (§10), `docs/specs/2026-10-01-arm-sandbox-design.md` ("Controllers"), `docs/PROJECT_STRUCTURE.md`, `CLAUDE.md`, `README.md`, `docs/LEARNING_RESOURCES.md`, `docs/MEMORY.md`
- Create: `src/arm_sandbox_controllers/README.md`

- [ ] **Step 1: Decisions log.** Append to `docs/REQUIREMENTS.md` §10, and add `, D20–D22 added 2026-10-04 (Plan 04)` to the status line after `(Plan 03)`:

```markdown
| D20 | Task-space controllers share one ROS-free core (`RobotDynamics`: Pinocchio + armature; `TaskSpaceControl` with OSC and Cartesian impedance laws) under two thin `ros2_control` plugins; targets on `~/target_pose` | One tested implementation of the control math (and later the Gym env's); plugins stay thin |
| D21 | `sim.launch.py` injects URDF, joint names, frames, rest pose and `compensate_gravity = not gravcomp` into the task-space controllers | robot.yaml and the URDF stay the single source; gravity is compensated exactly once (a wrong flag misses targets by cm) |
| D22 | The sim's `ExternalWrenchPlugin` is opt-in (`external_wrench:=true`) | It makes `ros2_control_node` segfault on shutdown in mujoco_ros2_control 0.1.2 |
```

- [ ] **Step 2: Design spec, "Controllers".** Replace the second bullet ("Real-time safe: …") with:

```markdown
- Real-time safe: targets go through `realtime_tools::RealtimeBuffer` (Humble has no `RealtimeThreadSafeBox`), state through `RealtimePublisher`. No allocation in `update()`: `KinematicChain` has allocation-free `fk`/`jacobian` overloads, and tests built with `EIGEN_RUNTIME_NO_MALLOC` prove the control core never allocates. Gains, armature and limits come from YAML.
- The model is Pinocchio on the URDF, reduced to the arm joints, plus the MJCF's armature (0.1 per joint); it matches MuJoCo's M(q) and g(q) to 1e-6 (`test_dynamics_model.py`). Controllers add g(q) only if `compensate_gravity` (= not `gravcomp`, injected by the launch with the URDF, names and rest pose, D21).
- `sim.launch.py arm_controller:=` chooses the active arm controller (all are loaded). Measured on the reach task: OSC ≈ 0.1 mm in about 1 s, impedance 0.3–2.7 mm, IK + JTC 1.3–4.0 mm without simulator gravity compensation.
```

- [ ] **Step 3: `docs/PROJECT_STRUCTURE.md` and `CLAUDE.md`**
- `PROJECT_STRUCTURE.md`:
  - Status note: `M0–M2 done, M3 controllers done (Plans 01–04)`, and add `arm_sandbox_controllers` to the list of packages that exist.
  - `arm_sandbox_controllers` row, Purpose: `ROS-free control core (Pinocchio dynamics, OSC and Cartesian impedance laws) + ros2_control plugins`.
  - Launch arguments line: add `arm_controller`, `external_wrench`.
- `CLAUDE.md` status line: `> **Status:** M0–M2 done, M3 part 1 (task-space controllers) done (Plans 01–04). Next: Plan 05 (drawer task). …` (keep the rest of the line).

- [ ] **Step 4: `src/arm_sandbox_controllers/README.md`**

```markdown
# arm_sandbox_controllers

Task-space controllers for torque-controlled arms: a ROS-free control core and two `ros2_control`
plugins on top of it. Select one with `make start_sim ARGS="arm_controller:=osc_controller"` (or
`cartesian_impedance_controller`), then publish a `geometry_msgs/PoseStamped` to `~/target_pose`.

## The model

Joint-space dynamics M(q) q'' + C(q, q') q' + g(q) = tau come from Pinocchio on the URDF, reduced to
the arm joints, plus the actuators' armature (reflected rotor inertia), which the MJCF has and the
URDF can't express. Kinematics (J, FK) come from arm_sandbox_kinematics.

## The laws

Both work on the end-effector error e = [p* - p; angle-axis(R* R^T)] and twist x' = J q':

    tau = J^T F + N^T tau_0 + C q' (+ g)        Lambda = (J M^-1 J^T)^-1,  N^T = I - J^T Jbar^T

- Operational space (Khatib): F = Lambda (Kp e - Kd x'). Lambda turns the end effector into a unit
  mass in every direction, so the gains give the same, decoupled response everywhere: stiff tracking.
- Cartesian impedance (Hogan): F = K e - D x'. A spring-damper: a push F moves the end effector by
  about K^-1 F (test_impedance_compliance.py: 10 N, 500 N/m, 2 cm). Right for contact.
- tau_0 pulls the redundant joint towards the rest pose without moving the end effector (N^T is the
  dynamically consistent null-space projector).
- g is added only when nothing else compensates gravity (`compensate_gravity`, set by the launch).

## Real time

`update()` never allocates: preallocated workspace, allocation-free kinematics overloads, and tests
that run `compute()` under Eigen's EIGEN_RUNTIME_NO_MALLOC guard.
```

- [ ] **Step 5: `docs/LEARNING_RESOURCES.md`.** Append:

```markdown
---

## Plan 04 — Task-Space Control (M3, part 1)

Plan: [plan/2026-10-04-plan-04-task-space-control.md](plan/2026-10-04-plan-04-task-space-control.md)

- **Khatib (1987)**, "A unified approach for motion and force control of robot manipulators: The
  operational space formulation", *IEEE Journal on Robotics and Automation*. The OSC law, Lambda, and
  the dynamically consistent null space.
- **Hogan (1985)**, "Impedance Control: An Approach to Manipulation" (parts I–III), *Journal of Dynamic
  Systems, Measurement, and Control*. Impedance control.
- **Siciliano et al.**, Ch. 7 (dynamics), Ch. 8 (motion control, incl. operational space) and Ch. 9
  (force control, incl. impedance). **Lynch & Park, *Modern Robotics***, Ch. 8 (dynamics) and Ch. 11
  (robot control: computed torque, task-space and impedance control).
- **Featherstone, *Rigid Body Dynamics Algorithms*** (Springer): CRBA (mass matrix), RNEA (C q' + g),
  ABA (forward dynamics) — the algorithms Pinocchio implements.
- **Pinocchio docs**: `crba`, `nonLinearEffects`, `computeGeneralizedGravity`, `aba`,
  `buildReducedModel`, `Model::armature`.
- **control.ros.org**: "Writing a new controller" and the `realtime_tools` docs. franka_ros2's
  Cartesian impedance example controller is a good real-robot comparison.
```

- [ ] **Step 6: `README.md` (the repository's front page)**
  - **Status table:** the M3 row becomes `Control | Operational-space and impedance controllers as ros2_control plugins; drawer opened compliantly | controllers done; drawer task in [Plan 05](docs/plan/2026-10-04-plan-05-drawer-task.md)`.
  - **Highlights:** add a bullet with the reach comparison from Task 6: three controllers on the same 5 targets, OSC ≈ 0.1 mm in about 1 s, impedance 0.3–2.7 mm, IK + JTC 1.3–4.0 mm. Also mention that the impedance controller deflects 2 cm under a 10 N push (k = 500 N/m), and that `compute()` is proven allocation-free.
  - **Quick start:** add `make start_sim ARGS="arm_controller:=osc_controller"` and `make start_reach ARGS="controller:=osc_controller"`.
  - **Repository layout:** add `| \`src/arm_sandbox_controllers\` | ROS-free control core (Pinocchio dynamics, OSC and impedance laws) + ros2_control plugins |`.
  - **Test count:** update the number in the "Tested headless" bullet to what `colcon test-result` reports (about 95).

- [ ] **Step 7: `docs/MEMORY.md`.** Append a new `## Session — <date>` section at the end (earlier sections stay unchanged). Cover the commits per task, what you learned, and the state at the end.

- [ ] **Step 8: Check, then commit**

```bash
grep -n "D20\|D21\|D22" docs/REQUIREMENTS.md docs/specs/2026-10-01-arm-sandbox-design.md   # hits in both
make test   # 0 failures
git add CLAUDE.md README.md docs src/arm_sandbox_controllers/README.md
git commit -m "docs: record D20-D22, document the task-space controllers, add README and reading list"
```

---

## Done when

- `make test` passes with no warnings: the kinematics and control-core real-time tests, the control laws in closed loop, the dynamics model vs MuJoCo, the impedance compliance in the sim, and the reach task with all three controllers in both gravcomp modes.
- `make start_sim ARGS="arm_controller:=osc_controller"` plus a `PoseStamped` on `/osc_controller/target_pose` moves the arm there. The impedance controller gives way by F/k when pushed.
- D20–D22 recorded; the docs and the controllers README match the code.
- Next: Plan 05 (drawer task, finishing M3).
