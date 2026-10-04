# Plan 05 — The Drawer Task (M3, part 2): Opening a Drawer Compliantly

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A cabinet with a drawer in the sim, and a drawer task that hooks the closed gripper behind the handle and pulls the drawer open with the Cartesian impedance controller from Plan 04. The run counts as a success when the drawer's joint (read from sim ground truth) stays open. This finishes M3 ("drawer task opened compliantly").

**Architecture:**
- **Task objects** are standalone MJCF files in `arm_sandbox_sim/objects/`. `compose_scene` attaches them at launch (`sim.launch.py objects:=drawer`) under the prefix `<file stem>/`, so the drawer's joint is `drawer/slide`.
- **`arm_sandbox_tasks`** gets a shared `TaskRunner` base (frames, TF, waiting, holding a condition, sending pose targets), which `reach_runner` now uses too. It also gets the ROS-free drawer task logic and a thin `drawer_runner` node.
- **Ground truth:** the success check reads the drawer joint from the sim's `/mujoco_actuators_states`, which lists every MuJoCo joint, actuated or not.

**Tech Stack:** MuJoCo 3.12 `MjSpec.attach`, C++17, yaml-cpp, `rclcpp`/`rclcpp_action` (`control_msgs/GripperCommand`), GoogleTest, pytest, `launch_testing`.

**Spec:** [`../specs/2026-10-01-arm-sandbox-design.md`](../specs/2026-10-01-arm-sandbox-design.md), sections "Simulation" (scene composition, ground truth) and "Tasks and Executive" (task files, `joint_opened`), and [`../REQUIREMENTS.md`](../REQUIREMENTS.md) (FR-15 task 5, FR-16, M3). It **builds on Plan 04** ([`2026-10-04-plan-04-task-space-control.md`](2026-10-04-plan-04-task-space-control.md)): the impedance controller and `sim.launch.py arm_controller:=`.

## Task overview

Click a task to jump to it.

- [Task 1: Task objects in the scene](#task-1-task-objects-in-the-scene): Task objects in the scene: `drawer.xml` attached under `drawer/` (`objects:=drawer`).
- [Task 2: Shared runner plumbing (refactor, behaviour unchanged)](#task-2-shared-runner-plumbing-refactor-behaviour-unchanged): Refactor: shared `TaskRunner` base and task-file readers; reach behaviour unchanged.
- [Task 3: The drawer task file and logic](#task-3-the-drawer-task-file-and-logic): Drawer task file and ROS-free logic (hook pose, pull path, `joint_opened`).
- [Task 4: `drawer_runner` — open the drawer (M3 acceptance)](#task-4-drawer_runner--open-the-drawer-m3-acceptance): `drawer_runner` hooks the handle and pulls the drawer open with impedance; `make start_drawer`; M3 acceptance.
- [Task 5: Record the decisions, update the docs, add READMEs, log the session](#task-5-record-the-decisions-update-the-docs-add-readmes-log-the-session): Docs: decisions D23–D25, READMEs, top-level README, reading list, session log.

## Global Constraints

- Everything from Plan 04's Global Constraints still holds (versions, real-time rules, no robot-specific values or magic numbers in code, headless tests, own `ROS_DOMAIN_ID` per launch test). This plan adds 47.
- **Pure logic + thin node:** task-file parsing and the pull path are ROS-free (`task_logic` library); the runners only wire them to ROS.
- **Ground truth is privileged** (spec "Simulation"): only a task's success check may read `/mujoco_actuators_states`. Controllers never do. The approach uses the drawer's known handle pose from the task file (perception replaces it in M5).
- **Scene changes need a relaunch, pose changes don't** (spec): objects come in through `sim.launch.py objects:=`.
- **No code duplication:** the two runners share `TaskRunner` and the task-file field readers (`yaml_fields.hpp`).

## Where each step runs

Same as Plan 04: dev container, `source /opt/ros/humble/setup.bash`, and `source install/setup.bash` after `make test`.

## Before Task 1: branch

Plan 04 must be merged (or this branch cut from its branch):

```bash
git switch master && git pull
git switch -c feat/m3-drawer
```

This plan's code was built and run end to end in a scratch workspace on 2026-10-04, on top of Plan 04: `colcon test` gave 108 tests, 0 failures. Numbers below come from that run.

## Design notes (found while prototyping)

- **Hook, don't grasp.** The Panda gripper in the MJCF is a weak position servo (100 N/m on the finger opening, like Menagerie's), so squeezing a 1.5 cm bar gives under 1 N of grip. Pulling a drawer that takes about 3 N just to start moving would slip. Instead, the closed fingers go down into the 4.8 cm gap between the handle bar and the drawer face, and the pull pushes on the bar through geometry, with no grip force needed. The handle sits 2 cm below the face top, so the hand (about 4.5 cm above the TCP) clears it.
- **The drawer needs real force:** damping 10 N·s/m and friction loss 3 N. Pulling with 2 N barely moves it, 4 N opens it at about 0.1 m/s, and 8 N slams it open.
- **Prefix required.** In MuJoCo 3.12, `MjSpec.attach` with an empty prefix composes fine, but the scene written back to XML doesn't load ("empty class name"). With a prefix it works, and the prefix doubles as a namespace (`drawer/slide`).
- **Result (scratch run):** the impedance controller opens the drawer **0.184 m** for a 0.20 m pull, in both gravcomp modes. The shortfall is about 0.6 cm of slack before the fingers touch the bar, plus the spring deflection under the drawer's resistance (about 4.5 N / 500 N/m ≈ 0.9 cm). The OSC opens it the same amount on this well-aligned pull. Compliance pays off when the path and the slide disagree; see the README.

## File Structure

| Path | Responsibility |
|---|---|
| `src/arm_sandbox_sim/arm_sandbox_sim/scene.py` | `compose_scene(…, objects=…)`: attach task objects under `<stem>/` |
| `src/arm_sandbox_sim/objects/drawer.xml` (new) | The cabinet and drawer (slide joint, damping, friction, handle bar) |
| `src/arm_sandbox_sim/CMakeLists.txt`, `test/test_scene.py` | Install `objects/`; three new composition tests |
| `src/arm_sandbox_bringup/launch/sim.launch.py` | `objects:=` argument |
| `src/arm_sandbox_tasks/src/yaml_fields.hpp` (new) | Task-file field readers shared by the loaders |
| `src/arm_sandbox_tasks/include/arm_sandbox_tasks/task_runner.hpp` (new) | `TaskRunner` base node shared by the runners |
| `src/arm_sandbox_tasks/src/reach_task.cpp`, `src/reach_runner.cpp` | Refactored onto the shared helpers (behaviour unchanged) |
| `src/arm_sandbox_tasks/include/arm_sandbox_tasks/drawer_task.hpp`, `src/drawer_task.cpp`, `config/tasks/drawer.yaml` | Drawer task file and logic (pull path) |
| `src/arm_sandbox_tasks/src/drawer_runner.cpp`, `config/drawer_runner.yaml`, `launch/drawer.launch.py` | The drawer runner |
| `src/arm_sandbox_tasks/test/test_drawer_task.cpp`, `test/test_drawer.py` | Unit tests; end-to-end test in both gravcomp modes |
| `src/arm_sandbox_tasks/CMakeLists.txt` | `task_logic` library (reach + drawer), `drawer_runner`, tests |
| `Makefile`, `docs/…`, `CLAUDE.md`, READMEs | `make start_drawer`; docs in Task 5 |

## Out of scope (later plans)

- Perceiving the handle pose (M5), randomized drawer poses and seeded resets (M4/M6), and behavior-tree recovery such as re-hooking after a slip (M5).
- Comparing OSC and impedance on a *misaligned* pull with contact-force metrics. That belongs in the evaluation tool (M6); this plan only reports the opening.

---

# Part 1 — The Drawer in the Scene

### Task 1: Task objects in the scene

**Files:**
- Modify: `src/arm_sandbox_sim/arm_sandbox_sim/scene.py`, `src/arm_sandbox_sim/CMakeLists.txt`, `src/arm_sandbox_sim/test/test_scene.py`
- Create: `src/arm_sandbox_sim/objects/drawer.xml`
- Modify: `src/arm_sandbox_bringup/launch/sim.launch.py`

**Interfaces:**
- Produces:
  - `compose_scene(scene_path, *, robot_root, gravcomp, objects: Sequence[Path] = ()) -> str` and the same `objects` argument on `write_composed_scene`. Objects are attached under `<file stem>/` and never get gravity compensation. Composition raises `FileNotFoundError` (missing object) or `ValueError` (doesn't compile, e.g. the same object twice).
  - `share/arm_sandbox_sim/objects/drawer.xml`: joint `drawer/slide` (0–0.3 m along −x), site `drawer/handle` at (0.544, 0, 0.23) in the robot base frame.
  - `sim.launch.py objects:=<name>[,<name>…]`.

- [ ] **Step 1: Write the failing tests**

Replace `src/arm_sandbox_sim/test/test_scene.py` (three new tests at the end):

```python
"""Scene composition: loads from anywhere, gravcomp switches the arm's gravity compensation, and task
objects are attached under their file name."""

from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from arm_sandbox_sim.scene import compose_scene, write_composed_scene

ROBOT_DIR = Path(get_package_share_directory("arm_sandbox_description")) / "panda"
SCENE = ROBOT_DIR / "mjcf" / "scene.xml"
DRAWER = Path(get_package_share_directory("arm_sandbox_sim")) / "objects" / "drawer.xml"
HOME_KEYFRAME = "home"
HOLD_DURATION_S = 0.5
# With gravcomp the unactuated arm stays put; without it, it sags (see test_panda_mjcf.py).
GRAVCOMP_DRIFT_LIMIT_RAD = 0.01
NO_GRAVCOMP_MIN_DRIFT_RAD = 0.1


@pytest.fixture(scope="module")
def robot_config() -> dict:
    return yaml.safe_load((ROBOT_DIR / "config" / "robot.yaml").read_text())


def load(xml: str) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(xml)


def robot_subtree(model: mujoco.MjModel, root_name: str) -> set[int]:
    """Body ids of the robot: the root body and all its descendants."""
    members = {model.body(root_name).id}
    for body in range(1, model.nbody):  # parents always have lower ids than their children
        if model.body_parentid[body] in members:
            members.add(body)
    return members


def arm_drift_after_zero_torque(model: mujoco.MjModel, robot_config: dict) -> float:
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key(HOME_KEYFRAME).id)
    addresses = [int(model.joint(joint).qposadr[0]) for joint in robot_config["arm_joints"]]
    data.ctrl[[model.actuator(joint).id for joint in robot_config["arm_joints"]]] = 0.0
    start = data.qpos[addresses].copy()
    for _ in range(int(HOLD_DURATION_S / model.opt.timestep)):
        mujoco.mj_step(model, data)
    return float(np.max(np.abs(data.qpos[addresses] - start)))


def test_composed_scene_loads_from_another_directory(tmp_path: Path, robot_config: dict) -> None:
    path = write_composed_scene(SCENE, robot_root=robot_config["base_frame"], gravcomp=False, output_dir=tmp_path)
    assert path.parent == tmp_path
    composed = mujoco.MjModel.from_xml_path(str(path))
    source = mujoco.MjModel.from_xml_path(str(SCENE))
    assert (composed.nq, composed.nu, composed.nbody, composed.nkey) == (source.nq, source.nu, source.nbody, source.nkey)


def test_gravcomp_flags_only_the_robot(robot_config: dict) -> None:
    model = load(compose_scene(SCENE, robot_root=robot_config["base_frame"], gravcomp=True))
    robot = robot_subtree(model, robot_config["base_frame"])
    for body in range(model.nbody):
        assert model.body_gravcomp[body] == (1.0 if body in robot else 0.0), model.body(body).name


def test_without_gravcomp_nothing_is_compensated(robot_config: dict) -> None:
    model = load(compose_scene(SCENE, robot_root=robot_config["base_frame"], gravcomp=False))
    assert np.all(model.body_gravcomp == 0.0)


def test_gravcomp_holds_the_unactuated_arm(robot_config: dict) -> None:
    root = robot_config["base_frame"]
    with_gravcomp = load(compose_scene(SCENE, robot_root=root, gravcomp=True))
    without_gravcomp = load(compose_scene(SCENE, robot_root=root, gravcomp=False))
    assert arm_drift_after_zero_torque(with_gravcomp, robot_config) < GRAVCOMP_DRIFT_LIMIT_RAD
    assert arm_drift_after_zero_torque(without_gravcomp, robot_config) > NO_GRAVCOMP_MIN_DRIFT_RAD


def test_unknown_robot_root_is_rejected() -> None:
    with pytest.raises(ValueError, match="no_such_body"):
        compose_scene(SCENE, robot_root="no_such_body", gravcomp=True)


def test_missing_scene_is_rejected(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        compose_scene(tmp_path / "missing.xml", robot_root="panda_link0", gravcomp=False)


def test_objects_are_attached_under_their_file_name(robot_config: dict) -> None:
    model = load(compose_scene(SCENE, robot_root=robot_config["base_frame"], gravcomp=True, objects=[DRAWER]))
    assert model.joint("drawer/slide").id >= 0
    assert model.site("drawer/handle").id >= 0
    # gravcomp is for the robot only: the drawer keeps its weight.
    assert model.body_gravcomp[model.body("drawer/tray").id] == 0.0
    assert model.body_gravcomp[model.body(robot_config["base_frame"]).id] == 1.0


def test_same_object_twice_is_rejected(robot_config: dict) -> None:
    with pytest.raises(ValueError, match="repeated name"):
        compose_scene(SCENE, robot_root=robot_config["base_frame"], gravcomp=False, objects=[DRAWER, DRAWER])


def test_missing_object_is_rejected(tmp_path: Path, robot_config: dict) -> None:
    with pytest.raises(FileNotFoundError):
        compose_scene(SCENE, robot_root=robot_config["base_frame"], gravcomp=False, objects=[tmp_path / "nothing.xml"])
```

Run: `make test`
Expected: the three new tests FAIL. `drawer.xml` isn't installed yet, and `compose_scene` has no `objects` argument (`TypeError: compose_scene() got an unexpected keyword argument 'objects'`).

- [ ] **Step 2: The drawer model**

`src/arm_sandbox_sim/objects/drawer.xml` (no `<default>` classes, see the design notes):

```xml
<mujoco model="drawer">
  <!-- A cabinet with one drawer in front of the robot (robot base at the origin, x forward).
       The drawer slides towards the robot (-x). Its handle is a bar along y, held in front of the
       drawer face, so the closed gripper fits behind it and pulls it like a hook.
       Scene composition attaches this file under the prefix "drawer/": its slide joint is
       drawer/slide, its handle site drawer/handle. -->
  <worldbody>
    <body name="cabinet" pos="0.80 0 0">
      <geom name="cabinet_bottom" type="box" pos="0 0 0.01" size="0.2 0.22 0.01" rgba="0.55 0.42 0.3 1"/>
      <geom name="cabinet_left" type="box" pos="0 0.21 0.15" size="0.2 0.01 0.13" rgba="0.55 0.42 0.3 1"/>
      <geom name="cabinet_right" type="box" pos="0 -0.21 0.15" size="0.2 0.01 0.13" rgba="0.55 0.42 0.3 1"/>
      <geom name="cabinet_back" type="box" pos="0.19 0 0.15" size="0.01 0.2 0.13" rgba="0.55 0.42 0.3 1"/>
      <body name="tray" pos="0 0 0.03">
        <!-- slides along -x (towards the robot) by up to 0.3 m; damping and friction loss so that
             opening it takes a steady pull of a few newtons -->
        <joint name="slide" type="slide" axis="-1 0 0" range="0 0.3" damping="10" frictionloss="3"/>
        <inertial pos="0 0 0.06" mass="1.0" diaginertia="0.02 0.02 0.02"/>
        <geom name="tray_bottom" type="box" pos="0 0 0.005" size="0.18 0.19 0.005" rgba="0.75 0.62 0.45 1"/>
        <geom name="tray_front" type="box" pos="-0.19 0 0.11" size="0.01 0.2 0.11" rgba="0.75 0.62 0.45 1"/>
        <geom name="tray_back" type="box" pos="0.17 0 0.07" size="0.01 0.19 0.07" rgba="0.75 0.62 0.45 1"/>
        <geom name="tray_left" type="box" pos="0 0.18 0.07" size="0.18 0.01 0.07" rgba="0.75 0.62 0.45 1"/>
        <geom name="tray_right" type="box" pos="0 -0.18 0.07" size="0.18 0.01 0.07" rgba="0.75 0.62 0.45 1"/>
        <geom name="handle_left_post" type="box" pos="-0.225 0.08 0.20" size="0.025 0.006 0.006" rgba="0.3 0.3 0.3 1"/>
        <geom name="handle_right_post" type="box" pos="-0.225 -0.08 0.20" size="0.025 0.006 0.006" rgba="0.3 0.3 0.3 1"/>
        <geom name="handle_bar" type="capsule" fromto="-0.256 0.09 0.20 -0.256 -0.09 0.20" size="0.008" rgba="0.3 0.3 0.3 1"/>
        <site name="handle" pos="-0.256 0 0.20" size="0.01" rgba="1 0 0 1"/>
      </body>
    </body>
  </worldbody>
  <contact>
    <exclude body1="cabinet" body2="tray"/>
  </contact>
</mujoco>
```

`src/arm_sandbox_sim/CMakeLists.txt` (installs `objects/`):

```cmake
cmake_minimum_required(VERSION 3.16)
project(arm_sandbox_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ROS-free Python module (scene composition), shared by the launch files and, later, the Gym env.
ament_python_install_package(${PROJECT_NAME})
# Task objects (standalone MJCF), attached to the scene by name: sim.launch.py objects:=drawer
install(DIRECTORY objects DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_scene test/test_scene.py)
endif()

ament_package()
```

- [ ] **Step 3: Attach objects in `compose_scene`**

`src/arm_sandbox_sim/arm_sandbox_sim/scene.py`:

```python
"""Compose the MuJoCo scene that the sim loads (design spec "Simulation").

ROS-free on purpose: the launch files use it now, and the Phase B Gymnasium environment will load
exactly the same model through it.
"""

import tempfile
from collections.abc import Sequence
from pathlib import Path

import mujoco

COMPOSED_FILE_NAME = "composed_scene.xml"


def _absolute_asset_dirs(spec: mujoco.MjSpec, source: Path) -> None:
    """Relative mesh and texture paths resolve against the source file's folder. Make them absolute
    so the composed file can be written anywhere."""
    model_dir = source.resolve().parent
    spec.meshdir = str(model_dir / spec.meshdir)
    spec.texturedir = str(model_dir / spec.texturedir)


def compose_scene(scene_path: Path, *, robot_root: str, gravcomp: bool, objects: Sequence[Path] = ()) -> str:
    """Return the scene as one MJCF string that loads from any directory.

    Args:
        scene_path: The robot's scene file (e.g. share/arm_sandbox_description/panda/mjcf/scene.xml).
        robot_root: Name of the robot's base body (robot.yaml `base_frame`).
        gravcomp: If True, MuJoCo cancels gravity on the robot's bodies, as the real Panda's torque
            interface does. If False (the default in the launch files, D15), controllers must
            compensate gravity themselves. Bodies outside the robot are never compensated.
        objects: Task object files (standalone MJCF, positions in the world frame) attached to the
            scene, e.g. the drawer of the drawer task, each under the prefix "<file stem>/". A
            task's objects change only on relaunch; poses are set per reset.
    """
    if not scene_path.is_file():
        raise FileNotFoundError(f"MuJoCo scene not found: {scene_path}")

    spec = mujoco.MjSpec.from_file(str(scene_path))
    _absolute_asset_dirs(spec, scene_path)

    root = spec.body(robot_root)
    if root is None:
        raise ValueError(f"robot root body '{robot_root}' not found in {scene_path}")
    if gravcomp:
        for body in [root, *root.find_all(mujoco.mjtObj.mjOBJ_BODY)]:
            body.gravcomp = 1.0

    for object_path in objects:
        if not object_path.is_file():
            raise FileNotFoundError(f"scene object not found: {object_path}")
        object_spec = mujoco.MjSpec.from_file(str(object_path))
        _absolute_asset_dirs(object_spec, object_path)
        # Each object's names get its file name as prefix ("drawer.xml": joint "slide" becomes
        # "drawer/slide"). MuJoCo 3.12 can't write an attachment back to XML without a prefix, and
        # it keeps objects from clashing with each other or the robot.
        spec.attach(object_spec, frame=spec.worldbody.add_frame(), prefix=f"{object_path.stem}/")

    try:
        spec.compile()
    except ValueError as error:
        raise ValueError(f"composed scene does not compile: {error}") from error
    return spec.to_xml()


def write_composed_scene(
    scene_path: Path,
    *,
    robot_root: str,
    gravcomp: bool,
    objects: Sequence[Path] = (),
    output_dir: Path | None = None,
) -> Path:
    """Write compose_scene()'s result to `output_dir` (a new temp dir if None) and return its path."""
    xml = compose_scene(scene_path, robot_root=robot_root, gravcomp=gravcomp, objects=objects)
    directory = output_dir if output_dir is not None else Path(tempfile.mkdtemp(prefix="arm_sandbox_scene_"))
    path = directory / COMPOSED_FILE_NAME
    path.write_text(xml)
    return path
```

`src/arm_sandbox_bringup/launch/sim.launch.py` (adds `objects:=`):

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
    objects_dir = Path(get_package_share_directory("arm_sandbox_sim")) / "objects"
    object_names = [name for name in LaunchConfiguration("objects").perform(context).split(",") if name]
    scene = write_composed_scene(
        require_file(description_dir / "mjcf" / "scene.xml", "MuJoCo scene"),
        robot_root=robot_config["base_frame"],
        gravcomp=is_true(context, "gravcomp"),
        objects=[require_file(objects_dir / f"{name}.xml", "scene object") for name in object_names],
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
                "objects",
                default_value="",
                description="Task objects to add, comma-separated names from arm_sandbox_sim/objects (e.g. drawer)",
            ),
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
Expected: `test_scene` 9 passed; everything else still passes.

- [ ] **Step 5: Look at it**

```bash
make start_sim ARGS="objects:=drawer"   # native viewer: a cabinet in front of the robot, handle bar facing it
source install/setup.bash && ros2 topic echo --once /mujoco_actuators_states --field name   # ends with 'drawer/slide'
```

- [ ] **Step 6: Commit**

```bash
git add src/arm_sandbox_sim src/arm_sandbox_bringup
git commit -m "feat(sim): task objects attached to the scene at launch (objects:=drawer)"
```

---

# Part 2 — The Drawer Task

### Task 2: Shared runner plumbing (refactor, behaviour unchanged)

`drawer_runner` needs most of what `reach_runner` already has: frames, the end-effector pose from TF, waiting for the stack, holding a condition for `hold_s`, sending pose targets. That moves into `TaskRunner`, and the task-file field readers move into `src/yaml_fields.hpp`. `reach_runner` and `reach_task.cpp` are rewritten on top of them. Their behaviour must not change: the reach unit and end-to-end tests are the check.

**Files:**
- Create: `src/arm_sandbox_tasks/src/yaml_fields.hpp`, `src/arm_sandbox_tasks/include/arm_sandbox_tasks/task_runner.hpp`
- Modify: `src/arm_sandbox_tasks/src/reach_task.cpp`, `src/arm_sandbox_tasks/src/reach_runner.cpp`, `src/arm_sandbox_tasks/CMakeLists.txt`

**Interfaces:**
- Produces:
  - Namespace `arm_sandbox_tasks::yaml_fields`: `vector3(node, what)`, `positive(node, what)`, `non_empty_string(node, what)`, and `load(path, kind, parse)`, which prefixes errors with `"<kind> task <path>: "`.
  - `class TaskRunner : public rclcpp::Node`. Protected members:
    - `TaskRunner(name)` declares `base_frame`, `ee_frame` and `startup_timeout_s`;
    - accessors `base_frame()`, `ee_frame()`, `startup_timeout_s()`;
    - `ee_pose()`;
    - `wait_until(pred, what)`;
    - `held_for(pred, hold_s, deadline)`;
    - `send_pose_target(publisher, pose)`;
    - `spin_for(period)`;
    - `seconds(double)`, `Clock`, `kPollPeriod`.
  - CMake: the ROS-free library is renamed from `reach_task` to `task_logic`.

- [ ] **Step 1: Write the shared helpers**

`src/arm_sandbox_tasks/src/yaml_fields.hpp`:

```cpp
#pragma once

// Field readers shared by the task-file loaders (reach_task.cpp, drawer_task.cpp). They throw
// std::invalid_argument naming the field; the loaders add the file name.

#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

namespace arm_sandbox_tasks::yaml_fields
{

inline Eigen::Vector3d vector3(const YAML::Node & node, const std::string & what)
{
  if (!node || !node.IsSequence() || node.size() != 3) {
    throw std::invalid_argument(what + " must be a list of 3 numbers");
  }
  return Eigen::Vector3d(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
}

inline double positive(const YAML::Node & node, const std::string & what)
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

inline std::string non_empty_string(const YAML::Node & node, const std::string & what)
{
  const std::string value = node ? node.as<std::string>() : "";
  if (value.empty()) {
    throw std::invalid_argument(what + " is missing");
  }
  return value;
}

/// Loads `path`, then runs `parse`, prefixing any error with "<kind> task <path>: ".
template <typename Parse>
auto load(const std::string & path, const std::string & kind, Parse parse)
{
  try {
    return parse(YAML::LoadFile(path));
  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(kind + " task " + path + ": " + error.what());
  } catch (const std::invalid_argument & error) {
    throw std::invalid_argument(kind + " task " + path + ": " + error.what());
  }
}

}  // namespace arm_sandbox_tasks::yaml_fields
```

`src/arm_sandbox_tasks/include/arm_sandbox_tasks/task_runner.hpp`:

```cpp
#pragma once

#include <chrono>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace arm_sandbox_tasks
{

/// ROS plumbing shared by the task runners (reach_runner, drawer_runner): robot frames, the
/// end-effector pose from TF, waiting for the stack, holding a condition, sending pose targets.
/// Runners are sequential programs: they spin themselves while they wait.
class TaskRunner : public rclcpp::Node
{
protected:
  using Clock = std::chrono::steady_clock;

  /// Declares `base_frame`, `ee_frame` (robot.yaml) and `startup_timeout_s`. Throws
  /// std::invalid_argument if one is missing.
  explicit TaskRunner(const std::string & name)
  : Node(name),
    base_frame_(declare_parameter<std::string>("base_frame", "")),
    ee_frame_(declare_parameter<std::string>("ee_frame", "")),
    startup_timeout_s_(declare_parameter<double>("startup_timeout_s", 0.0)),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    if (base_frame_.empty() || ee_frame_.empty() || startup_timeout_s_ <= 0.0) {
      RCLCPP_FATAL(get_logger(), "base_frame and ee_frame must be set, startup_timeout_s > 0");
      throw std::invalid_argument("invalid runner config");
    }
  }

  static Clock::duration seconds(double value)
  {
    return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(value));
  }

  const std::string & base_frame() const { return base_frame_; }
  const std::string & ee_frame() const { return ee_frame_; }
  double startup_timeout_s() const { return startup_timeout_s_; }

  /// End-effector pose in the base frame from TF, if available yet.
  std::optional<Eigen::Isometry3d> ee_pose() const
  {
    try {
      return tf2::transformToEigen(tf_buffer_.lookupTransform(base_frame_, ee_frame_, tf2::TimePointZero));
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  /// Spins until `ready()` or startup_timeout_s; logs what it waited for on timeout.
  template <typename Predicate>
  bool wait_until(Predicate ready, const std::string & what)
  {
    const auto deadline = Clock::now() + seconds(startup_timeout_s_);
    while (!ready()) {
      if (Clock::now() > deadline) {
        RCLCPP_FATAL(get_logger(), "timed out waiting for %s", what.c_str());
        return false;
      }
      spin_for();
    }
    return true;
  }

  /// Spins until `ok()` has stayed true for `hold_s` of sim time (true), or until `deadline` (false).
  template <typename Predicate>
  bool held_for(Predicate ok, double hold_s, Clock::time_point deadline)
  {
    std::optional<rclcpp::Time> ok_since;
    while (Clock::now() <= deadline) {
      if (!ok()) {
        ok_since.reset();
      } else if (!ok_since) {
        ok_since = now();
      } else if ((now() - *ok_since).seconds() >= hold_s) {
        return true;
      }
      spin_for();
    }
    return false;
  }

  /// Publishes `pose` (base frame) as a target for a task-space controller.
  void send_pose_target(rclcpp::Publisher<geometry_msgs::msg::PoseStamped> & publisher, const Eigen::Isometry3d & pose)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = base_frame_;
    msg.header.stamp = now();
    msg.pose = tf2::toMsg(pose);
    publisher.publish(msg);
  }

  void spin_for(std::chrono::milliseconds period = kPollPeriod)
  {
    rclcpp::spin_some(shared_from_this());
    std::this_thread::sleep_for(period);
  }

  static constexpr std::chrono::milliseconds kPollPeriod{20};

private:
  std::string base_frame_;
  std::string ee_frame_;
  double startup_timeout_s_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;  ///< runs its own node and thread
};

}  // namespace arm_sandbox_tasks
```

- [ ] **Step 2: Move `reach` onto them**

`src/arm_sandbox_tasks/src/reach_task.cpp`:

```cpp
#include "arm_sandbox_tasks/reach_task.hpp"

#include <algorithm>
#include <stdexcept>

#include "yaml_fields.hpp"

namespace arm_sandbox_tasks
{
namespace
{
constexpr const char * kSupportedSuccessType = "ee_at_pose";
}  // namespace

ReachTask load_reach_task(const std::string & path)
{
  using yaml_fields::positive;
  using yaml_fields::vector3;
  return yaml_fields::load(path, "reach", [](const YAML::Node & yaml) {
    ReachTask task;
    task.name = yaml_fields::non_empty_string(yaml["name"], "name");
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
  });
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

`src/arm_sandbox_tasks/src/reach_runner.cpp`:

```cpp
// reach_runner: solves the reach task (milestones M2, M3). Thin ROS wrapper: the math is in
// arm_sandbox_kinematics, the task logic in reach_task.hpp, the ROS plumbing in task_runner.hpp.
//
// Two ways to move (parameter `motion`):
// - joint_trajectory (M2): IK from the current joint positions (null-space pull towards home) ->
//   one FollowJointTrajectory goal to the joint trajectory controller.
// - pose_target (M3): publish the target pose to a task-space controller (OSC or impedance), which
//   does the rest; no IK.
// Either way, a target counts once TF shows the end effector held within the task tolerance.
// Exits 0 if every target was reached, 1 otherwise.

#include <algorithm>
#include <cstdio>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include "arm_sandbox_kinematics/ik.hpp"
#include "arm_sandbox_kinematics/kinematic_chain.hpp"
#include "arm_sandbox_tasks/reach_task.hpp"
#include "arm_sandbox_tasks/task_runner.hpp"

namespace arm_sandbox_tasks
{
namespace
{
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
constexpr const char * kJointTrajectory = "joint_trajectory";
constexpr const char * kPoseTarget = "pose_target";
}  // namespace

class ReachRunner : public TaskRunner
{
public:
  ReachRunner()
  : TaskRunner("reach_runner"),
    arm_joints_(declare_parameter<std::vector<std::string>>("arm_joints", std::vector<std::string>{})),
    task_(load_reach_task(declare_parameter<std::string>("task_file", ""))),
    velocity_scale_(declare_parameter<double>("velocity_scale", 0.0)),
    min_move_duration_s_(declare_parameter<double>("min_move_duration_s", 0.0)),
    settle_timeout_s_(declare_parameter<double>("settle_timeout_s", 0.0))
  {
    const auto motion = declare_parameter<std::string>("motion", "");
    if (motion != kJointTrajectory && motion != kPoseTarget) {
      RCLCPP_FATAL(get_logger(), "motion must be '%s' or '%s', got '%s'", kJointTrajectory, kPoseTarget, motion.c_str());
      throw std::invalid_argument("invalid motion");
    }
    use_pose_target_ = motion == kPoseTarget;
    const auto home = declare_parameter<std::vector<double>>("home", std::vector<double>{});
    if (arm_joints_.empty() || home.size() != arm_joints_.size()) {
      RCLCPP_FATAL(get_logger(), "robot config missing: arm_joints, home");
      throw std::invalid_argument("invalid robot config");
    }
    if (velocity_scale_ <= 0.0 || min_move_duration_s_ <= 0.0 || settle_timeout_s_ <= 0.0) {
      RCLCPP_FATAL(get_logger(), "velocity_scale, min_move_duration_s, settle_timeout_s must be > 0");
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
                         : !arm_client_->wait_for_action_server(seconds(startup_timeout_s()))) {
      return false;
    }

    const arm_sandbox_kinematics::KinematicChain chain(*robot_description_, base_frame(), ee_frame());
    if (chain.joint_names() != arm_joints_) {
      RCLCPP_FATAL(get_logger(), "the URDF chain %s -> %s doesn't match robot.yaml arm_joints", base_frame().c_str(), ee_frame().c_str());
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
      send_pose_target(*target_pub_, target);
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
    const bool held = held_for(
      [&] {
        const std::optional<Eigen::Isometry3d> ee = ee_pose();
        if (!ee) {
          return false;
        }
        error = pose_error(target, *ee);
        return within(error, task_.tolerance);
      },
      task_.hold_s, deadline);
    if (!held) {
      RCLCPP_ERROR(get_logger(), "target %zu: not held within tolerance (%.4f m, %.4f rad)", index, error.position, error.orientation);
      return false;
    }
    const double elapsed = std::chrono::duration<double>(Clock::now() - start_time).count();
    RCLCPP_INFO(get_logger(), "target %zu reached: %.4f m, %.4f rad in %.2f s (%s)",
                index, error.position, error.orientation, elapsed, how.c_str());
    return true;
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

  std::vector<std::string> arm_joints_;
  ReachTask task_;
  double velocity_scale_;
  double min_move_duration_s_;
  double settle_timeout_s_;
  Eigen::VectorXd home_;
  arm_sandbox_kinematics::IkOptions ik_options_;

  std::optional<std::string> robot_description_;
  std::optional<sensor_msgs::msg::JointState> latest_joint_state_;
  bool controller_active_ = false;
  bool use_pose_target_ = false;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
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

`src/arm_sandbox_tasks/CMakeLists.txt`. This is the final file. Until Tasks 3–4, leave out:
- `src/drawer_task.cpp` from `task_logic` (Task 3);
- the `drawer_runner` executable block and its name in `install(TARGETS …)` (Task 4);
- the `test_drawer_task` gtest block (Task 3) and the `test_drawer.py` launch test (Task 4).

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

# ROS-free task logic (task files, pose errors, move timing, pull paths), unit-tested on its own.
add_library(task_logic STATIC src/reach_task.cpp src/drawer_task.cpp)
set_target_properties(task_logic PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(task_logic PUBLIC include)
target_link_libraries(task_logic PUBLIC Eigen3::Eigen yaml-cpp)

# Thin ROS nodes on top of it (shared plumbing: include/arm_sandbox_tasks/task_runner.hpp).
add_executable(reach_runner src/reach_runner.cpp)
target_link_libraries(reach_runner task_logic arm_sandbox_kinematics::arm_sandbox_kinematics)
ament_target_dependencies(reach_runner control_msgs geometry_msgs rclcpp rclcpp_action sensor_msgs std_msgs tf2_eigen tf2_ros)

add_executable(drawer_runner src/drawer_runner.cpp)
target_link_libraries(drawer_runner task_logic)
ament_target_dependencies(drawer_runner control_msgs geometry_msgs rclcpp rclcpp_action sensor_msgs tf2_eigen tf2_ros)

install(TARGETS reach_runner drawer_runner DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_reach_task test/test_reach_task.cpp)
  target_link_libraries(test_reach_task task_logic)
  target_compile_definitions(test_reach_task PRIVATE
    REACH_TASK_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/tasks/reach.yaml")
  ament_add_gtest(test_drawer_task test/test_drawer_task.cpp)
  target_link_libraries(test_drawer_task task_logic)
  target_compile_definitions(test_drawer_task PRIVATE
    DRAWER_TASK_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/tasks/drawer.yaml")
  # Full stack: sim + reach_runner. Own ROS domain (see arm_sandbox_bringup/CMakeLists.txt).
  add_launch_test(test/test_reach.py TIMEOUT 900 ENV ROS_DOMAIN_ID=45)
  add_launch_test(test/test_drawer.py TIMEOUT 300 ENV ROS_DOMAIN_ID=47)
endif()

ament_package()
```

- [ ] **Step 3: Run the tests: behaviour unchanged**

Run: `make test`
Expected: 0 failures. `test_reach_task` 6 passed and `test_reach` 12 passed, exactly as before the refactor. If a reach case fails now, the refactor changed behaviour. Compare the failing log line with the Plan 04 table before changing anything else.

- [ ] **Step 4: Commit**

```bash
git add src/arm_sandbox_tasks
git commit -m "refactor(tasks): shared TaskRunner base and task-file readers; reach_runner uses them"
```

### Task 3: The drawer task file and logic

**Files:**
- Create: `src/arm_sandbox_tasks/include/arm_sandbox_tasks/drawer_task.hpp`, `src/arm_sandbox_tasks/src/drawer_task.cpp`, `src/arm_sandbox_tasks/config/tasks/drawer.yaml`
- Test: `src/arm_sandbox_tasks/test/test_drawer_task.cpp`
- Modify: `src/arm_sandbox_tasks/CMakeLists.txt` (restore `drawer_task.cpp` and `test_drawer_task`)

**Interfaces:**
- Consumes: `yaml_fields` (Task 2); `pose_from_xyz_rpy` (`reach_task.hpp`).
- Produces:
  - `struct DrawerTask { name; scene_objects; controller; hook_pose; approach_height; pull_direction (unit); pull_distance; pull_speed; joint; min_opening; hold_s; time_limit_s; }`
  - `DrawerTask load_drawer_task(const std::string & path)`, which throws `std::invalid_argument`, for example on `success.type` other than `joint_opened` or a zero pull direction.
  - `Eigen::Isometry3d pull_target(start, direction, speed, distance, t)`.

- [ ] **Step 1: Write the failing tests**

`src/arm_sandbox_tasks/config/tasks/drawer.yaml`:

```yaml
# Drawer task (REQUIREMENTS FR-15, task 5; milestone M3): hook the closed gripper behind the drawer
# handle and pull the drawer open with the Cartesian impedance controller, which gives way where
# the pulled path and the drawer's slide don't agree exactly.
# The sim must run with these (make start_sim ARGS="objects:=drawer arm_controller:=cartesian_impedance_controller"):
name: drawer
scene: [drawer] # objects from arm_sandbox_sim/objects
controller: cartesian_impedance_controller
hook:
  # End-effector pose behind the handle bar, in the robot base frame: between the bar and the
  # drawer face, gripper pointing down, fingers closed. Known from the drawer model (ground truth);
  # perception provides it from M5 on.
  position: [0.574, 0.0, 0.23]
  rpy: [3.14159265, 0.0, 0.0]
  approach_height: 0.10 # m: come down from this far above
pull:
  direction: [-1.0, 0.0, 0.0] # towards the robot, along the drawer's slide
  distance: 0.20 # m
  speed: 0.05 # m/s
success:
  type: joint_opened
  joint: drawer/slide # sim ground truth (/mujoco_actuators_states)
  min_opening: 0.15 # m
  hold_s: 0.5
time_limit_s: 60.0 # whole task
```

`src/arm_sandbox_tasks/test/test_drawer_task.cpp`:

```cpp
// ROS-free drawer task logic: task file parsing and validation, the pull path.

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <string>

#include "arm_sandbox_tasks/drawer_task.hpp"

namespace arm_sandbox_tasks
{
namespace
{
constexpr double kTolerance = 1e-12;

std::string write_task(const std::string & contents)
{
  const std::string path = ::testing::TempDir() + "drawer_task_test.yaml";
  std::ofstream(path) << contents;
  return path;
}

const char * const kValidTask = R"(
name: drawer
scene: [drawer]
controller: cartesian_impedance_controller
hook: {position: [0.5, 0.0, 0.2], rpy: [3.14159265, 0.0, 0.0], approach_height: 0.1}
pull: {direction: [-2.0, 0.0, 0.0], distance: 0.2, speed: 0.05}
success: {type: joint_opened, joint: drawer/slide, min_opening: 0.15, hold_s: 0.5}
time_limit_s: 60.0
)";

std::string with(std::string text, const std::string & from, const std::string & to)
{
  text.replace(text.find(from), from.size(), to);
  return text;
}

TEST(DrawerTask, ShippedTaskFileLoads)
{
  const DrawerTask task = load_drawer_task(DRAWER_TASK_FILE);
  EXPECT_EQ(task.name, "drawer");
  EXPECT_FALSE(task.scene_objects.empty());
  EXPECT_FALSE(task.controller.empty());
  EXPECT_LT(task.min_opening, task.pull_distance);  // the pull goes further than success needs
}

TEST(DrawerTask, ParsesFields)
{
  const DrawerTask task = load_drawer_task(write_task(kValidTask));
  EXPECT_EQ(task.scene_objects, std::vector<std::string>{"drawer"});
  EXPECT_EQ(task.controller, "cartesian_impedance_controller");
  EXPECT_TRUE(task.hook_pose.translation().isApprox(Eigen::Vector3d(0.5, 0.0, 0.2)));
  EXPECT_DOUBLE_EQ(task.approach_height, 0.1);
  EXPECT_TRUE(task.pull_direction.isApprox(-Eigen::Vector3d::UnitX()));  // normalized
  EXPECT_EQ(task.joint, "drawer/slide");
  EXPECT_DOUBLE_EQ(task.min_opening, 0.15);
}

TEST(DrawerTask, RejectsInvalidFiles)
{
  EXPECT_THROW(load_drawer_task("/no/such/file.yaml"), std::invalid_argument);
  EXPECT_THROW(load_drawer_task(write_task(with(kValidTask, "joint_opened", "ee_at_pose"))), std::invalid_argument);
  EXPECT_THROW(load_drawer_task(write_task(with(kValidTask, "[-2.0, 0.0, 0.0]", "[0.0, 0.0, 0.0]"))), std::invalid_argument);
  EXPECT_THROW(load_drawer_task(write_task(with(kValidTask, "scene: [drawer]", "scene: []"))), std::invalid_argument);
  EXPECT_THROW(load_drawer_task(write_task(with(kValidTask, "speed: 0.05", "speed: 0.0"))), std::invalid_argument);
  EXPECT_THROW(load_drawer_task(write_task(with(kValidTask, "joint: drawer/slide, ", ""))), std::invalid_argument);
}

TEST(DrawerTask, PullTargetMovesAtSpeedAndStopsAtDistance)
{
  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  start.translation() = Eigen::Vector3d(0.5, 0.1, 0.2);
  start.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  const Eigen::Vector3d direction = -Eigen::Vector3d::UnitX();
  EXPECT_TRUE(pull_target(start, direction, 0.05, 0.2, 0.0).isApprox(start, kTolerance));
  EXPECT_NEAR(pull_target(start, direction, 0.05, 0.2, 2.0).translation().x(), 0.4, kTolerance);
  EXPECT_NEAR(pull_target(start, direction, 0.05, 0.2, 100.0).translation().x(), 0.3, kTolerance);  // capped
  EXPECT_TRUE(pull_target(start, direction, 0.05, 0.2, 2.0).linear().isApprox(start.linear(), kTolerance));
}

}  // namespace
}  // namespace arm_sandbox_tasks
```

Restore `src/drawer_task.cpp` in `task_logic` and the `test_drawer_task` block in the CMake file. Run: `make test`
Expected: FAIL at CMake configure with `Cannot find source file` (`src/drawer_task.cpp`).

- [ ] **Step 2: Write the logic**

```cpp
#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace arm_sandbox_tasks
{

/// The drawer task (REQUIREMENTS FR-15, task 5): hook the closed gripper behind the drawer handle and
/// pull the drawer open. Loaded from `config/tasks/drawer.yaml`. ROS-free.
struct DrawerTask
{
  std::string name;
  /// What the sim must run with: its objects (sim.launch.py objects:=...) and the arm controller.
  std::vector<std::string> scene_objects;
  std::string controller;
  /// End-effector pose behind the handle, in the robot's base frame, and how far above it the
  /// approach starts.
  Eigen::Isometry3d hook_pose = Eigen::Isometry3d::Identity();
  double approach_height = 0.0;  ///< m, > 0
  Eigen::Vector3d pull_direction = Eigen::Vector3d::Zero();  ///< unit vector, base frame
  double pull_distance = 0.0;  ///< m, > 0
  double pull_speed = 0.0;     ///< m/s, > 0
  /// Success (`joint_opened`): the drawer joint, read from sim ground truth, stays at or beyond
  /// `min_opening` for `hold_s`.
  std::string joint;
  double min_opening = 0.0;  ///< m, > 0
  double hold_s = 0.0;       ///< s, > 0
  double time_limit_s = 0.0;  ///< whole task, > 0
};

/// Parse and validate a drawer task file. Throws std::invalid_argument with the file and field, e.g.
/// for an unsupported `success.type` or a zero pull direction (normalized otherwise).
DrawerTask load_drawer_task(const std::string & path);

/// Target pose `t` seconds into the pull: `start` moved along `direction` at `speed`, stopping after
/// `distance`. Orientation unchanged.
Eigen::Isometry3d pull_target(
  const Eigen::Isometry3d & start, const Eigen::Vector3d & direction, double speed, double distance, double t);

}  // namespace arm_sandbox_tasks
```

```cpp
#include "arm_sandbox_tasks/drawer_task.hpp"

#include <algorithm>
#include <stdexcept>

#include "arm_sandbox_tasks/reach_task.hpp"
#include "yaml_fields.hpp"

namespace arm_sandbox_tasks
{
namespace
{
constexpr const char * kSupportedSuccessType = "joint_opened";
}  // namespace

DrawerTask load_drawer_task(const std::string & path)
{
  using yaml_fields::non_empty_string;
  using yaml_fields::positive;
  using yaml_fields::vector3;
  return yaml_fields::load(path, "drawer", [](const YAML::Node & yaml) {
    DrawerTask task;
    task.name = non_empty_string(yaml["name"], "name");
    const YAML::Node scene = yaml["scene"];
    if (!scene || !scene.IsSequence() || scene.size() == 0) {
      throw std::invalid_argument("scene must be a non-empty list of sim object names");
    }
    task.scene_objects = scene.as<std::vector<std::string>>();
    task.controller = non_empty_string(yaml["controller"], "controller");

    const YAML::Node hook = yaml["hook"];
    if (!hook) {
      throw std::invalid_argument("hook is missing");
    }
    task.hook_pose = pose_from_xyz_rpy(vector3(hook["position"], "hook.position"), vector3(hook["rpy"], "hook.rpy"));
    task.approach_height = positive(hook["approach_height"], "hook.approach_height");

    const YAML::Node pull = yaml["pull"];
    if (!pull) {
      throw std::invalid_argument("pull is missing");
    }
    const Eigen::Vector3d direction = vector3(pull["direction"], "pull.direction");
    if (direction.norm() == 0.0) {
      throw std::invalid_argument("pull.direction must not be zero");
    }
    task.pull_direction = direction.normalized();
    task.pull_distance = positive(pull["distance"], "pull.distance");
    task.pull_speed = positive(pull["speed"], "pull.speed");

    const YAML::Node success = yaml["success"];
    if (!success || !success["type"] || success["type"].as<std::string>() != kSupportedSuccessType) {
      throw std::invalid_argument(std::string("success.type must be '") + kSupportedSuccessType + "'");
    }
    task.joint = non_empty_string(success["joint"], "success.joint");
    task.min_opening = positive(success["min_opening"], "success.min_opening");
    task.hold_s = positive(success["hold_s"], "success.hold_s");
    task.time_limit_s = positive(yaml["time_limit_s"], "time_limit_s");
    return task;
  });
}

Eigen::Isometry3d pull_target(
  const Eigen::Isometry3d & start, const Eigen::Vector3d & direction, double speed, double distance, double t)
{
  Eigen::Isometry3d target = start;
  target.translation() += direction * std::clamp(speed * t, 0.0, distance);
  return target;
}

}  // namespace arm_sandbox_tasks
```

- [ ] **Step 3: Run the tests to see them pass**

Run: `make test`
Expected: `test_drawer_task` 4 passed.

- [ ] **Step 4: Commit**

```bash
git add src/arm_sandbox_tasks
git commit -m "feat(tasks): drawer task file and logic (hook pose, pull path, joint_opened)"
```

### Task 4: `drawer_runner` — open the drawer (M3 acceptance)

The sequence:
1. Wait for the controller named in the task, the gripper, and the drawer joint in ground truth. If the joint never appears, the error asks whether the sim was started with `objects:=drawer`.
2. Close the fingers.
3. Move to the approach pose (10 cm above the hook pose) and hold it, then the hook pose.
4. Stream the pull path at 50 Hz.
5. Succeed once the drawer joint stays ≥ `min_opening` for `hold_s`.

**Files:**
- Create: `src/arm_sandbox_tasks/src/drawer_runner.cpp`, `src/arm_sandbox_tasks/config/drawer_runner.yaml`, `src/arm_sandbox_tasks/launch/drawer.launch.py`
- Test: `src/arm_sandbox_tasks/test/test_drawer.py`
- Modify: `src/arm_sandbox_tasks/CMakeLists.txt` (restore `drawer_runner` and the launch test), `Makefile`, `CLAUDE.md`, `docs/PROJECT_STRUCTURE.md`

**Interfaces:**
- Consumes: `TaskRunner` (Task 2), `DrawerTask` (Task 3), `/cartesian_impedance_controller/target_pose` and `/pose_error` (Plan 04), `/gripper_controller/gripper_cmd`, `/mujoco_actuators_states`.
- Produces: `ros2 launch arm_sandbox_tasks drawer.launch.py robot:=panda task:=drawer` (it reads the controller from the task file), and `make start_drawer`.

- [ ] **Step 1: Write the failing launch test**

`src/arm_sandbox_tasks/test/test_drawer.py`:

```python
"""M3 acceptance: with the drawer in the sim, drawer_runner opens it with the impedance controller.

Runs once per gravity-compensation mode (D15), like the reach test.
"""

import unittest
from pathlib import Path

import launch_testing
import launch_testing.asserts
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest

TASK = "drawer"
# Startup (~10 s) plus the approach, the pull (4 s) and the hold.
DRAWER_TIMEOUT_S = 120.0


def launch_file(package: str, name: str) -> str:
    return str(Path(get_package_share_directory(package)) / "launch" / name)


def task_config() -> dict:
    path = Path(get_package_share_directory("arm_sandbox_tasks")) / "config" / "tasks" / f"{TASK}.yaml"
    return yaml.safe_load(path.read_text())


@pytest.mark.launch_test
@launch_testing.parametrize("gravcomp", ["false", "true"])
def generate_test_description(gravcomp: str):
    task = task_config()
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file("arm_sandbox_bringup", "sim.launch.py")),
        launch_arguments={
            "robot": "panda",
            "viewer": "false",
            "rerun": "false",
            "gravcomp": gravcomp,
            "objects": ",".join(task["scene"]),
            "arm_controller": task["controller"],
        }.items(),
    )
    drawer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file("arm_sandbox_tasks", "drawer.launch.py")),
        launch_arguments={"robot": "panda", "task": TASK}.items(),
    )
    return LaunchDescription([sim, drawer, ReadyToTest()])


class TestDrawer(unittest.TestCase):
    def test_drawer_runner_finishes(self, proc_info) -> None:
        proc_info.assertWaitForShutdown(process="drawer_runner", timeout=DRAWER_TIMEOUT_S)


@launch_testing.post_shutdown_test()
class TestDrawerResult(unittest.TestCase):
    def test_drawer_opened(self, proc_info, proc_output) -> None:
        # drawer_runner exits 0 only if the drawer stayed open for hold_s.
        launch_testing.asserts.assertExitCodes(proc_info, process="drawer_runner")
        launch_testing.asserts.assertInStderr(proc_output, "drawer opened", "drawer_runner")
```

Restore the `drawer_runner` lines and the `test_drawer.py` launch test in the CMake file. Run: `make test`
Expected: FAIL at CMake configure with `Cannot find source file` (`src/drawer_runner.cpp`).

- [ ] **Step 2: Write the runner, its config and launch file**

`src/arm_sandbox_tasks/config/drawer_runner.yaml`:

```yaml
# drawer_runner: hooks the drawer handle and pulls it open through a task-space controller.
# drawer.launch.py passes the robot's frames (robot.yaml), the task file, the controller named in
# it, and that controller's topics.
drawer_runner:
  ros__parameters:
    startup_timeout_s: 60.0
    gripper_action: /gripper_controller/gripper_cmd
    gripper_closed_position: 0.0 # m per finger: fully closed, the fingers form the hook
    gripper_max_effort: 50.0 # N
    # Approach and hook poses must be held this closely before moving on.
    approach_position_tolerance: 0.01 # m
    approach_orientation_tolerance: 0.05 # rad
    approach_hold_s: 0.3
    approach_timeout_s: 15.0
    target_rate_hz: 50.0 # pull path streamed at this rate
    # Ground truth (privileged): only the success check reads it.
    ground_truth_topic: /mujoco_actuators_states
```

`src/arm_sandbox_tasks/launch/drawer.launch.py`:

```python
"""Run the drawer task (milestone M3) against a running sim with the drawer.

    make start_sim ARGS="objects:=drawer arm_controller:=cartesian_impedance_controller"
    ros2 launch arm_sandbox_tasks drawer.launch.py robot:=panda task:=drawer

drawer_runner gets the robot's frames from its robot.yaml, the task from config/tasks/<task>.yaml
(which also names the controller it needs), and its settings from config/drawer_runner.yaml.
It exits 0 if the drawer was opened; the launch then ends.
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

ROBOT_CONFIG_KEYS = ("base_frame", "ee_frame")


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
    task_file = require_file(tasks_dir / "config" / "tasks" / f"{task}.yaml", "task file")
    controller = yaml.safe_load(task_file.read_text())["controller"]

    drawer_runner = Node(
        package="arm_sandbox_tasks",
        executable="drawer_runner",
        name="drawer_runner",
        parameters=[
            str(require_file(tasks_dir / "config" / "drawer_runner.yaml", "drawer_runner config")),
            {key: robot_config[key] for key in ROBOT_CONFIG_KEYS},
            {
                "task_file": str(task_file),
                "controller": controller,
                "pose_target_topic": f"/{controller}/target_pose",
                "controller_state_topic": f"/{controller}/pose_error",
                "use_sim_time": True,
            },
        ],
        output="screen",
    )
    return [
        drawer_runner,
        RegisterEventHandler(OnProcessExit(target_action=drawer_runner, on_exit=[EmitEvent(event=Shutdown())])),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="panda", description="Robot folder in arm_sandbox_description"),
            DeclareLaunchArgument("task", default_value="drawer", description="Task file in config/tasks/ (without .yaml)"),
            OpaqueFunction(function=launch_setup),
        ]
    )
```

`src/arm_sandbox_tasks/src/drawer_runner.cpp`:

```cpp
// drawer_runner: solves the drawer task (milestone M3) with a task-space controller, normally the
// Cartesian impedance controller. Thin ROS wrapper: task logic in drawer_task.hpp, plumbing in
// task_runner.hpp.
//
// Close the fingers, come down behind the drawer handle (approach pose, then hook pose), pull along
// the task's direction by streaming pose targets, and succeed once the drawer joint (sim ground
// truth) stays open for hold_s. The controller's compliance takes up the difference between the
// pulled path and the drawer's slide. Exits 0 on success, 1 otherwise.

#include <algorithm>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "arm_sandbox_tasks/drawer_task.hpp"
#include "arm_sandbox_tasks/reach_task.hpp"
#include "arm_sandbox_tasks/task_runner.hpp"

namespace arm_sandbox_tasks
{
namespace
{
using GripperCommand = control_msgs::action::GripperCommand;
}  // namespace

class DrawerRunner : public TaskRunner
{
public:
  DrawerRunner()
  : TaskRunner("drawer_runner"),
    task_(load_drawer_task(declare_parameter<std::string>("task_file", ""))),
    approach_tolerance_{
      declare_parameter<double>("approach_position_tolerance", 0.0),
      declare_parameter<double>("approach_orientation_tolerance", 0.0)},
    approach_hold_s_(declare_parameter<double>("approach_hold_s", 0.0)),
    approach_timeout_s_(declare_parameter<double>("approach_timeout_s", 0.0)),
    target_rate_hz_(declare_parameter<double>("target_rate_hz", 0.0)),
    gripper_closed_position_(declare_parameter<double>("gripper_closed_position", -1.0)),
    gripper_max_effort_(declare_parameter<double>("gripper_max_effort", 0.0))
  {
    if (approach_tolerance_.position <= 0.0 || approach_tolerance_.orientation <= 0.0 || approach_hold_s_ <= 0.0 ||
        approach_timeout_s_ <= 0.0 || target_rate_hz_ <= 0.0 || gripper_closed_position_ < 0.0 || gripper_max_effort_ <= 0.0) {
      RCLCPP_FATAL(get_logger(), "drawer_runner config: tolerances, times, rate and gripper effort must be > 0, "
                                 "gripper_closed_position >= 0");
      throw std::invalid_argument("invalid drawer_runner config");
    }
    const auto controller = declare_parameter<std::string>("controller", "");
    if (controller != task_.controller) {
      RCLCPP_FATAL(get_logger(), "task '%s' needs controller '%s', launched with '%s'",
                   task_.name.c_str(), task_.controller.c_str(), controller.c_str());
      throw std::invalid_argument("wrong controller for the task");
    }

    // The controller publishes its pose error only while active, so the first message means "ready".
    pose_error_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      declare_parameter<std::string>("controller_state_topic", ""), rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistStamped &) { controller_active_ = true; });
    target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      declare_parameter<std::string>("pose_target_topic", ""), rclcpp::SystemDefaultsQoS());
    gripper_client_ = rclcpp_action::create_client<GripperCommand>(this, declare_parameter<std::string>("gripper_action", ""));
    // Ground truth (privileged, design spec "Simulation"): only success checks may read it.
    ground_truth_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      declare_parameter<std::string>("ground_truth_topic", ""), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState & msg) { on_ground_truth(msg); });
  }

  bool run()
  {
    if (!wait_until([this] { return controller_active_; }, "controller '" + task_.controller + "' to become active") ||
        !wait_until([this] { return target_pub_->get_subscription_count() > 0; }, "the controller's target subscription") ||
        !wait_until([this] { return opening_.has_value(); },
                    "joint '" + task_.joint + "' in ground truth (sim started with objects:=" + task_.scene_objects.front() + "?)") ||
        !gripper_client_->wait_for_action_server(seconds(startup_timeout_s()))) {
      return false;
    }
    const auto deadline = Clock::now() + seconds(task_.time_limit_s);

    if (!close_gripper(deadline)) {
      RCLCPP_ERROR(get_logger(), "closing the gripper failed");
      return false;
    }
    Eigen::Isometry3d approach = task_.hook_pose;
    approach.translation().z() += task_.approach_height;
    if (!move_and_hold(approach, "approach") || !move_and_hold(task_.hook_pose, "hook")) {
      return false;
    }

    // Stream the pull path; the impedance controller follows it as far as the drawer lets it.
    const auto pull_start = Clock::now();
    const double pull_time_s = task_.pull_distance / task_.pull_speed;
    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / target_rate_hz_));
    for (double t = 0.0; t <= pull_time_s; t = std::chrono::duration<double>(Clock::now() - pull_start).count()) {
      send_pose_target(*target_pub_, pull_target(task_.hook_pose, task_.pull_direction, task_.pull_speed, task_.pull_distance, t));
      spin_for(period);
    }
    send_pose_target(*target_pub_, pull_target(task_.hook_pose, task_.pull_direction, task_.pull_speed, task_.pull_distance, pull_time_s));

    const bool opened = held_for([this] { return *opening_ >= task_.min_opening; }, task_.hold_s, deadline);
    if (!opened) {
      RCLCPP_ERROR(get_logger(), "task '%s': drawer opened only %.3f m (needs %.3f m)", task_.name.c_str(), *opening_, task_.min_opening);
      return false;
    }
    RCLCPP_INFO(get_logger(), "task '%s': drawer opened %.3f m (needs %.3f m)", task_.name.c_str(), *opening_, task_.min_opening);
    return true;
  }

private:
  void on_ground_truth(const sensor_msgs::msg::JointState & msg)
  {
    const auto it = std::find(msg.name.begin(), msg.name.end(), task_.joint);
    const auto index = static_cast<std::size_t>(it - msg.name.begin());
    if (it != msg.name.end() && index < msg.position.size()) {
      opening_ = msg.position[index];
    }
  }

  bool close_gripper(Clock::time_point deadline)
  {
    GripperCommand::Goal goal;
    goal.command.position = gripper_closed_position_;
    goal.command.max_effort = gripper_max_effort_;
    auto goal_future = gripper_client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_future, deadline - Clock::now()) !=
        rclcpp::FutureReturnCode::SUCCESS || !goal_future.get()) {
      return false;
    }
    auto result_future = gripper_client_->async_get_result(goal_future.get());
    return rclcpp::spin_until_future_complete(shared_from_this(), result_future, deadline - Clock::now()) ==
             rclcpp::FutureReturnCode::SUCCESS &&
           result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
  }

  /// Sends `target` and waits until the end effector holds it within the approach tolerance.
  bool move_and_hold(const Eigen::Isometry3d & target, const std::string & what)
  {
    send_pose_target(*target_pub_, target);
    PoseError error;
    const bool held = held_for(
      [&] {
        const std::optional<Eigen::Isometry3d> ee = ee_pose();
        if (!ee) {
          return false;
        }
        error = pose_error(target, *ee);
        return within(error, approach_tolerance_);
      },
      approach_hold_s_, Clock::now() + seconds(approach_timeout_s_));
    if (!held) {
      RCLCPP_ERROR(get_logger(), "%s pose not reached (%.4f m, %.4f rad)", what.c_str(), error.position, error.orientation);
      return false;
    }
    RCLCPP_INFO(get_logger(), "%s pose reached (%.4f m, %.4f rad)", what.c_str(), error.position, error.orientation);
    return true;
  }

  DrawerTask task_;
  PoseTolerance approach_tolerance_;
  double approach_hold_s_;
  double approach_timeout_s_;
  double target_rate_hz_;
  double gripper_closed_position_;
  double gripper_max_effort_;

  bool controller_active_ = false;
  std::optional<double> opening_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr pose_error_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ground_truth_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
};

}  // namespace arm_sandbox_tasks

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  bool opened = false;
  {
    const auto runner = std::make_shared<arm_sandbox_tasks::DrawerRunner>();
    opened = runner->run();
  }
  rclcpp::shutdown();
  return opened ? 0 : 1;
}
```

- [ ] **Step 3: Run the tests to see them pass**

Run: `make test`
Expected: 0 failures. `test_drawer` runs once per gravcomp mode, with 2 tests each.

- [ ] **Step 4: Run it by hand, and check the failure message**

```bash
make start_sim ARGS="viewer:=false rerun:=false objects:=drawer arm_controller:=cartesian_impedance_controller"   # terminal 1
source install/setup.bash && ros2 launch arm_sandbox_tasks drawer.launch.py                                      # terminal 2
```

Expected:

```
approach pose reached (0.0077 m, 0.0014 rad)
hook pose reached (0.0039 m, 0.0019 rad)
task 'drawer': drawer opened 0.184 m (needs 0.150 m)
```

Now restart the sim *without* `objects:=drawer` and run terminal 2 again. Expected: after `startup_timeout_s` (60 s), `timed out waiting for joint 'drawer/slide' in ground truth (sim started with objects:=drawer?)` and exit code 1.

- [ ] **Step 5: `make start_drawer`**

In the `Makefile`, add `start_drawer` to `.PHONY` and to the container-targets comment, then append:

```make
# Container: run the drawer task against a running sim started with
# make start_sim ARGS="objects:=drawer arm_controller:=cartesian_impedance_controller"
start_drawer:
	$(ROS_SETUP) && source install/setup.bash && ros2 launch arm_sandbox_tasks drawer.launch.py $(ARGS)
```

Add the matching lines to `CLAUDE.md` ("Inside the dev container": `- \`make start_drawer\` - run the drawer task (M3) against a sim started with objects:=drawer and the impedance controller`) and to the `docs/PROJECT_STRUCTURE.md` Commands table (`| \`make start_drawer\` | container | Run the drawer task against a running sim with the drawer |`).

- [ ] **Step 6: Watch it** [needs a display]

```bash
make start_sim ARGS="objects:=drawer arm_controller:=cartesian_impedance_controller"   # terminal 1
make start_drawer                                                                     # terminal 2
```

Expected: the closed gripper comes down behind the handle and pulls the drawer about 18 cm open. This run is the M3 demo video.

- [ ] **Step 7: Commit**

```bash
git add src/arm_sandbox_tasks Makefile CLAUDE.md docs/PROJECT_STRUCTURE.md
git commit -m "feat(tasks): drawer_runner opens the drawer with the impedance controller; make start_drawer"
```

---

# Part 3 — Docs

### Task 5: Record the decisions, update the docs, add READMEs, log the session

**Files:**
- Modify: `docs/REQUIREMENTS.md` (§10), `docs/specs/2026-10-01-arm-sandbox-design.md` ("Simulation", "Tasks and Executive"), `docs/PROJECT_STRUCTURE.md`, `CLAUDE.md`, `README.md`, `docs/LEARNING_RESOURCES.md`, `docs/MEMORY.md`
- Modify: `src/arm_sandbox_sim/README.md`, `src/arm_sandbox_tasks/README.md`

- [ ] **Step 1: Decisions log.** Append to `docs/REQUIREMENTS.md` §10, and add `, D23–D25 added (Plan 05)` to the status line:

```markdown
| D23 | The drawer is opened by hooking the closed gripper behind the handle (geometric contact), not by grasping | The gripper's position servo gives < 1 N of grip on a thin bar; a hook needs none, and the contact makes compliance matter |
| D24 | Task objects are standalone MJCF files in `arm_sandbox_sim/objects`, attached at launch (`objects:=`) under the prefix `<file stem>/` | Keeps task scenes out of the robot model; MuJoCo 3.12 can't write an unprefixed attachment back to XML, and the prefix namespaces object names |
| D25 | Task success checks may read sim ground truth from `/mujoco_actuators_states` (all MuJoCo joints, e.g. `drawer/slide`) | Ground truth without a new publisher; only success checks use it, never controllers or the approach |
```

- [ ] **Step 2: Design spec.**
  - In "Simulation", replace the "Scene." bullet with: `- **Scene.** \`arm_sandbox_description/<robot>/mjcf/scene.xml\` (floor, lights) plus task objects from \`arm_sandbox_sim/objects/\`, attached at launch by \`compose_scene\` under \`<file stem>/\` (\`sim.launch.py objects:=drawer\`, D24). Gravity compensation (D15) never applies to objects.`
  - In "Tasks and Executive", after the reach bullet, add: `- The drawer task (M3) is \`config/tasks/drawer.yaml\`: scene objects, the controller it needs (impedance), the hook pose, the pull (direction, distance, speed) and \`success: {type: joint_opened, joint: drawer/slide, min_opening, hold_s}\`, read from ground truth (D25). \`drawer_runner\` hooks the handle (D23) and streams the pull; the drawer opens 0.184 m for a 0.20 m pull.`

- [ ] **Step 3: `PROJECT_STRUCTURE.md` and `CLAUDE.md`**
  - `PROJECT_STRUCTURE.md`:
    - status: `M0–M3 done (Plans 01–05)`;
    - `arm_sandbox_sim` row purpose: `Scene composition at launch (gravcomp, task objects in objects/); later reset/randomize services`;
    - `arm_sandbox_tasks` row: `Task configs and runners: reach, drawer; later BehaviorTree.CPP executive and skills`;
    - launch arguments line: add `objects`.
  - `CLAUDE.md` status line: `> **Status:** M0–M3 done (Plans 01–05): … \`arm_sandbox_controllers\` (OSC, impedance), \`arm_sandbox_tasks\` (reach, drawer; \`make start_reach\`, \`make start_drawer\`). Next: M4 (MoveIt 2). …`

- [ ] **Step 4: READMEs.** Append to `src/arm_sandbox_sim/README.md`:

```markdown

## Task objects

Standalone MJCF files in `objects/` (positions in the world frame), added at launch with
`sim.launch.py objects:=drawer`. Each is attached under its file name: the drawer's slide joint is
`drawer/slide`, its handle site `drawer/handle`. Changing objects needs a relaunch; their poses can
change per reset.
```

Append to `src/arm_sandbox_tasks/README.md`:

```markdown

## Drawer (M3)

    make start_sim ARGS="objects:=drawer arm_controller:=cartesian_impedance_controller"   # terminal 1
    make start_drawer                                                                     # terminal 2

The closed gripper goes down behind the handle bar and pulls along the drawer's slide (a hook: the
gripper's position servo is too weak to grip a thin bar). The impedance controller follows the
streamed pull path as far as the drawer lets it, so contact forces stay those of a spring:
F = K e. A stiff controller (OSC) would push as hard as its gains allow wherever the path and the
slide disagree, which is why contact tasks use impedance. Success is read from sim ground truth:
the drawer joint must stay at least 0.15 m open for 0.5 s (it reaches 0.184 m for a 0.20 m pull).
```

- [ ] **Step 5: `docs/LEARNING_RESOURCES.md`.** Append:

```markdown
---

## Plan 05 — The Drawer Task (M3, part 2)

Plan: [plan/2026-10-04-plan-05-drawer-task.md](plan/2026-10-04-plan-05-drawer-task.md)

- **Hogan (1985)** again (Plan 04), and **Siciliano et al., Ch. 9** (force control: compliance and
  impedance in contact with a constrained environment, e.g. a sliding drawer).
- **Villani & De Schutter, "Force Control"**, *Springer Handbook of Robotics*, chapter on force control:
  interaction with constrained environments.
- **MuJoCo docs**: "Model Editing" (`MjSpec`, `attach`), joint `damping`/`frictionloss`, and the contact
  parameters (friction, `solref`/`solimp`) that decide how the hook and the handle interact.
```

- [ ] **Step 6: `README.md` (the repository's front page)**
  - **Status table:** the M3 row becomes `done`. The `M4–M6` row stays `planned`.
  - **Highlights:** add a bullet on the drawer task: a cabinet added at launch, the closed gripper hooked behind the handle, and the impedance controller pulling it 18.4 cm open (success read from sim ground truth).
  - **Quick start:** add the drawer run: `make start_sim ARGS="objects:=drawer arm_controller:=cartesian_impedance_controller"`, then `make start_drawer`.
  - **Repository layout:** the `arm_sandbox_sim` row becomes `Composes the MuJoCo scene at launch: gravity compensation, task objects (objects/)`, and the `arm_sandbox_tasks` row becomes `Task files and runners (reach, drawer)`.
  - **Test count:** update the number in the "Tested headless" bullet to what `colcon test-result` reports (about 108).

- [ ] **Step 7: `docs/MEMORY.md`.** Append a new `## Session — <date>` section at the end (earlier sections stay unchanged), with the commits, what you learned, and the state at the end (M3 done).

- [ ] **Step 8: Check, then commit**

```bash
grep -n "D23\|D24\|D25" docs/REQUIREMENTS.md docs/specs/2026-10-01-arm-sandbox-design.md   # hits in both
make test   # 0 failures
git add CLAUDE.md README.md docs src/arm_sandbox_sim/README.md src/arm_sandbox_tasks/README.md
git commit -m "docs: record D23-D25, document task objects and the drawer task"
```

---

## Done when

- `make test` passes: object composition, the unchanged reach tests after the refactor, the drawer task logic, and the drawer task end to end in both gravcomp modes.
- `make start_sim ARGS="objects:=drawer arm_controller:=cartesian_impedance_controller"` + `make start_drawer` opens the drawer at least 15 cm (**M3** with Plan 04).
- D23–D25 recorded; the docs and READMEs match the code.
- Next: Plan 06, M4 (MoveIt 2: collision-free planning, Servo teleop, pick-and-place with ground-truth poses).
