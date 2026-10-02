# Plan 02 — Sim Bring-up (M1) and Browser Viewer (finish M0)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Drive the torque-actuated Panda in MuJoCo through `ros2_control` (JTC on effort and a gripper action), select the robot with `robot:=panda`, and show the same running sim in the browser (Rerun) and in the native MuJoCo viewer at the same time. This completes M1 and the remaining part of M0.

**Architecture:** `arm_sandbox_description/panda/` holds the robot: a torque-actuated MJCF renamed to URDF names, a new URDF xacro built from the same Menagerie meshes and inertials, and `robot.yaml`. A ROS-free Python module in `arm_sandbox_sim` composes the MuJoCo scene at launch (gravity compensation on or off). `arm_sandbox_bringup/launch/sim.launch.py` starts `mujoco_ros2_control` (D13), `robot_state_publisher`, and the controllers in a fixed order (reset to `home`, then activate the arm controller). It can also start `rerun --serve-web` and the C++ `arm_sandbox_viz` bridge, which logs `/robot_description`, `/tf`, `/tf_static`, and `/joint_states` to Rerun.

**Tech Stack:** ROS 2 Humble, MuJoCo 3.12.0 (`mujoco_vendor` + pip), `mujoco_ros2_control` 0.1.2, `ros2_controllers` 2.54 (JTC, `GripperActionController`), xacro, Pinocchio (test reference), Rerun 0.38.1 (viewer CLI + C++ SDK), C++17 (`rclcpp`), pytest, `launch_testing`.

**Spec:** [`../specs/2026-10-01-arm-sandbox-design.md`](../specs/2026-10-01-arm-sandbox-design.md) (design) and [`../REQUIREMENTS.md`](../REQUIREMENTS.md) (requirements, D1–D14). Read both before starting. Plan 01 ([`2026-10-01-plan-01-foundation.md`](2026-10-01-plan-01-foundation.md)) built the environment this plan uses.

## Global Constraints

- ROS 2 **Humble** (Ubuntu 22.04), D7. MuJoCo **3.12.0** everywhere, D14. Sim backend **`mujoco_ros2_control` 0.1.2**, D13. Rerun **0.38.1** (`RERUN_VERSION` in `scripts/install_deps.sh`).
- **No lockstep:** physics free-runs on sim time; every node uses `use_sim_time: true` (spec, Decisions).
- **Torque-actuated arm:** arm joints use MJCF `motor` actuators and the `effort` command interface. The gripper keeps a `position` actuator (spec, Decisions).
- **Gravity compensation is a launch argument, `gravcomp:=false` by default** (user decision 2026-10-02, recorded as D15 in Task 9). `false`: controllers compensate gravity. `true`: MuJoCo compensates the arm's gravity, like the real Panda's torque interface.
- **The URDF is our own xacro**, with kinematics from `moveit_resources_panda_description` and joint ranges, torque limits, inertials, and meshes from the vendored Menagerie model (user decision 2026-10-02, recorded as D16 in Task 9).
- **One dependency list:** `scripts/install_deps.sh`. The Dockerfile runs it. Never add dependencies anywhere else (Plan 01).
- ROS Python is **`/usr/bin/python3`** (Makefile `PYTHON`). MuJoCo pip bindings live there.
- **Robot-specific values only in** `src/arm_sandbox_description/<robot>/` and `src/arm_sandbox_bringup/config/<robot>_controllers.yaml`. Launch files and code take the robot from `robot:=` and read names from `robot.yaml` (FR-7, FR-8, CLAUDE.md FORBIDDEN list).
- **No magic numbers in code:** gains, tolerances, rates, and URLs go in YAML. Tests may define named constants at the top of the test file.
- C++17, `-Wall -Wextra -Wpedantic -Werror`. No `std::cout` in nodes, no raw `new`/`delete`, fail loudly on bad config (CLAUDE.md).
- **All tests run headless** (`viewer:=false`). Each `launch_testing` test gets its own `ROS_DOMAIN_ID` so parallel packages can't see each other's sim.
- Viewers depend **only on standard ROS 2 topics** (FR-14a).

## Where each step runs

- **[container]**: a terminal inside the `sandbox` dev container (default for every step).
- **[host]**: a terminal on the PC itself, outside the container. Needs Docker and a browser.

Every container step assumes `source /opt/ros/humble/setup.bash`. After `make test` has built the workspace, also `source install/setup.bash`.

## Before Task 1: branch

Plan 01's last commits are on `feat/mujoco` (`574e8bd add arm description`). Branch from wherever those commits are now (merged `main`, or `feat/mujoco` itself):

```bash
git fetch origin
git log --oneline --all | grep "add arm description"   # find the branch that has it
git switch -c feat/m1-sim-viewer <that branch>
```

## File Structure

| Path | Responsibility |
|---|---|
| `src/arm_sandbox_description/panda/mjcf/panda.xml` | Modified Menagerie Panda: URDF names, `motor` arm actuators, tendon `position` gripper, TCP site, `home` keyframe, 1 ms timestep |
| `src/arm_sandbox_description/panda/mjcf/{hand,panda_nohand}.xml` | Removed (unused Menagerie variants with the old names) |
| `src/arm_sandbox_description/panda/config/robot.yaml` | Robot config for generic code: arm joints, base/EE frames, gripper, home pose |
| `src/arm_sandbox_description/panda/urdf/panda.urdf.xacro` | URDF (ROS side), same frames, limits, inertials, and meshes as the MJCF |
| `src/arm_sandbox_description/panda/urdf/panda.ros2_control.xacro` | `<ros2_control>` block: `MujocoSystemInterface`, effort arm, position gripper |
| `src/arm_sandbox_description/test/test_panda_mjcf.py` | MJCF checks (rewritten) |
| `src/arm_sandbox_description/test/test_panda_urdf.py` | URDF↔MJCF agreement (FK vs Pinocchio, limits, mass) and `ros2_control` block |
| `src/arm_sandbox_sim/` | New package. `arm_sandbox_sim/scene.py`: ROS-free scene composition (gravcomp, absolute asset paths) |
| `src/arm_sandbox_bringup/` | New package. `launch/sim.launch.py`, `scripts/activate_at_home` (reset + activate back to back), `config/panda_controllers.yaml`, launch tests |
| `src/arm_sandbox_viz/` | New package. C++ `rerun_bridge` node, `config/rerun_bridge.yaml`, launch test |
| `scripts/install_deps.sh` | Adds the Rerun C++ SDK build (`/opt/rerun_cpp_sdk`) |
| `tests/env/test_toolchain.py` | Adds Rerun C++ SDK smoke tests |
| `Makefile` | Adds `start_sim` |
| `CLAUDE.md`, `docs/…`, package `README.md`s | Updated in Task 9 |

## Out of scope (later plans)

- Cameras (`CameraPlugin`), `robot.yaml` `cameras:` entry: M5.
- `arm_sandbox_sim` reset service with seeded randomization, ground truth, table and task objects, `arm_sandbox_interfaces`: M4.
- MoveIt 2, Servo: M4. Kinematics library: M2. Custom controllers and controller-state plots in Rerun: M3.
- Images, markers, and planned paths in the Rerun bridge: added with the topics that produce them (M4/M5).
- CI workflow and `make lint`.

---

# Part 1 — Robot Model (M1)

### Task 1: Torque-actuated MJCF with URDF names, and `robot.yaml`

The vendored Menagerie Panda uses position-servo `general` actuators and its own names (`joint1`, `link0`, `hand`). `mujoco_ros2_control` matches `ros2_control` joint names to MJCF joint and actuator names, so the MJCF must use the URDF names (`panda_joint1`, …). The arm switches to `motor` (torque) actuators. The gripper becomes a `position` actuator on the finger tendon, and it must be **named after the joint it reports** (`panda_finger_joint1`), because that is how `mujoco_ros2_control` 0.1.2 maps tendon actuators.

**Files:**
- Modify: `src/arm_sandbox_description/panda/mjcf/panda.xml`
- Delete: `src/arm_sandbox_description/panda/mjcf/hand.xml`, `src/arm_sandbox_description/panda/mjcf/panda_nohand.xml`
- Create: `src/arm_sandbox_description/panda/config/robot.yaml`
- Modify: `src/arm_sandbox_description/package.xml`
- Test: `src/arm_sandbox_description/test/test_panda_mjcf.py` (rewrite)

**Interfaces:**
- Consumes: the vendored Menagerie model (Plan 01, Task 8).
- Produces:
  - MJCF joint names `panda_joint1`…`panda_joint7`, `panda_finger_joint1`, `panda_finger_joint2`. Body names `panda_link0`…`panda_link7`, `panda_hand`, `panda_leftfinger`, `panda_rightfinger`. Site `panda_hand_tcp`.
  - Actuators: `motor` named `panda_joint1`…`panda_joint7` (ctrlrange = ±torque limit), `position` named `panda_finger_joint1` on tendon `split` (ctrlrange `0 0.04`, meters per finger).
  - Keyframe `home` equal to `robot.yaml` `home`, fingers open (`0.04`). Timestep `0.001` s.
  - `robot.yaml` keys: `arm_joints` (list[str]), `base_frame` (str), `ee_frame` (str), `gripper.type`, `gripper.joint`, `gripper.max_width` (m, both fingers), `home` (list[float], rad).

- [ ] **Step 1: Write `robot.yaml`**

Create `src/arm_sandbox_description/panda/config/robot.yaml`:

```yaml
# Robot config for generic code (design spec "Robot Description and Config").
# The single source of joint/frame names and the home pose: code never contains these values.
# The MJCF keyframe named `home` must equal `home` below (test/test_panda_mjcf.py checks it).
# Camera names are added with the cameras (M5).
arm_joints: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7]
base_frame: panda_link0
ee_frame: panda_hand_tcp
gripper:
  type: parallel
  joint: panda_finger_joint1
  max_width: 0.08 # m, both fingers fully open (0.04 each)
home: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
```

- [ ] **Step 2: Write the failing tests**

Replace `src/arm_sandbox_description/test/test_panda_mjcf.py` with:

```python
"""The Panda MJCF is torque-actuated, uses the URDF names, and matches robot.yaml."""

from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml

ROBOT_DIR = Path(__file__).resolve().parents[1] / "panda"
MJCF_DIR = ROBOT_DIR / "mjcf"
ROBOT_CONFIG = ROBOT_DIR / "config" / "robot.yaml"

HOME_KEYFRAME = "home"
# One physics step per controller update: controller_manager runs at 1 kHz (Task 4).
EXPECTED_TIMESTEP_S = 0.001
# With zero torque the arm must visibly sag within this time (no hidden position servo).
FALL_TEST_DURATION_S = 0.5
FALL_THRESHOLD_RAD = 0.1


@pytest.fixture(scope="module")
def robot_config() -> dict:
    return yaml.safe_load(ROBOT_CONFIG.read_text())


@pytest.fixture(scope="module")
def model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(MJCF_DIR / "panda.xml"))


def arm_actuator_ids(model: mujoco.MjModel, robot_config: dict) -> list[int]:
    return [model.actuator(joint).id for joint in robot_config["arm_joints"]]


def arm_qpos_addresses(model: mujoco.MjModel, robot_config: dict) -> list[int]:
    return [int(model.joint(joint).qposadr[0]) for joint in robot_config["arm_joints"]]


def test_names_from_robot_config_exist(model: mujoco.MjModel, robot_config: dict) -> None:
    for joint in robot_config["arm_joints"] + [robot_config["gripper"]["joint"]]:
        assert model.joint(joint).id >= 0
    assert model.body(robot_config["base_frame"]).id >= 0
    assert model.site(robot_config["ee_frame"]).id >= 0


def test_timestep_matches_controller_rate(model: mujoco.MjModel) -> None:
    assert model.opt.timestep == pytest.approx(EXPECTED_TIMESTEP_S)


def test_arm_actuators_are_torque_motors(model: mujoco.MjModel, robot_config: dict) -> None:
    for joint in robot_config["arm_joints"]:
        actuator_id = model.actuator(joint).id  # named after its joint, as ros2_control expects
        assert model.actuator_trntype[actuator_id] == mujoco.mjtTrn.mjTRN_JOINT
        assert model.actuator_trnid[actuator_id][0] == model.joint(joint).id
        assert model.actuator_biastype[actuator_id] == mujoco.mjtBias.mjBIAS_NONE  # no servo
        assert model.actuator_gainprm[actuator_id][0] == 1.0
        assert model.actuator_gear[actuator_id][0] == 1.0
        low, high = model.actuator_ctrlrange[actuator_id]
        assert high > 0.0 and low == -high


def test_gripper_is_a_position_servo_on_the_finger_tendon(model: mujoco.MjModel, robot_config: dict) -> None:
    gripper = robot_config["gripper"]
    actuator_id = model.actuator(gripper["joint"]).id
    assert model.actuator_trntype[actuator_id] == mujoco.mjtTrn.mjTRN_TENDON
    assert model.actuator_biastype[actuator_id] == mujoco.mjtBias.mjBIAS_AFFINE
    np.testing.assert_allclose(model.actuator_ctrlrange[actuator_id], [0.0, gripper["max_width"] / 2])


def test_home_keyframe_matches_robot_config(model: mujoco.MjModel, robot_config: dict) -> None:
    key = model.key(HOME_KEYFRAME)
    np.testing.assert_allclose(key.qpos[arm_qpos_addresses(model, robot_config)], robot_config["home"])
    np.testing.assert_allclose(key.ctrl[arm_actuator_ids(model, robot_config)], 0.0)


def test_unactuated_arm_falls_under_gravity(model: mujoco.MjModel, robot_config: dict) -> None:
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key(HOME_KEYFRAME).id)
    data.ctrl[arm_actuator_ids(model, robot_config)] = 0.0
    addresses = arm_qpos_addresses(model, robot_config)
    start = data.qpos[addresses].copy()
    for _ in range(int(FALL_TEST_DURATION_S / model.opt.timestep)):
        mujoco.mj_step(model, data)
    assert np.max(np.abs(data.qpos[addresses] - start)) > FALL_THRESHOLD_RAD


def test_scene_simulates_one_second_without_nan() -> None:
    model = mujoco.MjModel.from_xml_path(str(MJCF_DIR / "scene.xml"))
    data = mujoco.MjData(model)
    for _ in range(int(1.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    assert np.all(np.isfinite(data.qpos))
```

- [ ] **Step 3: Run them to see them fail**

Run: `/usr/bin/python3 -m pytest -q src/arm_sandbox_description/test/test_panda_mjcf.py`
Expected: FAIL. `test_names_from_robot_config_exist` fails with `KeyError: "Invalid name 'panda_joint1'..."`, and most others fail the same way.

- [ ] **Step 4: Rename bodies and joints, set the timestep, add the TCP site**

```bash
F=src/arm_sandbox_description/panda/mjcf/panda.xml
sed -i -E \
  -e 's/<body name="(link[0-7])"/<body name="panda_\1"/' \
  -e 's/<body name="hand"/<body name="panda_hand"/' \
  -e 's/<body name="left_finger"/<body name="panda_leftfinger"/' \
  -e 's/<body name="right_finger"/<body name="panda_rightfinger"/' \
  -e 's/<joint name="(joint[1-7])"/<joint name="panda_\1"/' \
  -e 's/<joint name="(finger_joint[12])"/<joint name="panda_\1"/' \
  -e 's|<option integrator="implicitfast"/>|<option integrator="implicitfast" timestep="0.001"/>|' \
  "$F"
# The arm no longer has `general` actuators, so their class default goes too.
sed -i '/<general dyntype="none" biastype="affine"/d' "$F"
# Tool centre point between the fingertips (robot.yaml ee_frame), same offset as the URDF.
sed -i '/<body name="panda_hand" /a\                      <site name="panda_hand_tcp" pos="0 0 0.1034"/>' "$F"
grep -oE '<(body|joint|site) name="panda_[a-z0-9_]+"' "$F" | cut -d' ' -f1 | sort | uniq -c
# Expected: 11 <body (panda_link0..7, panda_hand, panda_leftfinger, panda_rightfinger),
#           9 <joint (panda_joint1..7, panda_finger_joint1, panda_finger_joint2), 1 <site (panda_hand_tcp)
```

- [ ] **Step 5: Replace the tendon/actuator/keyframe/contact sections**

Everything from the line `  <tendon>` to the end of the file is replaced:

```bash
F=src/arm_sandbox_description/panda/mjcf/panda.xml
sed -i '/^  <tendon>/,$d' "$F"
cat >> "$F" <<'EOF'
  <tendon>
    <!-- Mean opening of both fingers: the controlled gripper coordinate (0 to 0.04 m per finger). -->
    <fixed name="split">
      <joint joint="panda_finger_joint1" coef="0.5"/>
      <joint joint="panda_finger_joint2" coef="0.5"/>
    </fixed>
  </tendon>

  <equality>
    <joint joint1="panda_finger_joint1" joint2="panda_finger_joint2" solimp="0.95 0.99 0.001" solref="0.005 1"/>
  </equality>

  <actuator>
    <!-- Arm: torque (motor) actuators, like the real Panda's torque interface. ctrlrange is the
         joint torque limit in N m. Exposed to ros2_control as the `effort` command interface.
         Each actuator is named after its joint. -->
    <motor name="panda_joint1" joint="panda_joint1" ctrlrange="-87 87"/>
    <motor name="panda_joint2" joint="panda_joint2" ctrlrange="-87 87"/>
    <motor name="panda_joint3" joint="panda_joint3" ctrlrange="-87 87"/>
    <motor name="panda_joint4" joint="panda_joint4" ctrlrange="-87 87"/>
    <motor name="panda_joint5" joint="panda_joint5" ctrlrange="-12 12"/>
    <motor name="panda_joint6" joint="panda_joint6" ctrlrange="-12 12"/>
    <motor name="panda_joint7" joint="panda_joint7" ctrlrange="-12 12"/>
    <!-- Gripper: position servo on the finger tendon (same stiffness and damping as Menagerie's
         actuator8), exposed as the `position` command interface of panda_finger_joint1.
         mujoco_ros2_control requires a tendon actuator to carry the name of the joint it reports. -->
    <position name="panda_finger_joint1" tendon="split" kp="100" kv="10" ctrlrange="0 0.04" forcerange="-100 100"/>
  </actuator>

  <keyframe>
    <!-- qpos must equal `home` in ../config/robot.yaml (test/test_panda_mjcf.py checks it).
         ctrl: zero arm torque, gripper open. -->
    <key name="home" qpos="0 -0.785 0 -2.356 0 1.571 0.785 0.04 0.04" ctrl="0 0 0 0 0 0 0 0.04"/>
  </keyframe>

  <contact>
    <exclude body1="panda_link0" body2="panda_link1"/>
  </contact>
</mujoco>
EOF
```

- [ ] **Step 6: Add the modification notice** (Apache-2.0 §4b requires modified files to say so)

Insert directly after the first line `<mujoco model="panda">`:

```xml
  <!-- Modified for arm-sandbox from MuJoCo Menagerie franka_emika_panda (commit in
       MENAGERIE_COMMIT): bodies and joints renamed to the URDF names (panda_*); position-servo
       `general` arm actuators replaced by torque `motor` actuators; gripper actuator replaced by a
       tendon `position` actuator named panda_finger_joint1; panda_hand_tcp site added; `home`
       keyframe set to robot.yaml's home pose; timestep 0.001 s. hand.xml and panda_nohand.xml
       removed. -->
```

- [ ] **Step 7: Remove the unused variants, add test dependencies**

```bash
git rm -q src/arm_sandbox_description/panda/mjcf/hand.xml src/arm_sandbox_description/panda/mjcf/panda_nohand.xml
```

In `src/arm_sandbox_description/package.xml`, after `<test_depend>ament_cmake_pytest</test_depend>` add:

```xml
  <test_depend>python3-numpy</test_depend>
  <test_depend>python3-yaml</test_depend>
```

- [ ] **Step 8: Run the tests to see them pass**

Run: `/usr/bin/python3 -m pytest -q src/arm_sandbox_description/test/test_panda_mjcf.py`
Expected: 7 passed.

Run: `python3 -m mujoco.viewer --mjcf=src/arm_sandbox_description/panda/mjcf/scene.xml` [container, needs a display]
Expected: the Panda starts in the home pose and slowly collapses (zero torque). The TCP site shows as a small sphere between the fingertips. Close the window.

- [ ] **Step 9: Commit**

```bash
git add src/arm_sandbox_description
git commit -m "feat(description): torque-actuated Panda MJCF with URDF names, robot.yaml"
```

### Task 2: Panda URDF xacro that agrees with the MJCF

The upstream URDF (`moveit_resources_panda_description`) has `.dae` visual meshes and no `panda_hand_tcp`. This xacro keeps its kinematics (joint origins and axes) and takes everything else from the vendored Menagerie model, so the ROS side and MuJoCo describe the same robot. A test checks this with Pinocchio, which is the main risk listed in the spec ("URDF and MJCF disagree").

Two frame details, both matching the MJCF:
- `panda_hand` is rotated −π/4 about z from `panda_link8`.
- `panda_rightfinger` is rotated π about z, with joint axis `0 1 0`. Both fingers then use the same mesh and axis. (Upstream uses no rotation and axis `0 -1 0`. The finger positions are identical, only the link frame differs.)

**Files:**
- Create: `src/arm_sandbox_description/panda/urdf/panda.urdf.xacro`
- Modify: `src/arm_sandbox_description/CMakeLists.txt`, `src/arm_sandbox_description/package.xml`
- Test: `src/arm_sandbox_description/test/test_panda_urdf.py`

**Interfaces:**
- Consumes: Task 1 MJCF names and `robot.yaml`.
- Produces: `panda/urdf/panda.urdf.xacro` (robot name `panda`), links `panda_link0`…`panda_link8`, `panda_hand`, `panda_hand_tcp`, `panda_leftfinger`, `panda_rightfinger`. Joints as in the MJCF plus fixed `panda_joint8`, `panda_hand_joint`, `panda_hand_tcp_joint`. `panda_finger_joint2` mimics `panda_finger_joint1`. Meshes at `package://arm_sandbox_description/panda/mjcf/assets/...`.

- [ ] **Step 1: Write the failing tests**

Create `src/arm_sandbox_description/test/test_panda_urdf.py`:

```python
"""The URDF (ROS side) and the MJCF (MuJoCo side) describe the same robot.

Kinematics are compared with Pinocchio (URDF) against MuJoCo (MJCF) on random configurations.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pinocchio as pin
import pytest
import xacro
import yaml

ROBOT_DIR = Path(__file__).resolve().parents[1] / "panda"
URDF_XACRO = ROBOT_DIR / "urdf" / "panda.urdf.xacro"
MJCF = ROBOT_DIR / "mjcf" / "panda.xml"
ROBOT_CONFIG = ROBOT_DIR / "config" / "robot.yaml"

# Links that are bodies in both models (panda_link8 is a URDF-only flange frame).
SHARED_LINKS = [f"panda_link{i}" for i in range(8)] + ["panda_hand", "panda_leftfinger", "panda_rightfinger"]
FINGER_JOINTS = ["panda_finger_joint1", "panda_finger_joint2"]
NUM_RANDOM_CONFIGS = 50
RANDOM_SEED = 0
# The MJCF stores the hand rotation with 7 significant digits, hence 1e-6 rather than 1e-9.
POSE_TOLERANCE = 1e-6
LIMIT_TOLERANCE = 1e-9


@pytest.fixture(scope="module")
def robot_config() -> dict:
    return yaml.safe_load(ROBOT_CONFIG.read_text())


@pytest.fixture(scope="module")
def urdf_xml() -> str:
    return xacro.process_file(str(URDF_XACRO)).toxml()


@pytest.fixture(scope="module")
def urdf_joints(urdf_xml: str) -> dict[str, ET.Element]:
    # Top-level <joint> only: the <ros2_control> block (Task 4) has <joint> elements of its own.
    return {joint.get("name"): joint for joint in ET.fromstring(urdf_xml).findall("joint")}


@pytest.fixture(scope="module")
def mj_model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(MJCF))


def random_configuration(mj_model: mujoco.MjModel, robot_config: dict, rng: np.random.Generator) -> dict[str, float]:
    q = {joint: rng.uniform(*mj_model.joint(joint).range) for joint in robot_config["arm_joints"]}
    finger = rng.uniform(*mj_model.joint(FINGER_JOINTS[0]).range)
    q.update({joint: finger for joint in FINGER_JOINTS})  # mimic: both fingers move together
    return q


def test_robot_config_names_exist_in_urdf(urdf_xml: str, urdf_joints: dict, robot_config: dict) -> None:
    links = {link.get("name") for link in ET.fromstring(urdf_xml).iter("link")}
    assert {robot_config["base_frame"], robot_config["ee_frame"]} <= links
    for joint in robot_config["arm_joints"]:
        assert urdf_joints[joint].get("type") == "revolute"
    assert urdf_joints[robot_config["gripper"]["joint"]].get("type") == "prismatic"


def test_forward_kinematics_match(urdf_xml: str, mj_model: mujoco.MjModel, robot_config: dict) -> None:
    pin_model = pin.buildModelFromXML(urdf_xml)
    pin_data = pin_model.createData()
    mj_data = mujoco.MjData(mj_model)
    rng = np.random.default_rng(RANDOM_SEED)

    for _ in range(NUM_RANDOM_CONFIGS):
        q = random_configuration(mj_model, robot_config, rng)
        pin_q = pin.neutral(pin_model)
        for joint, value in q.items():
            pin_q[pin_model.joints[pin_model.getJointId(joint)].idx_q] = value
            mj_data.qpos[mj_model.joint(joint).qposadr[0]] = value
        pin.framesForwardKinematics(pin_model, pin_data, pin_q)
        mujoco.mj_kinematics(mj_model, mj_data)

        for link in SHARED_LINKS:
            expected = pin_data.oMf[pin_model.getFrameId(link)]
            np.testing.assert_allclose(mj_data.body(link).xpos, expected.translation, atol=POSE_TOLERANCE, err_msg=link)
            np.testing.assert_allclose(
                mj_data.body(link).xmat.reshape(3, 3), expected.rotation, atol=POSE_TOLERANCE, err_msg=link
            )
        ee_frame = robot_config["ee_frame"]
        expected = pin_data.oMf[pin_model.getFrameId(ee_frame)]
        np.testing.assert_allclose(mj_data.site(ee_frame).xpos, expected.translation, atol=POSE_TOLERANCE)
        np.testing.assert_allclose(mj_data.site(ee_frame).xmat.reshape(3, 3), expected.rotation, atol=POSE_TOLERANCE)


def test_joint_ranges_match(urdf_joints: dict, mj_model: mujoco.MjModel, robot_config: dict) -> None:
    for joint in robot_config["arm_joints"] + FINGER_JOINTS:
        limit = urdf_joints[joint].find("limit")
        urdf_range = [float(limit.get("lower")), float(limit.get("upper"))]
        np.testing.assert_allclose(urdf_range, mj_model.joint(joint).range, atol=LIMIT_TOLERANCE, err_msg=joint)


def test_torque_limits_match(urdf_joints: dict, mj_model: mujoco.MjModel, robot_config: dict) -> None:
    for joint in robot_config["arm_joints"]:
        urdf_effort = float(urdf_joints[joint].find("limit").get("effort"))
        assert urdf_effort == pytest.approx(mj_model.actuator(joint).ctrlrange[1]), joint


def test_total_mass_matches(urdf_xml: str, mj_model: mujoco.MjModel) -> None:
    urdf_mass = sum(float(mass.get("value")) for mass in ET.fromstring(urdf_xml).iter("mass"))
    assert urdf_mass == pytest.approx(mujoco.mj_getTotalmass(mj_model))
```

- [ ] **Step 2: Run them to see them fail**

Run: `/usr/bin/python3 -m pytest -q src/arm_sandbox_description/test/test_panda_urdf.py`
Expected: FAIL, every test errors with `XacroException: No such file or directory: .../panda.urdf.xacro`.

- [ ] **Step 3: Write the xacro**

Create `src/arm_sandbox_description/panda/urdf/panda.urdf.xacro`:

```xml
<?xml version="1.0"?>
<!--
  Franka Panda for arm-sandbox (ROS side), selected by robot:=panda.
  - Joint origins and axes follow the upstream Panda URDF (moveit_resources_panda_description).
  - Joint ranges, torque limits, inertials and meshes come from the vendored MuJoCo Menagerie model
    in ../mjcf/, so the URDF and the MJCF describe the same robot (test/test_panda_urdf.py).
  - Link frames equal the MJCF body frames. panda_rightfinger is rotated by pi about z, as in the
    MJCF, so both fingers share one mesh and one joint axis.
  - panda_hand_tcp is the tool centre point between the fingertips (robot.yaml ee_frame).
-->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="panda">
  <xacro:property name="mesh_dir" value="package://arm_sandbox_description/panda/mjcf/assets"/>
  <!-- MJCF material colors. Set inline per visual: not every URDF consumer resolves named materials. -->
  <xacro:property name="colors" value="${dict(white='1 1 1 1', off_white='0.901961 0.921569 0.929412 1', black='0.25 0.25 0.25 1', green='0 1 0 1', light_blue='0.039216 0.541176 0.780392 1')}"/>

  <!-- One visual per MJCF visual geom of the body. -->
  <xacro:macro name="visual_part" params="mesh color">
    <visual>
      <geometry>
        <mesh filename="${mesh_dir}/${mesh}.obj"/>
      </geometry>
      <material name="${color}">
        <color rgba="${colors[color]}"/>
      </material>
    </visual>
  </xacro:macro>

  <xacro:macro name="collision_mesh" params="file">
    <collision>
      <geometry>
        <mesh filename="${mesh_dir}/${file}"/>
      </geometry>
    </collision>
  </xacro:macro>

  <!-- Same values as the MJCF <inertial> (fullinertia order: ixx iyy izz ixy ixz iyz). -->
  <xacro:macro name="link_inertial" params="mass xyz ixx iyy izz ixy ixz iyz">
    <inertial>
      <origin xyz="${xyz}" rpy="0 0 0"/>
      <mass value="${mass}"/>
      <inertia ixx="${ixx}" iyy="${iyy}" izz="${izz}" ixy="${ixy}" ixz="${ixz}" iyz="${iyz}"/>
    </inertial>
  </xacro:macro>

  <!-- Limits: MJCF joint ranges and actuator torque limits; velocity from the Franka datasheet. -->
  <xacro:macro name="arm_joint" params="index xyz rpy lower upper effort velocity">
    <joint name="panda_joint${index}" type="revolute">
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <parent link="panda_link${index - 1}"/>
      <child link="panda_link${index}"/>
      <axis xyz="0 0 1"/>
      <limit lower="${lower}" upper="${upper}" effort="${effort}" velocity="${velocity}"/>
    </joint>
  </xacro:macro>

  <link name="panda_link0">
    <xacro:link_inertial mass="0.629769" xyz="-0.041018 -0.00014 0.049974"
      ixx="0.00315" iyy="0.00388" izz="0.004285" ixy="8.2904e-7" ixz="0.00015" iyz="8.2299e-6"/>
    <xacro:visual_part mesh="link0_0" color="off_white"/>
    <xacro:visual_part mesh="link0_1" color="black"/>
    <xacro:visual_part mesh="link0_2" color="off_white"/>
    <xacro:visual_part mesh="link0_3" color="black"/>
    <xacro:visual_part mesh="link0_4" color="off_white"/>
    <xacro:visual_part mesh="link0_5" color="black"/>
    <xacro:visual_part mesh="link0_7" color="white"/>
    <xacro:visual_part mesh="link0_8" color="white"/>
    <xacro:visual_part mesh="link0_9" color="black"/>
    <xacro:visual_part mesh="link0_10" color="off_white"/>
    <xacro:visual_part mesh="link0_11" color="white"/>
    <xacro:collision_mesh file="link0.stl"/>
  </link>

  <xacro:arm_joint index="1" xyz="0 0 0.333" rpy="0 0 0" lower="-2.8973" upper="2.8973" effort="87" velocity="2.1750"/>
  <link name="panda_link1">
    <xacro:link_inertial mass="4.970684" xyz="0.003875 0.002081 -0.04762"
      ixx="0.70337" iyy="0.70661" izz="0.0091170" ixy="-0.00013900" ixz="0.0067720" iyz="0.019169"/>
    <xacro:visual_part mesh="link1" color="white"/>
    <xacro:collision_mesh file="link1.stl"/>
  </link>

  <xacro:arm_joint index="2" xyz="0 0 0" rpy="${-pi/2} 0 0" lower="-1.7628" upper="1.7628" effort="87" velocity="2.1750"/>
  <link name="panda_link2">
    <xacro:link_inertial mass="0.646926" xyz="-0.003141 -0.02872 0.003495"
      ixx="0.0079620" iyy="2.8110e-2" izz="2.5995e-2" ixy="-3.925e-3" ixz="1.0254e-2" iyz="7.04e-4"/>
    <xacro:visual_part mesh="link2" color="white"/>
    <xacro:collision_mesh file="link2.stl"/>
  </link>

  <xacro:arm_joint index="3" xyz="0 -0.316 0" rpy="${pi/2} 0 0" lower="-2.8973" upper="2.8973" effort="87" velocity="2.1750"/>
  <link name="panda_link3">
    <xacro:link_inertial mass="3.228604" xyz="2.7518e-2 3.9252e-2 -6.6502e-2"
      ixx="3.7242e-2" iyy="3.6155e-2" izz="1.083e-2" ixy="-4.761e-3" ixz="-1.1396e-2" iyz="-1.2805e-2"/>
    <xacro:visual_part mesh="link3_0" color="white"/>
    <xacro:visual_part mesh="link3_1" color="white"/>
    <xacro:visual_part mesh="link3_2" color="white"/>
    <xacro:visual_part mesh="link3_3" color="black"/>
    <xacro:collision_mesh file="link3.stl"/>
  </link>

  <xacro:arm_joint index="4" xyz="0.0825 0 0" rpy="${pi/2} 0 0" lower="-3.0718" upper="-0.0698" effort="87" velocity="2.1750"/>
  <link name="panda_link4">
    <xacro:link_inertial mass="3.587895" xyz="-5.317e-2 1.04419e-1 2.7454e-2"
      ixx="2.5853e-2" iyy="1.9552e-2" izz="2.8323e-2" ixy="7.796e-3" ixz="-1.332e-3" iyz="8.641e-3"/>
    <xacro:visual_part mesh="link4_0" color="white"/>
    <xacro:visual_part mesh="link4_1" color="white"/>
    <xacro:visual_part mesh="link4_2" color="black"/>
    <xacro:visual_part mesh="link4_3" color="white"/>
    <xacro:collision_mesh file="link4.stl"/>
  </link>

  <xacro:arm_joint index="5" xyz="-0.0825 0.384 0" rpy="${-pi/2} 0 0" lower="-2.8973" upper="2.8973" effort="12" velocity="2.6100"/>
  <link name="panda_link5">
    <xacro:link_inertial mass="1.225946" xyz="-1.1953e-2 4.1065e-2 -3.8437e-2"
      ixx="3.5549e-2" iyy="2.9474e-2" izz="8.627e-3" ixy="-2.117e-3" ixz="-4.037e-3" iyz="2.29e-4"/>
    <xacro:visual_part mesh="link5_0" color="black"/>
    <xacro:visual_part mesh="link5_1" color="white"/>
    <xacro:visual_part mesh="link5_2" color="white"/>
    <xacro:collision_mesh file="link5_collision_0.obj"/>
    <xacro:collision_mesh file="link5_collision_1.obj"/>
    <xacro:collision_mesh file="link5_collision_2.obj"/>
  </link>

  <xacro:arm_joint index="6" xyz="0 0 0" rpy="${pi/2} 0 0" lower="-0.0175" upper="3.7525" effort="12" velocity="2.6100"/>
  <link name="panda_link6">
    <xacro:link_inertial mass="1.666555" xyz="6.0149e-2 -1.4117e-2 -1.0517e-2"
      ixx="1.964e-3" iyy="4.354e-3" izz="5.433e-3" ixy="1.09e-4" ixz="-1.158e-3" iyz="3.41e-4"/>
    <xacro:visual_part mesh="link6_0" color="off_white"/>
    <xacro:visual_part mesh="link6_1" color="white"/>
    <xacro:visual_part mesh="link6_2" color="black"/>
    <xacro:visual_part mesh="link6_3" color="white"/>
    <xacro:visual_part mesh="link6_4" color="white"/>
    <xacro:visual_part mesh="link6_5" color="white"/>
    <xacro:visual_part mesh="link6_6" color="white"/>
    <xacro:visual_part mesh="link6_7" color="light_blue"/>
    <xacro:visual_part mesh="link6_8" color="light_blue"/>
    <xacro:visual_part mesh="link6_9" color="black"/>
    <xacro:visual_part mesh="link6_10" color="black"/>
    <xacro:visual_part mesh="link6_11" color="white"/>
    <xacro:visual_part mesh="link6_12" color="green"/>
    <xacro:visual_part mesh="link6_13" color="white"/>
    <xacro:visual_part mesh="link6_14" color="black"/>
    <xacro:visual_part mesh="link6_15" color="black"/>
    <xacro:visual_part mesh="link6_16" color="white"/>
    <xacro:collision_mesh file="link6.stl"/>
  </link>

  <xacro:arm_joint index="7" xyz="0.088 0 0" rpy="${pi/2} 0 0" lower="-2.8973" upper="2.8973" effort="12" velocity="2.6100"/>
  <link name="panda_link7">
    <xacro:link_inertial mass="7.35522e-01" xyz="1.0517e-2 -4.252e-3 6.1597e-2"
      ixx="1.2516e-2" iyy="1.0027e-2" izz="4.815e-3" ixy="-4.28e-4" ixz="-1.196e-3" iyz="-7.41e-4"/>
    <xacro:visual_part mesh="link7_0" color="white"/>
    <xacro:visual_part mesh="link7_1" color="black"/>
    <xacro:visual_part mesh="link7_2" color="black"/>
    <xacro:visual_part mesh="link7_3" color="black"/>
    <xacro:visual_part mesh="link7_4" color="black"/>
    <xacro:visual_part mesh="link7_5" color="black"/>
    <xacro:visual_part mesh="link7_6" color="black"/>
    <xacro:visual_part mesh="link7_7" color="white"/>
    <xacro:collision_mesh file="link7.stl"/>
  </link>

  <!-- Flange (no body in the MJCF; the hand body there sits directly at this offset). -->
  <joint name="panda_joint8" type="fixed">
    <origin xyz="0 0 0.107" rpy="0 0 0"/>
    <parent link="panda_link7"/>
    <child link="panda_link8"/>
  </joint>
  <link name="panda_link8"/>

  <joint name="panda_hand_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 ${-pi/4}"/>
    <parent link="panda_link8"/>
    <child link="panda_hand"/>
  </joint>
  <link name="panda_hand">
    <xacro:link_inertial mass="0.73" xyz="-0.01 0 0.03" ixx="0.001" iyy="0.0025" izz="0.0017" ixy="0" ixz="0" iyz="0"/>
    <xacro:visual_part mesh="hand_0" color="off_white"/>
    <xacro:visual_part mesh="hand_1" color="black"/>
    <xacro:visual_part mesh="hand_2" color="black"/>
    <xacro:visual_part mesh="hand_3" color="white"/>
    <xacro:visual_part mesh="hand_4" color="off_white"/>
    <xacro:collision_mesh file="hand.stl"/>
  </link>

  <joint name="panda_hand_tcp_joint" type="fixed">
    <origin xyz="0 0 0.1034" rpy="0 0 0"/>
    <parent link="panda_hand"/>
    <child link="panda_hand_tcp"/>
  </joint>
  <link name="panda_hand_tcp"/>

  <!-- Both fingers are identical links; the right one is turned by its joint origin below. -->
  <xacro:macro name="finger" params="side">
    <link name="panda_${side}finger">
      <xacro:link_inertial mass="0.015" xyz="0 0 0" ixx="2.375e-6" iyy="2.375e-6" izz="7.5e-7" ixy="0" ixz="0" iyz="0"/>
      <xacro:visual_part mesh="finger_0" color="off_white"/>
      <xacro:visual_part mesh="finger_1" color="black"/>
      <xacro:collision_mesh file="finger_0.obj"/>
    </link>
  </xacro:macro>
  <xacro:finger side="left"/>
  <xacro:finger side="right"/>

  <joint name="panda_finger_joint1" type="prismatic">
    <origin xyz="0 0 0.0584" rpy="0 0 0"/>
    <parent link="panda_hand"/>
    <child link="panda_leftfinger"/>
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="0.04" effort="20" velocity="0.2"/>
  </joint>
  <joint name="panda_finger_joint2" type="prismatic">
    <origin xyz="0 0 0.0584" rpy="0 0 ${pi}"/>
    <parent link="panda_hand"/>
    <child link="panda_rightfinger"/>
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="0.04" effort="20" velocity="0.2"/>
    <mimic joint="panda_finger_joint1"/>
  </joint>
</robot>
```

- [ ] **Step 4: Register the test and its dependencies**

`src/arm_sandbox_description/CMakeLists.txt`, inside `if(BUILD_TESTING)`, after the existing line:

```cmake
  ament_add_pytest_test(test_panda_urdf test/test_panda_urdf.py)
```

`src/arm_sandbox_description/package.xml`, after the test dependencies added in Task 1:

```xml
  <test_depend>pinocchio</test_depend>
  <test_depend>xacro</test_depend>
```

- [ ] **Step 5: Run the tests to see them pass**

Run: `/usr/bin/python3 -m pytest -q src/arm_sandbox_description/test/test_panda_urdf.py`
Expected: 5 passed.

If `test_forward_kinematics_match` fails, the error names the link. Compare that joint's `<origin>` with the MJCF body's `pos`/`quat`. Don't loosen `POSE_TOLERANCE`.

Then: `make test`
Expected: 0 errors, 0 failures (`test_panda_mjcf`, `test_panda_urdf`).

- [ ] **Step 6: Look at it in RViz2** [container, needs a display]

```bash
source install/setup.bash
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(xacro src/arm_sandbox_description/panda/urdf/panda.urdf.xacro)" &
ros2 run joint_state_publisher_gui joint_state_publisher_gui &   # skip if not installed
rviz2   # Fixed Frame: panda_link0; Add → RobotModel (topic /robot_description)
```

Expected: the colored Panda with fingers on both sides. Kill the processes afterwards (`kill %1 %2`). If `joint_state_publisher_gui` isn't installed, RViz shows only the fixed links. That's fine, because the FK test already covers the moving joints.

- [ ] **Step 7: Commit**

```bash
git add src/arm_sandbox_description
git commit -m "feat(description): Panda URDF xacro from Menagerie meshes, checked against the MJCF"
```

---

# Part 2 — Sim behind `ros2_control` (M1)

### Task 3: Scene composition with switchable gravity compensation (`arm_sandbox_sim`)

`mujoco_ros2_control` loads the MJCF file given in the URDF's `mujoco_model` parameter. The launch file composes that file: it optionally sets `gravcomp="1"` on the robot's bodies (D15), and it makes mesh paths absolute so the file can live in a temp directory. The code is ROS-free, so the Phase B Gymnasium environment can load exactly the same model later. MuJoCo's `MjSpec` is the official model-editing API (no XML string surgery).

Gravity compensation applies to the **robot subtree only** (`robot.yaml` `base_frame` and its descendants). Task objects added in M4 must keep their weight.

**Files:**
- Create: `src/arm_sandbox_sim/package.xml`, `src/arm_sandbox_sim/CMakeLists.txt`
- Create: `src/arm_sandbox_sim/arm_sandbox_sim/__init__.py` (empty), `src/arm_sandbox_sim/arm_sandbox_sim/scene.py`
- Test: `src/arm_sandbox_sim/test/test_scene.py`

**Interfaces:**
- Consumes: `panda/mjcf/scene.xml` and `robot.yaml` (Task 1), installed under `share/arm_sandbox_description/panda/`.
- Produces (Python, importable as `arm_sandbox_sim.scene`):
  - `compose_scene(scene_path: Path, *, robot_root: str, gravcomp: bool) -> str`: MJCF XML string.
  - `write_composed_scene(scene_path: Path, *, robot_root: str, gravcomp: bool, output_dir: Path | None = None) -> Path`: writes `composed_scene.xml` (to a new temp dir when `output_dir` is `None`) and returns its path.
  - Raises `FileNotFoundError` (missing scene) and `ValueError` (unknown `robot_root`).

- [ ] **Step 1: Create the package skeleton**

`src/arm_sandbox_sim/package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>arm_sandbox_sim</name>
  <version>0.1.0</version>
  <description>MuJoCo scenes for arm-sandbox: composition at launch (later: seeded reset, ground truth).</description>
  <maintainer email="robertocw_18@hotmail.com">Roberto Cai</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <!-- MuJoCo Python bindings come from pip (pinned by scripts/install_deps.sh, D14): no rosdep key. -->
  <test_depend>ament_cmake_pytest</test_depend>
  <test_depend>arm_sandbox_description</test_depend>
  <test_depend>python3-numpy</test_depend>
  <test_depend>python3-yaml</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

`src/arm_sandbox_sim/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(arm_sandbox_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ROS-free Python module (scene composition), shared by the launch files and, later, the Gym env.
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_scene test/test_scene.py)
endif()

ament_package()
```

```bash
mkdir -p src/arm_sandbox_sim/arm_sandbox_sim src/arm_sandbox_sim/test
touch src/arm_sandbox_sim/arm_sandbox_sim/__init__.py
```

- [ ] **Step 2: Write the failing tests**

Create `src/arm_sandbox_sim/test/test_scene.py`:

```python
"""Scene composition: loads from anywhere, and gravcomp switches the arm's gravity compensation."""

from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from arm_sandbox_sim.scene import compose_scene, write_composed_scene

ROBOT_DIR = Path(get_package_share_directory("arm_sandbox_description")) / "panda"
SCENE = ROBOT_DIR / "mjcf" / "scene.xml"
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
```

- [ ] **Step 3: Run them to see them fail**

Run: `make test` (builds the new package so `ament_index` and `arm_sandbox_sim` resolve), or after one build: `source install/setup.bash && /usr/bin/python3 -m pytest -q src/arm_sandbox_sim/test`
Expected: FAIL with `ModuleNotFoundError: No module named 'arm_sandbox_sim.scene'`.

- [ ] **Step 4: Implement `scene.py`**

Create `src/arm_sandbox_sim/arm_sandbox_sim/scene.py`:

```python
"""Compose the MuJoCo scene that the sim loads (design spec "Simulation").

ROS-free on purpose: the launch files use it now, and the Phase B Gymnasium environment will load
exactly the same model through it.
"""

import tempfile
from pathlib import Path

import mujoco

COMPOSED_FILE_NAME = "composed_scene.xml"


def compose_scene(scene_path: Path, *, robot_root: str, gravcomp: bool) -> str:
    """Return the scene as one MJCF string that loads from any directory.

    Args:
        scene_path: The robot's scene file (e.g. share/arm_sandbox_description/panda/mjcf/scene.xml).
        robot_root: Name of the robot's base body (robot.yaml `base_frame`).
        gravcomp: If True, MuJoCo cancels gravity on the robot's bodies, as the real Panda's torque
            interface does. If False (the default in the launch files, D15), controllers must
            compensate gravity themselves. Bodies outside the robot are never compensated.
    """
    if not scene_path.is_file():
        raise FileNotFoundError(f"MuJoCo scene not found: {scene_path}")

    spec = mujoco.MjSpec.from_file(str(scene_path))
    # Relative mesh and texture paths resolve against the source file's folder. Make them absolute
    # so the composed file can be written anywhere.
    model_dir = scene_path.resolve().parent
    spec.meshdir = str(model_dir / spec.meshdir)
    spec.texturedir = str(model_dir / spec.texturedir)

    root = spec.body(robot_root)
    if root is None:
        raise ValueError(f"robot root body '{robot_root}' not found in {scene_path}")
    if gravcomp:
        for body in [root, *root.find_all(mujoco.mjtObj.mjOBJ_BODY)]:
            body.gravcomp = 1.0

    return spec.to_xml()


def write_composed_scene(
    scene_path: Path, *, robot_root: str, gravcomp: bool, output_dir: Path | None = None
) -> Path:
    """Write compose_scene()'s result to `output_dir` (a new temp dir if None) and return its path."""
    xml = compose_scene(scene_path, robot_root=robot_root, gravcomp=gravcomp)
    directory = output_dir if output_dir is not None else Path(tempfile.mkdtemp(prefix="arm_sandbox_scene_"))
    path = directory / COMPOSED_FILE_NAME
    path.write_text(xml)
    return path
```

- [ ] **Step 5: Run the tests to see them pass**

Run: `make test`
Expected: 0 failures. `test_scene` shows 6 passed in `colcon test-result --verbose`.

- [ ] **Step 6: Commit**

```bash
git add src/arm_sandbox_sim
git commit -m "feat(sim): compose the MuJoCo scene at launch with switchable gravity compensation"
```

### Task 4: Bring-up — `ros2_control` block, controllers, `sim.launch.py`

This task starts the sim behind `ros2_control` and activates the controllers in a safe order. Without gravity compensation, the arm sags as soon as physics starts, because zero torque is commanded until a controller is active. JTC holds whatever pose it sees when it activates. So the launch:

1. spawns `joint_state_broadcaster` (active) and, in parallel, `arm_controller` + `gripper_controller` with `--inactive` (loaded and configured),
2. then runs `activate_at_home`, a small node that calls `/mujoco_ros2_control_node/reset_world` (`keyframe: home`) and `/controller_manager/switch_controller` back to back, so the controllers latch the home pose.

Why a node and not "reset, then spawner": this was tried while writing the plan. Starting a spawner process takes about 2 s of wall time. The arm falls during that time, and JTC then holds it at the joint limits. Pausing the sim doesn't help either: `mujoco_ros2_control`'s control loop sleeps on sim time, so a paused sim also blocks the controller switch. The back-to-back calls leave a few milliseconds, and the arm holds home within about 1e-4 rad.

`mujoco_ros2_control` 0.1.2 serves its services from the node `mujoco_ros2_control_node`. Its own docs say `/ros2_control_node/...`, which is wrong for this version (the launch log prints `Created reset_world service at: /mujoco_ros2_control_node/reset_world`).

This plan's code was executed end to end in a scratch workspace on 2026-10-02 (`colcon test`: 38 tests, 0 failures, including every launch test below). The starting gains needed no tuning.

**Files:**
- Create: `src/arm_sandbox_description/panda/urdf/panda.ros2_control.xacro`
- Modify: `src/arm_sandbox_description/panda/urdf/panda.urdf.xacro` (args + include)
- Modify: `src/arm_sandbox_description/test/test_panda_urdf.py` (one test)
- Create: `src/arm_sandbox_bringup/package.xml`, `src/arm_sandbox_bringup/CMakeLists.txt`
- Create: `src/arm_sandbox_bringup/config/panda_controllers.yaml`
- Create: `src/arm_sandbox_bringup/scripts/activate_at_home`
- Create: `src/arm_sandbox_bringup/launch/sim.launch.py`
- Test: `src/arm_sandbox_bringup/test/test_sim_bringup.py`

**Interfaces:**
- Consumes: Task 2 xacro, Task 3 `write_composed_scene`, `robot.yaml`.
- Produces:
  - xacro args `mujoco_model` (path, default `""`) and `headless` (`true|false`, default `false`).
  - `ros2 launch arm_sandbox_bringup sim.launch.py robot:=panda viewer:=true|false gravcomp:=false|true`.
  - Executable `arm_sandbox_bringup/activate_at_home`. Parameters: `reset_service`, `keyframe`, `switch_service` (str), `controllers` (list[str]), `timeout_s` (float > 0). Exit code 0 on success, 1 on failure.
  - Controllers: `joint_state_broadcaster` (100 Hz), `arm_controller` (`joint_trajectory_controller/JointTrajectoryController`, effort), `gripper_controller` (`position_controllers/GripperActionController`).
  - Topics/actions: `/joint_states`, `/tf`, `/robot_description`, `/clock`, `/arm_controller/follow_joint_trajectory`, `/gripper_controller/gripper_cmd`.

- [ ] **Step 1: Write the failing description test**

Append to `src/arm_sandbox_description/test/test_panda_urdf.py`:

```python
def test_ros2_control_block_drives_mujoco(robot_config: dict) -> None:
    urdf = xacro.process_file(
        str(URDF_XACRO), mappings={"mujoco_model": "/tmp/composed_scene.xml", "headless": "true"}
    ).toxml()
    block = ET.fromstring(urdf).find("ros2_control")
    assert block is not None
    hardware = block.find("hardware")
    assert hardware.findtext("plugin") == "mujoco_ros2_control/MujocoSystemInterface"
    params = {param.get("name"): param.text for param in hardware.findall("param")}
    assert params == {"mujoco_model": "/tmp/composed_scene.xml", "headless": "true", "initial_keyframe": "home"}

    commands = {
        joint.get("name"): [interface.get("name") for interface in joint.findall("command_interface")]
        for joint in block.findall("joint")
    }
    expected = {joint: ["effort"] for joint in robot_config["arm_joints"]}
    expected[robot_config["gripper"]["joint"]] = ["position"]
    assert commands == expected
```

Run: `/usr/bin/python3 -m pytest -q src/arm_sandbox_description/test/test_panda_urdf.py::test_ros2_control_block_drives_mujoco`
Expected: FAIL. `assert block is not None` fails, or xacro rejects the unknown `mujoco_model` mapping.

- [ ] **Step 2: Write the `ros2_control` xacro**

Create `src/arm_sandbox_description/panda/urdf/panda.ros2_control.xacro`:

```xml
<?xml version="1.0"?>
<!--
  ros2_control interfaces of the Panda (design spec "Simulation" → Command interfaces).
  Arm: `effort` commands (MJCF motor actuators). Gripper: `position` command on panda_finger_joint1
  (MJCF tendon position actuator; panda_finger_joint2 follows through an equality constraint).
  The hardware is mujoco_ros2_control (D13). A real-arm driver would replace only <hardware>.
-->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="panda_state_interfaces">
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </xacro:macro>

  <xacro:macro name="panda_effort_joint" params="name">
    <joint name="${name}">
      <command_interface name="effort"/>
      <xacro:panda_state_interfaces/>
    </joint>
  </xacro:macro>

  <!-- Reads the top-level xacro args textually: ${...} would turn "true" into Python's "True". -->
  <xacro:macro name="panda_ros2_control">
    <ros2_control name="MujocoSystem" type="system">
      <hardware>
        <plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>
        <!-- Composed at launch by arm_sandbox_sim.scene (gravcomp on/off, absolute mesh paths). -->
        <param name="mujoco_model">$(arg mujoco_model)</param>
        <param name="headless">$(arg headless)</param>
        <!-- MJCF keyframe equal to robot.yaml `home`. -->
        <param name="initial_keyframe">home</param>
      </hardware>
      <xacro:panda_effort_joint name="panda_joint1"/>
      <xacro:panda_effort_joint name="panda_joint2"/>
      <xacro:panda_effort_joint name="panda_joint3"/>
      <xacro:panda_effort_joint name="panda_joint4"/>
      <xacro:panda_effort_joint name="panda_joint5"/>
      <xacro:panda_effort_joint name="panda_joint6"/>
      <xacro:panda_effort_joint name="panda_joint7"/>
      <joint name="panda_finger_joint1">
        <command_interface name="position"/>
        <xacro:panda_state_interfaces/>
      </joint>
    </ros2_control>
  </xacro:macro>
</robot>
```

In `panda.urdf.xacro`, directly after the opening `<robot ...>` line, add:

```xml
  <!-- Set by sim.launch.py. Empty defaults keep the file loadable for tests and RViz. -->
  <xacro:arg name="mujoco_model" default=""/>
  <xacro:arg name="headless" default="false"/>
  <xacro:include filename="panda.ros2_control.xacro"/>
```

and directly before `</robot>`:

```xml
  <xacro:panda_ros2_control/>
```

Run: `/usr/bin/python3 -m pytest -q src/arm_sandbox_description/test/test_panda_urdf.py`
Expected: 6 passed.

- [ ] **Step 3: Write the controllers config**

Create `src/arm_sandbox_bringup/config/panda_controllers.yaml`. The gains are starting values from an offline MuJoCo check of the same model (torque-controlled at 1 kHz, gravcomp off): they hold home within ~0.005 rad and track a 0.3 rad step on every joint. Task 5 tunes them if its test fails.

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
```

- [ ] **Step 4: Create the bringup package**

`src/arm_sandbox_bringup/package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>arm_sandbox_bringup</name>
  <version>0.1.0</version>
  <description>Top-level launch files and per-robot controller configs for arm-sandbox.</description>
  <maintainer email="robertocw_18@hotmail.com">Roberto Cai</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>arm_sandbox_description</exec_depend>
  <exec_depend>arm_sandbox_sim</exec_depend>
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
  <exec_depend>python3-yaml</exec_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>

  <test_depend>action_msgs</test_depend>
  <test_depend>control_msgs</test_depend>
  <test_depend>launch_testing_ament_cmake</test_depend>
  <test_depend>launch_testing_ros</test_depend>
  <test_depend>rclpy</test_depend>
  <test_depend>rosgraph_msgs</test_depend>
  <test_depend>sensor_msgs</test_depend>
  <test_depend>trajectory_msgs</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

`src/arm_sandbox_bringup/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(arm_sandbox_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/activate_at_home DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Each launch test gets its own ROS domain, so sims started by tests of different packages
  # (colcon runs packages in parallel) or a `make start_sim` in another terminal can't interfere.
  add_launch_test(test/test_sim_bringup.py TIMEOUT 180 ENV ROS_DOMAIN_ID=41)
endif()

ament_package()
```

- [ ] **Step 5: Write the failing launch test**

Create `src/arm_sandbox_bringup/test/test_sim_bringup.py`:

```python
"""Sim bring-up: the Panda comes up in MuJoCo behind ros2_control and holds its home pose.

Headless, gravcomp off (the default and the harder case: controllers carry the gravity load).
"""

import time
import unittest
from pathlib import Path

import pytest
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from controller_manager.test_utils import check_controllers_running
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

ROBOT = "panda"
CONTROLLERS = ["joint_state_broadcaster", "arm_controller", "gripper_controller"]
STARTUP_TIMEOUT_S = 60.0
SETTLE_TIME_S = 2.0
HOME_TOLERANCE_RAD = 0.05
NANOSECONDS_PER_SECOND = 1_000_000_000


def robot_config() -> dict:
    path = Path(get_package_share_directory("arm_sandbox_description")) / ROBOT / "config" / "robot.yaml"
    return yaml.safe_load(path.read_text())


@pytest.mark.launch_test
def generate_test_description() -> LaunchDescription:
    sim_launch = Path(get_package_share_directory("arm_sandbox_bringup")) / "launch" / "sim.launch.py"
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_launch)),
        launch_arguments={"robot": ROBOT, "viewer": "false", "gravcomp": "false"}.items(),
    )
    return LaunchDescription([sim, ReadyToTest()])


class TestSimBringup(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.node = rclpy.create_node("test_sim_bringup")

    def tearDown(self) -> None:
        self.node.destroy_node()

    def wait_for_message(self, msg_type, topic: str, predicate=lambda _msg: True, timeout_s: float = STARTUP_TIMEOUT_S):
        received = []
        # Best-effort subscription: receives from reliable and best-effort publishers alike.
        subscription = self.node.create_subscription(
            msg_type, topic, lambda msg: received.append(msg) if predicate(msg) else None, qos_profile_sensor_data
        )
        deadline = time.monotonic() + timeout_s
        try:
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        finally:
            self.node.destroy_subscription(subscription)
        self.assertTrue(received, f"no matching message on {topic} within {timeout_s} s")
        return received[0]

    def spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def test_controllers_become_active(self) -> None:
        check_controllers_running(self.node, CONTROLLERS, timeout=STARTUP_TIMEOUT_S)

    def test_clock_advances(self) -> None:
        def nanoseconds(msg: Clock) -> int:
            return msg.clock.sec * NANOSECONDS_PER_SECOND + msg.clock.nanosec

        first = self.wait_for_message(Clock, "/clock")
        self.spin_for(SETTLE_TIME_S)
        second = self.wait_for_message(Clock, "/clock")
        self.assertGreater(nanoseconds(second), nanoseconds(first))

    def test_arm_holds_home_pose(self) -> None:
        check_controllers_running(self.node, CONTROLLERS, timeout=STARTUP_TIMEOUT_S)
        self.spin_for(SETTLE_TIME_S)
        config = robot_config()
        joints = config["arm_joints"]
        msg = self.wait_for_message(JointState, "/joint_states", lambda m: set(joints) <= set(m.name))
        positions = dict(zip(msg.name, msg.position))
        errors = [abs(positions[joint] - home) for joint, home in zip(joints, config["home"])]
        self.assertLess(max(errors), HOME_TOLERANCE_RAD, dict(zip(joints, errors)))
```

Run: `make test`
Expected: FAIL. `test_sim_bringup` errors because `launch/sim.launch.py` doesn't exist (`FileNotFoundError` / "file not found" in the launch output).

- [ ] **Step 6: Write `activate_at_home`**

Create `src/arm_sandbox_bringup/scripts/activate_at_home` (then `chmod +x`):

```python
#!/usr/bin/env python3
"""Reset the sim to the home keyframe, then activate the arm controllers right away.

A torque-controlled arm without gravity compensation sags as soon as nothing commands it, and JTC
holds whatever pose it sees when it activates. Starting a spawner process after the reset takes
about 2 s, enough for the arm to fall. This node makes the two service calls back to back, so the
controllers latch the home pose. The controllers must already be loaded and configured (inactive).
"""

import sys

import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
from mujoco_ros2_control_msgs.srv import ResetWorld
from rclpy.node import Node


class ActivateAtHome(Node):
    def __init__(self) -> None:
        super().__init__("activate_at_home")
        self.reset_service = self.declare_parameter("reset_service", "").value
        self.keyframe = self.declare_parameter("keyframe", "").value
        self.controllers = list(self.declare_parameter("controllers", [""]).value)
        self.switch_service = self.declare_parameter("switch_service", "").value
        self.timeout_s = float(self.declare_parameter("timeout_s", 0.0).value)
        if not all([self.reset_service, self.keyframe, self.switch_service, self.timeout_s > 0.0]) or not all(
            self.controllers
        ):
            raise ValueError("activate_at_home: set reset_service, keyframe, controllers, switch_service, timeout_s")

    def call(self, service_type, name: str, request):
        client = self.create_client(service_type, name)
        if not client.wait_for_service(timeout_sec=self.timeout_s):
            raise RuntimeError(f"service {name} not available after {self.timeout_s} s")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_s)
        if future.result() is None:
            raise RuntimeError(f"no response from {name} within {self.timeout_s} s")
        return future.result()

    def run(self) -> None:
        reset = self.call(ResetWorld, self.reset_service, ResetWorld.Request(keyframe=self.keyframe))
        if not reset.success:
            raise RuntimeError(f"reset to '{self.keyframe}' failed: {reset.message}")
        switch = self.call(
            SwitchController,
            self.switch_service,
            SwitchController.Request(
                activate_controllers=self.controllers,
                strictness=SwitchController.Request.STRICT,
                activate_asap=True,
                timeout=Duration(sec=int(self.timeout_s)),
            ),
        )
        if not switch.ok:
            raise RuntimeError(f"activating {self.controllers} failed")
        self.get_logger().info(f"reset to '{self.keyframe}' and activated {self.controllers}")


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = ActivateAtHome()
        node.run()
        return 0
    except (RuntimeError, ValueError) as error:
        (node.get_logger() if node else rclpy.logging.get_logger("activate_at_home")).fatal(str(error))
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 7: Write `sim.launch.py`**

Create `src/arm_sandbox_bringup/launch/sim.launch.py`:

```python
"""Bring up one robot in MuJoCo behind ros2_control (milestone M1).

    ros2 launch arm_sandbox_bringup sim.launch.py robot:=panda viewer:=true gravcomp:=false

Startup order: with a torque-controlled arm and no gravity compensation, the arm sags from the
moment physics starts, because zero torque is commanded until a controller is active, and JTC holds
whatever pose it sees when it activates. So the arm and gripper controllers are loaded inactive, and
activate_at_home resets the sim to the `home` keyframe and activates them in back-to-back service
calls (a spawner started after the reset would leave ~2 s for the arm to fall).
"""

from pathlib import Path

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
ARM_AND_GRIPPER_CONTROLLERS = ["arm_controller", "gripper_controller"]
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


def launch_setup(context: LaunchContext) -> list:
    robot = LaunchConfiguration("robot").perform(context)
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
    control_node = Node(
        # mujoco_ros2_control ships its own ros2_control_node (same parameters as upstream's).
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        parameters=[{"use_sim_time": True}, str(controllers_file)],
        # Humble's controller_manager reads the URDF from ~/robot_description.
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
        on_exit=Shutdown(),
    )
    joint_state_broadcaster = spawner("joint_state_broadcaster")
    # Loaded and configured, but not active: activate_at_home activates them right after the reset.
    arm_and_gripper = spawner(*ARM_AND_GRIPPER_CONTROLLERS, "--inactive")
    activate_at_home = Node(
        package="arm_sandbox_bringup",
        executable="activate_at_home",
        parameters=[
            {
                "reset_service": RESET_WORLD_SERVICE,
                "keyframe": HOME_KEYFRAME,
                "switch_service": SWITCH_CONTROLLER_SERVICE,
                "controllers": ARM_AND_GRIPPER_CONTROLLERS,
                "timeout_s": STARTUP_SERVICE_TIMEOUT_S,
            }
        ],
        output="screen",
    )
    return [
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
                "gravcomp",
                default_value="false",
                description="MuJoCo compensates the arm's gravity (like the real Panda); false: controllers do",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
```

- [ ] **Step 8: Run it by hand first**

```bash
make test   # builds everything; test_sim_bringup may still fail, that's fine here
source install/setup.bash
ros2 launch arm_sandbox_bringup sim.launch.py viewer:=false
```

In a second container terminal:

```bash
source install/setup.bash
ros2 control list_controllers      # Expected: 3 controllers, all "active"
ros2 topic echo --once /joint_states | head -20   # Expected: panda_joint1..7 within ~1e-3 rad of robot.yaml home
```

Expected in the launch output, in this order: `Created reset_world service at: /mujoco_ros2_control_node/reset_world`, `Configured and activated joint_state_broadcaster`, and `[activate_at_home]: reset to 'home' and activated ['arm_controller', 'gripper_controller']`.

If the service path printed differs, change `RESET_WORLD_SERVICE`. If a controller fails to load, the launch output names the parameter. Fix it in `panda_controllers.yaml`. Stop the launch with Ctrl+C.

- [ ] **Step 9: Run the tests to see them pass**

Run: `make test`
Expected: 0 failures. `test_sim_bringup` reports 3 tests passed.

If `test_arm_holds_home_pose` fails with large errors on `panda_joint2`/`panda_joint4`, the arm fell before the controllers activated. Check that `activate_at_home` logged success and that nothing else spawns `arm_controller` as active.

- [ ] **Step 10: Commit**

```bash
git add src/arm_sandbox_description src/arm_sandbox_bringup
git commit -m "feat(bringup): Panda in MuJoCo behind ros2_control; reset to home and activate back to back"
```

### Task 5: The arm follows a joint trajectory (M1 acceptance)

**Files:**
- Test: `src/arm_sandbox_bringup/test/test_arm_trajectory.py`
- Modify: `src/arm_sandbox_bringup/CMakeLists.txt` (register the test)
- Modify: `src/arm_sandbox_bringup/config/panda_controllers.yaml` (only if gains need tuning)
- Modify: `Makefile` (`start_sim` target)

**Interfaces:**
- Consumes: Task 4 launch, actions `/arm_controller/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`) and `/gripper_controller/gripper_cmd` (`control_msgs/action/GripperCommand`).
- Produces: `make start_sim [ARGS="viewer:=false gravcomp:=true ..."]`.

- [ ] **Step 1: Write the test**

Create `src/arm_sandbox_bringup/test/test_arm_trajectory.py`:

```python
"""M1 acceptance: the arm follows a joint trajectory through ros2_control, and the gripper moves.

Runs once per gravity-compensation mode (D15): controllers compensate (false) or MuJoCo does (true).
"""

import time
import unittest
from pathlib import Path

import launch_testing
import pytest
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from controller_manager.test_utils import check_controllers_running
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ROBOT = "panda"
CONTROLLERS = ["joint_state_broadcaster", "arm_controller", "gripper_controller"]
STARTUP_TIMEOUT_S = 60.0
ACTION_TIMEOUT_S = 30.0
MOVE_DURATION_S = 2
# Offset from home for the test goal: every joint moves, and all stay inside their limits.
GOAL_OFFSET_RAD = [0.3, 0.3, -0.3, 0.3, 0.3, 0.3, 0.3]
FINAL_TOLERANCE_RAD = 0.02  # the JTC `goal` tolerance in panda_controllers.yaml
# Partly closed, short of finger contact (the fingertip pads touch near 0).
GRIPPER_PARTLY_CLOSED_M = 0.01
GRIPPER_TOLERANCE_M = 0.003
GRIPPER_MAX_EFFORT_N = 50.0


def robot_config() -> dict:
    path = Path(get_package_share_directory("arm_sandbox_description")) / ROBOT / "config" / "robot.yaml"
    return yaml.safe_load(path.read_text())


@pytest.mark.launch_test
@launch_testing.parametrize("gravcomp", ["false", "true"])
def generate_test_description(gravcomp: str) -> LaunchDescription:
    sim_launch = Path(get_package_share_directory("arm_sandbox_bringup")) / "launch" / "sim.launch.py"
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_launch)),
        launch_arguments={"robot": ROBOT, "viewer": "false", "gravcomp": gravcomp}.items(),
    )
    return LaunchDescription([sim, ReadyToTest()])


class TestArmTrajectory(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.node = rclpy.create_node("test_arm_trajectory")
        check_controllers_running(self.node, CONTROLLERS, timeout=STARTUP_TIMEOUT_S)

    def tearDown(self) -> None:
        self.node.destroy_node()

    def latest_positions(self, joints: list[str]) -> list[float]:
        received = []
        subscription = self.node.create_subscription(
            JointState,
            "/joint_states",
            lambda msg: received.append(msg) if set(joints) <= set(msg.name) else None,
            qos_profile_sensor_data,
        )
        deadline = time.monotonic() + STARTUP_TIMEOUT_S
        try:
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        finally:
            self.node.destroy_subscription(subscription)
        self.assertTrue(received, "no /joint_states with the requested joints")
        positions = dict(zip(received[0].name, received[0].position))
        return [positions[joint] for joint in joints]

    def send_goal(self, client: ActionClient, goal):
        self.assertTrue(client.wait_for_server(timeout_sec=STARTUP_TIMEOUT_S), "action server not available")
        goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, goal_future, timeout_sec=ACTION_TIMEOUT_S)
        handle = goal_future.result()
        self.assertTrue(handle is not None and handle.accepted, "goal rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=ACTION_TIMEOUT_S)
        response = result_future.result()
        self.assertIsNotNone(response, "no result before timeout")
        return response

    def test_arm_follows_joint_trajectory(self) -> None:
        config = robot_config()
        joints = config["arm_joints"]
        target = [home + offset for home, offset in zip(config["home"], GOAL_OFFSET_RAD)]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        goal.trajectory.points = [
            JointTrajectoryPoint(
                positions=target, velocities=[0.0] * len(joints), time_from_start=Duration(sec=MOVE_DURATION_S)
            )
        ]
        client = ActionClient(self.node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")

        response = self.send_goal(client, goal)

        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(response.result.error_code, FollowJointTrajectory.Result.SUCCESSFUL, response.result.error_string)
        errors = [abs(actual - wanted) for actual, wanted in zip(self.latest_positions(joints), target)]
        self.assertLess(max(errors), FINAL_TOLERANCE_RAD, dict(zip(joints, errors)))

    def test_gripper_closes_and_opens(self) -> None:
        gripper = robot_config()["gripper"]
        client = ActionClient(self.node, GripperCommand, "/gripper_controller/gripper_cmd")
        for target in (GRIPPER_PARTLY_CLOSED_M, gripper["max_width"] / 2):
            goal = GripperCommand.Goal()
            goal.command.position = target
            goal.command.max_effort = GRIPPER_MAX_EFFORT_N

            response = self.send_goal(client, goal)

            self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
            self.assertAlmostEqual(self.latest_positions([gripper["joint"]])[0], target, delta=GRIPPER_TOLERANCE_M)
```

Register it in `src/arm_sandbox_bringup/CMakeLists.txt`, after the `test_sim_bringup` line:

```cmake
  add_launch_test(test/test_arm_trajectory.py TIMEOUT 300 ENV ROS_DOMAIN_ID=42)
```

- [ ] **Step 2: Run it**

Run: `make test`
Expected: PASS for both parametrizations (`gravcomp=false` and `gravcomp=true`): 2 tests each.

This test is new behavior, not a red/green step on new code. The controllers already exist, so it can pass right away. If it fails, go to Step 3. Otherwise skip Step 3.

- [ ] **Step 3: Tune the gains (only if Step 2 failed)**

Watch the tracking error live:

```bash
ros2 launch arm_sandbox_bringup sim.launch.py viewer:=true   # terminal 1
ros2 topic echo /arm_controller/state --field error.positions   # terminal 2
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7],
    points: [{positions: [0.3, -0.485, -0.3, -2.056, 0.3, 1.871, 1.085], time_from_start: {sec: 2}}]}}"   # terminal 3
```

Change one joint at a time in `panda_controllers.yaml`:
- Steady error that doesn't shrink: raise `i` (keep `i_clamp` at or below half the joint's torque limit).
- Oscillation or vibration: raise `d`, or lower `p`.
- Large error during the move only: raise `p`.

Rebuild isn't needed for YAML changes with `--symlink-install`. Restart the launch. Re-run `make test` until it passes. Put the final values and a one-line reason in the commit message.

- [ ] **Step 4: Add `make start_sim`**

In the `Makefile`, add `start_sim` to `.PHONY` and append:

```make
# Container: run the sim. ARGS are launch arguments, e.g. make start_sim ARGS="viewer:=false gravcomp:=true"
start_sim:
	$(ROS_SETUP) && source install/setup.bash && ros2 launch arm_sandbox_bringup sim.launch.py $(ARGS)
```

Also update the header comment's container target list: `# Container targets (in dev container): smoke, test, viewer, start_sim`.

Document it where the other targets are listed: in `CLAUDE.md` ("Inside the dev container"), add `- \`make start_sim\` - run the sim (\`ros2 launch arm_sandbox_bringup sim.launch.py\`); \`make start_sim ARGS="viewer:=false gravcomp:=true"\`` and change `- \`make sim\`, \`make lint\` - added by later plans` to `- \`make lint\` - added by later plans`. In `docs/PROJECT_STRUCTURE.md`, add a Commands-table row `| \`make start_sim\` | container | Run the sim (\`ARGS="viewer:=false gravcomp:=true"\`) |` and change the note below the table to `\`make lint\` is added by a later plan.`

Run: `make start_sim ARGS="viewer:=false"` and stop it with Ctrl+C after "Configured and activated arm_controller".
Expected: no errors.

- [ ] **Step 5: Watch it once** [container, needs a display]

```bash
make start_sim   # native MuJoCo viewer opens; the arm stands in the home pose
```

In a second terminal, send the `ros2 action send_goal` command from Step 3. Expected: the arm moves smoothly to the new pose in about 2 s and stays there. Ctrl+C the launch.

- [ ] **Step 6: Commit**

```bash
git add src/arm_sandbox_bringup Makefile
git commit -m "test(bringup): arm follows a joint trajectory and gripper moves, with and without gravcomp; make start_sim"
```

---

# Part 3 — Browser Viewer (finish M0)

### Task 6: Rerun C++ SDK in the toolchain

`arm_sandbox_viz` is C++ (spec), and the Rerun C++ SDK isn't packaged for apt. `install_deps.sh` builds it once from the release bundle, at the same version as the viewer, and installs it to `/opt/rerun_cpp_sdk`. Colcon builds then find it offline. The bundle builds Apache Arrow, which takes **about 10 minutes** with all cores. This was checked for 0.38.1 while writing this plan: `find_package(rerun_sdk)` plus `target_link_libraries(... rerun_sdk)` builds cleanly with `-Wall -Wextra -Wpedantic -Werror`.

**Files:**
- Modify: `scripts/install_deps.sh`
- Modify: `tests/env/test_toolchain.py`

**Interfaces:**
- Produces: CMake package `rerun_sdk` (target `rerun_sdk`) under prefix `/opt/rerun_cpp_sdk`. Version equals `RERUN_VERSION` (0.38.1).

- [ ] **Step 1: Write the failing smoke tests**

In `tests/env/test_toolchain.py`, add `import re` and `from pathlib import Path` to the imports. Then add these constants below `MUJOCO_VENDOR_LIB`:

```python
RERUN_CPP_SDK_PREFIX = Path("/opt/rerun_cpp_sdk")
RERUN_CPP_SDK_VERSION_FILE = RERUN_CPP_SDK_PREFIX / "lib" / "cmake" / "rerun_sdk" / "rerun_sdkConfigVersion.cmake"
```

and append:

```python
def test_rerun_cpp_sdk_matches_viewer_version() -> None:
    # The viz bridge (C++ SDK) and the viewer (`rerun` CLI) must be the same Rerun version.
    assert RERUN_CPP_SDK_VERSION_FILE.is_file(), "Rerun C++ SDK not installed (scripts/install_deps.sh)"
    match = re.search(r'set\(PACKAGE_VERSION "([^"]+)"\)', RERUN_CPP_SDK_VERSION_FILE.read_text())
    assert match, "no PACKAGE_VERSION in the Rerun C++ SDK config"
    viewer = subprocess.run(["rerun", "--version"], capture_output=True, text=True, check=True).stdout
    assert f"rerun-cli {match.group(1)} " in viewer, (match.group(1), viewer)


def test_rerun_cpp_sdk_builds_and_links(tmp_path: Path) -> None:
    (tmp_path / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.16)\n"
        "project(rerun_smoke CXX)\n"
        "set(CMAKE_CXX_STANDARD 17)\n"
        "find_package(rerun_sdk REQUIRED)\n"
        "add_executable(rerun_smoke main.cpp)\n"
        "target_compile_options(rerun_smoke PRIVATE -Wall -Wextra -Wpedantic -Werror)\n"
        "target_link_libraries(rerun_smoke PRIVATE rerun_sdk)\n"
    )
    (tmp_path / "main.cpp").write_text(
        '#include <rerun.hpp>\nint main() { const rerun::RecordingStream rec("smoke"); return 0; }\n'
    )
    build = tmp_path / "build"
    for command in (
        ["cmake", "-S", str(tmp_path), "-B", str(build), f"-DCMAKE_PREFIX_PATH={RERUN_CPP_SDK_PREFIX}"],
        ["cmake", "--build", str(build)],
        [str(build / "rerun_smoke")],
    ):
        result = subprocess.run(command, capture_output=True, text=True, check=False)
        assert result.returncode == 0, f"{command}: {result.stdout}\n{result.stderr}"
```

Run: `make smoke`
Expected: the two new tests FAIL (`Rerun C++ SDK not installed`, and `find_package(rerun_sdk)` not found). All others pass.

- [ ] **Step 2: Add the SDK build to `install_deps.sh`**

Below `readonly RERUN_VENV="/opt/rerun"` add:

```bash
readonly RERUN_CPP_SDK_PREFIX="/opt/rerun_cpp_sdk"
```

Update the header comment's bullet list with:

```bash
#   - Rerun C++ SDK (built from the release bundle) in /opt/rerun_cpp_sdk, for arm_sandbox_viz
```

Add this function after `install_rerun()`:

```bash
install_rerun_cpp_sdk() {
    # arm_sandbox_viz links the Rerun C++ SDK, which has no apt package. Build it from the release
    # bundle at the viewer's version, so colcon builds find it offline (find_package(rerun_sdk)).
    # The bundle builds Apache Arrow: ~10 minutes.
    local version_file="${RERUN_CPP_SDK_PREFIX}/lib/cmake/rerun_sdk/rerun_sdkConfigVersion.cmake"
    if [ -f "${version_file}" ] && grep -qF "set(PACKAGE_VERSION \"${RERUN_VERSION}\")" "${version_file}"; then
        echo "Rerun C++ SDK ${RERUN_VERSION} already installed."
        return
    fi

    echo "Building the Rerun C++ SDK ${RERUN_VERSION} into ${RERUN_CPP_SDK_PREFIX}..."
    local work
    work="$(mktemp -d)"
    curl -fsSL -o "${work}/rerun_cpp_sdk.zip" \
        "https://github.com/rerun-io/rerun/releases/download/${RERUN_VERSION}/rerun_cpp_sdk.zip"
    "${SYSTEM_PYTHON}" -m zipfile -e "${work}/rerun_cpp_sdk.zip" "${work}"
    cmake -S "${work}/rerun_cpp_sdk" -B "${work}/build" \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${RERUN_CPP_SDK_PREFIX}"
    cmake --build "${work}/build" --config Release --target rerun_sdk -j"$(nproc)"
    "${SUDO[@]}" cmake --install "${work}/build"
    rm -rf "${work}"
}
```

and call it in `main()` right after `install_rerun`:

```bash
    install_rerun_cpp_sdk
```

- [ ] **Step 3: Install it in the running container**

```bash
scripts/install_deps.sh   # re-runs everything; apt/pip steps are no-ops, the SDK build takes ~10 min
```

Expected: ends with "arm-sandbox dependencies installed". `ls /opt/rerun_cpp_sdk/lib` shows `librerun_sdk.a`, `librerun_c__linux_x64.a`, `libarrow.a`, and `libarrow_bundled_dependencies.a`.

- [ ] **Step 4: Run the smoke tests to see them pass**

Run: `make smoke`
Expected: all pass, including the two new tests.

- [ ] **Step 5: Rebuild the image** [host]

```bash
make build   # the Dockerfile runs install_deps.sh, so the image now contains the SDK (+~10 min)
```

Then restart the dev container on the new image (`make down`, then VS Code **Dev Containers: Reopen in Container**) and run `make smoke` again. Expected: all pass.

- [ ] **Step 6: Commit**

```bash
git add scripts/install_deps.sh tests/env/test_toolchain.py
git commit -m "feat(env): build the Rerun C++ SDK into /opt/rerun_cpp_sdk for the viz bridge"
```

### Task 7: `arm_sandbox_viz` — ROS 2 → Rerun bridge node

The bridge subscribes only to standard topics (FR-14a) and logs them in the way Rerun's own URDF example does. This was checked against 0.38.1 while writing this plan:

- `/robot_description` → `try_log_file_from_contents("robot_description.urdf", …, "robot", static)`. Rerun's URDF loader resolves `package://` meshes from the ROS environment. It logs the geometry under `/robot/<robot name>/visual_geometries/<link>/…`, attaches each link to a coordinate frame named after the link, and logs the zero-pose joint transforms to `/tf_static`.
- Every `/tf` transform → `Transform3D(...).with_parent_frame(header.frame_id).with_child_frame(child_frame_id)`, logged to its own entity `ros/tf/<child_frame>` on the `sim_time` timeline. A newer transform for a known child frame replaces the URDF's static one, so the robot moves.
- `/tf_static` → the same transform, static, to `ros/tf_static/<child_frame>`. One entity per frame, so later static publishers don't overwrite each other.
- `/joint_states` → `Scalars` time series `joint_states/<joint>/{position,velocity,effort}`, throttled by `joint_state_log_period_s`.

**Files:**
- Create: `src/arm_sandbox_viz/package.xml`, `src/arm_sandbox_viz/CMakeLists.txt`
- Create: `src/arm_sandbox_viz/include/arm_sandbox_viz/rerun_bridge.hpp`
- Create: `src/arm_sandbox_viz/src/rerun_bridge.cpp`, `src/arm_sandbox_viz/src/main.cpp`
- Create: `src/arm_sandbox_viz/config/rerun_bridge.yaml`
- Test: `src/arm_sandbox_viz/test/test_rerun_bridge.py`

**Interfaces:**
- Consumes: `rerun_sdk` from `/opt/rerun_cpp_sdk` (Task 6).
- Produces: executable `arm_sandbox_viz/rerun_bridge` (node name `rerun_bridge`). Parameters: `application_id` (str), `grpc_url` (str), `save_path` (str; exactly one of `grpc_url`/`save_path` is non-empty), `joint_state_log_period_s` (float, ≥ 0). Config `share/arm_sandbox_viz/config/rerun_bridge.yaml`. Entity layout as listed above.

- [ ] **Step 1: Package skeleton and config**

`src/arm_sandbox_viz/package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>arm_sandbox_viz</name>
  <version>0.1.0</version>
  <description>ROS 2 to Rerun bridge: shows the running sim in the browser from standard topics only.</description>
  <maintainer email="robertocw_18@hotmail.com">Roberto Cai</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- The Rerun C++ SDK comes from scripts/install_deps.sh (/opt/rerun_cpp_sdk): no rosdep key. -->
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>tf2_msgs</depend>

  <test_depend>geometry_msgs</test_depend>
  <test_depend>launch_testing_ament_cmake</test_depend>
  <test_depend>rclpy</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

`src/arm_sandbox_viz/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(arm_sandbox_viz)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
# Built and installed by scripts/install_deps.sh; not an apt/rosdep package.
set(RERUN_CPP_SDK_PREFIX "/opt/rerun_cpp_sdk" CACHE PATH "Rerun C++ SDK install prefix")
find_package(rerun_sdk REQUIRED HINTS ${RERUN_CPP_SDK_PREFIX})

add_executable(rerun_bridge src/main.cpp src/rerun_bridge.cpp)
target_include_directories(rerun_bridge PRIVATE include)
ament_target_dependencies(rerun_bridge rclcpp sensor_msgs std_msgs tf2_msgs)
# Plain signature: ament_target_dependencies uses it too, and CMake forbids mixing the two.
target_link_libraries(rerun_bridge rerun_sdk)

install(TARGETS rerun_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_rerun_bridge.py TIMEOUT 120 ENV ROS_DOMAIN_ID=43)
endif()

ament_package()
```

`src/arm_sandbox_viz/config/rerun_bridge.yaml`:

```yaml
rerun_bridge:
  ros__parameters:
    application_id: arm_sandbox
    # The viewer started by sim.launch.py (`rerun --serve-web`) takes gRPC on 9876 and serves the web
    # viewer on http://localhost:9090 (Rerun 0.38 defaults, checked by tests/env/test_rerun_web.py).
    grpc_url: rerun+http://127.0.0.1:9876/proxy
    # Non-empty: record to this .rrd file instead of streaming (then grpc_url must be empty).
    save_path: ""
    # Log /joint_states at most this often (s). JSB publishes at 100 Hz, which is more than plots need.
    joint_state_log_period_s: 0.02
```

- [ ] **Step 2: Write the failing test**

Create `src/arm_sandbox_viz/test/test_rerun_bridge.py`. It runs the bridge alone with a tiny box-only URDF (no meshes, no sim), publishes one message per topic, and inspects the recorded `.rrd` after shutdown, when the bridge has flushed.

```python
"""The Rerun bridge logs the URDF, TF and joint states from standard topics (FR-12, FR-14a).

The bridge records to a file; after shutdown, `rerun rrd print` shows which entities it logged.
"""

import subprocess
import tempfile
import time
import unittest
from pathlib import Path

import launch_testing
import launch_testing.asserts
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

RECORDING = Path(tempfile.mkdtemp(prefix="rerun_bridge_test_")) / "bridge.rrd"
PUBLISH_DURATION_S = 3.0
PUBLISH_PERIOD_S = 0.05
ROBOT_DESCRIPTION = """<?xml version="1.0"?>
<robot name="test_bot">
  <link name="base_link"><visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual></link>
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/><child link="arm_link"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <link name="arm_link"><visual><geometry><box size="0.2 0.05 0.05"/></geometry></visual></link>
</robot>
"""
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


@pytest.mark.launch_test
def generate_test_description() -> LaunchDescription:
    bridge = Node(
        package="arm_sandbox_viz",
        executable="rerun_bridge",
        name="rerun_bridge",
        parameters=[{"save_path": str(RECORDING), "grpc_url": "", "joint_state_log_period_s": 0.0}],
        output="screen",
    )
    return LaunchDescription([bridge, ReadyToTest()])


def transform(parent: str, child: str, stamp) -> TransformStamped:
    msg = TransformStamped()
    msg.header.frame_id = parent
    msg.header.stamp = stamp
    msg.child_frame_id = child
    msg.transform.rotation.w = 1.0
    return msg


class TestPublish(unittest.TestCase):
    def test_publish_standard_topics(self) -> None:
        rclpy.init()
        node = rclpy.create_node("rerun_bridge_test_publisher")
        try:
            description = node.create_publisher(String, "/robot_description", LATCHED)
            tf_static = node.create_publisher(TFMessage, "/tf_static", LATCHED)
            tf = node.create_publisher(TFMessage, "/tf", 10)
            joint_states = node.create_publisher(JointState, "/joint_states", 10)

            description.publish(String(data=ROBOT_DESCRIPTION))
            tf_static.publish(TFMessage(transforms=[transform("world", "base_link", node.get_clock().now().to_msg())]))
            deadline = time.monotonic() + PUBLISH_DURATION_S
            while time.monotonic() < deadline:
                stamp = node.get_clock().now().to_msg()
                tf.publish(TFMessage(transforms=[transform("base_link", "arm_link", stamp)]))
                state = JointState(name=["arm_joint"], position=[0.5], velocity=[0.0], effort=[0.0])
                state.header.stamp = stamp
                joint_states.publish(state)
                rclpy.spin_once(node, timeout_sec=PUBLISH_PERIOD_S)
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestRecording(unittest.TestCase):
    def test_bridge_exited_cleanly(self, proc_info) -> None:
        launch_testing.asserts.assertExitCodes(proc_info)

    def test_recording_has_robot_tf_and_joint_states(self) -> None:
        self.assertTrue(RECORDING.is_file(), f"no recording at {RECORDING}")
        printed = subprocess.run(["rerun", "rrd", "print", str(RECORDING)], capture_output=True, text=True, check=True)
        entities = printed.stdout
        self.assertIn("/robot/test_bot/visual_geometries/arm_link", entities)
        self.assertRegex(entities, r"/ros/tf/arm_link - data columns: \[Transform3D:child_frame")
        self.assertRegex(entities, r"/ros/tf_static/base_link - data columns: \[Transform3D:child_frame")
        self.assertIn("/joint_states/arm_joint/position - data columns: [Scalars:scalars]", entities)
```

Run: `make test`
Expected: FAIL at build time. CMake can't find `src/main.cpp` (or, if you created empty files, linking fails).

- [ ] **Step 3: Write the node**

`src/arm_sandbox_viz/include/arm_sandbox_viz/rerun_bridge.hpp`:

```cpp
#pragma once

#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace arm_sandbox_viz
{

/// Forwards standard ROS 2 topics to Rerun (FR-12). Reads no sim internals (FR-14a).
///
/// - /robot_description: logged through Rerun's URDF loader, which names each link's coordinate
///   frame after the link and resolves package:// meshes from the ROS environment.
/// - /tf, /tf_static: each transform becomes a Transform3D between named frames, so the URDF
///   geometry follows the robot. /tf is timed by its header stamp (sim time), /tf_static is static.
/// - /joint_states: position, velocity and effort per joint as time series (throttled).
class RerunBridge : public rclcpp::Node
{
public:
  explicit RerunBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RerunBridge() override;

private:
  void on_robot_description(const std_msgs::msg::String & msg) const;
  void on_tf(const tf2_msgs::msg::TFMessage & msg, bool is_static) const;
  void on_joint_states(const sensor_msgs::msg::JointState & msg);

  rerun::RecordingStream recording_;
  rclcpp::Duration joint_state_log_period_;
  std::optional<rclcpp::Time> last_joint_state_log_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
};

}  // namespace arm_sandbox_viz
```

`src/arm_sandbox_viz/src/rerun_bridge.cpp`:

```cpp
#include "arm_sandbox_viz/rerun_bridge.hpp"

#include <cstddef>
#include <stdexcept>
#include <string>

namespace arm_sandbox_viz
{
namespace
{
constexpr const char * kTimeline = "sim_time";
// File name hint: tells Rerun to use its URDF loader for the /robot_description contents.
constexpr const char * kRobotDescriptionFileName = "robot_description.urdf";
constexpr const char * kRobotEntityPrefix = "robot";

rerun::Transform3D to_rerun(const geometry_msgs::msg::TransformStamped & msg)
{
  const auto & t = msg.transform.translation;
  const auto & r = msg.transform.rotation;
  return rerun::Transform3D::from_translation_rotation(
           {static_cast<float>(t.x), static_cast<float>(t.y), static_cast<float>(t.z)},
           rerun::Quaternion::from_xyzw(
             static_cast<float>(r.x), static_cast<float>(r.y), static_cast<float>(r.z),
             static_cast<float>(r.w)))
    .with_parent_frame(msg.header.frame_id)
    .with_child_frame(msg.child_frame_id);
}

double to_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return rclcpp::Time(stamp).seconds();
}
}  // namespace

RerunBridge::RerunBridge(const rclcpp::NodeOptions & options)
: Node("rerun_bridge", options),
  recording_(declare_parameter<std::string>("application_id", "arm_sandbox")),
  joint_state_log_period_(
    rclcpp::Duration::from_seconds(declare_parameter<double>("joint_state_log_period_s", 0.0)))
{
  const auto grpc_url = declare_parameter<std::string>("grpc_url", "");
  const auto save_path = declare_parameter<std::string>("save_path", "");
  if (grpc_url.empty() == save_path.empty()) {
    RCLCPP_FATAL(get_logger(), "set exactly one of 'grpc_url' or 'save_path'");
    throw std::invalid_argument("invalid rerun_bridge sink config");
  }
  if (joint_state_log_period_ < rclcpp::Duration(0, 0)) {
    RCLCPP_FATAL(get_logger(), "'joint_state_log_period_s' must be >= 0");
    throw std::invalid_argument("invalid rerun_bridge throttle config");
  }

  const rerun::Error error =
    save_path.empty() ? recording_.connect_grpc(grpc_url) : recording_.save(save_path);
  if (error.is_err()) {
    RCLCPP_FATAL(get_logger(), "cannot open the Rerun sink: %s", error.description.c_str());
    throw std::runtime_error("rerun sink failed");
  }
  RCLCPP_INFO(
    get_logger(), "logging to %s", save_path.empty() ? grpc_url.c_str() : save_path.c_str());

  const auto latched = rclcpp::QoS(1).transient_local().reliable();
  robot_description_sub_ = create_subscription<std_msgs::msg::String>(
    "/robot_description", latched,
    [this](const std_msgs::msg::String & msg) { on_robot_description(msg); });
  tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", rclcpp::QoS(100), [this](const tf2_msgs::msg::TFMessage & msg) { on_tf(msg, false); });
  tf_static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(100).transient_local().reliable(),
    [this](const tf2_msgs::msg::TFMessage & msg) { on_tf(msg, true); });
  joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(10),
    [this](const sensor_msgs::msg::JointState & msg) { on_joint_states(msg); });
}

RerunBridge::~RerunBridge()
{
  // Make sure a recording file is complete when the node stops.
  const rerun::Error error = recording_.flush_blocking();
  if (error.is_err()) {
    RCLCPP_ERROR(get_logger(), "flushing the Rerun recording failed: %s", error.description.c_str());
  }
}

void RerunBridge::on_robot_description(const std_msgs::msg::String & msg) const
{
  const auto * contents = reinterpret_cast<const std::byte *>(msg.data.data());
  const rerun::Error error = recording_.try_log_file_from_contents(
    kRobotDescriptionFileName, contents, msg.data.size(), kRobotEntityPrefix, true);
  if (error.is_err()) {
    RCLCPP_ERROR(get_logger(), "cannot log the robot description: %s", error.description.c_str());
  }
}

void RerunBridge::on_tf(const tf2_msgs::msg::TFMessage & msg, bool is_static) const
{
  for (const auto & transform : msg.transforms) {
    if (is_static) {
      recording_.log_static("ros/tf_static/" + transform.child_frame_id, to_rerun(transform));
      continue;
    }
    recording_.set_time_duration_secs(kTimeline, to_seconds(transform.header.stamp));
    recording_.log("ros/tf/" + transform.child_frame_id, to_rerun(transform));
  }
}

void RerunBridge::on_joint_states(const sensor_msgs::msg::JointState & msg)
{
  const rclcpp::Time stamp(msg.header.stamp);
  // A sim reset can move time backwards; then log right away.
  const bool too_soon = last_joint_state_log_ && stamp >= *last_joint_state_log_ &&
                        stamp - *last_joint_state_log_ < joint_state_log_period_;
  if (too_soon) {
    return;
  }
  last_joint_state_log_ = stamp;

  recording_.set_time_duration_secs(kTimeline, stamp.seconds());
  for (std::size_t i = 0; i < msg.name.size(); ++i) {
    const std::string prefix = "joint_states/" + msg.name[i];
    if (i < msg.position.size()) {
      recording_.log(prefix + "/position", rerun::Scalars(msg.position[i]));
    }
    if (i < msg.velocity.size()) {
      recording_.log(prefix + "/velocity", rerun::Scalars(msg.velocity[i]));
    }
    if (i < msg.effort.size()) {
      recording_.log(prefix + "/effort", rerun::Scalars(msg.effort[i]));
    }
  }
}

}  // namespace arm_sandbox_viz
```

`src/arm_sandbox_viz/src/main.cpp`:

```cpp
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "arm_sandbox_viz/rerun_bridge.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // The node is destroyed (and the recording flushed) before shutdown.
  rclcpp::spin(std::make_shared<arm_sandbox_viz::RerunBridge>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 4: Run the tests to see them pass**

Run: `make test`
Expected: build with no warnings, then `test_rerun_bridge` shows 3 passed (1 active, 2 post-shutdown).

If the `/robot/test_bot/visual_geometries/...` assertion fails, run `rerun rrd print <path printed in the test output>` and compare the entity names. Rerun's URDF loader decides them, so fix the test's expected path, not the bridge.

- [ ] **Step 5: Commit**

```bash
git add src/arm_sandbox_viz
git commit -m "feat(viz): C++ ROS 2 to Rerun bridge for robot description, TF and joint states"
```

### Task 8: Viewer in the sim launch, and the M0 check

**Files:**
- Modify: `src/arm_sandbox_bringup/launch/sim.launch.py`
- Modify: `src/arm_sandbox_bringup/package.xml`, `src/arm_sandbox_bringup/CMakeLists.txt`
- Test: `src/arm_sandbox_bringup/test/test_sim_recording.py`

**Interfaces:**
- Consumes: Task 7 bridge and config, Task 4 launch.
- Produces: launch args `rerun:=true|false` (default `true`: start `rerun --serve-web` and the bridge) and `rerun_save:=<file.rrd>` (record to a file instead of starting a viewer).

- [ ] **Step 1: Write the failing test**

Create `src/arm_sandbox_bringup/test/test_sim_recording.py`:

```python
"""Full stack: the running Panda sim shows up in Rerun (recorded to a file here, headless)."""

import subprocess
import tempfile
import time
import unittest
from pathlib import Path

import launch_testing
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager.test_utils import check_controllers_running
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest

RECORDING = Path(tempfile.mkdtemp(prefix="sim_recording_test_")) / "sim.rrd"
CONTROLLERS = ["joint_state_broadcaster", "arm_controller", "gripper_controller"]
STARTUP_TIMEOUT_S = 60.0
RECORD_TIME_S = 3.0


@pytest.mark.launch_test
def generate_test_description() -> LaunchDescription:
    sim_launch = Path(get_package_share_directory("arm_sandbox_bringup")) / "launch" / "sim.launch.py"
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_launch)),
        launch_arguments={"robot": "panda", "viewer": "false", "rerun": "true", "rerun_save": str(RECORDING)}.items(),
    )
    return LaunchDescription([sim, ReadyToTest()])


class TestRunSim(unittest.TestCase):
    def test_sim_runs_while_recording(self) -> None:
        rclpy.init()
        node = rclpy.create_node("test_sim_recording")
        try:
            check_controllers_running(node, CONTROLLERS, timeout=STARTUP_TIMEOUT_S)
            deadline = time.monotonic() + RECORD_TIME_S
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestRecording(unittest.TestCase):
    def test_recording_shows_the_moving_panda(self) -> None:
        self.assertTrue(RECORDING.is_file(), f"no recording at {RECORDING}")
        printed = subprocess.run(["rerun", "rrd", "print", str(RECORDING)], capture_output=True, text=True, check=True)
        entities = printed.stdout
        # Meshes from the URDF (robot name "panda"), moving joints from /tf, plots from /joint_states.
        self.assertRegex(entities, r"/robot/panda/visual_geometries/panda_link1/visual_0 - data columns: \[Asset3D")
        self.assertRegex(entities, r"/ros/tf/panda_link2 - data columns: \[Transform3D:child_frame")
        self.assertIn("/joint_states/panda_joint1/position", entities)
```

Register it in `src/arm_sandbox_bringup/CMakeLists.txt`:

```cmake
  add_launch_test(test/test_sim_recording.py TIMEOUT 180 ENV ROS_DOMAIN_ID=44)
```

Add `<exec_depend>arm_sandbox_viz</exec_depend>` to `src/arm_sandbox_bringup/package.xml` (keep the list alphabetical).

The existing bring-up tests don't pass `rerun`, so they'd start a viewer once the default is `true`. In `test_sim_bringup.py` and `test_arm_trajectory.py`, add `"rerun": "false"` to `launch_arguments`.

Run: `make test`
Expected: `test_sim_recording` FAILS. The launch rejects or ignores `rerun_save`, so there's no recording at the path.

- [ ] **Step 2: Add the viewer to `sim.launch.py`**

Add this helper below `spawner()`:

```python
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
        # Web viewer on http://localhost:9090, gRPC on 9876 (bridge config `grpc_url`).
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
```

In `launch_setup()`, change `return [` to `return rerun_actions(context) + [`.

In `generate_launch_description()`, add after the `gravcomp` argument:

```python
            DeclareLaunchArgument(
                "rerun", default_value="true", description="Show the sim in the Rerun web viewer (http://localhost:9090)"
            ),
            DeclareLaunchArgument(
                "rerun_save", default_value="", description="Record to this .rrd file instead of starting a viewer"
            ),
```

Extend the module docstring's usage line to:

```python
    ros2 launch arm_sandbox_bringup sim.launch.py robot:=panda viewer:=true gravcomp:=false rerun:=true
```

- [ ] **Step 3: Run the tests to see them pass**

Run: `make test`
Expected: 0 failures across all packages (`test_panda_mjcf`, `test_panda_urdf`, `test_scene`, `test_sim_bringup`, `test_arm_trajectory`, `test_rerun_bridge`, `test_sim_recording`).

- [ ] **Step 4: M0 check — the same sim in the browser and the native viewer** [container + host]

```bash
make start_sim   # [container] native MuJoCo viewer opens; rerun --serve-web starts
```

On the host, open `http://localhost:9090`. Expected: the colored Panda in the home pose, plus `joint_states/...` time series. In a second container terminal, send the trajectory goal from Task 5, Step 3.
Expected: the arm moves **in both viewers at the same time**, and the joint plots in Rerun follow. This is M0's "done when" (REQUIREMENTS §7).

Optional, for the milestone video: record the screen while doing this, or replay a recording with `make start_sim ARGS="rerun_save:=/tmp/m0.rrd"` and then `rerun /tmp/m0.rrd`.

Ctrl+C the launch.

- [ ] **Step 5: Commit**

```bash
git add src/arm_sandbox_bringup
git commit -m "feat(bringup): Rerun web viewer and bridge in sim.launch.py (rerun, rerun_save args)"
```

---

# Part 4 — Docs

### Task 9: Record the decisions, update the docs, add package READMEs

**Files:**
- Modify: `docs/REQUIREMENTS.md` (§10 decisions, status line)
- Modify: `docs/specs/2026-10-01-arm-sandbox-design.md` (Decisions, Simulation, Robot Description, Visualization)
- Modify: `docs/PROJECT_STRUCTURE.md`, `CLAUDE.md`
- Create: `src/arm_sandbox_description/README.md`, `src/arm_sandbox_sim/README.md`, `src/arm_sandbox_bringup/README.md`, `src/arm_sandbox_viz/README.md` (required by NFR-6 and CLAUDE.md "Code is considered ready when")

**Interfaces:**
- Consumes: everything above.
- Produces: docs that match the code. The next plan (M2, kinematics) is written from them.

- [ ] **Step 1: Decisions log** — in `docs/REQUIREMENTS.md` §10, append:

```markdown
| D15 | Gravity compensation is a launch argument, `gravcomp:=false` by default: controllers compensate gravity; `true` makes MuJoCo compensate the arm (like the real Panda's torque interface) | Keeps gravity compensation a skill to build (M3) while allowing real-robot behavior; only the robot's bodies are compensated, never task objects |
| D16 | Own Panda URDF xacro: upstream kinematics, Menagerie meshes, inertials, ranges and torque limits, plus `panda_hand_tcp` | The upstream URDF's `.dae` visuals don't load in Rerun and it has no TCP frame; one mesh source keeps URDF and MJCF in agreement (tested against Pinocchio) |
| D17 | Rerun C++ SDK built from the release bundle by `install_deps.sh` into `/opt/rerun_cpp_sdk`, same version as the viewer | No apt package; one dependency list, offline colcon builds |
```

Also change the header line `> Status: fourth draft, 2026-10-01.` to `> Status: fourth draft, 2026-10-01; D15–D17 added 2026-10-02 (Plan 02).`

- [ ] **Step 2: Design spec**

In `docs/specs/2026-10-01-arm-sandbox-design.md`:

1. In "Decisions", change the intro line to `Decisions from the requirements (D1–D12, short form; D13–D17 are covered below):`, and after the D14 bullet add:

```markdown
- **Gravity compensation is switchable, off by default, D15.** `sim.launch.py gravcomp:=true` sets MuJoCo `gravcomp` on the robot's bodies (scene composed at launch by `arm_sandbox_sim.scene`), which mirrors the real Panda, whose torque interface compensates gravity. With the default `false`, JTC's integral term (M1) and the custom controllers' g(q) from Pinocchio (M3) carry the load.
- **Own URDF xacro, D16.** `panda.urdf.xacro` keeps upstream kinematics and takes meshes, inertials, ranges and torque limits from the Menagerie MJCF. `test_panda_urdf.py` checks FK (vs Pinocchio), limits and mass against the MJCF.
- **Startup order.** Without gravity compensation the arm sags until a controller is active, and JTC holds the pose it sees on activation. `sim.launch.py` loads the arm and gripper controllers inactive. Then `activate_at_home` calls `/mujoco_ros2_control_node/reset_world` (`home` keyframe) and `switch_controller` back to back. A spawner started after the reset would leave about 2 s for the arm to fall. The sim can't be paused for this either, because the control loop sleeps on sim time.
```

2. In "Simulation", replace the "Models." bullet with:

```markdown
- **Models.** URDF/xacro (ROS side) written for this project (D16) from upstream kinematics and the MJCF's meshes, inertials and limits. MJCF from MuJoCo Menagerie, renamed to the URDF names and changed to torque actuators. A unit test checks that URDF FK (Pinocchio) and MJCF FK agree on random configurations (the main risk of keeping two model files).
```

and replace the "Scene." bullet with:

```markdown
- **Scene.** For now `arm_sandbox_description/<robot>/mjcf/scene.xml` (floor, lights). `arm_sandbox_sim.scene.compose_scene()` turns it into one file at launch (absolute asset paths, optional gravcomp, D15). The table, cameras and per-task objects are added to the composition in M4/M5.
```

3. In "Robot Description and Config", replace the tree and the YAML block with:

````markdown
```
arm_sandbox_description/panda/
  urdf/panda.urdf.xacro          # selected by robot:=panda; args mujoco_model, headless
  urdf/panda.ros2_control.xacro  # <ros2_control> block (MujocoSystemInterface)
  mjcf/panda.xml                 # torque-actuated, URDF names, `home` keyframe
  mjcf/scene.xml
  config/robot.yaml
arm_sandbox_bringup/config/panda_controllers.yaml
```

`robot.yaml` is the single source for generic code (cameras are added in M5):

```yaml
arm_joints: [panda_joint1, ..., panda_joint7]
base_frame: panda_link0
ee_frame: panda_hand_tcp
gripper: {type: parallel, joint: panda_finger_joint1, max_width: 0.08}
home: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
```
````

4. In "Visualization", after the second bullet ("It logs the URDF once…"), add:

```markdown
- Entity layout (Rerun 0.38): the URDF via Rerun's URDF loader under `/robot/<robot>/…` (one coordinate frame per link, zero-pose joints on `/tf_static`); each `/tf` transform as a `Transform3D` with parent/child frames on `ros/tf/<child>` (timeline `sim_time`), which moves the robot; `/tf_static` on `ros/tf_static/<child>` (static); `/joint_states` as `joint_states/<joint>/{position,velocity,effort}`. The Rerun C++ SDK comes from `install_deps.sh` (D17). `sim.launch.py rerun:=true` (default) starts `rerun --serve-web` and the bridge; `rerun_save:=<file.rrd>` records instead.
```

- [ ] **Step 3: `docs/PROJECT_STRUCTURE.md`**

- Status note at the top: replace with `> **Status: M0 and M1 done (Plan 02).** These exist: the dev environment, \`tests/env/\`, \`arm_sandbox_description\` (Panda MJCF, URDF xacro, robot.yaml), \`arm_sandbox_sim\` (scene composition only), \`arm_sandbox_bringup\` (sim launch, controllers), and \`arm_sandbox_viz\` (Rerun bridge). Everything else is the target layout from \`docs/REQUIREMENTS.md\` (§6). Update it as packages are created.`
- Packages table, `arm_sandbox_sim` row, Lang column: `Python (C++ later)`. Purpose: `Scene composition at launch (gravcomp, D15); later reset/randomize services, ground truth`.
- Project tree under `panda/`: remove the `meshes/` line (meshes live in `mjcf/assets/`).
- Commands table: change the `make start_sim` row (added in Task 5) to `| \`make start_sim\` | container | Run the sim: native viewer + Rerun (\`ARGS="viewer:=false gravcomp:=true rerun:=false"\`) |`, and change the note below the table to `\`make lint\` is added by a later plan.`
- Configuration → Robot selection: add `- Launch arguments of \`sim.launch.py\`: \`robot\`, \`viewer\`, \`gravcomp\`, \`rerun\`, \`rerun_save\``.

- [ ] **Step 4: `CLAUDE.md`**

- Status line: `> **Status:** M0 and M1 done (Plans 01–02): dev environment, \`arm_sandbox_description\` (MJCF, URDF, robot.yaml), \`arm_sandbox_sim\` (scene composition), \`arm_sandbox_bringup\` (\`make start_sim\`), \`arm_sandbox_viz\` (Rerun bridge). Next: M2 (kinematics). Every other package and command below is still the **plan** from \`docs/REQUIREMENTS.md\`. Update this file as they become real.`
- "Inside the dev container" list: change the `make start_sim` line (added in Task 5) to `- \`make start_sim\` - run the sim (native viewer + Rerun at http://localhost:9090); \`make start_sim ARGS="viewer:=false gravcomp:=true"\``, and change `- \`make lint\` - added by later plans` to `- \`make lint\` - added by a later plan`.
- "Development mode" → Rerun bullet: `- **Rerun web viewer**: http://localhost:9090, started by \`make start_sim\` (\`rerun:=false\` to skip)`.

- [ ] **Step 5: Package READMEs** (short, theory first, NFR-6)

`src/arm_sandbox_description/README.md`:

```markdown
# arm_sandbox_description

All robot-specific files, one folder per robot (`robot:=<folder>`). Nothing outside this package and
`arm_sandbox_bringup/config/<robot>_controllers.yaml` names a joint or link.

## Two models of one robot

ROS tools (robot_state_publisher, MoveIt, Pinocchio) read the URDF; MuJoCo reads the MJCF. Keeping
two files is the price of using both, and the risk is that they drift apart. Here they share
meshes, inertials and limits, use the same link/joint names, and `test/test_panda_urdf.py` compares
forward kinematics (Pinocchio on the URDF vs MuJoCo on the MJCF) on random configurations.

## Torque actuation

The MJCF arm uses `motor` actuators: the command is joint torque, as on the real Panda's torque
interface. Position control is a controller's job (JTC's PID in M1, OSC/impedance in M3). The
gripper is a position servo on a tendon that averages both fingers.

## Files (panda)

- `urdf/panda.urdf.xacro`, `urdf/panda.ros2_control.xacro` - URDF and its `ros2_control` block
- `mjcf/panda.xml`, `mjcf/scene.xml` - MuJoCo model (modified Menagerie, see its header)
- `config/robot.yaml` - joint names, base/EE frames, gripper, home pose for generic code
```

`src/arm_sandbox_sim/README.md`:

```markdown
# arm_sandbox_sim

What arm-sandbox adds on top of `mujoco_ros2_control` (D13). For now: composing the MuJoCo scene at
launch (`arm_sandbox_sim.scene`). Later: seeded randomized reset and ground truth (M4).

## Gravity compensation (D15)

A torque-controlled arm falls unless something cancels gravity, τ = g(q). Either the controller
computes g(q) (default here, `gravcomp:=false`), or the simulator applies it as a passive force
(`gravcomp:=true`, MuJoCo's body `gravcomp`), which is what the real Panda does inside its torque
interface. Only the robot's bodies are compensated, so task objects keep their weight.

The module is ROS-free so the Phase B Gymnasium environment loads exactly the same model.
```

`src/arm_sandbox_bringup/README.md`:

```markdown
# arm_sandbox_bringup

Top-level launch files and per-robot controller configs.

    make start_sim                                  # = ros2 launch arm_sandbox_bringup sim.launch.py
    make start_sim ARGS="viewer:=false gravcomp:=true rerun:=false"

Arguments: `robot` (default `panda`), `viewer` (native MuJoCo window), `gravcomp` (D15), `rerun`
(web viewer at http://localhost:9090), `rerun_save` (record to a `.rrd` file instead).

## Control

`arm_controller` is the stock Joint Trajectory Controller on the `effort` interface. It interpolates
the trajectory and closes a PID loop per joint, τ = Kp·e + Ki·∫e + Kd·ė. Without simulator gravity
compensation, the integral term carries the gravity load, so tracking settles within a second rather
than instantly; the M3 controllers add g(q) explicitly. Gains are in `config/panda_controllers.yaml`.

## Startup order

JTC holds the pose it sees when it activates, and an unpowered arm falls within a fraction of a
second. The launch therefore loads the arm and gripper controllers inactive, and `activate_at_home`
resets the sim to the `home` keyframe and activates them in back-to-back service calls.
```

`src/arm_sandbox_viz/README.md`:

```markdown
# arm_sandbox_viz

`rerun_bridge` (C++) shows the running sim in the browser through Rerun. It subscribes only to
standard topics (FR-14a), so any viewer can replace it (M12) without touching the rest.

## How the robot moves in Rerun

Rerun 0.38 models transforms like TF: named coordinate frames connected by parent→child transforms.
The URDF loader puts each link's meshes in a frame named after the link and logs the joint transforms
at zero. The bridge then logs every `/tf` transform with the same parent/child frame names, timed by
the message stamp on the `sim_time` timeline; the newest transform wins, so the meshes follow the
robot. `/joint_states` become time-series plots.

Config: `config/rerun_bridge.yaml` (`grpc_url` to stream, or `save_path` to record).
The Rerun C++ SDK is installed by `scripts/install_deps.sh` (D17).
```

- [ ] **Step 6: Check, then commit**

```bash
grep -n "D15\|D16\|D17" docs/REQUIREMENTS.md docs/specs/2026-10-01-arm-sandbox-design.md   # Expected: hits in both
grep -n "make start_sim" CLAUDE.md docs/PROJECT_STRUCTURE.md   # Expected: hits in both
make test    # Expected: still 0 failures
git add CLAUDE.md docs src/*/README.md
git commit -m "docs: record D15-D17, document sim bring-up and viewer, add package READMEs"
git push -u origin feat/m1-sim-viewer
```

---

## Done when

- `make smoke` and `make test` pass in the dev container, with no compiler warnings.
- `make start_sim` shows the Panda in the native MuJoCo viewer **and** at `http://localhost:9090` at the same time, and a `FollowJointTrajectory` goal moves it in both (**M0**).
- The arm follows a joint trajectory through `ros2_control` with `gravcomp:=false` and `true`, and the robot is selected by `robot:=panda` (**M1**).
- D15–D17 are recorded; CLAUDE.md, PROJECT_STRUCTURE.md, and the spec match the code. Next: Plan 03 (M2, hand-written kinematics checked against Pinocchio, reach task).
