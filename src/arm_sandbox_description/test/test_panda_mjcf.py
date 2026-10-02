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
