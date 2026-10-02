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
