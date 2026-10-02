"""The vendored MuJoCo Menagerie Panda loads and simulates."""

from pathlib import Path

import mujoco
import numpy as np

MJCF_DIR = Path(__file__).resolve().parents[1] / "panda" / "mjcf"


def load(file_name: str) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(MJCF_DIR / file_name))


def test_panda_has_seven_arm_and_two_finger_joints() -> None:
    model = load("panda.xml")
    names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)}
    assert {f"joint{i}" for i in range(1, 8)} <= names
    assert {"finger_joint1", "finger_joint2"} <= names


def test_scene_simulates_one_second_without_nan() -> None:
    model = load("scene.xml")
    data = mujoco.MjData(model)
    steps = int(1.0 / model.opt.timestep)
    for _ in range(steps):
        mujoco.mj_step(model, data)
    assert np.all(np.isfinite(data.qpos))
