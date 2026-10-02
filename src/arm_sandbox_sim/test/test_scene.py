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
