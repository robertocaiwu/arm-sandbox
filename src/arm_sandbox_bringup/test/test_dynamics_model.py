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
