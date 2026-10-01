"""Toolchain smoke tests: prove the dev image has what Phase A needs.

Run inside the dev container: `make smoke`.
These are the acceptance gate for docker/Dockerfile (Plan 01, Task 3).
"""

import os
import shutil
import subprocess

import pytest

RERUN_VENV_PYTHON = "/opt/rerun/bin/python"


def test_ros_distro_is_jazzy() -> None:
    assert os.environ.get("ROS_DISTRO") == "jazzy"


def test_rclpy_imports() -> None:
    import rclpy  # noqa: F401  (fails if pip broke the ROS Python environment)


def test_ros_python_keeps_numpy_1() -> None:
    # Jazzy's compiled Python bindings (e.g. pinocchio/eigenpy) are built against NumPy 1.x.
    import numpy

    assert numpy.__version__.startswith("1."), numpy.__version__


@pytest.mark.parametrize(
    "package",
    [
        "controller_manager",
        "joint_trajectory_controller",
        "moveit_ros_move_group",
        "moveit_resources_panda_moveit_config",
        "robot_state_publisher",
        "xacro",
        "behaviortree_cpp",
        "vision_msgs",
        "rviz2",
    ],
)
def test_ros_package_installed(package: str) -> None:
    result = subprocess.run(
        ["ros2", "pkg", "prefix", package], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr


def test_pinocchio_imports() -> None:
    import pinocchio  # noqa: F401


def test_mujoco_simulates_a_falling_body() -> None:
    import mujoco

    model = mujoco.MjModel.from_xml_string(
        "<mujoco><worldbody><body pos='0 0 1'><freejoint/><geom size='0.1'/></body>"
        "</worldbody></mujoco>"
    )
    data = mujoco.MjData(model)
    start_height = data.qpos[2]
    for _ in range(100):
        mujoco.mj_step(model, data)
    assert data.qpos[2] < start_height


def test_mujoco_offscreen_render() -> None:
    # The backend comes from MUJOCO_GL: egl (GPU) or osmesa (CPU / CI fallback).
    import mujoco

    model = mujoco.MjModel.from_xml_string(
        "<mujoco><worldbody><light pos='0 0 3'/>"
        "<geom type='box' size='0.2 0.2 0.2' rgba='1 0 0 1'/></worldbody></mujoco>"
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, height=120, width=160)
    try:
        renderer.update_scene(data)
        image = renderer.render()
    finally:
        renderer.close()
    assert image.shape == (120, 160, 3)
    assert image.max() > 0, "rendered image is completely black"


def test_rerun_cli_available() -> None:
    assert shutil.which("rerun") is not None


def test_rerun_sdk_in_its_own_venv() -> None:
    # rerun-sdk needs NumPy 2, so it lives in /opt/rerun, away from ROS Python.
    result = subprocess.run(
        [RERUN_VENV_PYTHON, "-c", "import rerun"], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr


def test_claude_cli_available() -> None:
    assert shutil.which("claude") is not None


def test_gpu_visible() -> None:
    if shutil.which("nvidia-smi") is None:
        pytest.skip("no NVIDIA GPU in this environment (e.g. CI)")
    result = subprocess.run(["nvidia-smi", "-L"], capture_output=True, text=True, check=False)
    assert result.returncode == 0 and "GPU" in result.stdout, result.stderr
