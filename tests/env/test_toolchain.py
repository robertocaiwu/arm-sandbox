"""Toolchain smoke tests: prove the dev image has what Phase A needs.

Run inside the dev container: `make smoke`.
These are the acceptance gate for docker/Dockerfile (Plan 01, Task 3).
"""

import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest

# Headless by default; the compose file / docker/.env can override (e.g. osmesa).
os.environ.setdefault("MUJOCO_GL", "egl")

RERUN_VENV_PYTHON = "/opt/rerun/bin/python"
MUJOCO_VENDOR_LIB = "/opt/ros/humble/opt/mujoco_vendor/lib"
RERUN_CPP_SDK_PREFIX = Path("/opt/rerun_cpp_sdk")
RERUN_CPP_SDK_VERSION_FILE = RERUN_CPP_SDK_PREFIX / "lib" / "cmake" / "rerun_sdk" / "rerun_sdkConfigVersion.cmake"


def test_ros_distro_is_humble() -> None:
    assert os.environ.get("ROS_DISTRO") == "humble"


def test_rclpy_imports() -> None:
    import rclpy  # noqa: F401  (fails if pip broke the ROS Python environment)


def test_ros_python_keeps_numpy_1() -> None:
    # Humble's compiled Python bindings (e.g. pinocchio/eigenpy) are built against NumPy 1.x.
    import numpy

    assert numpy.__version__.startswith("1."), numpy.__version__


@pytest.mark.parametrize(
    "package",
    [
        "controller_manager",
        "joint_trajectory_controller",
        "mujoco_ros2_control",
        "mujoco_ros2_control_plugins",
        "moveit_ros_move_group",
        "moveit_servo",
        "gripper_controllers",
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


def test_pip_mujoco_matches_ros_sim_mujoco() -> None:
    # The Gymnasium path (pip mujoco) and the ROS 2 sim (mujoco_vendor) must run the same
    # engine version (decision D14). mujoco_vendor ships libmujoco.so.<version>.
    import mujoco

    vendor_libs = [p for p in os.listdir(MUJOCO_VENDOR_LIB) if p.startswith("libmujoco.so.")]
    assert vendor_libs, f"no libmujoco.so.<version> in {MUJOCO_VENDOR_LIB}"
    vendor_version = vendor_libs[0].removeprefix("libmujoco.so.")
    assert mujoco.__version__ == vendor_version


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


def test_gpu_visible() -> None:
    if shutil.which("nvidia-smi") is None:
        pytest.skip("no NVIDIA GPU in this environment (e.g. CI)")
    result = subprocess.run(["nvidia-smi", "-L"], capture_output=True, text=True, check=False)
    assert result.returncode == 0 and "GPU" in result.stdout, result.stderr


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
