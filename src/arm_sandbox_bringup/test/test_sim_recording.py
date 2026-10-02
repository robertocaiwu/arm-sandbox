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
