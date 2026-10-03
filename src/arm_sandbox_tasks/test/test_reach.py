"""M2 acceptance: with the sim running, reach_runner reaches every target of the reach task.

Runs once per gravity-compensation mode (D15), like test_arm_trajectory.py.
"""

import unittest
from pathlib import Path

import launch_testing
import launch_testing.asserts
import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest

# Startup (~10 s) plus every target's move and settling.
REACH_TIMEOUT_S = 180.0


def launch_file(package: str, name: str) -> str:
    return str(Path(get_package_share_directory(package)) / "launch" / name)


@pytest.mark.launch_test
@launch_testing.parametrize("gravcomp", ["false", "true"])
def generate_test_description(gravcomp: str):
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file("arm_sandbox_bringup", "sim.launch.py")),
        launch_arguments={"robot": "panda", "viewer": "false", "rerun": "false", "gravcomp": gravcomp}.items(),
    )
    reach = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file("arm_sandbox_tasks", "reach.launch.py")),
        launch_arguments={"robot": "panda", "task": "reach"}.items(),
    )
    return LaunchDescription([sim, reach, ReadyToTest()])


class TestReach(unittest.TestCase):
    def test_reach_runner_finishes(self, proc_info) -> None:
        proc_info.assertWaitForShutdown(process="reach_runner", timeout=REACH_TIMEOUT_S)


@launch_testing.post_shutdown_test()
class TestReachResult(unittest.TestCase):
    def test_every_target_reached(self, proc_info, proc_output) -> None:
        # reach_runner exits 0 only if all targets were reached.
        launch_testing.asserts.assertExitCodes(proc_info, process="reach_runner")
        # ROS loggers write to stderr.
        launch_testing.asserts.assertInStderr(proc_output, "targets reached", "reach_runner")
