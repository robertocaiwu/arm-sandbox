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
        launch_arguments={"robot": ROBOT, "viewer": "false", "rerun": "false", "gravcomp": "false"}.items(),
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
