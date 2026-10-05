"""M3: the Cartesian impedance controller behaves like a spring in the running sim.

A steady push F on the hand (the sim's external-wrench plugin) displaces the end effector by about
F / k, k being the controller's translational stiffness; after the push it returns to its target.
"""

import time
import unittest
from pathlib import Path

import numpy as np
import pytest
import rclpy
import tf2_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from controller_manager.test_utils import check_controllers_running
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from mujoco_ros2_control_msgs.msg import ExternalWrench
from mujoco_ros2_control_msgs.srv import ApplyExternalWrench

ROBOT = "panda"
CONTROLLER = "cartesian_impedance_controller"
CONTROLLERS = ["joint_state_broadcaster", CONTROLLER, "gripper_controller"]
WRENCH_SERVICE = "/external_wrench/apply_wrench"
PUSHED_BODY = "panda_hand"
PUSH_FORCE_N = 10.0
PUSH_DURATION_S = 3
STARTUP_TIMEOUT_S = 60.0
SETTLE_TIME_S = 2.0  # the push is measured this long after it starts, the return this long after it ends
DEFLECTION_TOLERANCE = 0.2  # 20 % of F / k
RETURN_TOLERANCE_M = 0.002


def controller_parameters() -> dict:
    path = Path(get_package_share_directory("arm_sandbox_bringup")) / "config" / f"{ROBOT}_controllers.yaml"
    return yaml.safe_load(path.read_text())[CONTROLLER]["ros__parameters"]


def robot_config() -> dict:
    path = Path(get_package_share_directory("arm_sandbox_description")) / ROBOT / "config" / "robot.yaml"
    return yaml.safe_load(path.read_text())


@pytest.mark.launch_test
def generate_test_description() -> LaunchDescription:
    sim_launch = Path(get_package_share_directory("arm_sandbox_bringup")) / "launch" / "sim.launch.py"
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_launch)),
        launch_arguments={
            "robot": ROBOT,
            "viewer": "false",
            "rerun": "false",
            "arm_controller": CONTROLLER,
            "external_wrench": "true",
        }.items(),
    )
    return LaunchDescription([sim, ReadyToTest()])


class TestImpedanceCompliance(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.node = rclpy.create_node("test_impedance_compliance")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def tearDown(self) -> None:
        self.node.destroy_node()

    def spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def ee_position(self) -> np.ndarray:
        config = robot_config()
        transform = self.tf_buffer.lookup_transform(config["base_frame"], config["ee_frame"], rclpy.time.Time())
        t = transform.transform.translation
        return np.array([t.x, t.y, t.z])

    def test_push_deflects_like_a_spring_and_releases(self) -> None:
        check_controllers_running(self.node, CONTROLLERS, timeout=STARTUP_TIMEOUT_S)
        client = self.node.create_client(ApplyExternalWrench, WRENCH_SERVICE)
        self.assertTrue(client.wait_for_service(timeout_sec=STARTUP_TIMEOUT_S), f"{WRENCH_SERVICE} not available")
        self.spin_for(SETTLE_TIME_S)
        rest = self.ee_position()

        push = ExternalWrench()
        push.wrench.header.frame_id = PUSHED_BODY
        push.wrench.wrench.force.y = PUSH_FORCE_N  # in the body frame
        push.duration = Duration(sec=PUSH_DURATION_S)
        request = ApplyExternalWrench.Request()
        request.wrenches.external_wrenches = [push]
        # The service answers only when the push is over, so call it asynchronously.
        future = client.call_async(request)
        self.spin_for(SETTLE_TIME_S)
        pushed = self.ee_position()

        rclpy.spin_until_future_complete(self.node, future, timeout_sec=PUSH_DURATION_S + STARTUP_TIMEOUT_S)
        self.assertTrue(future.result() is not None and future.result().success, "push was not applied")
        self.spin_for(SETTLE_TIME_S)
        released = self.ee_position()

        expected = PUSH_FORCE_N / controller_parameters()["translational_stiffness"]
        deflection = np.linalg.norm(pushed - rest)
        self.assertAlmostEqual(deflection, expected, delta=DEFLECTION_TOLERANCE * expected, msg=f"deflection {deflection:.4f} m")
        self.assertLess(np.linalg.norm(released - rest), RETURN_TOLERANCE_M)
