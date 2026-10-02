"""M1 acceptance: the arm follows a joint trajectory through ros2_control, and the gripper moves.

Runs once per gravity-compensation mode (D15): controllers compensate (false) or MuJoCo does (true).
"""

import time
import unittest
from pathlib import Path

import launch_testing
import pytest
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from controller_manager.test_utils import check_controllers_running
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ROBOT = "panda"
CONTROLLERS = ["joint_state_broadcaster", "arm_controller", "gripper_controller"]
STARTUP_TIMEOUT_S = 60.0
ACTION_TIMEOUT_S = 30.0
MOVE_DURATION_S = 2
# Offset from home for the test goal: every joint moves, and all stay inside their limits.
GOAL_OFFSET_RAD = [0.3, 0.3, -0.3, 0.3, 0.3, 0.3, 0.3]
FINAL_TOLERANCE_RAD = 0.02  # the JTC `goal` tolerance in panda_controllers.yaml
# Partly closed, short of finger contact (the fingertip pads touch near 0).
GRIPPER_PARTLY_CLOSED_M = 0.01
GRIPPER_TOLERANCE_M = 0.003
GRIPPER_MAX_EFFORT_N = 50.0


def robot_config() -> dict:
    path = Path(get_package_share_directory("arm_sandbox_description")) / ROBOT / "config" / "robot.yaml"
    return yaml.safe_load(path.read_text())


@pytest.mark.launch_test
@launch_testing.parametrize("gravcomp", ["false", "true"])
def generate_test_description(gravcomp: str) -> LaunchDescription:
    sim_launch = Path(get_package_share_directory("arm_sandbox_bringup")) / "launch" / "sim.launch.py"
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_launch)),
        launch_arguments={"robot": ROBOT, "viewer": "false", "rerun": "false", "gravcomp": gravcomp}.items(),
    )
    return LaunchDescription([sim, ReadyToTest()])


class TestArmTrajectory(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.node = rclpy.create_node("test_arm_trajectory")
        check_controllers_running(self.node, CONTROLLERS, timeout=STARTUP_TIMEOUT_S)

    def tearDown(self) -> None:
        self.node.destroy_node()

    def latest_positions(self, joints: list[str]) -> list[float]:
        received = []
        subscription = self.node.create_subscription(
            JointState,
            "/joint_states",
            lambda msg: received.append(msg) if set(joints) <= set(msg.name) else None,
            qos_profile_sensor_data,
        )
        deadline = time.monotonic() + STARTUP_TIMEOUT_S
        try:
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        finally:
            self.node.destroy_subscription(subscription)
        self.assertTrue(received, "no /joint_states with the requested joints")
        positions = dict(zip(received[0].name, received[0].position))
        return [positions[joint] for joint in joints]

    def send_goal(self, client: ActionClient, goal):
        self.assertTrue(client.wait_for_server(timeout_sec=STARTUP_TIMEOUT_S), "action server not available")
        goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, goal_future, timeout_sec=ACTION_TIMEOUT_S)
        handle = goal_future.result()
        self.assertTrue(handle is not None and handle.accepted, "goal rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=ACTION_TIMEOUT_S)
        response = result_future.result()
        self.assertIsNotNone(response, "no result before timeout")
        return response

    def test_arm_follows_joint_trajectory(self) -> None:
        config = robot_config()
        joints = config["arm_joints"]
        target = [home + offset for home, offset in zip(config["home"], GOAL_OFFSET_RAD)]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        goal.trajectory.points = [
            JointTrajectoryPoint(
                positions=target, velocities=[0.0] * len(joints), time_from_start=Duration(sec=MOVE_DURATION_S)
            )
        ]
        client = ActionClient(self.node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")

        response = self.send_goal(client, goal)

        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(response.result.error_code, FollowJointTrajectory.Result.SUCCESSFUL, response.result.error_string)
        errors = [abs(actual - wanted) for actual, wanted in zip(self.latest_positions(joints), target)]
        self.assertLess(max(errors), FINAL_TOLERANCE_RAD, dict(zip(joints, errors)))

    def test_gripper_closes_and_opens(self) -> None:
        gripper = robot_config()["gripper"]
        client = ActionClient(self.node, GripperCommand, "/gripper_controller/gripper_cmd")
        for target in (GRIPPER_PARTLY_CLOSED_M, gripper["max_width"] / 2):
            goal = GripperCommand.Goal()
            goal.command.position = target
            goal.command.max_effort = GRIPPER_MAX_EFFORT_N

            response = self.send_goal(client, goal)

            self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
            self.assertAlmostEqual(self.latest_positions([gripper["joint"]])[0], target, delta=GRIPPER_TOLERANCE_M)
