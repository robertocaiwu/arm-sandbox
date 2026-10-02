"""The Rerun bridge logs the URDF, TF and joint states from standard topics (FR-12, FR-14a).

The bridge records to a file; after shutdown, `rerun rrd print` shows which entities it logged.
"""

import subprocess
import tempfile
import time
import unittest
from pathlib import Path

import launch_testing
import launch_testing.asserts
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

RECORDING = Path(tempfile.mkdtemp(prefix="rerun_bridge_test_")) / "bridge.rrd"
PUBLISH_DURATION_S = 3.0
PUBLISH_PERIOD_S = 0.05
ROBOT_DESCRIPTION = """<?xml version="1.0"?>
<robot name="test_bot">
  <link name="base_link"><visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual></link>
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/><child link="arm_link"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <link name="arm_link"><visual><geometry><box size="0.2 0.05 0.05"/></geometry></visual></link>
</robot>
"""
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


@pytest.mark.launch_test
def generate_test_description() -> LaunchDescription:
    bridge = Node(
        package="arm_sandbox_viz",
        executable="rerun_bridge",
        name="rerun_bridge",
        parameters=[{"save_path": str(RECORDING), "grpc_url": "", "joint_state_log_period_s": 0.0}],
        output="screen",
    )
    return LaunchDescription([bridge, ReadyToTest()])


def transform(parent: str, child: str, stamp) -> TransformStamped:
    msg = TransformStamped()
    msg.header.frame_id = parent
    msg.header.stamp = stamp
    msg.child_frame_id = child
    msg.transform.rotation.w = 1.0
    return msg


class TestPublish(unittest.TestCase):
    def test_publish_standard_topics(self) -> None:
        rclpy.init()
        node = rclpy.create_node("rerun_bridge_test_publisher")
        try:
            description = node.create_publisher(String, "/robot_description", LATCHED)
            tf_static = node.create_publisher(TFMessage, "/tf_static", LATCHED)
            tf = node.create_publisher(TFMessage, "/tf", 10)
            joint_states = node.create_publisher(JointState, "/joint_states", 10)

            description.publish(String(data=ROBOT_DESCRIPTION))
            tf_static.publish(TFMessage(transforms=[transform("world", "base_link", node.get_clock().now().to_msg())]))
            deadline = time.monotonic() + PUBLISH_DURATION_S
            while time.monotonic() < deadline:
                stamp = node.get_clock().now().to_msg()
                tf.publish(TFMessage(transforms=[transform("base_link", "arm_link", stamp)]))
                state = JointState(name=["arm_joint"], position=[0.5], velocity=[0.0], effort=[0.0])
                state.header.stamp = stamp
                joint_states.publish(state)
                rclpy.spin_once(node, timeout_sec=PUBLISH_PERIOD_S)
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestRecording(unittest.TestCase):
    def test_bridge_exited_cleanly(self, proc_info) -> None:
        launch_testing.asserts.assertExitCodes(proc_info)

    def test_recording_has_robot_tf_and_joint_states(self) -> None:
        self.assertTrue(RECORDING.is_file(), f"no recording at {RECORDING}")
        printed = subprocess.run(["rerun", "rrd", "print", str(RECORDING)], capture_output=True, text=True, check=True)
        entities = printed.stdout
        self.assertIn("/robot/test_bot/visual_geometries/arm_link", entities)
        self.assertRegex(entities, r"/ros/tf/arm_link - data columns: \[Transform3D:child_frame")
        self.assertRegex(entities, r"/ros/tf_static/base_link - data columns: \[Transform3D:child_frame")
        self.assertIn("/joint_states/arm_joint/position - data columns: [Scalars:scalars]", entities)
