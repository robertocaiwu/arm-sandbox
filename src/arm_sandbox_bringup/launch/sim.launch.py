"""Bring up one robot in MuJoCo behind ros2_control (milestone M1).

    ros2 launch arm_sandbox_bringup sim.launch.py robot:=panda viewer:=true gravcomp:=false

Startup order: with a torque-controlled arm and no gravity compensation, the arm sags from the
moment physics starts, because zero torque is commanded until a controller is active, and JTC holds
whatever pose it sees when it activates. So the arm and gripper controllers are loaded inactive, and
activate_at_home resets the sim to the `home` keyframe and activates them in back-to-back service
calls (a spawner started after the reset would leave ~2 s for the arm to fall).
"""

from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from arm_sandbox_sim.scene import write_composed_scene

# mujoco_ros2_control 0.1.2 (D13) serves its services from this node.
RESET_WORLD_SERVICE = "/mujoco_ros2_control_node/reset_world"
SWITCH_CONTROLLER_SERVICE = "/controller_manager/switch_controller"
# Every robot's MJCF defines this keyframe, equal to `home` in its robot.yaml.
HOME_KEYFRAME = "home"
ARM_AND_GRIPPER_CONTROLLERS = ["arm_controller", "gripper_controller"]
# Startup service calls give up after this long (sim start with meshes takes a few seconds).
STARTUP_SERVICE_TIMEOUT_S = 30.0


def is_true(context: LaunchContext, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).lower() == "true"


def require_file(path: Path, what: str) -> Path:
    if not path.is_file():
        raise FileNotFoundError(f"{what} not found: {path}")
    return path


def spawner(*arguments: str) -> Node:
    return Node(package="controller_manager", executable="spawner", arguments=list(arguments), output="screen")


def launch_setup(context: LaunchContext) -> list:
    robot = LaunchConfiguration("robot").perform(context)
    description_dir = Path(get_package_share_directory("arm_sandbox_description")) / robot
    bringup_dir = Path(get_package_share_directory("arm_sandbox_bringup"))

    robot_config = yaml.safe_load(require_file(description_dir / "config" / "robot.yaml", "robot config").read_text())
    controllers_file = require_file(bringup_dir / "config" / f"{robot}_controllers.yaml", "controllers config")
    scene = write_composed_scene(
        require_file(description_dir / "mjcf" / "scene.xml", "MuJoCo scene"),
        robot_root=robot_config["base_frame"],
        gravcomp=is_true(context, "gravcomp"),
    )
    robot_description = xacro.process_file(
        str(require_file(description_dir / "urdf" / f"{robot}.urdf.xacro", "URDF")),
        mappings={"mujoco_model": str(scene), "headless": str(not is_true(context, "viewer")).lower()},
    ).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": ParameterValue(robot_description, value_type=str), "use_sim_time": True}],
        output="screen",
    )
    control_node = Node(
        # mujoco_ros2_control ships its own ros2_control_node (same parameters as upstream's).
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        parameters=[{"use_sim_time": True}, str(controllers_file)],
        # Humble's controller_manager reads the URDF from ~/robot_description.
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
        on_exit=Shutdown(),
    )
    joint_state_broadcaster = spawner("joint_state_broadcaster")
    # Loaded and configured, but not active: activate_at_home activates them right after the reset.
    arm_and_gripper = spawner(*ARM_AND_GRIPPER_CONTROLLERS, "--inactive")
    activate_at_home = Node(
        package="arm_sandbox_bringup",
        executable="activate_at_home",
        parameters=[
            {
                "reset_service": RESET_WORLD_SERVICE,
                "keyframe": HOME_KEYFRAME,
                "switch_service": SWITCH_CONTROLLER_SERVICE,
                "controllers": ARM_AND_GRIPPER_CONTROLLERS,
                "timeout_s": STARTUP_SERVICE_TIMEOUT_S,
            }
        ],
        output="screen",
    )
    return [
        robot_state_publisher,
        control_node,
        joint_state_broadcaster,
        arm_and_gripper,
        RegisterEventHandler(OnProcessExit(target_action=arm_and_gripper, on_exit=[activate_at_home])),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="panda", description="Robot folder in arm_sandbox_description"),
            DeclareLaunchArgument("viewer", default_value="true", description="Open the native MuJoCo viewer"),
            DeclareLaunchArgument(
                "gravcomp",
                default_value="false",
                description="MuJoCo compensates the arm's gravity (like the real Panda); false: controllers do",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
