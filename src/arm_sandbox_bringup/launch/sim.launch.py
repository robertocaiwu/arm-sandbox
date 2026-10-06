"""Bring up one robot in MuJoCo behind ros2_control (milestone M1).

    ros2 launch arm_sandbox_bringup sim.launch.py robot:=panda viewer:=true gravcomp:=false rerun:=true

Startup order: with a torque-controlled arm and no gravity compensation, the arm sags from the
moment physics starts, because zero torque is commanded until a controller is active, and JTC holds
whatever pose it sees when it activates. So the arm and gripper controllers are loaded inactive, and
activate_at_home resets the sim to the `home` keyframe and activates them in back-to-back service
calls (a spawner started after the reset would leave ~2 s for the arm to fall).
"""

from pathlib import Path

import tempfile

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
JOINT_TRAJECTORY_CONTROLLER = "arm_controller"
# Task-space controllers (arm_sandbox_controllers): they get the robot's URDF, names and rest pose
# injected (see task_space_parameters_file). Only one arm controller is active at a time.
TASK_SPACE_CONTROLLERS = ["osc_controller", "cartesian_impedance_controller"]
ARM_CONTROLLERS = [JOINT_TRAJECTORY_CONTROLLER, *TASK_SPACE_CONTROLLERS]
GRIPPER_CONTROLLER = "gripper_controller"
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


class _NoAliasDumper(yaml.SafeDumper):
    """rcl's params parser rejects YAML aliases (&id001 / *id001), which PyYAML emits for shared objects."""

    def ignore_aliases(self, data) -> bool:
        return True


def task_space_parameters_file(robot_config: dict, robot_description: str, compensate_gravity: bool) -> Path:
    """Parameters every task-space controller needs from the robot, as a ROS params file.

    Written per launch so robot.yaml and the URDF stay the single source (no copies in the
    controllers YAML). compensate_gravity is the opposite of the sim's gravcomp: gravity must be
    compensated exactly once.
    """
    robot_parameters = {
        "joints": robot_config["arm_joints"],
        "base_frame": robot_config["base_frame"],
        "ee_frame": robot_config["ee_frame"],
        "rest_configuration": [float(value) for value in robot_config["home"]],
        "robot_description": robot_description,
        "compensate_gravity": compensate_gravity,
    }
    contents = {name: {"ros__parameters": robot_parameters} for name in TASK_SPACE_CONTROLLERS}
    path = Path(tempfile.mkdtemp(prefix="arm_sandbox_controllers_")) / "task_space_controllers.yaml"
    path.write_text(yaml.dump(contents, Dumper=_NoAliasDumper))
    return path


def rerun_actions(context: LaunchContext) -> list:
    """The Rerun viewer (own process) and the bridge that feeds it (design spec "Visualization")."""
    if not is_true(context, "rerun"):
        return []
    bridge_config = require_file(
        Path(get_package_share_directory("arm_sandbox_viz")) / "config" / "rerun_bridge.yaml", "Rerun bridge config"
    )
    overrides = {"use_sim_time": True}
    actions = []
    save_path = LaunchConfiguration("rerun_save").perform(context)
    if save_path:
        overrides.update({"save_path": save_path, "grpc_url": ""})
    else:
        # Web viewer on http://localhost:9090/?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9876%2Fproxy, gRPC on 9876 (bridge config `grpc_url`).
        actions.append(ExecuteProcess(cmd=["rerun", "--serve-web"], output="screen"))
    actions.append(
        Node(
            package="arm_sandbox_viz",
            executable="rerun_bridge",
            name="rerun_bridge",
            parameters=[str(bridge_config), overrides],
            output="screen",
        )
    )
    return actions


def launch_setup(context: LaunchContext) -> list:
    robot = LaunchConfiguration("robot").perform(context)
    arm_controller = LaunchConfiguration("arm_controller").perform(context)
    if arm_controller not in ARM_CONTROLLERS:
        raise ValueError(f"arm_controller must be one of {ARM_CONTROLLERS}, got '{arm_controller}'")
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
    # Opt-in: mujoco_ros2_control 0.1.2's ExternalWrenchPlugin makes ros2_control_node segfault on
    # shutdown (exit code -11), so only tests and demos that push on the robot enable it.
    sim_plugin_files = (
        [str(require_file(bringup_dir / "config" / "sim_plugins.yaml", "sim plugins config"))]
        if is_true(context, "external_wrench")
        else []
    )
    control_node = Node(
        # mujoco_ros2_control ships its own ros2_control_node (same parameters as upstream's).
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        parameters=[
            {"use_sim_time": True},
            str(controllers_file),
            str(task_space_parameters_file(robot_config, robot_description, not is_true(context, "gravcomp"))),
            *sim_plugin_files,
        ],
        # Humble's controller_manager reads the URDF from ~/robot_description.
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
        on_exit=Shutdown(),
    )
    joint_state_broadcaster = spawner("joint_state_broadcaster")
    # Loaded and configured, but not active: activate_at_home activates them right after the reset.
    arm_and_gripper = spawner(*ARM_CONTROLLERS, GRIPPER_CONTROLLER, "--inactive")
    activate_at_home = Node(
        package="arm_sandbox_bringup",
        executable="activate_at_home",
        parameters=[
            {
                "reset_service": RESET_WORLD_SERVICE,
                "keyframe": HOME_KEYFRAME,
                "switch_service": SWITCH_CONTROLLER_SERVICE,
                "controllers": [arm_controller, GRIPPER_CONTROLLER],
                "timeout_s": STARTUP_SERVICE_TIMEOUT_S,
            }
        ],
        output="screen",
    )
    return rerun_actions(context) + [
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
                "external_wrench",
                default_value="false",
                description="Enable the sim's ~/apply_wrench service (push on a body; for tests and demos)",
            ),
            DeclareLaunchArgument(
                "arm_controller",
                default_value=JOINT_TRAJECTORY_CONTROLLER,
                description=f"Arm controller to activate: one of {ARM_CONTROLLERS}",
            ),
            DeclareLaunchArgument(
                "gravcomp",
                default_value="false",
                description="MuJoCo compensates the arm's gravity (like the real Panda); false: controllers do",
            ),
            DeclareLaunchArgument(
                "rerun", default_value="true", description="Show the sim in the Rerun web viewer (http://localhost:9090/?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9876%2Fproxy)"
            ),
            DeclareLaunchArgument(
                "rerun_save", default_value="", description="Record to this .rrd file instead of starting a viewer"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
