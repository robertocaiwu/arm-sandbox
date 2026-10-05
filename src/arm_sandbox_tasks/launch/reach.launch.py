"""Run the reach task (milestone M2) against a running sim (`make start_sim`).

    ros2 launch arm_sandbox_tasks reach.launch.py robot:=panda task:=reach controller:=arm_controller

`controller` must be the arm controller the sim activated (sim.launch.py arm_controller:=...):
arm_controller (IK + joint trajectory, M2) or a task-space controller (pose target, M3).

reach_runner gets the robot's names and home pose from its robot.yaml, the task from
config/tasks/<task>.yaml, and its solver/motion settings from config/reach_runner.yaml.
It exits 0 if every target was reached; the launch then ends.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_CONFIG_KEYS = ("arm_joints", "base_frame", "ee_frame", "home")
JOINT_TRAJECTORY_CONTROLLER = "arm_controller"
TASK_SPACE_CONTROLLERS = ("osc_controller", "cartesian_impedance_controller")


def motion_parameters(controller: str) -> dict:
    """How reach_runner moves the arm with `controller`, and which of its topics to use."""
    if controller == JOINT_TRAJECTORY_CONTROLLER:
        return {
            "motion": "joint_trajectory",
            "arm_action": f"/{controller}/follow_joint_trajectory",
            "controller_state_topic": f"/{controller}/state",
        }
    if controller in TASK_SPACE_CONTROLLERS:
        return {
            "motion": "pose_target",
            "pose_target_topic": f"/{controller}/target_pose",
            "controller_state_topic": f"/{controller}/pose_error",
        }
    raise ValueError(f"controller must be one of {(JOINT_TRAJECTORY_CONTROLLER, *TASK_SPACE_CONTROLLERS)}, got '{controller}'")


def require_file(path: Path, what: str) -> Path:
    if not path.is_file():
        raise FileNotFoundError(f"{what} not found: {path}")
    return path


def launch_setup(context: LaunchContext) -> list:
    robot = LaunchConfiguration("robot").perform(context)
    task = LaunchConfiguration("task").perform(context)
    controller = LaunchConfiguration("controller").perform(context)
    tasks_dir = Path(get_package_share_directory("arm_sandbox_tasks"))
    robot_config_file = Path(get_package_share_directory("arm_sandbox_description")) / robot / "config" / "robot.yaml"
    robot_config = yaml.safe_load(require_file(robot_config_file, "robot config").read_text())

    reach_runner = Node(
        package="arm_sandbox_tasks",
        executable="reach_runner",
        name="reach_runner",
        parameters=[
            str(require_file(tasks_dir / "config" / "reach_runner.yaml", "reach_runner config")),
            {key: robot_config[key] for key in ROBOT_CONFIG_KEYS},
            motion_parameters(controller),
            {
                "task_file": str(require_file(tasks_dir / "config" / "tasks" / f"{task}.yaml", "task file")),
                "use_sim_time": True,
            },
        ],
        output="screen",
    )
    return [
        reach_runner,
        RegisterEventHandler(OnProcessExit(target_action=reach_runner, on_exit=[EmitEvent(event=Shutdown())])),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="panda", description="Robot folder in arm_sandbox_description"),
            DeclareLaunchArgument("task", default_value="reach", description="Task file in config/tasks/ (without .yaml)"),
            DeclareLaunchArgument(
                "controller",
                default_value=JOINT_TRAJECTORY_CONTROLLER,
                description="Active arm controller: arm_controller, osc_controller or cartesian_impedance_controller",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
