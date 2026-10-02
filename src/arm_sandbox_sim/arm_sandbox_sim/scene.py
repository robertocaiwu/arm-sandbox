"""Compose the MuJoCo scene that the sim loads (design spec "Simulation").

ROS-free on purpose: the launch files use it now, and the Phase B Gymnasium environment will load
exactly the same model through it.
"""

import tempfile
from pathlib import Path

import mujoco

COMPOSED_FILE_NAME = "composed_scene.xml"


def compose_scene(scene_path: Path, *, robot_root: str, gravcomp: bool) -> str:
    """Return the scene as one MJCF string that loads from any directory.

    Args:
        scene_path: The robot's scene file (e.g. share/arm_sandbox_description/panda/mjcf/scene.xml).
        robot_root: Name of the robot's base body (robot.yaml `base_frame`).
        gravcomp: If True, MuJoCo cancels gravity on the robot's bodies, as the real Panda's torque
            interface does. If False (the default in the launch files, D15), controllers must
            compensate gravity themselves. Bodies outside the robot are never compensated.
    """
    if not scene_path.is_file():
        raise FileNotFoundError(f"MuJoCo scene not found: {scene_path}")

    spec = mujoco.MjSpec.from_file(str(scene_path))
    # Relative mesh and texture paths resolve against the source file's folder. Make them absolute
    # so the composed file can be written anywhere.
    model_dir = scene_path.resolve().parent
    spec.meshdir = str(model_dir / spec.meshdir)
    spec.texturedir = str(model_dir / spec.texturedir)

    root = spec.body(robot_root)
    if root is None:
        raise ValueError(f"robot root body '{robot_root}' not found in {scene_path}")
    if gravcomp:
        for body in [root, *root.find_all(mujoco.mjtObj.mjOBJ_BODY)]:
            body.gravcomp = 1.0

    return spec.to_xml()


def write_composed_scene(
    scene_path: Path, *, robot_root: str, gravcomp: bool, output_dir: Path | None = None
) -> Path:
    """Write compose_scene()'s result to `output_dir` (a new temp dir if None) and return its path."""
    xml = compose_scene(scene_path, robot_root=robot_root, gravcomp=gravcomp)
    directory = output_dir if output_dir is not None else Path(tempfile.mkdtemp(prefix="arm_sandbox_scene_"))
    path = directory / COMPOSED_FILE_NAME
    path.write_text(xml)
    return path
