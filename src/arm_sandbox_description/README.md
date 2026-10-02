# arm_sandbox_description

All robot-specific files, one folder per robot (`robot:=<folder>`). Nothing outside this package and
`arm_sandbox_bringup/config/<robot>_controllers.yaml` names a joint or link.

## Two models of one robot

ROS tools (robot_state_publisher, MoveIt, Pinocchio) read the URDF; MuJoCo reads the MJCF. Keeping
two files is the price of using both, and the risk is that they drift apart. Here they share
meshes, inertials and limits, use the same link/joint names, and `test/test_panda_urdf.py` compares
forward kinematics (Pinocchio on the URDF vs MuJoCo on the MJCF) on random configurations.

## Torque actuation

The MJCF arm uses `motor` actuators: the command is joint torque, as on the real Panda's torque
interface. Position control is a controller's job (JTC's PID in M1, OSC/impedance in M3). The
gripper is a position servo on a tendon that averages both fingers.

## Files (panda)

- `urdf/panda.urdf.xacro`, `urdf/panda.ros2_control.xacro` - URDF and its `ros2_control` block
- `mjcf/panda.xml`, `mjcf/scene.xml` - MuJoCo model (modified Menagerie, see its header)
- `config/robot.yaml` - joint names, base/EE frames, gripper, home pose for generic code
