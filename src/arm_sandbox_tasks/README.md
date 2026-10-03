# arm_sandbox_tasks

Task definitions and the programs that solve them. Task files live in `config/tasks/`; the task logic
(`reach_task.hpp`) is ROS-free so the eval runner and the Gym environment can reuse it.

## Reach (M2)

    make start_sim      # terminal 1
    make start_reach    # terminal 2: IK → joint trajectory → pose check, exits 0 if all targets reached

`reach_runner` solves each target in `config/tasks/reach.yaml` with `arm_sandbox_kinematics`, starting
from the current joint positions with a null-space pull towards home. It sends the joint trajectory
controller one goal, timed so no joint exceeds `velocity_scale` × its velocity limit, and counts the
target as reached once TF shows the end effector within tolerance for `hold_s`.

Without simulator gravity compensation the controller's integral term carries the arm's weight, so
the end effector settles within a few millimetres (≤ 4.1 mm after 0.5 s); with `gravcomp:=true` it
is within 0.3 mm. Explicit gravity compensation in the controller is M3.
