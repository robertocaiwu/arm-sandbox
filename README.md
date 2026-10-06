# arm-sandbox

A ROS 2 Humble + MuJoCo sandbox around a Franka Panda arm, built to learn and demonstrate
manipulation: hand-written kinematics, custom controllers, motion planning and perception first
(Phase A), then robot learning (imitation learning, RL, VLAs, VLMs) compared against that baseline
(Phase B). The arm is torque-controlled in MuJoCo and driven only through `ros2_control`, exactly as a
real arm would be, and nothing robot-specific is hard-coded.

## Status

| Milestone | What | State |
|---|---|---|
| M0 Environment | Docker dev container, MuJoCo Panda, native viewer + Rerun in the browser | done |
| M1 ros2_control | Torque-actuated Panda behind `ros2_control`, joint trajectory controller, `robot:=` launch argument | done |
| M2 Kinematics | Hand-written FK / Jacobian / IK in C++, checked against Pinocchio; reach task solved | done |
| M3 Control | Operational-space and impedance controllers as `ros2_control` plugins; drawer opened compliantly | controllers done; drawer task in [Plan 05](docs/plan/2026-10-04-plan-05-drawer-task.md) |
| M4–M6 | MoveIt 2 and pick-and-place, perception, evaluation tool | planned |
| M7–M11 | Demonstrations, imitation learning, RL, VLA/VLM, comparison table | planned |

Details and acceptance criteria: [docs/REQUIREMENTS.md](docs/REQUIREMENTS.md) §7.

## Highlights so far

- **Kinematics written by hand, verified against a reference.** `arm_sandbox_kinematics` builds a
  chain from any URDF. Its FK and geometric Jacobian match Pinocchio to 1e-9 on random configurations,
  and the Jacobian also matches finite differences. The damped-least-squares IK uses
  manipulability-scaled damping, an exact null-space projector and joint clamping. It solves 200/200
  round trips from a nearby seed and 82.5 % of random workspace targets from the home pose.
- **One robot, two model files that agree.** The URDF (ROS side) and the MJCF (MuJoCo side)
  share meshes, inertials and limits; a test checks that their kinematics agree.
- **Reach task, end to end.** `make start_reach` drives the arm through 5 target poses with IK and the
  joint trajectory controller. Every target is held within 5 mm / 2° (1.3–4.1 mm when the controller
  carries gravity, 0.1–0.3 mm with simulator gravity compensation).
- **Task-space control on Pinocchio dynamics.** `arm_sandbox_controllers` adds operational-space and
  Cartesian impedance controllers as `ros2_control` plugins over one ROS-free core. On the same 5 reach
  targets, OSC settles to about 0.1 mm in about 1 s, impedance to 0.3–2.8 mm, and IK + the joint
  trajectory controller to 1.2–4.0 mm. The impedance controller gives 2 cm under a 10 N push
  (k = 500 N/m) and springs back, and a test proves `compute()` never allocates.
- **Watch it anywhere.** A C++ bridge streams `/robot_description`, `/tf` and `/joint_states` to the
  Rerun web viewer, next to MuJoCo's native viewer.
- **Tested headless.** `make test` runs 95 GoogleTest, pytest and `launch_testing` tests,
  including full-stack runs of the simulator.

## Quick start

Prerequisites: Docker (with Compose v2), VS Code with the Dev Containers extension, git. An NVIDIA GPU
is optional (Phase A runs on the CPU); it is detected automatically.

```bash
git clone <this repo> arm-sandbox   # the folder must be named arm-sandbox
cd arm-sandbox
make build                          # build the dev image (first time: a while)
code .                              # then: "Dev Containers: Reopen in Container"
```

Inside the dev container:

```bash
make smoke          # toolchain checks
make test           # colcon build + all tests
make start_sim      # the sim: MuJoCo window + Rerun web viewer
make start_reach    # in a second terminal: the reach task
make start_sim ARGS="arm_controller:=osc_controller"   # or cartesian_impedance_controller
make start_reach ARGS="controller:=osc_controller"     # the reach task with pose targets
make viewer         # just the MuJoCo model, no ROS
```

Rerun web viewer: <http://localhost:9090/?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9876%2Fproxy>
(the bare `:9090` page shows Rerun's welcome screen).

Useful launch arguments: `make start_sim ARGS="viewer:=false gravcomp:=true rerun:=false"`.
`gravcomp:=true` lets MuJoCo carry the arm's weight, as the real Panda's torque interface does.

## Repository layout

| Package | Role |
|---|---|
| `src/arm_sandbox_description` | Panda URDF/xacro, MJCF (from MuJoCo Menagerie, torque-actuated), `robot.yaml`; the only place robot names live |
| `src/arm_sandbox_sim` | Composes the MuJoCo scene at launch (gravity compensation on/off) |
| `src/arm_sandbox_bringup` | `sim.launch.py`, controller configs, startup ordering |
| `src/arm_sandbox_kinematics` | ROS-free C++ kinematics: FK, Jacobian, IK, manipulability |
| `src/arm_sandbox_controllers` | ROS-free control core (Pinocchio dynamics, OSC and impedance laws) + ros2_control plugins |
| `src/arm_sandbox_tasks` | Task files and runners (reach) |
| `src/arm_sandbox_viz` | ROS 2 → Rerun bridge |

Each package has a README with the theory behind it. Tooling: `docker/`, `scripts/install_deps.sh`
(the single dependency list), `Makefile`, `tests/env/` (toolchain smoke tests).

## Documentation

- [docs/REQUIREMENTS.md](docs/REQUIREMENTS.md): goals, requirements, milestones, decisions log
- [docs/specs/](docs/specs/): design spec
- [docs/plan/](docs/plan/): implementation plans, one per milestone group
- [docs/PROJECT_STRUCTURE.md](docs/PROJECT_STRUCTURE.md): architecture map, topics, commands
- [docs/LEARNING_RESOURCES.md](docs/LEARNING_RESOURCES.md): background reading for each plan
- [docs/MEMORY.md](docs/MEMORY.md): change history, session by session

## Stack

ROS 2 Humble · MuJoCo 3.12 via `mujoco_ros2_control` · `ros2_control` · C++17 / Eigen · Pinocchio
(reference and dynamics) · MoveIt 2 and BehaviorTree.CPP (coming) · Rerun · Docker. Phase B adds
LeRobot, Stable-Baselines3 and a VLM through Ollama.

## License

Apache-2.0 (see [LICENSE](LICENSE)). The Panda MJCF and meshes come from
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) (Apache-2.0), modified as
noted at the top of `panda.xml`.
