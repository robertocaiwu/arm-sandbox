# PROJECT_STRUCTURE.md

> Architectural map for AI agents and developers. Enables quick navigation and dependency analysis.
>
> **Status: M0–M2 done, M3 controllers done (Plans 01–04).** These exist: the dev environment, `tests/env/`, `arm_sandbox_description`, `arm_sandbox_sim` (scene composition only), `arm_sandbox_bringup`, `arm_sandbox_viz`, `arm_sandbox_kinematics`, `arm_sandbox_controllers`, and `arm_sandbox_tasks` (reach task only). Everything else is the target layout from `docs/REQUIREMENTS.md` (§6). Update it as packages are created.

## Overview

**arm-sandbox** is a **ROS 2 colcon workspace** for robot-arm manipulation in simulation, built with **ROS 2 Humble + MuJoCo + ros2_control + MoveIt 2**, in C++ by default and Python where easier.

### Stack

| Layer           | Technology                                                       |
| --------------- | ---------------------------------------------------------------- |
| Middleware      | ROS 2 Humble (Ubuntu 22.04)                                      |
| Physics         | MuJoCo 3.12.0 via `mujoco_ros2_control` 0.1.2 (D13, D14)         |
| Control         | ros2_control, custom C++ controllers (OSC, impedance)            |
| Planning        | MoveIt 2, MoveIt Servo                                           |
| Kinematics      | Hand-written C++ (Eigen), checked against Pinocchio              |
| Task executive  | BehaviorTree.CPP                                                 |
| Visualization   | Rerun (browser), RViz2, native MuJoCo viewer                     |
| Learning (B)    | LeRobot, Stable-Baselines3, Gymnasium, Qwen-VL via Ollama        |
| Build           | colcon, ament_cmake / ament_python                               |
| Testing         | GoogleTest, pytest, launch_testing                               |
| Infra           | Docker + NVIDIA Container Toolkit, devcontainer, GitHub Actions  |

---

## Project Tree

<details>
<summary>Expand planned structure</summary>

```
arm-sandbox/
├── CLAUDE.md
├── README.md
├── Makefile                         # host: build/up/shell/down; container: smoke/test
├── .devcontainer/
│   └── devcontainer.json            # one config; initializeCommand runs scripts/select_gpu.sh
├── .github/workflows/               # CI: build + headless tests
├── docker/
│   ├── Dockerfile                   # runs scripts/install_deps.sh; user mirrors the host user
│   ├── docker-compose.yml           # `sandbox` dev service, CPU-only (host network, X11, repo parent at /workspace, ~/.claude mount)
│   └── docker-compose.local.yml     # generated per PC by select_gpu.sh (GPU or CPU), gitignored
├── docs/
│   ├── REQUIREMENTS.md              # requirements, decisions log, milestones
│   ├── PROJECT_STRUCTURE.md         # this file
│   ├── MEMORY.md                    # change history, one appended section per session
│   ├── LEARNING_RESOURCES.md        # background reading per plan (theory behind the code)
│   ├── specs/                       # design specs
│   └── plan/                        # implementation plans (one per milestone group)
├── scripts/
│   ├── install_deps.sh              # all dependencies (Ubuntu 22.04); used by the Dockerfile too
│   └── select_gpu.sh                # writes docker/docker-compose.local.yml (GPU if available)
├── tests/env/                       # toolchain smoke tests (make smoke)
└── src/                             # colcon workspace
    ├── arm_sandbox_description/     # per-robot models + config
    │   └── panda/
    │       ├── urdf/                # URDF/xacro (+ ros2_control tags)
    │       ├── mjcf/                # MuJoCo model (from MuJoCo Menagerie)
    │       └── config/robot.yaml    # arm joints, base/EE frames, gripper, home pose
    ├── arm_sandbox_sim/             # MuJoCo bring-up, scenes, sensors, reset/randomize services
    ├── arm_sandbox_kinematics/      # pure C++ library (no ROS): FK, Jacobian, IK
    ├── arm_sandbox_controllers/     # custom ros2_control controllers (OSC, impedance)
    ├── arm_sandbox_moveit_config/   # MoveIt 2 config per robot
    ├── arm_sandbox_perception/      # object detection / pose estimation from RGB-D
    ├── arm_sandbox_tasks/           # task definitions, BT executive, skills
    ├── arm_sandbox_learning/        # Phase B: Gymnasium wrapper, data collection, policy nodes
    ├── arm_sandbox_eval/            # evaluation runner, metrics, results table
    ├── arm_sandbox_viz/             # ROS 2 → Rerun bridge node
    ├── arm_sandbox_interfaces/      # custom msg/srv (only if standard ones don't fit)
    └── arm_sandbox_bringup/         # launch files, top-level configs
```

</details>

---

## Packages

| Package                     | Lang          | Phase | Purpose                                                                                   |
| --------------------------- | ------------- | ----- | ----------------------------------------------------------------------------------------- |
| `arm_sandbox_description`   | data          | A     | All robot-specific files, one folder per robot. The only place robot names appear         |
| `arm_sandbox_sim`           | Python (C++ later) | A | Scene composition at launch (gravcomp, D15); later reset/randomize services, ground truth |
| `arm_sandbox_kinematics`    | C++           | A     | ROS-free kinematics library (Eigen), unit-tested against Pinocchio                        |
| `arm_sandbox_controllers`   | C++           | A     | ROS-free control core (Pinocchio dynamics, OSC and Cartesian impedance laws) + ros2_control plugins |
| `arm_sandbox_moveit_config` | config        | A     | SRDF, kinematics solver, joint limits, planning pipelines                                 |
| `arm_sandbox_perception`    | C++           | A     | RGB-D → object poses → TF + planning scene                                                |
| `arm_sandbox_tasks`         | C++           | A     | Task configs and runners. Now: the reach task (`reach_runner`); later BehaviorTree.CPP executive and skills |
| `arm_sandbox_eval`          | Python        | A/B   | Run any approach × task × N seeds; metrics, videos, rosbags, CSV/JSON                     |
| `arm_sandbox_viz`           | C++           | A     | Subscribes to standard topics, logs to Rerun (web viewer)                                 |
| `arm_sandbox_learning`      | Python        | B     | Gymnasium-on-MuJoCo wrapper, LeRobotDataset collection, IL/RL/VLA policy nodes, VLM planner |
| `arm_sandbox_interfaces`    | msg/srv       | A     | Custom interfaces, e.g. task reset/randomize, only when standard types don't fit         |
| `arm_sandbox_bringup`       | Python launch | A     | Top-level launch files, `robot:=` argument                                                |

### Dependency direction

```
bringup ──▶ everything
tasks ──▶ moveit_config, controllers, perception, kinematics
controllers ──▶ kinematics
sim, viz, eval ──▶ standard ROS 2 topics only
learning ──▶ description (configs), MuJoCo directly (training only)
description ◀── read by all (via robot.yaml / URDF), depends on nothing
```

---

## Key Interfaces

| Interface                          | Type                                    | Producer                 | Consumers                    |
| ---------------------------------- | --------------------------------------- | ------------------------ | ---------------------------- |
| `/joint_states`                    | `sensor_msgs/JointState`                | `joint_state_broadcaster` | MoveIt 2, viz, eval, policies |
| `/tf`, `/tf_static`                | `tf2_msgs/TFMessage`                    | robot_state_publisher, perception | all                 |
| `/clock`                           | `rosgraph_msgs/Clock`                   | sim                      | all (`use_sim_time`)         |
| `/camera/{scene,wrist}/*`          | `Image`, `CameraInfo`, `PointCloud2`    | sim                      | perception, viz, policies    |
| `/scene/markers`                   | `visualization_msgs/MarkerArray`        | sim                      | viz, RViz2                   |
| Controller command topics/actions  | `FollowJointTrajectory`, etc.           | MoveIt 2, skills, policies | ros2_control              |
| Reset / randomize                  | service                                 | sim                      | tasks, eval                  |

> Viewers use only these standard topics (requirement FR-14a), so a custom browser viewer (M12) can replace Rerun.

---

## Configuration

### Robot selection

- Launch argument: `robot:=panda`. Resolves to `arm_sandbox_description/<robot>/`, its `ros2_control` YAML, and its MoveIt config
- Adding a robot = adding a description folder + configs. No code changes
- Launch arguments of `sim.launch.py`: `robot`, `viewer`, `gravcomp`, `rerun`, `rerun_save`, `arm_controller`, `external_wrench`

### Config files

| File                                         | Purpose                                         |
| -------------------------------------------- | ----------------------------------------------- |
| `description/<robot>/config/robot.yaml`      | Arm joints, base/EE frames, gripper, home pose  |
| `bringup/config/<robot>_controllers.yaml`    | `ros2_control` controllers and gains            |
| `tasks/config/<task>.yaml`                   | Scene, randomization, success criterion, limits |
| `eval/config/*.yaml`                         | Approach × task × seeds experiments             |

### Build

- `colcon build --symlink-install`, C++ with `-Wall -Wextra -Wpedantic`
- C++17/20, Eigen, `ament_clang_format`, `ament_clang_tidy`; Python with `ruff`

### Testing

- GoogleTest for C++ libraries and controllers, pytest for Python
- `launch_testing` for sim bring-up and controller integration
- All tests headless (CI has no GPU or display)

---

## Commands

| Command        | Where     | Description                                   |
| -------------- | --------- | --------------------------------------------- |
| `make build`   | host      | Build the `arm-sandbox:dev` image             |
| `make up/down` | host      | Start / stop the `sandbox` container          |
| `make shell`   | host      | Shell into the running container              |
| `scripts/install_deps.sh` | host or container (Ubuntu 22.04) | Install all dependencies (sudo) |
| `make smoke`   | container | Toolchain smoke tests (`tests/env/`)          |
| `make test`    | container | `colcon build` + `colcon test`                |
| `make viewer` | container | Native MuJoCo viewer on the robot scene, no ROS (`ROBOT=panda`) |
| `make start_sim` | container | Run the sim: native viewer + Rerun (`ARGS="viewer:=false gravcomp:=true rerun:=false"`) |
| `make start_reach` | container | Run the reach task against a running sim (`ARGS="task:=reach"`) |

`make lint` is added by a later plan.

---

## Key Architectural Patterns

### Hardware interface as swap point

Controllers, MoveIt 2, and policies only talk to `ros2_control`. MuJoCo is one hardware-interface backend, and a real arm's driver would be another.

### Pure library + thin node

Math and logic (kinematics, gating, metrics) live in ROS-free libraries with unit tests. Nodes only wire them to topics, services, and parameters.

### Train on MuJoCo, deploy through ROS 2 (Phase B)

The Gymnasium wrapper uses MuJoCo directly for fast training, with the same robot and task configs. Trained policies run as ROS 2 nodes and go through the same evaluation tool as Phase A.

---

## External Integrations

| Integration       | Purpose                                     | Config Location                       |
| ----------------- | ------------------------------------------- | ------------------------------------- |
| MuJoCo Menagerie  | Panda MJCF model                            | `arm_sandbox_description/panda/mjcf/` |
| Rerun             | Browser visualization                       | `arm_sandbox_viz/config/`             |
| LeRobot           | IL/VLA training, datasets (Phase B)         | `arm_sandbox_learning/config/`        |
| Ollama (Qwen-VL)  | VLM planner (Phase B), runs on the host      | `arm_sandbox_learning/config/`        |

---

## Maintenance

### When to Update This File

- A package is created, renamed, or removed under `src/`
- A new robot is added to `arm_sandbox_description/`
- A key topic, service, or launch argument changes
- A Makefile target is added or changed
- An architectural pattern is introduced

### Verification Commands

```bash
ls src/
colcon list
ros2 launch arm_sandbox_bringup sim.launch.py --show-args
grep -E '^[a-z-]+:' Makefile
```

### Sync Checklist

- [ ] Package table matches `colcon list`
- [ ] Tree matches `src/`
- [ ] Interfaces table matches running topics (`ros2 topic list`)
- [ ] Commands match the Makefile
- [ ] "Planned" markers removed for things that now exist

---

> **Note**: This document is a navigation aid. Keep it accurate but don't over-document. Update when architecture changes, not for every file addition.
