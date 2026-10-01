# PROJECT_STRUCTURE.md

> Architectural map for AI agents and developers. Enables quick navigation and dependency analysis.
>
> **Status: planned.** No code exists yet. This is the target layout from `docs/REQUIREMENTS.md` (§6). Update it as packages are created.

## Overview

**arm-sandbox** is a **ROS 2 colcon workspace** for robot-arm manipulation in simulation, built with **ROS 2 Jazzy + MuJoCo + ros2_control + MoveIt 2**, in C++ by default and Python where easier.

### Stack

| Layer           | Technology                                                       |
| --------------- | ---------------------------------------------------------------- |
| Middleware      | ROS 2 Jazzy (Ubuntu 24.04)                                       |
| Physics         | MuJoCo 3.x (`mujoco_ros2_control` or custom hardware interface)  |
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
├── .devcontainer/                   # VS Code devcontainer
├── .github/workflows/               # CI: build + headless tests
├── docker/
│   ├── Dockerfile                   # ROS 2 Jazzy + MuJoCo + MoveIt 2 + Rerun; user mirrors the host user
│   └── docker-compose.yml           # `sandbox` dev service (GPU, host network, X11, ~/.claude mount)
├── docs/
│   ├── REQUIREMENTS.md              # requirements, decisions log, milestones
│   ├── PROJECT_STRUCTURE.md         # this file
│   ├── specs/                       # design specs
│   └── plan/                        # implementation plans (one per milestone group)
├── tests/env/                       # toolchain smoke tests (make smoke)
└── src/                             # colcon workspace
    ├── arm_sandbox_description/     # per-robot models + config
    │   └── panda/
    │       ├── urdf/                # URDF/xacro (+ ros2_control tags)
    │       ├── mjcf/                # MuJoCo model (from MuJoCo Menagerie)
    │       ├── meshes/
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
| `arm_sandbox_sim`           | C++           | A     | Loads MJCF + scene, runs physics, publishes cameras, reset/randomize services             |
| `arm_sandbox_kinematics`    | C++           | A     | ROS-free kinematics library (Eigen), unit-tested against Pinocchio                        |
| `arm_sandbox_controllers`   | C++           | A     | `controller_interface` plugins: operational-space, impedance                              |
| `arm_sandbox_moveit_config` | config        | A     | SRDF, kinematics solver, joint limits, planning pipelines                                 |
| `arm_sandbox_perception`    | C++           | A     | RGB-D → object poses → TF + planning scene                                                |
| `arm_sandbox_tasks`         | C++           | A     | Task configs (scene, success, time limit), BehaviorTree.CPP executive and skills          |
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
| `make smoke`   | container | Toolchain smoke tests (`tests/env/`)          |
| `make test`    | container | `colcon build` + `colcon test`                |

`make sim` and `make lint` are added by later plans.

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
