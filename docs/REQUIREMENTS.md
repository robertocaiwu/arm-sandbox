# arm-sandbox — Requirements (Draft v0.4)

> Status: fourth draft, 2026-10-01. Decisions made so far are in §10. Open questions are in §11.

## 1. Purpose

`arm-sandbox` is a ROS 2 simulation sandbox built around a robot arm. It is for learning and showing manipulation skills in two phases:

1. **Phase A: applied manipulation.** Kinematics, control, motion planning, perception, and pick-and-place, built with the standard industry ROS 2 stack.
2. **Phase B: robot learning.** Imitation learning, RL, VLAs, and VLMs, solving the same tasks and compared against the Phase A baseline.

### 1.1 Goals

- **G1. Learning.** Build hands-on depth in manipulator kinematics, dynamics, control, and planning first, then in robot learning.
- **G2. Portfolio.** Produce public evidence of these skills for job applications: clean code, reproducible results, comparison tables, and short demo videos.
- **G3. Robot-agnostic.** No robot-specific code. Supporting a new arm is a matter of adding a description and config package.
- **G4. Modular.** Add a new approach (controller, planner, policy) or a new task without touching the rest.

### 1.2 Non-goals

- **No real hardware.** The project is sim-only, but nothing may block a later port to a real arm (see §3.2).
- Not a general-purpose robotics framework. Prefer existing ROS 2 packages and libraries over writing our own.
- No mobile manipulation, multi-robot setups, or AMR integration. Those stay in `apps/amr-sim-lab`.
- No Isaac Sim/Isaac Lab. It proved hard to get running and too heavy for this machine.

## 2. Target Skills

Ordered by phase. Each skill must be demonstrated by at least one concrete module or experiment.

### Phase A: applied manipulation

| Area | Skill to demonstrate |
|---|---|
| Kinematics | FK, analytic and numerical IK (Jacobian pseudo-inverse, damped least squares), singularity handling, null-space redundancy resolution |
| Dynamics and control | Joint PD + gravity compensation, computed torque, operational-space and impedance control, trajectory generation (min-jerk, splines, time parameterization) |
| ROS 2 control stack | `ros2_control` hardware interfaces and controllers (joint trajectory, forward command, custom controller), URDF/xacro, TF2 |
| Motion planning | MoveIt 2: collision-aware planning, planning scene, Cartesian paths, MoveIt Servo for real-time teleop |
| Perception | Camera-based object detection and pose estimation (RGB-D, point clouds), simulated hand-eye calibration |
| Manipulation | Grasp selection, pick-and-place state machine / behavior tree, error recovery |

### Phase B: robot learning

| Area | Skill to demonstrate |
|---|---|
| Imitation learning | Behavior Cloning, ACT, and Diffusion Policy trained on scripted and teleop demos |
| Reinforcement learning | PPO/SAC on manipulation tasks, reward design, domain randomization |
| VLA | Evaluate and fine-tune a small open VLA (e.g. SmolVLA) on sandbox tasks |
| VLM | VLM as a high-level task planner or success detector that calls Phase A skills |

### Both phases

Reproducible experiments, containerized environment, tests, CI.

## 3. Functional Requirements

### 3.1 Simulation

- **FR-1** Use **MuJoCo** as the physics engine. Simulate a 6/7-DoF arm with a parallel gripper, with joint limits, torque limits, and contact.
- **FR-2** Expose the sim to ROS 2 through **`ros2_control`**, so controllers and MoveIt 2 talk to the sim the same way they would talk to a real arm.
- **FR-3** Publish simulated sensors as standard ROS 2 topics: `/joint_states`, `/clock`, RGB and depth images with `CameraInfo`, and point clouds. Provide at least one fixed scene camera and one wrist camera.
- **FR-4** Run both headless (CI, training) and with a viewer (debugging, demos).
- **FR-5** Deterministic given a seed: same seed + same commands = same rollout.
- **FR-6** Support scene and domain randomization: object poses, colors and textures, lighting, camera pose, physics parameters.

### 3.2 Robot-agnostic design

- **FR-7** Keep everything robot-specific in a per-robot package, never in code:
  - Robot description: URDF/xacro for ROS 2 plus an MJCF model for MuJoCo
  - `ros2_control` config: joints, interfaces, controllers
  - MoveIt 2 config: SRDF, kinematics solver, joint limits
  - A small robot config file: arm joints, base and end-effector frames, gripper type, home pose
- **FR-8** Write all kinematics, control, planning, task, and policy code against generic interfaces (joint names, frames, and limits read from config, never hard-coded).
- **FR-9** Use the `ros2_control` hardware interface as the swap point. The MuJoCo backend is one implementation, and a real arm's driver would be another, with no change above it.
- **FR-10** Ship **one arm** for now (Franka Panda, 7-DoF). Selecting the robot is still a launch argument (`robot:=panda`), so a second arm can be added later as a new package with no code changes.

### 3.3 Visualization

- **FR-11** **Native viewer:** the MuJoCo viewer for low-level physics debugging (contacts, forces). Runs through WSLg on this machine.
- **FR-12** **Browser viewer:** watch the running simulation from a web browser, with no local GUI needed. Shows the robot model, TF, camera images, planned trajectories, and live plots (joint states, controller errors).
- **FR-13** Both viewers show the **same running simulation**. Physics runs once, and the viewers only display it.
- **FR-14** RViz2 stays available for MoveIt 2 interactive planning.

- **FR-14a** Viewers depend only on **standard ROS 2 topics** (`/joint_states`, `/tf`, images, `visualization_msgs`), never on sim internals. Any browser viewer can then be swapped in without touching the rest of the stack, including a custom one later (M12).

> Approach for FR-12: **Rerun** (open source, runs locally, no account). A small `arm_sandbox_viz` bridge node subscribes to the ROS 2 topics above and logs them to Rerun: the robot from its URDF, TF, scene objects, camera images, trajectories, and time-series plots. The Rerun viewer is served as a web page (`--web-viewer`) and can also record sessions for later replay. LeRobot uses Rerun to view datasets too, so Phase B can use the same viewer.
>
> Optional extra: `web_video_server` to stream the MuJoCo-rendered camera images as plain video.

### 3.4 Tasks

- **FR-15** A small task set of increasing difficulty:
  1. Reach a target pose
  2. Pick and place a cube (known pose, from sim ground truth)
  3. Pick and place with perception (pose estimated from the camera)
  4. Stack two blocks
  5. Open a drawer (articulated object, needs compliant control)
  6. Language-conditioned: "put the red block in the bowl" (Phase B)
- **FR-16** Each task defines its scene, success criterion, reward (for Phase B), and time limit, in config.
- **FR-17** Tasks can be reset and randomized through a ROS 2 service.

### 3.5 Phase A: applied manipulation stack

- **FR-18** Hand-written kinematics library (FK, Jacobian, IK), cross-checked against a reference library such as Pinocchio or KDL.
- **FR-19** At least one **custom `ros2_control` controller** (e.g. an operational-space or impedance controller) next to the standard ones.
- **FR-20** MoveIt 2 integration: plan and execute collision-free motions, Cartesian paths, and Servo-based teleop (keyboard/gamepad).
- **FR-21** Perception node: detect objects and estimate their poses from RGB-D and publish them to TF and the planning scene.
- **FR-22** Task-level executive (behavior tree or state machine) that runs pick-and-place end to end and recovers from failures (missed grasp, planning failure).

### 3.6 Phase B: robot learning

- **FR-23** A common policy interface: `obs (+ optional language instruction) → action`, so classical skills and learned policies run in the same evaluation loop.
- **FR-24** A **Gymnasium** wrapper over the same tasks for training, so any RL/IL library can plug in.
- **FR-25** Collect demonstrations from scripted Phase A skills and from teleop (MoveIt Servo), and store them in **LeRobotDataset** format.
- **FR-26** Train and evaluate BC / ACT / Diffusion Policy, at least one RL policy (PPO or SAC), and one small VLA.
- **FR-27** A VLM planner breaks a language goal into Phase A skill calls (`pick(obj)`, `place(obj, target)`). It can reuse the local Ollama/Qwen setup (`scripts/python/start_ollama_qwen.py`).
- **FR-28** Training runs directly on MuJoCo through the Gymnasium wrapper (no ROS 2 in the loop, for speed). It uses the same robot and task configs as the ROS 2 stack. Trained policies are then deployed as ROS 2 nodes and evaluated under the same conditions as the Phase A stack.

### 3.7 Evaluation

- **FR-29** One evaluation tool that runs any approach on any task over N seeded episodes and reports success rate, time to completion, smoothness (jerk), constraint violations (joint limits, collisions), and planning/inference latency.
- **FR-30** Record rollouts automatically as video and as rosbag2.
- **FR-31** Write results to CSV/JSON and build a comparison table across approaches. This table is the main portfolio artifact.

## 4. Non-Functional Requirements

- **NFR-1 Hardware budget.** Must run on the local workstation: **RTX 3070, 8 GB VRAM, WSL2**. Phase A must also run on a PC without an NVIDIA GPU (CPU rendering), for development on a laptop. That's plenty for Phase A. For Phase B, everything runs locally by default (small VLAs such as SmolVLA ~0.45B). Renting cloud GPUs is allowed only for occasional heavy jobs (e.g. a larger VLA fine-tune), and nothing may require it.
- **NFR-2 Containerized.** One Docker image (ROS 2 + MuJoCo + dev tools) with NVIDIA GPU passthrough, plus a VS Code devcontainer, following the `apps/amr-sim-lab` pattern. Self-contained: no dependency on the `ai-station` image.
- **NFR-3 Reproducible.** Pinned dependencies, seeds, and configs saved with every run's outputs.
- **NFR-4 Tested.** Unit tests for kinematics and control math (FK∘IK round-trip, Jacobian vs finite differences), `launch_testing` integration tests for the ROS 2 bring-up, and smoke tests per task. All tests run headless.
- **NFR-5 CI.** GitHub Actions builds the workspace and runs tests headless on every push.
- **NFR-6 Language and readability.** **C++ by default** (controllers, hardware interface, kinematics, perception, task executive). Python where it is clearly easier (launch files, eval scripts, Gymnasium wrapper, learning code, VLM planner). Modern C++ (C++17/20) with typed Python, clear package boundaries, and a short README per package explaining the theory behind it. The audience is hiring teams reading the code.
- **NFR-7 Performance.** Sim runs at least real-time with visualization on. Phase B training uses a faster non-ROS path (FR-28).

## 5. Proposed Tech Stack

| Concern | Choice | Notes |
|---|---|---|
| ROS 2 | **Humble** (Ubuntu 22.04) | Runs natively in the current dev environment (Ubuntu 22.04) and matches `amr-sim-lab`. LTS until May 2027; a later move to Jazzy (Ubuntu 24.04) is a known follow-up. |
| Physics | **MuJoCo** 3.x | Light, stable, fits 8 GB easily. |
| Sim ↔ ROS 2 | `mujoco_ros2_control` (ros-controls) | Exists as a ros-controls project. Available as `ros-humble-mujoco-ros2-control`; maturity must be checked in M0, with a thin custom hardware interface as the fallback. |
| Robot model | Franka Panda | MJCF from MuJoCo Menagerie. URDF and MoveIt config from upstream ROS 2 packages. |
| Kinematics | Eigen (hand-written C++) + Pinocchio | Pinocchio is used to check the hand-written implementation. |
| Planning | MoveIt 2 (+ MoveIt Servo) | |
| Task executive | BehaviorTree.CPP | |
| Browser visualization | **Rerun** (web viewer) + custom bridge node | Plus RViz2 and the native MuJoCo viewer. Optional `web_video_server` for camera video. Custom viewer later (M12). |
| IL / VLA (Phase B) | LeRobot | ACT, Diffusion Policy, SmolVLA, and LeRobotDataset in one library. |
| RL (Phase B) | Stable-Baselines3 | |
| VLM (Phase B) | Qwen-VL via Ollama | Already set up in this workspace. |

## 6. High-Level Architecture (draft)

```
                         ┌──────────────── Browser (Rerun) / RViz2 ──────────────────┐
                         │      robot model, TF, cameras, trajectories, plots        │
                         └───────────────────────────▲───────────────────────────────┘
                                                     │ ROS 2 topics → arm_sandbox_viz
 ┌────────────────────────────────────────────────────┴─────────────────────────────────┐
 │  Task executive (BT)  ──▶  MoveIt 2 / custom skills  ──▶  ros2_control controllers  │
 │        ▲                         ▲                         (JTC, OSC, impedance)    │
 │        │ obj poses               │ planning scene                    │              │
 │  Perception node  ◀── RGB-D ─────┘                                   │              │
 │                                                                      ▼              │
 │                                         ┌──── hardware interface (swap point) ────┐ │
 │  Phase B: policy node (IL/RL/VLA) ──────▶  MuJoCo backend   │   (future) real arm │ │
 │           VLM planner → skill calls     └──────────┬──────────────────────────────┘ │
 └────────────────────────────────────────────────────┼─────────────────────────────────┘
                                                      ▼
                                  MuJoCo physics (+ native viewer)
 Robot-specific: robot_description + ros2_control config + MoveIt config + robot.yaml (per arm)
```

Proposed ROS 2 packages (colcon workspace under `src/`):

| Package | Role |
|---|---|
| `arm_sandbox_description` | Per-robot URDF/xacro, MJCF, meshes, `robot.yaml` (one folder per arm) |
| `arm_sandbox_sim` | MuJoCo bring-up, scenes, sensors, reset/randomize services |
| `arm_sandbox_kinematics` | Pure, ROS-free C++ kinematics library (Eigen) + tests |
| `arm_sandbox_controllers` | Custom `ros2_control` controllers (OSC, impedance) |
| `arm_sandbox_moveit_config` | MoveIt 2 configs per robot |
| `arm_sandbox_perception` | Object detection / pose estimation |
| `arm_sandbox_tasks` | Task definitions, executive, skills |
| `arm_sandbox_learning` | Phase B: Gymnasium wrapper, data collection, policy nodes |
| `arm_sandbox_eval` | Evaluation runner and results table |
| `arm_sandbox_viz` | ROS 2 → Rerun bridge node (C++) |
| `arm_sandbox_bringup` | Launch files and top-level configs |

## 7. Milestones

### Phase A: applied manipulation

| # | Milestone | Done when |
|---|---|---|
| M0 | Environment | Container builds; Panda loads in MuJoCo; the arm is visible in the browser **and** in the native viewer at the same time |
| M1 | ros2_control | Arm follows a joint trajectory through `ros2_control`; robot selected by launch arg |
| M2 | Kinematics | Hand-written FK/Jacobian/IK library, tested against Pinocchio; reach task solved |
| M3 | Control | Custom operational-space / impedance controller as a `ros2_control` plugin; drawer task opened compliantly |
| M4 | MoveIt 2 | Collision-free planning + Servo teleop; pick-and-place with ground-truth poses |
| M5 | Perception | Pick-and-place using camera-estimated poses; behavior tree with failure recovery |
| M6 | Evaluation | Eval runner, metrics, videos, rosbags; Phase A results table published |

### Phase B: robot learning

| # | Milestone | Done when |
|---|---|---|
| M7 | Data | Scripted and teleop demos stored as LeRobotDataset |
| M8 | Imitation learning | ACT and Diffusion Policy trained, run as ROS 2 nodes, evaluated against Phase A |
| M9 | RL | PPO/SAC on reach + pick with domain randomization |
| M10 | VLA / VLM | SmolVLA fine-tuned; VLM planner solves the language-conditioned task |
| M11 | Portfolio polish | Final comparison table, demo videos, write-ups |

### Future (optional)

| # | Milestone | Done when |
|---|---|---|
| M12 | Custom browser viewer | Own web viewer (e.g. three.js + URDF loader over a WebSocket bridge) shows the robot, TF, and scene objects live, and replaces Rerun for the 3D view without changes outside the viz layer (FR-14a) |

Each milestone should end with something worth showing (a video plus numbers), so the project is useful for job applications even if it stops early.

## 8. Success Criteria

- Phase A: perception-based pick-and-place running end to end in ROS 2, with no robot-specific values in code.
- Phase B: at least three learned approaches evaluated against the Phase A baseline in one comparison table.
- The simulation can be watched live from a browser.
- Every result reproducible from one command plus a config.
- A public `arm-sandbox` repository with CI, a clear README, and demo videos.

## 9. Repository

- Name: **`arm-sandbox`**, a standalone git repository. It is not part of the `projects` repo (no submodule, nothing tracked there, no references outside its own folder). On the original PC it is checked out at `/workspace/apps/arm-sandbox`, where `projects` ignores `apps/*`. Inside the dev container it is always mounted at `/workspace/arm-sandbox`.
- Layout follows `apps/amr-sim-lab`: `src/` (colcon workspace), `docker/`, `docs/`, a `Makefile` for common commands.

## 10. Decisions Log

| # | Decision | Reason |
|---|---|---|
| D1 | MuJoCo, not Isaac Sim | Isaac Sim didn't work here and is too heavy for 8 GB; MuJoCo is lighter and more stable |
| D2 | Browser viewing required, alongside the native viewer | Watch the sim without a local GUI and share demos easily |
| D3 | Applied manipulation first (Phase A), then robot learning (Phase B) | Learning priority |
| D4 | ROS 2 is the backbone | Industry standard, and it builds on existing AMR/ROS 2 experience |
| D5 | No real hardware, but robot-agnostic via `ros2_control` + per-robot configs | Keep a later port to a real arm possible |
| D6 | Standalone repository `arm-sandbox`, independent of the `projects` repo | Portfolio visibility; develop on any PC |
| D7 | ROS 2 Humble (Ubuntu 22.04), changed from Jazzy | The dev environment is Ubuntu 22.04, where Jazzy has no binaries; Humble installs natively there and in Docker |
| D8 | One arm (Franka Panda) for now; design stays robot-agnostic | Keep scope small |
| D9 | Phase B trains directly on MuJoCo, deploys and evaluates through ROS 2 | Training speed |
| D10 | C++ by default, Python where it is clearly easier | Matches job postings |
| D11 | Mostly local compute; occasional cloud GPU rental allowed | Cost |
| D12 | Rerun for the browser viewer (no Foxglove); custom viewer as a future milestone | Open source, local, no frontend to maintain now; keeps the option to build one |

## 11. Open Questions

None at the moment.
