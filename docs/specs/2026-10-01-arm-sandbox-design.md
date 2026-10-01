# arm-sandbox - Design Spec

ROS 2 Jazzy + MuJoCo sandbox for a Franka Panda arm. Phase A builds an applied-manipulation stack (kinematics, control, MoveIt 2, perception, pick-and-place). Phase B adds robot learning (IL, RL, VLA, VLM) and compares it against Phase A. Portfolio and learning project. Runs locally in Docker on an RTX 3070 (8 GB) under WSL2.

Requirements, IDs (FR-x, NFR-x), the decisions log, and milestones are in [`../REQUIREMENTS.md`](../REQUIREMENTS.md). This spec describes **how** they are met.

## Decisions

Decisions from the requirements (D1–D12, short form):

- MuJoCo, not Isaac Sim. ROS 2 Jazzy is the backbone.
- Applied manipulation first (Phase A), robot learning second (Phase B).
- One arm (Franka Panda) for now. The design stays robot-agnostic through `ros2_control` and per-robot config.
- Phase B trains directly on MuJoCo, then deploys and evaluates through ROS 2.
- C++ by default, Python where clearly easier.
- Mostly local compute. Occasional cloud GPU rental is allowed but never required.
- Rerun for the browser viewer. A custom viewer is a future milestone (M12).
- Own repository, `arm-sandbox`.

Design decisions added by this spec:

- **Torque-actuated arm in MuJoCo.** The Panda MJCF uses `motor` (torque) actuators, like the real Panda's torque interface. Every arm controller writes the `effort` command interface. "Position" control means the Joint Trajectory Controller (JTC) running its built-in PID on effort. One actuator type means no actuator swapping when switching controllers. The gripper keeps a position actuator.
- **Physics runs inside the `ros2_control` loop, in lockstep.** The hardware interface steps MuJoCo once per controller update (1 kHz) and publishes `/clock`. All nodes use `use_sim_time`. The sim pauses when the stack can't keep up instead of drifting.
- **Sim backend chosen by an M0 spike.** First choice is the ros-controls `mujoco_ros2_control`. If it is missing anything below (effort interface, cameras, `/clock`, reset), write a thin `MujocoSystem` hardware interface in `arm_sandbox_sim`. The rest of the design doesn't depend on which one wins.
- **Hand-written kinematics, library dynamics.** FK, Jacobian, and IK are hand-written in C++/Eigen (the learning goal). The mass matrix, Coriolis, and gravity terms come from Pinocchio at first. A hand-written RNEA/CRBA is an optional stretch goal, tested against Pinocchio.
- **Shared ROS-free control core.** Controller math (OSC, impedance, joint impedance) lives in a ROS-free C++ library. The `ros2_control` plugins wrap it, and pybind11 bindings expose it to the Phase B Gymnasium environment. Training and deployment use the same controller code (DRY, and a smaller gap between the two).
- **Skills as ROS 2 actions.** `Pick`, `Place`, `MoveToPose`, and `Gripper` are action servers. The behavior tree, the VLM planner, and the eval runner all call the same actions. This is why `arm_sandbox_interfaces` exists.
- **Perception and ground truth share one message type.** Both publish `vision_msgs/Detection3DArray`. For M4, a ground-truth publisher stands in for perception. In M5 the perception node replaces it, and nothing downstream changes.
- **Scene changes need a relaunch, pose changes don't.** A task's objects are composed into the MJCF at launch. Object poses, colors, lighting, and camera pose are randomized at reset time without a relaunch.
- **Determinism is scoped.** The Gymnasium path is bit-for-bit deterministic per seed. The ROS 2 path is deterministic in scene setup (seeded reset) but not in executor timing, so the eval runner relies on N seeded episodes and reports spread, not single runs. This refines FR-5.
- **Fully containerized.** Nothing is installed on the host. Development happens in a VS Code dev container, the same pattern as `apps/amr-sim-lab`.
- **CI is optional**, as in fleet_master. The workflow file is kept, and the same checks run locally with `make test`.

## Architecture

```
            Browser: Rerun web viewer          RViz2 (MoveIt)        MuJoCo native viewer
                     ▲                              ▲                        ▲
                     │ gRPC                         │                        │ (same process as physics)
             arm_sandbox_viz ◀──── standard ROS 2 topics ────────────────────┤
                                                                             │
 eval runner ──▶ Executive (BehaviorTree.CPP) ──▶ skill actions (Pick/Place/MoveToPose/Gripper)
 VLM planner ──┘          │                                  │
                          │ Detection3DArray                 ├──▶ MoveIt 2 ──▶ JTC (effort+PID)
                 perception | ground-truth                   └──▶ switch ──▶ OSC / impedance controller
                          ▲                                                      │
                     RGB-D cameras                                     effort command interface
                          │                                                      ▼
                          └──────────── controller_manager + hardware interface (1 kHz lockstep) ── MuJoCo
 Phase B: policy node ──▶ joint_impedance_controller (same core as the Gym env)
```

Core loop (Phase A, M5): reset(seed) → cameras → perception publishes object poses → executive picks a target → `Pick` action (MoveIt plans approach → Cartesian descent → close gripper → check grasp width → lift) → `Place` action → success checked from ground truth → metrics logged.

## Simulation (`arm_sandbox_sim`)

- **Models.** URDF/xacro (ROS side) from upstream Franka / MoveIt resources. MJCF from MuJoCo Menagerie, changed to torque actuators. A unit test checks that URDF FK and MJCF FK agree on random configurations (the main risk of keeping two model files).
- **Scene.** `scenes/base.xml` (table, lights, cameras) plus a per-task object include, composed at launch.
- **Command interfaces.** Arm joints: `effort`. Gripper: `position`. State interfaces: `position`, `velocity`, `effort` for all joints.
- **Cameras.** `scene_camera` (fixed) and `wrist_camera`. Rendered offscreen in their own thread at 15–30 Hz, at 640×480. Each publishes `image_raw`, `depth`, `camera_info`, and `points`. Uses EGL on the GPU, or OSMesa in CI.
- **Services.** `~/reset` (seed, randomization on/off) and `~/randomize`. These need custom `.srv` types in `arm_sandbox_interfaces`.
- **Ground truth (privileged).** `/sim/ground_truth/objects` (`Detection3DArray`) and `/scene/markers` (`MarkerArray`). Only the eval runner, the M4 baseline, and the viewers may read them. Policies and perception may not.
- **Native viewer.** MuJoCo's passive viewer runs in the same process, enabled by the `viewer:=true` launch argument. It shows contacts and forces.

## Robot Description and Config (robot-agnostic)

```
arm_sandbox_description/panda/
  urdf/panda.urdf.xacro        # includes <ros2_control> block, selected by robot:=panda
  mjcf/panda.xml               # torque-actuated
  config/robot.yaml
```

`robot.yaml` is the single source for generic code:

```yaml
arm_joints: [panda_joint1, ..., panda_joint7]
base_frame: panda_link0
ee_frame: panda_hand_tcp
gripper: {type: parallel, joint: panda_finger_joint1, max_width: 0.08}
home: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
cameras: {wrist: wrist_camera, scene: scene_camera}
```

Launch loads it as parameters for every node. Code never contains these names. Adding a robot means adding a folder, its controllers YAML, and its MoveIt config.

## Kinematics (`arm_sandbox_kinematics`)

ROS-free C++17 library with Eigen. It is built from the URDF through `urdfdom`.

- `KinematicChain(urdf, base, tip)`
- `fk(q) → Isometry3d`
- `jacobian(q) → Matrix<6,n>` (geometric, base frame)
- `ik(target, q_seed, opts) → {q, converged, error}`: damped least squares with joint-limit clamping and an optional null-space task (stay near home / avoid limits)
- `manipulability(q)`
- Tests: FK∘IK round trip, Jacobian vs finite differences, FK and Jacobian vs Pinocchio at random configurations.

## Controllers (`arm_sandbox_controllers`)

| Controller | Interface | Input | Used for |
|---|---|---|---|
| `joint_trajectory_controller` (stock) | effort + PID | `FollowJointTrajectory` | MoveIt 2 execution, Servo |
| `osc_controller` | effort | `PoseStamped` target topic | Reach, Cartesian moves (M3) |
| `cartesian_impedance_controller` | effort | `PoseStamped` + stiffness params | Drawer task, contact (M3) |
| `joint_impedance_controller` | effort | `JointState` target topic | Phase B policy output |
| `parallel_gripper_action_controller` (stock) | position | `ParallelGripperCommand` action | Gripper |

- Each custom controller is a thin `controller_interface::ControllerInterface` wrapper around a class in the ROS-free core (`osc.hpp`, `impedance.hpp`). Dynamics come from Pinocchio.
- Real-time safe: targets go through `realtime_tools::RealtimeThreadSafeBox`. No allocation in `update()`. Gains come from YAML.
- The executive switches controllers through the `controller_manager` `switch_controller` service. Only one arm controller is active at a time.
- Each controller publishes a state topic (target, actual, error) for plots.

## Motion Planning (`arm_sandbox_moveit_config`)

- MoveIt 2 config for the Panda (SRDF, `kinematics.yaml` with KDL first, OMPL plus Pilz for linear moves).
- The planning scene gets the table as a static collision object and detected objects from `Detection3DArray` (boxes).
- MoveIt Servo runs in the same `move_group` launch, with keyboard/gamepad input for teleop (Phase B demos too).

## Perception (`arm_sandbox_perception`)

- Classical first: an HSV color mask on the scene RGB, back-projected with depth into a point cloud. The object pose is the centroid plus PCA for yaw (the task objects are boxes on a table). Output is `Detection3DArray` + TF `object_<id>`.
- Simulated hand-eye calibration: the camera extrinsic is randomized slightly per reset, then estimated from an ArUco/AprilTag on the gripper (M5 stretch).
- Learned detectors are out of scope for Phase A.

## Tasks and Executive (`arm_sandbox_tasks`)

Each task is a YAML file. The sim, the executive, the eval runner, and the Gym environment all read the same file:

```yaml
name: pick_place_cube
scene: objects/cube_and_bin.xml
randomize: {cube_xy: [[0.35, -0.2], [0.6, 0.2]], cube_yaw: [-3.14, 3.14], colors: true, lighting: true}
success: {type: object_in_region, object: cube, region: bin, hold_s: 1.0}
time_limit_s: 30
language: "put the red cube in the bin"   # used by Phase B
```

- Success predicates are a small fixed set (`ee_at_pose`, `object_in_region`, `object_on_object`, `joint_opened`). They are evaluated on ground truth.
- The executive is BehaviorTree.CPP 4 with one tree XML per task. Nodes: `DetectObjects`, `SelectGrasp` (top-down grasp from the object pose and yaw), `Pick`, `Place`, `OpenDrawer`, `CheckGrasp`. Recovery: `RetryUntilSuccessful` around `Pick`, and a `Fallback` to re-detect after a missed grasp.
- Skill actions (`Pick`, `Place`, `MoveToPose`) live in `arm_sandbox_tasks` as action servers and are what the BT nodes call.

## Evaluation (`arm_sandbox_eval`)

- Python CLI: `eval run --approach bt|policy:<name> --task pick_place_cube --episodes 50 --seed 0`.
- Per episode: `reset(seed+i)` → start the approach → poll the success predicate until success or timeout → log metrics.
- Metrics: success, time to completion, EE path length, jerk (RMS), joint-limit and collision violations (from MuJoCo contacts via ground truth), planning/inference latency.
- Output: `results/<date>_<approach>_<task>/` containing `episodes.csv`, `summary.json`, a rosbag2 (MCAP) per episode, and an MP4 of the scene camera for the first K episodes. `eval table` merges summaries into the comparison table (Markdown + CSV).

## Visualization (`arm_sandbox_viz`)

- C++ node using the Rerun C++ SDK. It subscribes only to standard topics (FR-14a): `/robot_description`, `/tf`, `/tf_static`, `/joint_states`, camera images and depth, `/scene/markers`, controller state topics, and `/display_planned_path`.
- It logs the URDF once (Rerun's URDF loader), then transforms per update, images, markers as boxes, planned paths as line strips, and joint and controller-error time series.
- The Rerun viewer runs as its own process (`rerun --serve-web`) in the container. The node connects to it over gRPC. With `network_mode: host`, open the web viewer in the host browser. Exact ports and flags get pinned to the Rerun version in M0.
- Optional: `web_video_server` for plain camera video.

## Phase B: Learning (`arm_sandbox_learning`)

- **Gym environment** (`ArmSandboxEnv`, Python): MuJoCo Python bindings load the same composed MJCF and task YAML. It steps physics at 1 kHz, and the shared controller core (pybind11) converts the policy action into torques.
- **Policy I/O (LeRobot-compatible):**
  - Observation: joint positions and velocities, gripper width, scene and wrist RGB (256×256), and the language instruction.
  - Action: absolute joint position targets + gripper command at 10 Hz, executed by the joint impedance core.
  - RL uses the same environment with low-dim observations (object pose from ground truth) for reach and pick.
- **Data:** the Phase A BT and Servo teleop are recorded to LeRobotDataset through the ROS 2 path (the realistic one), and also through the Gym path for scripted bulk demos.
- **Policies:** `policy_node` (Python) loads a checkpoint (ACT, Diffusion Policy, SmolVLA, SB3), subscribes to joint states and cameras, and publishes to `joint_impedance_controller` at 10 Hz. It runs on the GPU locally, and the eval runner treats it as `policy:<name>`.
- **VLM planner:** a Python node sends the scene image + instruction to Qwen-VL (Ollama on the host, over HTTP). It asks for a JSON skill list, validates it against a schema, and executes it through the skill actions. On invalid output it re-prompts once, then fails the episode.
- **Budget:** small models only locally (SmolVLA ~0.45B, ACT, DP). Larger fine-tunes are optional cloud jobs and are not part of any milestone's "done".

## Containers and Dev Environment

- `docker/Dockerfile`: `ros:jazzy` base, plus MoveIt 2, `ros2_control`, MuJoCo (pinned), Pinocchio, BehaviorTree.CPP, the Rerun SDK and CLI, and Python dependencies. Phase B adds a separate stage with PyTorch and LeRobot, so the Phase A image stays lean.
- `docker-compose.yml`, service `sandbox`: GPU reservation, `network_mode: host`, WSLg mounts (`/tmp/.X11-unix`, `/mnt/wslg`) for the native viewer and RViz2, workspace bind mount, and an idle `dev` command. The VS Code dev container attaches to it.
- Headless runs set `MUJOCO_GL=egl` (or `osmesa` in CI) and `viewer:=false`.

## Testing

- Kinematics/controller core: GoogleTest, including checks against Pinocchio. Written test-first.
- URDF vs MJCF FK agreement test.
- ROS 2 integration (`launch_testing`, headless): controllers load and activate, JTC reaches a joint goal, OSC reaches a Cartesian goal within tolerance, reset is deterministic in object poses for a given seed.
- Tasks: smoke test per task (reset + BT run headless, success within the time limit) for Phase A milestones.
- Python: pytest for the eval metrics, the Gym environment (`gymnasium.utils.env_checker`), and VLM output validation.
- Lint: `ament_clang_format`, `ament_clang_tidy`, `ruff`. Builds use `-Wall -Wextra -Wpedantic` with warnings as errors.

## CI (GitHub Actions)

Optional, as in fleet_master: the workflow is kept but may not run. One job builds the image and runs `make lint` and `make test` headless (OSMesa, no GPU). The same commands run locally.

## Repo Layout

See [`../PROJECT_STRUCTURE.md`](../PROJECT_STRUCTURE.md) for the full tree. In short:

```
arm-sandbox/
  src/arm_sandbox_*/      # colcon packages
  docker/  docker-compose.yml  .devcontainer/
  .github/workflows/ci.yml
  Makefile
  docs/  (REQUIREMENTS.md, PROJECT_STRUCTURE.md, specs/)
  results/                # eval outputs (gitignored)
```

## Risks

| Risk | Mitigation |
|---|---|
| `mujoco_ros2_control` immature on Jazzy, or missing cameras/effort | M0 spike. Fallback: own `MujocoSystem` hardware interface |
| URDF and MJCF disagree (frames, limits) | FK agreement test, with `robot.yaml` as the single naming source |
| GPU / OpenGL in Docker under WSL2 (EGL, WSLg) | Prove it in M0 (native viewer + offscreen camera); OSMesa fallback |
| 1 kHz lockstep too slow with cameras on | Cameras on their own thread at lower rate. Physics rate configurable (500 Hz fallback) |
| ROS 2 vs Gym behavior gap in Phase B | Shared controller core and same MJCF/task files. Gap measured by evaluating through ROS 2 |
| VRAM (8 GB) shared by rendering + policy inference | Small models, low camera resolution, inference in fp16 |

## Out of Scope

Real hardware, a second arm (the design allows it), Isaac Sim, mobile manipulation, learned perception in Phase A, multi-arm setups, and a custom browser viewer before M12.
