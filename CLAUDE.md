# arm-sandbox

A ROS 2 Humble + MuJoCo simulation sandbox built around a robot arm (Franka Panda) for learning and showing manipulation skills. Phase A covers applied manipulation (kinematics, control, MoveIt 2, perception, pick-and-place). Phase B covers robot learning (IL, RL, VLA, VLM), compared against the Phase A baseline.

**ALWAYS RESPOND IN ENGLISH**

> **Status:** requirements stage. No code exists yet. The structure, packages, and commands below are the **plan** from `docs/REQUIREMENTS.md`. Update this file as they become real.

## 📋 Core Working Principles

1. For maximum efficiency, whenever you need to perform multiple independent operations, invoke all relevant tools simultaneously and in parallel.
2. Before you finish, please verify your solution
3. Do what has been asked; nothing more, nothing less.
4. NEVER create files unless they're absolutely necessary for achieving your goal.
5. ALWAYS prefer editing an existing file to creating a new one.
6. NEVER proactively create documentation files (\*.md) or README files. Only create documentation files if explicitly requested by the User.
7. REQUIREMENTS, DECISIONS (§10) AND MILESTONES ARE IN ./docs/REQUIREMENTS.md. DESIGN IS IN ./docs/specs/. IMPLEMENTATION PLANS ARE IN ./docs/plan/. Read them before any design decision
8. PROJECT STRUCTURE IS IN ./docs/PROJECT_STRUCTURE.md

## 🏗️ Project Stack

- **ROS 2 Humble** (Ubuntu 22.04) - backbone; colcon workspace under `src/`
- **MuJoCo 3.x** - physics; connected to ROS 2 through `ros2_control` (`mujoco_ros2_control`, or a thin custom hardware interface as fallback)
- **C++17/20** (default) - controllers, hardware interface, kinematics (Eigen), perception, task executive, viz bridge
- **Python 3** (where clearly easier) - launch files, eval scripts, Gymnasium wrapper, learning code, VLM planner
- **ros2_control** - controllers and the hardware swap point (sim ↔ future real arm)
- **MoveIt 2** (+ MoveIt Servo) - motion planning and teleop
- **BehaviorTree.CPP** - task executive
- **Pinocchio** - reference to check the hand-written kinematics
- **Rerun** - browser viewer (via the `arm_sandbox_viz` bridge node); RViz2 and the native MuJoCo viewer for debugging
- **Phase B:** LeRobot (ACT, Diffusion Policy, SmolVLA, LeRobotDataset), Stable-Baselines3, Qwen-VL via Ollama
- **Testing** - GoogleTest (C++), pytest (Python), `launch_testing` (ROS 2 integration)
- **Docker** + NVIDIA Container Toolkit, VS Code devcontainer, GitHub Actions CI

## 🏛️ Architectural Principles

**"As simple as possible, but not simpler"**

- **KISS + DRY + YAGNI + Occam's Razor**: each new entity must justify its existence
- **Prior-art first**: look for existing ROS 2 packages and libraries first, then write our own. The exception is code written on purpose to learn (kinematics, custom controllers), which must be checked against a reference library
- **Documentation = part of code**: architectural decisions are recorded in code, comments, and the decisions log in `docs/REQUIREMENTS.md` §10
- **No premature optimization**
- **100% certainty**: evaluate cascading effects before changes

## 🚨 Code Quality Standards

**All code checks are mandatory - code must be ✅ CLEAN!**
No build errors. No compiler warnings (`-Wall -Wextra -Wpedantic`). No formatting or lint issues.

**Architectural standards:**

- Minimally sufficient patterns (don't overcomplicate)
- Decomposition: break tasks into subtasks
- Cascading effects: evaluate impact of changes
- Pure logic (kinematics, gating, layout math) lives in ROS-free libraries; ROS nodes are thin wrappers

## 🎯 Main Project Features

1. **MuJoCo sim behind `ros2_control`** - the arm is driven exactly as a real arm would be
2. **Robot-agnostic design** - everything robot-specific lives in config; the robot is a launch argument
3. **Applied manipulation stack (Phase A)** - hand-written kinematics, custom controllers (OSC/impedance), MoveIt 2, perception, behavior-tree pick-and-place
4. **Robot learning (Phase B)** - IL/RL/VLA trained on MuJoCo, deployed and evaluated as ROS 2 nodes
5. **Browser + native visualization** - Rerun in the browser and the MuJoCo viewer, both showing the same running sim
6. **Evaluation tool** - any approach × any task × N seeds → metrics, videos, rosbags, comparison table

## 🏗️ Architectural Patterns

### Control Flow (Phase A)

```
Perception → Task executive (BT) → MoveIt 2 / skills → ros2_control controllers → hardware interface → MuJoCo
```

- Object poses go from perception to TF and the MoveIt planning scene
- Controllers only talk to the hardware interface, never to MuJoCo directly

### Robot-Agnostic Layering

- **Per-robot package** (`arm_sandbox_description/<robot>/`): URDF/xacro, MJCF, meshes, `robot.yaml` (arm joints, base/EE frames, gripper, home pose)
- **Per-robot configs**: `ros2_control` controllers YAML, MoveIt 2 config
- **Generic code**: reads joint names, frames, and limits from parameters/config. Never hard-codes them
- **Swap point**: the `ros2_control` hardware interface (MuJoCo now, a real driver later)

### Training vs Deployment (Phase B)

- **Training**: Gymnasium wrapper directly on MuJoCo (no ROS 2 in the loop, for speed), using the same robot and task configs
- **Deployment/eval**: the trained policy runs as a ROS 2 node through the same stack and evaluation tool as Phase A
- **Common policy interface**: `obs (+ optional language instruction) → action`

### Visualization Data Flow

- Viewers depend only on standard ROS 2 topics (`/joint_states`, `/tf`, images, `visualization_msgs`), never on sim internals
- `arm_sandbox_viz` (C++) → Rerun web viewer. A custom browser viewer can replace it later (milestone M12) without touching the rest

## 📁 Project Structure (planned)

```
arm-sandbox/
  📦 src/          # colcon workspace: arm_sandbox_* ROS 2 packages
  🐳 docker/       # Dockerfile, docker-compose.yml (CPU) + docker-compose.gpu.yml (NVIDIA override)
  🛠️ scripts/      # install_deps.sh (all dependencies; also used by the Dockerfile)
  🐳 .devcontainer/ # gpu/ and cpu/ configs (VS Code asks which on Reopen in Container)
  📚 docs/         # REQUIREMENTS.md, PROJECT_STRUCTURE.md, design notes
  🛠️ Makefile      # build / test / run shortcuts
  ⚙️ .github/      # CI workflows
```

> 📖 **Detailed architecture**: full structure in `docs/PROJECT_STRUCTURE.md`

## ✅ Verification Checkpoints

**Stop and check** at these moments:

- After implementing a complete function, controller, or node
- After changing a URDF/MJCF, controller YAML, or launch file
- Before starting a new package or milestone
- Before declaring "done"

Run check (inside the dev container):

```bash
make smoke   # environment
make test    # colcon build + colcon test
```

For anything touching bring-up, also launch the sim headless and confirm it comes up without errors.

> Why: This prevents error accumulation and ensures code stability.

## 💻 Coding Standards

### FORBIDDEN:

- **NO robot-specific values in code** (joint names, link names, limits, frames, poses). Use `robot.yaml` / ROS parameters
- **NO magic numbers** - gains, tolerances, and timeouts go in config YAML
- **NO accessing MuJoCo from controllers, planners, or policies** - only through `ros2_control` / ROS 2 topics (training code via the Gymnasium wrapper is the one exception)
- **NO `std::cout` / `print`** in nodes - use `RCLCPP_*` / node loggers
- **NO raw `new`/`delete`** - use RAII and smart pointers
- **NO blocking calls or allocations in real-time paths** (`ros2_control` `update()`)
- **NO ignoring errors** - check return values, handle exceptions, fail loudly at startup on bad config
- **NO code duplication** - share via libraries, not copy-paste
- **NO TODOs** in final code

### Mandatory rules:

- C++ by default, Python where it is clearly easier
- C++: modern C++17/20, `const`-correctness, Eigen for linear algebra, follow `ament_clang_format` / `ament_clang_tidy`
- Python: type hints, follow `ruff`
- Pure-logic libraries have no ROS dependency and are unit-tested in isolation
- Every package has `package.xml` with complete dependencies
- **Meaningful names** for variables, functions, topics, and parameters
- **Early returns** to reduce nesting
- **Error handling** explicit and clear

## 📊 Implementation Standards

### Code is considered ready when:

- ✓ `colcon build` passes with no errors or warnings
- ✓ `colcon test` passes
- ✓ Formatting and lint applied (clang-format, ruff)
- ✓ Works end to end in sim (headless launch at minimum)
- ✓ Old/unused code removed
- ✓ Code is understandable to a junior developer
- ✓ Each package has a short README explaining the theory behind it (requirement NFR-6)

### Testing Strategy

- **Unit (GoogleTest / pytest)**: kinematics and control math. FK∘IK round-trip, Jacobian vs finite differences, results checked against Pinocchio
- **Integration (`launch_testing`)**: sim bring-up, controllers load, the arm reaches a commanded pose
- **Smoke tests**: each task resets and runs headless
- Math and control code → write tests first
- All tests run headless (CI has no GPU or display)

### **Security always**:

- No secrets or tokens in code, configs, or Dockerfiles
- Containers don't request `privileged` or root unless needed (GPU and display access are documented exceptions)
- Validate external inputs (service requests, config files)

## 🤝 Problem Solving

When stuck or confused:

1. **Stop** - Don't overcomplicate the solution
2. **Step back** - Re-read `docs/REQUIREMENTS.md`
3. **Simplify** - Simple solution is usually better
4. **Ask** - "I see two approaches: [A] vs [B]. Which aligns better with project standards?"

Your ideas for improvement are welcome - ask!

## 📘 Code Pattern Examples

### Robot-agnostic config (no hard-coded joints)

```cpp
// Joint names and frames come from parameters loaded from robot.yaml.
// Fail loudly at startup if the config is incomplete.
const auto joints = node->declare_parameter<std::vector<std::string>>("arm_joints", {});
const auto ee_frame = node->declare_parameter<std::string>("ee_frame", "");
if (joints.empty() || ee_frame.empty()) {
  RCLCPP_FATAL(node->get_logger(), "robot config missing 'arm_joints' or 'ee_frame'");
  throw std::runtime_error("invalid robot config");
}
```

### Pure library + thin node

```
arm_sandbox_kinematics/
  include/arm_sandbox_kinematics/kinematics.hpp   # pure C++ + Eigen, no rclcpp
  src/kinematics.cpp
  test/test_kinematics.cpp                        # GoogleTest, checked against Pinocchio
```

## 🛠️ Development Commands

### Main commands

Host (needs Docker):

- `make build` - build the `arm-sandbox:dev` image
- `make up` / `make down` - start / stop the `sandbox` container
- `make shell` - shell in the running container
- VS Code: **Dev Containers: Reopen in Container** (preferred) - pick **arm-sandbox (GPU)** on a PC with an NVIDIA GPU, **arm-sandbox (CPU)** anywhere else
- `make up` adds the GPU override automatically when Docker has the `nvidia` runtime (`GPU=auto`); force with `GPU=1` / `GPU=0`
- Without an NVIDIA GPU everything still works: MuJoCo renders on the CPU through Mesa (llvmpipe), which is fine for Phase A but too slow for Phase B training

Without Docker (any Ubuntu 22.04 machine or container):

- `scripts/install_deps.sh` - install all dependencies (uses sudo; safe to re-run). The same script builds the Docker image

Inside the dev container (or after `scripts/install_deps.sh`):

- `make smoke` - toolchain smoke tests (`tests/env/`)
- `make test` - `colcon build` + `colcon test`
- `make sim`, `make lint` - added by later plans

The repo is always mounted at `/workspace/arm-sandbox`, and host `~/.claude` is mounted into the container, so Claude Code sessions survive rebuilds and can move between PCs. The container user mirrors the host user (name, UID, GID). Local overrides (e.g. `MUJOCO_GL=osmesa`) go in `docker/.env`.

Python environments: ROS 2 nodes use the system Python with NumPy 1.x (`numpy<2`, required by Humble's compiled bindings). Tools that need NumPy 2 get their own venv: `rerun-sdk` lives in `/opt/rerun` (only its `rerun` CLI is on `PATH`).

### Development mode

- **Rerun web viewer**: served by the `arm_sandbox_viz` bridge (port set in its config)
- **Native viewer**: MuJoCo viewer through WSLg
- **RViz2**: MoveIt 2 interactive planning
- **Hot reloading**: not applicable. Rebuild with `colcon build --symlink-install` (Python and launch files pick up changes without a rebuild)

## 🌟 Key Project Features

### Hardware budget

- Local workstation: RTX 3070 (8 GB VRAM) under WSL2. Everything must run locally
- Phase B: small VLAs only locally (e.g. SmolVLA). Cloud GPUs only for occasional heavy jobs, never required

### Milestones

- Phase A: M0 environment → M1 ros2_control → M2 kinematics → M3 control → M4 MoveIt 2 → M5 perception → M6 evaluation
- Phase B: M7 data → M8 IL → M9 RL → M10 VLA/VLM → M11 polish
- Future: M12 custom browser viewer
- Each milestone ends with a demo video plus metrics. Details in `docs/REQUIREMENTS.md` §7

### Non-goals

- No real hardware (but nothing may block a port to one), no Isaac Sim, no mobile manipulation or AMR integration (that's `apps/amr-sim-lab`)

---

# Important Instructions Reminders

Do what has been asked; nothing more, nothing less.
NEVER create files unless they're absolutely necessary for achieving your goal.
ALWAYS prefer editing an existing file to creating a new one.
NEVER proactively create documentation files (\*.md) or README files. Only create documentation files if explicitly requested by the User.
