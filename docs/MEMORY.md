# arm-sandbox — Change History

A log of what changed, session by session, and why. **Append a new `## Session — <date>` section
at the bottom; never rewrite earlier sessions.** Each "State at end of session" note describes that
date only. The newest one is the current state.

Decisions are recorded in `docs/REQUIREMENTS.md` §10, designs in `docs/specs/`, and plans in
`docs/plan/`. This file says when things happened and what was learned along the way.

---

## Session — 2026-10-01

**Plan:** [Plan 01 — Foundation](plan/2026-10-01-plan-01-foundation.md) (repository, dev environment, PC handoff, M0 toolchain).

### Changes

- Created the standalone `arm-sandbox` repository with the requirements, the design spec, and Plan 01 (`22e0a9a`).
- Added the Docker dev environment: one image (`docker/Dockerfile`, which runs `scripts/install_deps.sh`), a Compose service `sandbox`, a VS Code dev container, the `Makefile`, and toolchain smoke tests in `tests/env/` (`7db8ecb`).
- Switched from ROS 2 Jazzy to **Humble** (D7), because the dev environment is Ubuntu 22.04 (`341dbce`).
- Split the GPU and CPU setups: `scripts/select_gpu.sh` writes a per-PC `docker/docker-compose.local.yml`, so the same config works with and without an NVIDIA GPU (`8466efb`, `f1adab0`).
- Merged the dev environment as PR #1 (`bd21b4e`).
- Ran the sim-backend spike (`5c95bed`), which led to:
  - **D13**: `mujoco_ros2_control` 0.1.2, with physics free-running on sim time (no lockstep).
  - **D14**: MuJoCo 3.12.0 everywhere, with the pip version pinned to the one `mujoco_vendor` ships.
- Vendored the MuJoCo Menagerie Panda MJCF as `arm_sandbox_description`.
- Checked the Rerun web viewer at 0.38.1: web viewer on port 9090, gRPC on port 9876.

### State at end of session

M0 environment working: `make smoke` passes, the Panda loads in MuJoCo, and the Rerun web viewer works. The browser bridge and `ros2_control` don't exist yet.

---

## Session — 2026-10-02

**Plan:** [Plan 02 — Sim Bring-up (M1) and Browser Viewer (finish M0)](plan/2026-10-02-plan-02-sim-bringup-and-viewer.md). Branch `feat/m1-sim-viewer`.

### Decisions (to be recorded as D15–D17 in Plan 02, Task 9)

- **D15, gravity compensation:** a launch argument, `gravcomp:=false` by default. With `false`, controllers compensate gravity. With `true`, MuJoCo compensates the robot's bodies only, like the real Panda's torque interface. Chosen over making it always on or always off.
- **D16, own URDF xacro:** upstream kinematics, plus the Menagerie meshes, inertials and limits, plus `panda_hand_tcp`. The upstream URDF uses `.dae` visual meshes that Rerun can't render.
- **D17, Rerun C++ SDK:** built by `install_deps.sh` into `/opt/rerun_cpp_sdk`, at the same version as the viewer (not done yet, Task 6).

### Changes

- Wrote Plan 02 (`e7e0577`). Before handing it over, its code was built and run in a scratch workspace (38 tests passing). That run found three problems, fixed in the plan before any task started:
  - **The arm fell at startup.** A spawner started after `reset_world` takes about 2 s, and the unpowered arm falls in that time. The fix is the `activate_at_home` helper.
  - **xacro turned `true` into `True`.** `${...}` evaluates the value, so the plan uses `$(arg ...)`.
  - **The URDF test read the wrong joints.** It picked up the `<ros2_control>` `<joint>` entries as if they were URDF joints.
- **Task 1** (`5f4f3e0`): the MJCF now uses the URDF names (`panda_*`), torque (`motor`) arm actuators and a position-controlled gripper on the finger tendon. It also gained the `panda_hand_tcp` site, a `home` keyframe and a 1 ms timestep. Added `panda/config/robot.yaml` and removed the unused `hand.xml` and `panda_nohand.xml`.
- Added `make viewer`, which opens the robot's MJCF in the native MuJoCo viewer without ROS (`e855668`).
- Merged `feat/mujoco` into `master` as PR #2 (`70476f1`), then merged `master` into `feat/m1-sim-viewer` (`78bf3d5`).
- **Task 2** (`988d960`): added `panda.urdf.xacro`. A test checks it against the MJCF: forward kinematics against Pinocchio on 50 random configurations, plus joint ranges, torque limits and total mass.
- **Task 3** (`9855c5b`): added the `arm_sandbox_sim` package with `scene.py` (`compose_scene` / `write_composed_scene`). It builds the scene at launch with absolute mesh paths and optional gravity compensation for the robot only. It has no ROS dependency, so the Phase B Gym environment can reuse it.
- **Task 4** (`17f8408`): added the `ros2_control` block (effort arm, position gripper), the `arm_sandbox_bringup` package (`panda_controllers.yaml`, `activate_at_home`, `sim.launch.py` with the `robot`/`viewer`/`gravcomp` arguments), and a bring-up launch test.
- **Task 5** (`509da51`, plan rename in `c928d6d`): the M1 acceptance test. The arm follows a joint trajectory and the gripper moves, with gravcomp off and on. The starting JTC gains needed no tuning. Added `make start_sim` (the plan originally called it `make sim`).

### Learned

- `mujoco_ros2_control` 0.1.2 serves its services at `/mujoco_ros2_control_node/...`. Its own docs say `/ros2_control_node/...`, which is wrong for this version.
- Its control loop sleeps on sim time, so pausing the sim also blocks controller switches. Don't use pause, reset, activate.
- On Humble, `ros2_control_node` needs `~/robot_description` remapped to `/robot_description`.
- The sim package is `arm_sandbox_sim`, but the launch file lives in `arm_sandbox_bringup`: `ros2 launch arm_sandbox_bringup sim.launch.py`, or `make start_sim`.
- Rerun 0.38 resolves `package://` meshes from the ROS environment, even when the URDF comes from a string. Its named-frame `Transform3D` maps directly onto ROS TF (checked for Task 7).

### State at end of session

M1 is done: `make start_sim` brings the Panda up behind `ros2_control`, and it follows trajectories. `make test` reports 31 tests, 0 failures. Tasks 6–9 are still to do: the Rerun C++ SDK, the `arm_sandbox_viz` bridge, the viewer in the launch file (finishing M0), and the docs.
