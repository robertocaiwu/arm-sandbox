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

- **Task 6** (`248656b`): `install_deps.sh` now builds the Rerun C++ SDK 0.38.1 into `/opt/rerun_cpp_sdk`. It skips the build if that version is already installed. Two smoke tests check the version match and that a small program compiles and links. **Open:** the Docker image still needs a host-side `make build`.
- **Task 7** (`e72be52`): added the `arm_sandbox_viz` package with the C++ `rerun_bridge`, which logs `/robot_description`, `/tf`, `/tf_static` and `/joint_states`. It either streams over gRPC or records to an `.rrd` file, and refuses to start with both or neither set. Its launch test checks the contents of a recording.
- **Task 8** (`67bb1ff`): `sim.launch.py` gained `rerun:=true` (starts `rerun --serve-web` and the bridge) and `rerun_save:=<file.rrd>`. A new full-stack test records the running Panda.
- Fixed the web viewer address everywhere (`291bb48`). The bare `http://localhost:9090` only opens Rerun's welcome page; the link that shows the sim is `http://localhost:9090/?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9876%2Fproxy`.
- **Task 9** (`f17cf1c`): recorded D15–D17 in `REQUIREMENTS.md`, updated the spec, `PROJECT_STRUCTURE.md` and `CLAUDE.md`, and added READMEs for the four packages.

### Learned

- `mujoco_ros2_control` 0.1.2 serves its services at `/mujoco_ros2_control_node/...`. Its own docs say `/ros2_control_node/...`, which is wrong for this version.
- Its control loop sleeps on sim time, so pausing the sim also blocks controller switches. Don't use pause, reset, activate.
- On Humble, `ros2_control_node` needs `~/robot_description` remapped to `/robot_description`.
- The sim package is `arm_sandbox_sim`, but the launch file lives in `arm_sandbox_bringup`: `ros2 launch arm_sandbox_bringup sim.launch.py`, or `make start_sim`.
- Rerun 0.38 resolves `package://` meshes from the ROS environment, even when the URDF comes from a string. Its named-frame `Transform3D` maps directly onto ROS TF (checked for Task 7).
- The Rerun web page needs the `?url=` link above to show data; `rerun --serve-web` prints it at startup ("connect at …").

### State at end of session

Plan 02 is complete on `feat/m1-sim-viewer` (not pushed yet). M1 is done: the Panda runs behind `ros2_control` and follows trajectories. M0 is done in code: `make start_sim` shows the sim in the native viewer and in Rerun. The visual browser check is still for the user to do. `make test` reports 38 tests, 0 failures. Still open: the host-side `make build` for the Rerun C++ SDK, then a push and PR. Next: Plan 03, M2 (kinematics).

---

## Session — 2026-10-03

**Plan:** [Plan 03 — Kinematics (M2) and the Reach Task](plan/2026-10-02-plan-03-kinematics-and-reach.md). Branch `feat/kinematics`.

### Decisions (recorded as D18–D19 in this session's docs commit)

- **D18:** the reach task runs as a `reach_runner` node (IK → one JTC goal → TF check), not a `MoveToPose` action. Skill actions come with MoveIt in M4.
- **D19:** analytic IK is deferred. M2 ships damped-least-squares IK.

### Changes

- Plan 02 merged to `master` as PR #3 (`c2c5fb7`), and Plan 03 as PR #4 (`1bb018d`). Plan 03 was written on 2026-10-02 from code built and run in a scratch workspace first (63 tests).
- **Task 1** (`e8e4994`): the `arm_sandbox_kinematics` package and `KinematicChain` (FK, geometric Jacobian, manipulability; Eigen + urdfdom, no ROS). The tests run on the real Panda URDF, which CMake generates from the xacro. FK and the Jacobian match Pinocchio to 1e-9, and the Jacobian also matches finite differences.
- **Task 2** (`1f88d39`): `solve_ik`, damped least squares with damping scaled by manipulability, an SVD null-space pull towards home, and joint clamping. Round trips from a nearby seed: 200/200; from home: 82.5% (floor 80%). The IK test binary takes about 0.6 s.
- Added `docs/LEARNING_RESOURCES.md`, background reading per plan (`71a5bfb`).
- **Task 3** (`c4218cc`): the `arm_sandbox_tasks` package with the ROS-free reach logic (task file, rpy poses, pose error, hold, move timing) and `config/tasks/reach.yaml`.
- **Task 4** (`336fe55`): `reach_runner`, `reach.launch.py`, `make start_reach`, and the end-to-end test in both gravcomp modes. All 5 targets are reached. Error after the 0.5 s hold: 1.6–4.1 mm with gravcomp off, 0.1–0.3 mm with it on.
- **Task 5** (this session's docs commit): D18–D19, the spec's Kinematics and Tasks sections, `PROJECT_STRUCTURE.md`, the `CLAUDE.md` status, and READMEs for `arm_sandbox_kinematics` and `arm_sandbox_tasks`.

### Learned

- **Plant a bug to test the tests.** A deliberately reversed roll/pitch/yaw order passed all the original task tests, because they only rotated about one axis, where order doesn't matter. Added a two-axis case (Task 3) and synced Plan 03. The same check showed that the Jacobian, joint-clamping and null-space tests each catch their own bug.
- **The red step fails at CMake configure** with `Cannot find source file` (the `.cpp` listed in CMake doesn't exist yet), before any "missing header" compile error. Plan 03 now says so.
- **`ros2 launch` (and `make start_reach`) exits 0 even when `reach_runner` exits 1.** The launch test is fine, because it checks the runner's own exit code. A shell script would need the launch to pass the code through (not done yet).
- **`mujoco_ros2_control` starts its MuJoCo window with both side panels hidden** (`ui0_enable`/`ui1_enable = false` in `mujoco_simulation.cpp`). Tab and Shift+Tab bring them back. The Control sliders don't move the arm, because `ros2_control` overwrites the actuator controls every update; use `ros2 action send_goal`.
- **Testing a temporary task file** with `--symlink-install` leaves a dangling symlink in `install/` after the source file is deleted. Remove it by hand.

### State at end of session

M2 is done: `make start_sim` + `make start_reach` reaches all 5 targets with the hand-written IK, and `make test` reports 63 tests, 0 failures. Still open: the M2 demo video (watch it in both viewers), pushing `feat/kinematics` and opening a PR, and the host-side `make build` for the Rerun C++ SDK (from Plan 02). Next: Plan 04, M3 (operational-space / impedance control with Pinocchio dynamics, drawer task).
