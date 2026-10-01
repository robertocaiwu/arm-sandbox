# Plan 01 — Foundation: Repository, Dev Environment, PC Handoff, M0 Toolchain

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn `arm-sandbox` into a standalone git repository (independent of the `projects` repo), give it a reproducible Docker dev environment that works on any Linux/WSL2 PC with an NVIDIA GPU, move development (including this Claude Code session) to a new PC, and prove the M0 toolchain: ROS 2 Jazzy, MuJoCo with the Panda, offscreen rendering, the Rerun web viewer, and a decision on the sim backend.

**Architecture:** One Docker image (`ros:jazzy-ros-base` plus MuJoCo, MoveIt 2, `ros2_control`, Rerun, and the Claude Code CLI). One Compose service, `sandbox`, idles so VS Code (Dev Containers) or `docker compose exec` can attach. The repo is mounted at the **same container path on every PC**, `/workspace/arm-sandbox`, and host `~/.claude` is mounted into the container. As a result, Claude Code sessions live on the host, survive container rebuilds, and can be moved between PCs by copying one file. Toolchain smoke tests (`tests/env/`) are the acceptance gate for the environment.

**Tech Stack:** git, GitHub CLI (`gh`), Docker + Compose v2, NVIDIA Container Toolkit, VS Code Dev Containers, ROS 2 Jazzy, MuJoCo (Python bindings), Rerun (`rerun-sdk`), pytest.

**Spec:** [`../specs/2026-10-01-arm-sandbox-design.md`](../specs/2026-10-01-arm-sandbox-design.md) (design) and [`../REQUIREMENTS.md`](../REQUIREMENTS.md) (requirements). Read both before starting.

## Global Constraints

- ROS 2 **Jazzy** (Ubuntu 24.04). MuJoCo **3.x**. (REQUIREMENTS §5, D7)
- Target hardware: **RTX 3070, 8 GB VRAM, WSL2**. Everything must run locally. (NFR-1)
- "Fully containerized. Nothing is installed on the host." The only host prerequisites are Docker, the NVIDIA driver + Container Toolkit, VS Code, git, and `gh`. (spec: Decisions)
- `arm-sandbox` is a **standalone repo**. Never add it to the `projects` repo (no submodule, no `git add` from `projects`), and never reference files outside the repo.
- Container path of the repo is **`/workspace/arm-sandbox`** on every PC, wherever it is cloned on the host. Don't change it, because session resume depends on it.
- Container user is **`ubuntu` (UID 1000)**, the default user of the Ubuntu 24.04 base image.
- Pinned dependencies (NFR-3): every pip package version is pinned in `docker/Dockerfile`.
- **No secrets in the repo** (CLAUDE.md, Security). The Claude session bundle contains private conversation history and is **never committed**.
- Robot-specific values live only in `src/arm_sandbox_description/` (FR-7).
- C++ by default, Python where clearly easier (D10). Every test in this plan is Python because it tests the toolchain, not product code.

## Where each step runs

Every step is tagged:

- **[host]**: a terminal on the PC itself (WSL2 shell or Linux shell), outside any container. Needs Docker.
- **[container]**: a terminal inside the `sandbox` dev container (VS Code terminal after "Reopen in Container", or `make shell`).
- **[new PC]**: a host terminal on the new development PC.

## File Structure

| Path | Responsibility |
|---|---|
| `.gitignore` | Ignore colcon output, eval results, local env, recordings, session bundles |
| `.gitattributes` | LF line endings everywhere (repo moves between Windows/WSL/Linux PCs) |
| `LICENSE` | Apache-2.0 (compatible with MuJoCo Menagerie) |
| `docker/Dockerfile` | The single dev image |
| `docker-compose.yml` | `sandbox` service: GPU, host network, X11, mounts (repo, `~/.claude`, git/ssh config) |
| `.devcontainer/devcontainer.json` | VS Code attaches to the `sandbox` service |
| `Makefile` | Host targets (`build`, `up`, `shell`, `down`) and container targets (`smoke`, `test`) |
| `tests/env/test_toolchain.py` | Smoke tests: ROS 2, ROS packages, MuJoCo sim + offscreen render, GPU, Claude CLI |
| `tests/env/test_rerun_web.py` | Smoke test: Rerun web viewer serves and accepts data |
| `src/arm_sandbox_description/` | First colcon package: vendored Menagerie Panda MJCF + its test |
| `CLAUDE.md`, `docs/PROJECT_STRUCTURE.md`, `docs/specs/…` | Updated to match what now exists |

## Out of scope for this plan (later plans)

- **Plan 02:** finish M0 (sim backend + `arm_sandbox_viz`, with the Panda visible in the browser and the native viewer at the same time) and M1 (`ros2_control` JTC, URDF, `robot.yaml`, torque-actuated MJCF).
- Plans 03+: one per milestone M2–M11, plus CI.

---

# Part 1 — Git Repository

### Task 1: Initialize the `arm-sandbox` repository

`arm-sandbox` is a **standalone repository**. It is not part of the `projects` repo: no submodule, nothing tracked by `projects`, and no references to files outside its own folder. On this PC it happens to be checked out at `/workspace/apps/arm-sandbox` (inside the `projects` checkout, where `apps/*` is gitignored), but that is only a location. On the new PC it is cloned on its own.

**Files:**
- Create: `.gitignore`, `.gitattributes`, `LICENSE`

**Interfaces:**
- Consumes: existing `CLAUDE.md`, `docs/REQUIREMENTS.md`, `docs/PROJECT_STRUCTURE.md`, `docs/specs/2026-10-01-arm-sandbox-design.md`, this plan.
- Produces: a git repo on branch `main` with one commit. Ignore rules later tasks rely on: `build/`, `install/`, `log/`, `results/`, `.env`, `*.claude-session.tar.gz`.

- [ ] **Step 1: Confirm the `projects` repo doesn't track it** [container or host, this PC]

```bash
cd /workspace/apps/arm-sandbox
git -C /workspace ls-files apps/arm-sandbox   # Expected: no output (nothing tracked by projects)
git -C /workspace check-ignore -q apps/arm-sandbox && echo ignored   # Expected: ignored
```

- [ ] **Step 2: Initialize on `main`**

```bash
git init -b main
git rev-parse --show-toplevel   # Expected: /workspace/apps/arm-sandbox (its own repo root, not /workspace)
git config user.name    # Expected: Roberto Cai (set globally). If empty: git config user.name "Roberto Cai"
```

- [ ] **Step 3: Write `.gitignore`**

```gitignore
# colcon
build/
install/
log/

# evaluation outputs (REQUIREMENTS FR-31; regenerated from configs)
results/

# local, per-PC settings (e.g. MUJOCO_GL override)
.env

# python
__pycache__/
*.pyc
.pytest_cache/

# recordings (rosbag2, Rerun)
*.mcap
*.db3
*.rrd

# Claude Code: local settings and exported session bundles (private history)
.claude/settings.local.json
*.claude-session.tar.gz
```

- [ ] **Step 4: Write `.gitattributes`**

```gitattributes
# The repo moves between Windows/WSL2 and Linux PCs: keep LF everywhere,
# otherwise shell scripts and the Dockerfile break with CRLF.
* text=auto eol=lf

*.png binary
*.jpg binary
*.stl binary
*.obj -diff
```

- [ ] **Step 5: Add the license**

```bash
curl -fsSL https://www.apache.org/licenses/LICENSE-2.0.txt -o LICENSE
grep -m1 "Version 2.0" LICENSE   # Expected: "Version 2.0, January 2004"
```

- [ ] **Step 6: Verify ignore rules**

```bash
mkdir -p build install log results && touch .env x.claude-session.tar.gz
git check-ignore build install log results .env x.claude-session.tar.gz
# Expected: all six paths printed
rm -rf build install log results .env x.claude-session.tar.gz
```

- [ ] **Step 7: First commit**

```bash
git add .gitignore .gitattributes LICENSE CLAUDE.md docs/
git status --short   # Expected: only the files above, staged (A)
git commit -m "chore: initialize arm-sandbox repository with requirements, design spec and plan"
git log --oneline    # Expected: one commit
git -C /workspace status --short | grep arm-sandbox   # Expected: no output (projects repo unaffected)
```

### Task 2: Create the GitHub remote and push

The repo starts **private**. Making it public is a portfolio step for later (REQUIREMENTS §8): `gh repo edit --visibility public --accept-visibility-change-consequences`.

**Files:** none

**Interfaces:**
- Produces: remote `origin` → `github.com/<your-login>/arm-sandbox`, with `main` tracking `origin/main`. Part 3 clones from it.

- [ ] **Step 1: Check GitHub auth** [host]

```bash
gh auth status   # Expected: "Logged in to github.com account <your-login>"
```

If it isn't logged in: `gh auth login` (choose SSH).

- [ ] **Step 2: Create the repo and push**

```bash
cd <arm-sandbox folder on this PC>   # /workspace/apps/arm-sandbox in the projects dev container
gh repo create arm-sandbox --private --source . --remote origin --push
```

Fallback without `gh`: create an empty private repo named `arm-sandbox` on github.com, then:

```bash
GH_LOGIN=$(gh api user -q .login 2>/dev/null || echo "<your-login>")
git remote add origin "git@github.com:${GH_LOGIN}/arm-sandbox.git"
git push -u origin main
```

- [ ] **Step 3: Verify**

```bash
git status -sb               # Expected: ## main...origin/main
git ls-remote origin main    # Expected: one line, same hash as `git rev-parse HEAD`
```

---

# Part 2 — Docker Dev Environment

### Task 3: Dev image, Compose service, and toolchain smoke tests

**Files:**
- Create: `tests/env/test_toolchain.py`, `docker/Dockerfile`, `docker-compose.yml`, `Makefile`

**Interfaces:**
- Consumes: Task 1 repo.
- Produces:
  - Image `arm-sandbox:dev`, Compose service `sandbox`, container user `ubuntu`, workdir `/workspace/arm-sandbox`.
  - Env in the container: `ROS_DISTRO=jazzy`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `MUJOCO_GL` (`egl` by default, overridable via `.env`), `NVIDIA_DRIVER_CAPABILITIES=all`.
  - Make targets: `build`, `up`, `shell`, `down` [host]; `smoke`, `test` [container].

- [ ] **Step 1: Write the failing smoke tests** [container or host]

Create `tests/env/test_toolchain.py`:

```python
"""Toolchain smoke tests: prove the dev image has what Phase A needs.

Run inside the dev container: `make smoke`.
These are the acceptance gate for docker/Dockerfile (Plan 01, Task 3).
"""

import os
import shutil
import subprocess

import pytest


def test_ros_distro_is_jazzy() -> None:
    assert os.environ.get("ROS_DISTRO") == "jazzy"


def test_rclpy_imports() -> None:
    import rclpy  # noqa: F401  (fails if pip broke the ROS Python environment)


@pytest.mark.parametrize(
    "package",
    [
        "controller_manager",
        "joint_trajectory_controller",
        "moveit_ros_move_group",
        "moveit_resources_panda_moveit_config",
        "robot_state_publisher",
        "xacro",
        "behaviortree_cpp",
        "vision_msgs",
        "rviz2",
    ],
)
def test_ros_package_installed(package: str) -> None:
    result = subprocess.run(
        ["ros2", "pkg", "prefix", package], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr


def test_pinocchio_imports() -> None:
    import pinocchio  # noqa: F401


def test_mujoco_simulates_a_falling_body() -> None:
    import mujoco

    model = mujoco.MjModel.from_xml_string(
        "<mujoco><worldbody><body pos='0 0 1'><freejoint/><geom size='0.1'/></body>"
        "</worldbody></mujoco>"
    )
    data = mujoco.MjData(model)
    start_height = data.qpos[2]
    for _ in range(100):
        mujoco.mj_step(model, data)
    assert data.qpos[2] < start_height


def test_mujoco_offscreen_render() -> None:
    # The backend comes from MUJOCO_GL: egl (GPU) or osmesa (CPU / CI fallback).
    import mujoco

    model = mujoco.MjModel.from_xml_string(
        "<mujoco><worldbody><light pos='0 0 3'/>"
        "<geom type='box' size='0.2 0.2 0.2' rgba='1 0 0 1'/></worldbody></mujoco>"
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, height=120, width=160)
    try:
        renderer.update_scene(data)
        image = renderer.render()
    finally:
        renderer.close()
    assert image.shape == (120, 160, 3)
    assert image.max() > 0, "rendered image is completely black"


def test_rerun_cli_available() -> None:
    assert shutil.which("rerun") is not None


def test_claude_cli_available() -> None:
    assert shutil.which("claude") is not None


def test_gpu_visible() -> None:
    if shutil.which("nvidia-smi") is None:
        pytest.skip("no NVIDIA GPU in this environment (e.g. CI)")
    result = subprocess.run(["nvidia-smi", "-L"], capture_output=True, text=True, check=False)
    assert result.returncode == 0 and "GPU" in result.stdout, result.stderr
```

- [ ] **Step 2: Run them outside the image to see them fail**

Run: `python3 -m pytest -q tests/env` (in the current workspace container, not the new image)
Expected: FAIL. At minimum `test_ros_distro_is_jazzy` fails (`None != 'jazzy'`), and `mujoco`/`rclpy` imports fail.

- [ ] **Step 3: Look up the versions to pin** [container or host, with internet]

```bash
python3 -m pip index versions mujoco 2>/dev/null | head -1      # e.g. "mujoco (3.x.y)"
python3 -m pip index versions rerun-sdk 2>/dev/null | head -1   # e.g. "rerun-sdk (0.x.y)"
```

Use the newest **MuJoCo 3.x** and the newest `rerun-sdk` in the ARG defaults in Step 4. Write the exact numbers, not ranges.

- [ ] **Step 4: Write `docker/Dockerfile`**

Replace `<MUJOCO_VERSION>` and `<RERUN_VERSION>` with the numbers from Step 3.

```dockerfile
# arm-sandbox dev image: ROS 2 Jazzy + MuJoCo + MoveIt 2 + ros2_control + Rerun + Claude Code.
# One image for development, tests and demos (design spec: "Containers and Dev Environment").
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git curl ca-certificates sudo \
        python3-pip python3-pytest python3-colcon-common-extensions \
        clang-format clang-tidy \
        libgl1 libegl1 libosmesa6 libglfw3 \
        ros-jazzy-rviz2 \
        ros-jazzy-xacro ros-jazzy-robot-state-publisher \
        ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
        ros-jazzy-moveit ros-jazzy-moveit-resources-panda-moveit-config \
        ros-jazzy-pinocchio ros-jazzy-behaviortree-cpp ros-jazzy-vision-msgs \
        ros-jazzy-ament-cmake-pytest \
        ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Ubuntu 24.04 marks the system Python as externally managed (PEP 668). ROS 2 runs on the
# system Python, so pip installs go there too (a venv would be invisible to ROS nodes).
# numpy<2: Jazzy's apt packages are built against NumPy 1.x.
ENV PIP_BREAK_SYSTEM_PACKAGES=1
ARG MUJOCO_VERSION=<MUJOCO_VERSION>
ARG RERUN_VERSION=<RERUN_VERSION>
RUN pip install --no-cache-dir \
        "mujoco==${MUJOCO_VERSION}" \
        "rerun-sdk==${RERUN_VERSION}" \
        "numpy<2"

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# The base image ships user `ubuntu` (UID 1000). Use it so files on the bind mount stay
# owned by the host user, and so the host ~/.claude can be mounted at /home/ubuntu/.claude.
RUN echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu \
    && chmod 0440 /etc/sudoers.d/ubuntu
USER ubuntu

# Claude Code CLI (native installer, goes to ~/.local/bin). Sessions/config are NOT stored in
# the image: ~/.claude and ~/.claude.json are bind-mounted from the host (docker-compose.yml).
RUN curl -fsSL https://claude.ai/install.sh | bash
ENV PATH="/home/ubuntu/.local/bin:${PATH}"

# Interactive shells get ROS 2 and the workspace overlay (if built).
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc \
    && echo '[ -f /workspace/arm-sandbox/install/setup.bash ] && source /workspace/arm-sandbox/install/setup.bash' >> ~/.bashrc

WORKDIR /workspace/arm-sandbox
CMD ["sleep", "infinity"]
```

- [ ] **Step 5: Write `docker-compose.yml`**

```yaml
# Dev environment for arm-sandbox. The `sandbox` service idles (sleep infinity) so VS Code
# Dev Containers or `make shell` can attach. See design spec "Containers and Dev Environment".
services:
  sandbox:
    build:
      context: .
      dockerfile: docker/Dockerfile
    image: arm-sandbox:dev
    network_mode: host # ROS 2 DDS discovery + Rerun web viewer reachable on host ports
    ipc: host # DDS shared-memory transport
    environment:
      - DISPLAY=${DISPLAY:-:0}
      - NVIDIA_DRIVER_CAPABILITIES=all # graphics (EGL/GL) too, not just CUDA
      - MUJOCO_GL=${MUJOCO_GL:-egl} # override in .env (e.g. osmesa) if EGL fails on a PC
    volumes:
      # Same container path on every PC: Claude Code keys sessions by working directory.
      - .:/workspace/arm-sandbox
      # X11 for the native MuJoCo viewer and RViz2 (WSLg exposes it here too).
      - /tmp/.X11-unix:/tmp/.X11-unix
      # Claude Code sessions, memory and login live on the host, not in the container.
      - ${HOME}/.claude:/home/ubuntu/.claude
      - ${HOME}/.claude.json:/home/ubuntu/.claude.json
      # git identity and SSH keys for push (read-only).
      - ${HOME}/.gitconfig:/home/ubuntu/.gitconfig:ro
      - ${HOME}/.ssh:/home/ubuntu/.ssh:ro
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
```

- [ ] **Step 6: Write `Makefile`** (recipe lines must start with a **tab**)

```make
# Host targets (need Docker):        build, up, shell, down
# Container targets (in dev container): smoke, test
SHELL := /bin/bash
COMPOSE := docker compose
ROS_SETUP := source /opt/ros/jazzy/setup.bash

.PHONY: build up shell down smoke test

build:
	$(COMPOSE) build sandbox

up:
	$(COMPOSE) up -d sandbox

shell:
	$(COMPOSE) exec sandbox bash

down:
	$(COMPOSE) down

smoke:
	$(ROS_SETUP) && python3 -m pytest -q tests/env

test:
	$(ROS_SETUP) && colcon build --symlink-install \
		&& colcon test && colcon test-result --verbose
```

- [ ] **Step 7: Make sure the bind-mount sources exist** [host]

If a bind-mount source file is missing, Docker creates a **directory** in its place, which breaks `~/.claude.json`.

```bash
mkdir -p ~/.claude ~/.ssh
touch ~/.claude.json ~/.gitconfig
id -u   # Expected: 1000. If not, see Task 4 (VS Code remaps the UID automatically).
```

- [ ] **Step 8: Build and start** [host]

```bash
cd <arm-sandbox folder>
make build   # first build pulls ~3-4 GB of ROS/MoveIt packages
make up
docker compose ps   # Expected: sandbox ... running
```

- [ ] **Step 9: Run the smoke tests in the container**

```bash
docker compose exec sandbox make smoke
```

Expected: all pass (`test_gpu_visible` passes on a GPU PC).

If `test_mujoco_offscreen_render` fails with an EGL error (a known risk on WSL2, see the spec's Risks table), switch to the CPU fallback and re-run:

```bash
echo "MUJOCO_GL=osmesa" > .env
make up                              # recreates the container with the new env
docker compose exec sandbox make smoke   # Expected: all pass
```

Record which backend worked on this PC in the commit message.

- [ ] **Step 10: Check the native viewer manually** [host]

```bash
xhost +local: 2>/dev/null || true   # native Linux X11 only. WSLg doesn't need it
docker compose exec sandbox python3 -m mujoco.viewer
```

Expected: an empty MuJoCo viewer window opens on your desktop. Close it.

- [ ] **Step 11: Commit**

```bash
git add docker/Dockerfile docker-compose.yml Makefile tests/env/test_toolchain.py
git commit -m "feat(env): dev image, compose service and toolchain smoke tests (MUJOCO_GL=<backend that worked>)"
git push
```

### Task 4: VS Code dev container with Claude Code

**Files:**
- Create: `.devcontainer/devcontainer.json`

**Interfaces:**
- Consumes: Compose service `sandbox` and user `ubuntu` (Task 3).
- Produces: "Reopen in Container" opens `/workspace/arm-sandbox` as `ubuntu`, with the Claude Code extension and CLI available. This is the entry point used in Part 3.

- [ ] **Step 1: Write `.devcontainer/devcontainer.json`**

```jsonc
{
  "name": "arm-sandbox",
  "dockerComposeFile": "../docker-compose.yml",
  "service": "sandbox",
  // Must stay identical on every PC: Claude Code keys sessions by this path.
  "workspaceFolder": "/workspace/arm-sandbox",
  "remoteUser": "ubuntu",
  // Remaps `ubuntu` to the host UID if it isn't 1000, so bind-mounted files keep the right owner.
  "updateRemoteUserUID": true,
  "shutdownAction": "stopCompose",
  "customizations": {
    "vscode": {
      "extensions": [
        "anthropic.claude-code",
        "ms-vscode.cpptools",
        "ms-vscode.cmake-tools",
        "ms-python.python",
        "charliermarsh.ruff"
      ]
    }
  }
}
```

- [ ] **Step 2: Open it** [host]

```bash
make down   # free the container started in Task 3
code <arm-sandbox folder>
```

In VS Code: Command Palette → **Dev Containers: Reopen in Container**.
Expected: VS Code reconnects. The bottom-left shows `Dev Container: arm-sandbox`.

- [ ] **Step 3: Verify inside the dev container** [container]

```bash
pwd                 # Expected: /workspace/arm-sandbox
whoami              # Expected: ubuntu
claude --version    # Expected: a version string
ls ~/.claude/projects | head   # Expected: the host's project folders (e.g. -workspace). The mount works
make smoke          # Expected: all pass
```

- [ ] **Step 4: Commit**

```bash
git add .devcontainer/devcontainer.json
git commit -m "feat(env): VS Code dev container attached to the sandbox service"
git push
```

### Task 5: Bring the docs in line with the environment

**Files:**
- Modify: `CLAUDE.md` (Core Working Principles item 7, "Development Commands (planned)" section, "Verification Checkpoints" run check)
- Modify: `docs/PROJECT_STRUCTURE.md` (Project Tree, Commands table)
- Modify: `docs/specs/2026-10-01-arm-sandbox-design.md` ("Containers and Dev Environment" section)

**Interfaces:**
- Consumes: Make targets and the paths from Tasks 3–4.
- Produces: docs that match reality, which the resumed Claude session on the new PC reads first.

- [ ] **Step 1: `CLAUDE.md`, item 7** — replace the line with:

```markdown
7. REQUIREMENTS, DECISIONS (§10) AND MILESTONES ARE IN ./docs/REQUIREMENTS.md. DESIGN IS IN ./docs/specs/. IMPLEMENTATION PLANS ARE IN ./docs/plan/. Read them before any design decision
```

- [ ] **Step 2: `CLAUDE.md`, "Development Commands (planned)"** — replace the "Main commands" list with:

```markdown
### Main commands

Host (needs Docker):

- `make build` - build the `arm-sandbox:dev` image
- `make up` / `make down` - start / stop the `sandbox` container
- `make shell` - shell in the running container
- VS Code: **Dev Containers: Reopen in Container** (preferred)

Inside the dev container:

- `make smoke` - toolchain smoke tests (`tests/env/`)
- `make test` - `colcon build` + `colcon test`
- `make sim`, `make lint` - added by later plans

The repo is always mounted at `/workspace/arm-sandbox`, and host `~/.claude` is mounted into the container, so Claude Code sessions survive rebuilds and can move between PCs.
```

- [ ] **Step 3: `CLAUDE.md`, "Verification Checkpoints"** — replace the "Run check (planned)" code block with:

```bash
make smoke   # environment
make test    # colcon build + colcon test
```

- [ ] **Step 4: `docs/PROJECT_STRUCTURE.md`** — in the Project Tree add these lines in alphabetical position under `arm-sandbox/`:

```
├── docker-compose.yml               # `sandbox` dev service (GPU, host network, X11, ~/.claude mount)
├── tests/env/                       # toolchain smoke tests (make smoke)
```

and under `docs/`:

```
│   ├── specs/                       # design specs
│   └── plan/                        # implementation plans (one per milestone group)
```

Replace the Commands table with:

```markdown
| Command        | Where     | Description                                   |
| -------------- | --------- | --------------------------------------------- |
| `make build`   | host      | Build the `arm-sandbox:dev` image             |
| `make up/down` | host      | Start / stop the `sandbox` container          |
| `make shell`   | host      | Shell into the running container              |
| `make smoke`   | container | Toolchain smoke tests                         |
| `make test`    | container | `colcon build` + `colcon test`                |
```

- [ ] **Step 5: Design spec, "Containers and Dev Environment"** — replace the `docker-compose.yml` bullet with:

```markdown
- `docker-compose.yml`, service `sandbox`: GPU reservation, `network_mode: host`, `ipc: host`, X11 socket mount (works for WSLg and native Linux), repo mounted at `/workspace/arm-sandbox` on every PC, host `~/.claude` + `~/.claude.json` mounted (Claude Code sessions persist and move between PCs), and an idle `sleep infinity` command. The VS Code dev container attaches to it as user `ubuntu`.
```

- [ ] **Step 6: Check, then commit**

```bash
grep -n "plan/" CLAUDE.md                    # Expected: item 7 line
grep -n "make smoke" CLAUDE.md docs/PROJECT_STRUCTURE.md   # Expected: hits in both
git add CLAUDE.md docs/
git commit -m "docs: document dev environment commands and plan location"
git push
```

---

# Part 3 — Move to the New PC (and resume this Claude session)

How it works: Claude Code stores each session as `~/.claude/projects/<encoded-cwd>/<session-id>.jsonl`. `<encoded-cwd>` is the working directory with `/` replaced by `-`. This planning session ran with cwd `/workspace` (the `projects` dev container), so it lives at `~/.claude/projects/-workspace/e1241580-0684-4d9c-9572-33d556f294fe.jsonl`. On the new PC, Claude runs in the `arm-sandbox` container at `/workspace/arm-sandbox` (encoded `-workspace-arm-sandbox`), so the file goes into that folder.

Older messages in this session refer to the repo as `/workspace/apps/arm-sandbox` (its location inside the `projects` checkout on this PC). After resuming, the first message tells Claude the repo is now at `/workspace/arm-sandbox`. CLAUDE.md and the docs only use repo-relative paths, so nothing else changes.

### Task 6: Export the session bundle (old PC, last thing before switching)

**Files:** none in the repo. The bundle goes to your home directory and is never committed (`*.claude-session.tar.gz` is ignored).

**Interfaces:**
- Produces: `~/arm-sandbox.claude-session.tar.gz` containing `<SESSION_ID>.jsonl` and `memory/`.

- [ ] **Step 1: Make sure everything is pushed** [host]

```bash
cd <arm-sandbox folder>
git status -sb   # Expected: ## main...origin/main, with no changes listed
```

- [ ] **Step 2: Exit the Claude Code session** you want to move (`/exit`, or close the panel), so the `.jsonl` file is complete.

- [ ] **Step 3: Create the bundle** [host]

```bash
SESSION_ID=e1241580-0684-4d9c-9572-33d556f294fe
SRC=~/.claude/projects/-workspace
ls -l "${SRC}/${SESSION_ID}.jsonl"   # Expected: the file exists (~1 MB)
tar -czf ~/arm-sandbox.claude-session.tar.gz -C "${SRC}" "${SESSION_ID}.jsonl" memory
tar -tzf ~/arm-sandbox.claude-session.tar.gz   # Expected: the .jsonl and memory/
```

If you continued the conversation in a different session, find its ID with `ls -t ~/.claude/projects/-workspace/*.jsonl | head -3` (most recent first) and use that instead.

- [ ] **Step 4: Copy the bundle to the new PC** through a private channel (USB drive, private cloud folder, `scp`). It contains the full conversation, so don't put it in git or anywhere public.

### Task 7: Set up the new PC and resume

**Files:** none

**Interfaces:**
- Consumes: GitHub repo (Task 2), dev container (Task 4), bundle (Task 6).
- Produces: a working dev container on the new PC, and this session resumable with `claude --resume`.

- [ ] **Step 1: Check prerequisites** [new PC]

```bash
docker --version && docker compose version     # Docker Engine (or Docker Desktop + WSL2) and Compose v2
docker run --rm --gpus all ubuntu:24.04 nvidia-smi -L   # Expected: GPU listed (NVIDIA driver + Container Toolkit OK)
git --version && gh auth status                # git + GitHub CLI logged in (SSH)
code --version                                 # VS Code, plus the "Dev Containers" extension installed
id -u                                          # 1000 expected. Other values are handled by updateRemoteUserUID
mkdir -p ~/.claude ~/.ssh && touch ~/.claude.json ~/.gitconfig
git config --global user.name >/dev/null || git config --global user.name "Roberto Cai"
```

- [ ] **Step 2: Clone** [new PC]. The host location doesn't matter, because the container path is fixed.

```bash
gh repo clone arm-sandbox ~/arm-sandbox
cd ~/arm-sandbox && git log --oneline | head -3   # Expected: the docs commit from Task 5 on top
```

- [ ] **Step 3: Restore the session** [new PC]

```bash
SESSION_ID=e1241580-0684-4d9c-9572-33d556f294fe
DEST=~/.claude/projects/-workspace-arm-sandbox
mkdir -p "${DEST}"
tar -xzf /path/to/arm-sandbox.claude-session.tar.gz -C "${DEST}"
ls "${DEST}"   # Expected: <SESSION_ID>.jsonl  memory
```

- [ ] **Step 4: Open the dev container** [new PC]

```bash
code ~/arm-sandbox
```

Command Palette → **Dev Containers: Reopen in Container**. The first build takes a while.

- [ ] **Step 5: Verify the environment** [container]

```bash
make smoke   # Expected: all pass. If the offscreen render fails: echo "MUJOCO_GL=osmesa" > .env, then rebuild the container
```

- [ ] **Step 6: Resume the session** [container]

```bash
claude            # first run only: log in when prompted, then /exit
claude --resume e1241580-0684-4d9c-9572-33d556f294fe
```

Expected: the earlier conversation history is shown. Send this first: *"The arm-sandbox repo is now a standalone checkout at /workspace/arm-sandbox (it used to be /workspace/apps/arm-sandbox). Use that path from now on."* Then check it by asking: *"Which task of docs/plan/2026-10-01-plan-01-foundation.md is next?"* The answer should be Part 4, Task 8. In the VS Code Claude Code panel, the session also appears in the past-conversations list for this workspace.

If `--resume` reports no such session, check that `pwd` is exactly `/workspace/arm-sandbox` and that the `.jsonl` sits in `~/.claude/projects/-workspace-arm-sandbox/`.

---

# Part 4 — M0 Toolchain (on the new PC)

### Task 8: Vendor the Panda MJCF as the first colcon package

Uses the unmodified MuJoCo Menagerie Panda. The torque-actuated variant, URDF, and `robot.yaml` come in Plan 02.

**Files:**
- Create: `src/arm_sandbox_description/package.xml`
- Create: `src/arm_sandbox_description/CMakeLists.txt`
- Create: `src/arm_sandbox_description/panda/mjcf/` (vendored from Menagerie `franka_emika_panda/`, plus a `MENAGERIE_COMMIT` file)
- Test: `src/arm_sandbox_description/test/test_panda_mjcf.py`

**Interfaces:**
- Consumes: `make test` (Task 3).
- Produces:
  - Package `arm_sandbox_description`, which installs `share/arm_sandbox_description/panda/`.
  - MJCF entry points `panda/mjcf/panda.xml` (arm + hand) and `panda/mjcf/scene.xml` (arm on a floor, with lights).
  - Menagerie joint names: `joint1`…`joint7`, `finger_joint1`, `finger_joint2`. Plan 02 maps them to URDF names.

- [ ] **Step 1: Write the failing test** [container]

Create `src/arm_sandbox_description/test/test_panda_mjcf.py`:

```python
"""The vendored MuJoCo Menagerie Panda loads and simulates."""

from pathlib import Path

import mujoco
import numpy as np

MJCF_DIR = Path(__file__).resolve().parents[1] / "panda" / "mjcf"


def load(file_name: str) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(MJCF_DIR / file_name))


def test_panda_has_seven_arm_and_two_finger_joints() -> None:
    model = load("panda.xml")
    names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)}
    assert {f"joint{i}" for i in range(1, 8)} <= names
    assert {"finger_joint1", "finger_joint2"} <= names


def test_scene_simulates_one_second_without_nan() -> None:
    model = load("scene.xml")
    data = mujoco.MjData(model)
    steps = int(1.0 / model.opt.timestep)
    for _ in range(steps):
        mujoco.mj_step(model, data)
    assert np.all(np.isfinite(data.qpos))
```

- [ ] **Step 2: Run it to see it fail**

Run: `python3 -m pytest -q src/arm_sandbox_description/test`
Expected: FAIL. MuJoCo can't find the `panda.xml` file (`ValueError: ... No such file or directory`).

- [ ] **Step 3: Vendor the model**

```bash
git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie.git /tmp/menagerie
MJCF=src/arm_sandbox_description/panda/mjcf
mkdir -p "${MJCF}"
cp -r /tmp/menagerie/franka_emika_panda/. "${MJCF}/"
git -C /tmp/menagerie rev-parse HEAD > "${MJCF}/MENAGERIE_COMMIT"
ls "${MJCF}"      # Expected: panda.xml scene.xml hand.xml assets/ LICENSE ...
du -sh "${MJCF}"  # Expected: a few tens of MB at most. Stop and ask if much larger
rm -rf /tmp/menagerie
```

- [ ] **Step 4: Write `package.xml` and `CMakeLists.txt`**

`src/arm_sandbox_description/package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>arm_sandbox_description</name>
  <version>0.1.0</version>
  <description>Robot models and per-robot config for arm-sandbox. The only place robot-specific values live.</description>
  <maintainer email="MAINTAINER_EMAIL">Roberto Cai</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <test_depend>ament_cmake_pytest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

`src/arm_sandbox_description/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(arm_sandbox_description)

find_package(ament_cmake REQUIRED)

# One folder per robot (FR-7). Installed as share/arm_sandbox_description/<robot>/.
install(DIRECTORY panda DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_panda_mjcf test/test_panda_mjcf.py)
endif()

ament_package()
```

Fill in the maintainer email from your git identity. The repo will become public, so use the address you're happy to publish (e.g. your GitHub noreply address):

```bash
sed -i "s/MAINTAINER_EMAIL/$(git config user.email)/" src/arm_sandbox_description/package.xml
grep maintainer src/arm_sandbox_description/package.xml
```

- [ ] **Step 5: Run the tests to see them pass**

Run: `python3 -m pytest -q src/arm_sandbox_description/test`
Expected: 2 passed.

Run: `make test`
Expected: build succeeds. `colcon test-result` reports 0 errors and 0 failures (`test_panda_mjcf` passed).

- [ ] **Step 6: Look at it in the native viewer**

```bash
python3 -m mujoco.viewer --mjcf=src/arm_sandbox_description/panda/mjcf/scene.xml
```

Expected: the Panda appears on a floor. Close the window.

- [ ] **Step 7: Commit**

```bash
git add src/arm_sandbox_description
git commit -m "feat(description): vendor MuJoCo Menagerie Panda (commit $(cat src/arm_sandbox_description/panda/mjcf/MENAGERIE_COMMIT))"
git push
```

### Task 9: Rerun web viewer smoke test

**Files:**
- Test: `tests/env/test_rerun_web.py`

**Interfaces:**
- Consumes: `rerun-sdk` from the image (Task 3).
- Produces: confirmed defaults that Plan 02's `arm_sandbox_viz` relies on: web viewer on **port 9090**, gRPC on **port 9876**, started with `rerun --serve-web`. If the pinned version uses different ports or flags, put the real ones in the test constants and in the design spec's Visualization section.

- [ ] **Step 1: Write the failing test** [container]

Create `tests/env/test_rerun_web.py`:

```python
"""Rerun web viewer serves over HTTP and accepts data over gRPC.

Plan 02's arm_sandbox_viz depends on these ports and this CLI flag.
"""

import subprocess
import time
import urllib.request

import pytest

WEB_VIEWER_URL = "http://localhost:9090"
STARTUP_TIMEOUT_S = 30.0


def wait_for_http(url: str, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            with urllib.request.urlopen(url, timeout=2) as response:
                if response.status == 200:
                    return True
        except OSError:
            time.sleep(0.5)
    return False


@pytest.fixture
def rerun_server():
    process = subprocess.Popen(
        ["rerun", "--serve-web"], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE
    )
    try:
        yield process
    finally:
        process.terminate()
        process.wait(timeout=10)


def test_web_viewer_serves_and_accepts_data(rerun_server: subprocess.Popen) -> None:
    import rerun as rr

    assert wait_for_http(WEB_VIEWER_URL, STARTUP_TIMEOUT_S), "web viewer did not come up"
    rr.init("arm_sandbox_smoke")
    rr.connect_grpc()  # default: the local gRPC server started by --serve-web
    rr.log("smoke/origin", rr.Points3D([[0.0, 0.0, 0.0]]))
    rr.disconnect()
    assert rerun_server.poll() is None, "rerun server exited while receiving data"
```

- [ ] **Step 2: Run it**

Run: `python3 -m pytest -q tests/env/test_rerun_web.py`
Expected: PASS if the pinned version uses these defaults. If it fails, run `rerun --help | grep -iE "serve|port"` and `python3 -c "import rerun as rr; help(rr.connect_grpc)"`. Change the URL, flag, or call in the test to the documented ones, then re-run until it passes. This test defines the contract, so it's expected to change if the defaults differ.

- [ ] **Step 3: Check it from the host browser**

Start `rerun --serve-web` in the container, and in another container terminal run:

```bash
python3 -c "import rerun as rr; rr.init('arm_sandbox_smoke'); rr.connect_grpc(); rr.log('smoke/origin', rr.Points3D([[0,0,0]], radii=0.05))"
```

Open `http://localhost:9090` in the host browser. Expected: the viewer loads and shows one point at the origin. Stop the server with Ctrl+C.

- [ ] **Step 4: Commit** (if the ports or flags differed, also update the design spec's Visualization section in the same commit)

```bash
git add tests/env/test_rerun_web.py docs/specs/2026-10-01-arm-sandbox-design.md
git commit -m "test(env): Rerun web viewer smoke test"
git push
```

### Task 10: Sim backend spike (`mujoco_ros2_control` vs own hardware interface)

A timeboxed investigation (max 1 day) whose deliverable is a **recorded decision**, not code in the repo. Plan 02 implements whichever backend wins.

**Files:**
- Modify: `docs/specs/2026-10-01-arm-sandbox-design.md` (Decisions bullet "Sim backend chosen by an M0 spike", Risks row 1)
- Modify: `docs/REQUIREMENTS.md` §10 (add a decision row)

**Interfaces:**
- Consumes: the Panda MJCF (Task 8), the dev image (Task 3).
- Produces: decision **D13: sim backend** = `mujoco_ros2_control` (with the version or commit) **or** "own `MujocoSystem` hardware interface". Plan 02 is written from this.

- [ ] **Step 1: Check for a binary package** [container]

```bash
sudo apt-get update && apt-cache policy ros-jazzy-mujoco-ros2-control
```

Note whether a candidate version exists.

- [ ] **Step 2: Build it from source in a scratch workspace** (outside the repo)

```bash
mkdir -p ~/spike_ws/src && cd ~/spike_ws/src
git clone https://github.com/ros-controls/mujoco_ros2_control.git
cd ~/spike_ws
source /opt/ros/jazzy/setup.bash
sudo rosdep init 2>/dev/null || true; rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install 2>&1 | tail -20
```

Expected: build finishes. If it fails, record the error, and that already counts toward the "own interface" side of the decision rule.

- [ ] **Step 3: Run its demo** following the repo README's launch command, then fill in this checklist from what you observe and from its source/docs:

| Need (design spec) | Supported? | Evidence (file/topic/param) |
|---|---|---|
| Builds on Jazzy, and the MuJoCo version it needs is compatible with the image's pin | | |
| `effort` command interface on arm joints | | |
| `position` command interface (gripper) | | |
| Publishes `/clock`; physics stepped in lockstep with the controller update | | |
| Camera rendering to ROS image topics (RGB + depth + CameraInfo) | | |
| Reset / set-state hook (keyframe or qpos) | | |
| Passive native viewer option | | |

- [ ] **Step 4: Apply the decision rule**

- **Adopt `mujoco_ros2_control`** if the first four rows are all "yes". Cameras, reset, and the viewer can be added in our own `arm_sandbox_sim` node next to it.
- **Otherwise, write our own `MujocoSystem`** hardware interface in `arm_sandbox_sim` (the fallback in the spec).

- [ ] **Step 5: Record the decision**

In `docs/specs/2026-10-01-arm-sandbox-design.md`, replace the "Sim backend chosen by an M0 spike" bullet with:

```markdown
- **Sim backend: <mujoco_ros2_control @ <commit/version> | own MujocoSystem hardware interface>.** Decided by the M0 spike (Plan 01, Task 10): <one sentence on the deciding checklist rows>. <If adopted: what arm_sandbox_sim adds on top (e.g. cameras, reset).>
```

Update the first row of the Risks table to match (resolved, or the remaining risk). In `docs/REQUIREMENTS.md` §10, add:

```markdown
| D13 | Sim backend: <choice> | M0 spike checklist (Plan 01, Task 10) |
```

- [ ] **Step 6: Commit and clean up**

```bash
cd /workspace/arm-sandbox
git add docs/
git commit -m "docs: record sim backend decision from M0 spike"
git push
rm -rf ~/spike_ws
```

---

## Done when

- `arm-sandbox` is its own git repo on GitHub (private), with every task's commit pushed.
- On the new PC: "Reopen in Container" works, `make smoke` and `make test` pass, the Panda shows in the native viewer, and Rerun shows data at `http://localhost:9090`.
- This Claude Code session resumes on the new PC with `claude --resume e1241580-0684-4d9c-9572-33d556f294fe`.
- Decision D13 (sim backend) is recorded. Next: write Plan 02 (finish M0 + M1) from the spec and D13.
