# arm-sandbox dev image: ROS 2 Jazzy + MuJoCo + MoveIt 2 + ros2_control + Rerun + Claude Code.
# One image for development, tests and demos (design spec: "Containers and Dev Environment").
ARG BASE_IMAGE=ros:jazzy-ros-base
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git curl ca-certificates sudo \
        python3-pip python3-venv python3-pytest python3-colcon-common-extensions \
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
# system Python, so packages ROS nodes import go there too (a venv would be invisible to them).
# numpy<2: Jazzy's compiled Python bindings (e.g. pinocchio/eigenpy) are built against NumPy 1.x.
ENV PIP_BREAK_SYSTEM_PACKAGES=1
ARG MUJOCO_VERSION=3.14.0
RUN pip install --no-cache-dir "mujoco==${MUJOCO_VERSION}" "numpy<2"

# rerun-sdk requires NumPy 2, so it gets its own venv; only its `rerun` CLI (the viewer) goes
# on PATH. ROS nodes never import it: the viz bridge uses the Rerun C++ SDK (design spec).
ARG RERUN_VERSION=0.38.1
RUN python3 -m venv /opt/rerun \
    && /opt/rerun/bin/pip install --no-cache-dir "rerun-sdk==${RERUN_VERSION}" \
    && ln -s /opt/rerun/bin/rerun /usr/local/bin/rerun

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Container user mirrors the host user (name, UID, GID come from docker-compose.yml build
# args), so files on the bind mount keep the right owner and host ~/.claude mounts at the same
# home path. The base image's `ubuntu` user holds UID/GID 1000 (the usual host IDs): remove it.
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN if id -u ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi

# Create a user matching the host's UID/GID
RUN groupadd -g ${USER_GID} ${USERNAME} && \
    useradd -l -u ${USER_UID} -g ${USERNAME} -m -s /bin/bash ${USERNAME} && \
    usermod -aG sudo ${USERNAME}

# Passwordless sudo: avoids syncing a host password into the container; sudo stays confined
# to this isolated dev container.
RUN mkdir -p /etc/sudoers.d && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

WORKDIR /workspace
RUN chown -R ${USERNAME}:${USER_GID} /workspace

USER ${USERNAME}

# Claude Code CLI (native installer, goes to ~/.local/bin). Sessions/config are NOT stored in
# the image: ~/.claude and ~/.claude.json are bind-mounted from the host (docker/docker-compose.yml).
RUN curl -fsSL https://claude.ai/install.sh | bash
ENV PATH="/home/${USERNAME}/.local/bin:${PATH}"

# Interactive shells get ROS 2 and the workspace overlay (if built).
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc \
    && echo '[ -f /workspace/arm-sandbox/install/setup.bash ] && source /workspace/arm-sandbox/install/setup.bash' >> ~/.bashrc

WORKDIR /workspace/arm-sandbox
CMD ["sleep", "infinity"]
