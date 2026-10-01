# arm-sandbox dev image: ROS 2 Humble + MuJoCo + MoveIt 2 + ros2_control + Rerun + Claude Code.
# One image for development, tests and demos (design spec: "Containers and Dev Environment").
# BASE_IMAGE must be Ubuntu 22.04 based; scripts/install_deps.sh adds ROS 2 if it's missing.
ARG BASE_IMAGE=ros:humble-ros-base
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# All dependencies come from one script, shared with non-Docker setups (DRY).
COPY scripts/install_deps.sh /tmp/install_deps.sh
RUN /tmp/install_deps.sh \
    && rm -rf /var/lib/apt/lists/* /tmp/install_deps.sh

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
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc \
    && echo '[ -f /workspace/arm-sandbox/install/setup.bash ] && source /workspace/arm-sandbox/install/setup.bash' >> ~/.bashrc

WORKDIR /workspace/arm-sandbox
CMD ["sleep", "infinity"]
