#!/usr/bin/env bash
# Install everything arm-sandbox needs on Ubuntu 22.04 (Jammy):
#   - ROS 2 Humble + MoveIt 2 + ros2_control + mujoco_ros2_control + Pinocchio + BehaviorTree.CPP (apt)
#   - build/test/lint tools and GL libraries for MuJoCo rendering (apt)
#   - MuJoCo Python bindings on the system Python (pip)
#   - Rerun viewer in its own venv (/opt/rerun), `rerun` CLI on PATH
#   - Rerun C++ SDK (built from the release bundle) in /opt/rerun_cpp_sdk, for arm_sandbox_viz
#
# Single source of truth for dependencies: docker/Dockerfile runs this script too.
# Safe to re-run. Privileged steps use sudo when not run as root.
#
# Usage:   scripts/install_deps.sh
# Pins:    MUJOCO_VERSION=x.y.z RERUN_VERSION=x.y.z scripts/install_deps.sh
set -euo pipefail

readonly ROS_DISTRO_NAME="humble"
readonly REQUIRED_CODENAME="jammy"
# Must match the MuJoCo bundled by ros-humble-mujoco-vendor (used by mujoco_ros2_control), so the
# Gymnasium training path and the ROS 2 sim run the same physics engine (decision D14).
readonly MUJOCO_VERSION="${MUJOCO_VERSION:-3.12.0}"
readonly RERUN_VERSION="${RERUN_VERSION:-0.38.1}"
readonly RERUN_VENV="/opt/rerun"
readonly RERUN_CPP_SDK_PREFIX="/opt/rerun_cpp_sdk"
# ROS 2 runs on the distro's Python. Never use whichever `python3` comes first on PATH
# (conda, pyenv, ...): packages installed there are invisible to ROS nodes.
readonly SYSTEM_PYTHON="/usr/bin/python3"

SUDO=()
if [ "$(id -u)" -ne 0 ]; then
    SUDO=(sudo)
fi

export DEBIAN_FRONTEND=noninteractive

check_os() {
    # shellcheck source=/dev/null
    . /etc/os-release
    if [ "${VERSION_CODENAME:-}" != "${REQUIRED_CODENAME}" ]; then
        echo "Error: ROS 2 ${ROS_DISTRO_NAME} needs Ubuntu 22.04 (${REQUIRED_CODENAME}), found ${PRETTY_NAME:-unknown}." >&2
        exit 1
    fi
}

ros_apt_repo_configured() {
    grep -rqs "packages.ros.org/ros2" /etc/apt/sources.list /etc/apt/sources.list.d/
}

setup_ros_apt_repo() {
    if ros_apt_repo_configured; then
        echo "ROS 2 apt repository already configured."
        return
    fi

    echo "Adding the ROS 2 apt repository..."
    "${SUDO[@]}" apt-get update
    "${SUDO[@]}" apt-get install -y --no-install-recommends software-properties-common curl ca-certificates
    "${SUDO[@]}" add-apt-repository -y universe

    # Official method: the ros2-apt-source package installs the repo and its signing key.
    local release
    release="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
    if [ -z "${release}" ]; then
        echo "Error: could not look up the latest ros-apt-source release." >&2
        exit 1
    fi

    local deb
    deb="$(mktemp --suffix=.deb)"
    curl -fsSL -o "${deb}" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${release}/ros2-apt-source_${release}.${REQUIRED_CODENAME}_all.deb"
    "${SUDO[@]}" dpkg -i "${deb}"
    rm -f "${deb}"
}

install_apt_packages() {
    echo "Installing apt packages..."
    "${SUDO[@]}" apt-get update
    "${SUDO[@]}" apt-get install -y --no-install-recommends \
        build-essential cmake git curl ca-certificates sudo \
        python3-pip python3-venv python3-pytest python3-colcon-common-extensions python3-rosdep \
        clang-format clang-tidy \
        libgl1 libegl1 libosmesa6 libglfw3 \
        "ros-${ROS_DISTRO_NAME}-ros-base" \
        "ros-${ROS_DISTRO_NAME}-rviz2" \
        "ros-${ROS_DISTRO_NAME}-xacro" \
        "ros-${ROS_DISTRO_NAME}-robot-state-publisher" \
        "ros-${ROS_DISTRO_NAME}-ros2-control" \
        "ros-${ROS_DISTRO_NAME}-ros2-controllers" \
        "ros-${ROS_DISTRO_NAME}-mujoco-ros2-control" \
        "ros-${ROS_DISTRO_NAME}-mujoco-ros2-control-plugins" \
        "ros-${ROS_DISTRO_NAME}-moveit" \
        "ros-${ROS_DISTRO_NAME}-moveit-servo" \
        "ros-${ROS_DISTRO_NAME}-moveit-resources-panda-moveit-config" \
        "ros-${ROS_DISTRO_NAME}-pinocchio" \
        "ros-${ROS_DISTRO_NAME}-behaviortree-cpp" \
        "ros-${ROS_DISTRO_NAME}-vision-msgs" \
        "ros-${ROS_DISTRO_NAME}-ament-cmake-pytest" \
        "ros-${ROS_DISTRO_NAME}-rmw-cyclonedds-cpp"
}

init_rosdep() {
    if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        return
    fi
    "${SUDO[@]}" rosdep init
}

install_python_packages() {
    # numpy<2: Humble's compiled Python bindings (e.g. pinocchio/eigenpy) are built against NumPy 1.x.
    echo "Installing MuJoCo ${MUJOCO_VERSION} for ${SYSTEM_PYTHON}..."
    "${SUDO[@]}" "${SYSTEM_PYTHON}" -m pip install --no-cache-dir \
        "mujoco==${MUJOCO_VERSION}" \
        "numpy<2"
}

install_rerun() {
    # rerun-sdk needs NumPy 2, so it lives in its own venv and only its `rerun` CLI (the viewer)
    # goes on PATH. ROS nodes never import it: the viz bridge uses the Rerun C++ SDK.
    echo "Installing Rerun ${RERUN_VERSION} into ${RERUN_VENV}..."
    if [ ! -x "${RERUN_VENV}/bin/python" ]; then
        "${SUDO[@]}" "${SYSTEM_PYTHON}" -m venv "${RERUN_VENV}"
    fi
    "${SUDO[@]}" "${RERUN_VENV}/bin/pip" install --no-cache-dir "rerun-sdk==${RERUN_VERSION}"
    "${SUDO[@]}" ln -sf "${RERUN_VENV}/bin/rerun" /usr/local/bin/rerun
}

install_rerun_cpp_sdk() {
    # arm_sandbox_viz links the Rerun C++ SDK, which has no apt package. Build it from the release
    # bundle at the viewer's version, so colcon builds find it offline (find_package(rerun_sdk)).
    # The bundle builds Apache Arrow: ~10 minutes.
    local version_file="${RERUN_CPP_SDK_PREFIX}/lib/cmake/rerun_sdk/rerun_sdkConfigVersion.cmake"
    if [ -f "${version_file}" ] && grep -qF "set(PACKAGE_VERSION \"${RERUN_VERSION}\")" "${version_file}"; then
        echo "Rerun C++ SDK ${RERUN_VERSION} already installed."
        return
    fi

    echo "Building the Rerun C++ SDK ${RERUN_VERSION} into ${RERUN_CPP_SDK_PREFIX}..."
    local work
    work="$(mktemp -d)"
    curl -fsSL -o "${work}/rerun_cpp_sdk.zip" \
        "https://github.com/rerun-io/rerun/releases/download/${RERUN_VERSION}/rerun_cpp_sdk.zip"
    "${SYSTEM_PYTHON}" -m zipfile -e "${work}/rerun_cpp_sdk.zip" "${work}"
    cmake -S "${work}/rerun_cpp_sdk" -B "${work}/build" \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${RERUN_CPP_SDK_PREFIX}"
    cmake --build "${work}/build" --config Release --target rerun_sdk -j"$(nproc)"
    "${SUDO[@]}" cmake --install "${work}/build"
    rm -rf "${work}"
}

main() {
    check_os
    setup_ros_apt_repo
    install_apt_packages
    init_rosdep
    install_python_packages
    install_rerun
    install_rerun_cpp_sdk

    cat <<EOF

arm-sandbox dependencies installed. Next steps (as your normal user):
  rosdep update
  source /opt/ros/${ROS_DISTRO_NAME}/setup.bash
  make smoke
EOF
}

main "$@"
