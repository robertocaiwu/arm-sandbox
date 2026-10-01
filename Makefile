# Host targets (need Docker):          build, up, shell, down (each runs select-gpu first)
# Container targets (in dev container): smoke, test
SHELL := /bin/bash
# GPU=auto (default) enables the NVIDIA GPU when Docker has the nvidia runtime; force with
# `make up GPU=1` or `GPU=0`. scripts/select_gpu.sh writes the per-PC override file.
GPU ?= auto
export GPU
COMPOSE := docker compose -f docker/docker-compose.yml -f docker/docker-compose.local.yml
# The container user mirrors the host user (docker/docker-compose.yml build args).
export USER_UID := $(shell id -u)
export USER_GID := $(shell id -g)
ROS_SETUP := source /opt/ros/humble/setup.bash
# ROS 2 uses the distro Python; never a conda/pyenv python3 that may be first on PATH.
PYTHON := /usr/bin/python3

.PHONY: select-gpu build up shell down smoke test

select-gpu:
	scripts/select_gpu.sh

build: select-gpu
	$(COMPOSE) build sandbox

up: select-gpu
	$(COMPOSE) up -d sandbox

shell: select-gpu
	$(COMPOSE) exec sandbox bash

down: select-gpu
	$(COMPOSE) down

smoke:
	$(ROS_SETUP) && $(PYTHON) -m pytest -q tests/env

test:
	$(ROS_SETUP) && colcon build --symlink-install \
		&& colcon test && colcon test-result --verbose
