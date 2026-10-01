# Host targets (need Docker):          build, up, shell, down
# Container targets (in dev container): smoke, test
SHELL := /bin/bash
COMPOSE := docker compose -f docker/docker-compose.yml
# The container user mirrors the host user (docker/docker-compose.yml build args).
export USER_UID := $(shell id -u)
export USER_GID := $(shell id -g)
ROS_SETUP := source /opt/ros/humble/setup.bash
# ROS 2 uses the distro Python; never a conda/pyenv python3 that may be first on PATH.
PYTHON := /usr/bin/python3

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
	$(ROS_SETUP) && $(PYTHON) -m pytest -q tests/env

test:
	$(ROS_SETUP) && colcon build --symlink-install \
		&& colcon test && colcon test-result --verbose
