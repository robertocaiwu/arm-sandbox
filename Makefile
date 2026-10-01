# Host targets (need Docker):          build, up, shell, down
# Container targets (in dev container): smoke, test
SHELL := /bin/bash
COMPOSE := docker compose -f docker/docker-compose.yml
# The container user mirrors the host user (docker/docker-compose.yml build args).
export USER_UID := $(shell id -u)
export USER_GID := $(shell id -g)
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
