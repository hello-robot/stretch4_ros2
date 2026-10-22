# Docker workflow for Stretch Simulation. Run these from this directory.

BASE_IMAGE := stretch4-ros2:latest
IMAGE := stretch-simulation:latest
CONTAINER_NAME := stretch-sim
CONTAINER_HOME := /home/hello-robot

# Host checkouts mounted over the container's copies, so edits on the host are used in the container.
# stretch4_mujoco is only mounted if there is a checkout at MUJOCO_SRC.
ROS2_SRC ?= $(abspath ..)
MUJOCO_SRC ?= $(HOME)/repos/stretch4_mujoco

MOUNTS := --volume $(ROS2_SRC):$(CONTAINER_HOME)/ament_ws/src/stretch4_ros2
ifneq ($(wildcard $(MUJOCO_SRC)/pyproject.toml),)
MOUNTS += --volume $(MUJOCO_SRC):$(CONTAINER_HOME)/repos/stretch4_mujoco
endif

.PHONY: help build run shell test-gpu

help:
	@echo "make build     Build the CI base image, then the simulation image on top of it"
	@echo "make run       Run the simulation container with the GPU, X11 and the dev mounts"
	@echo "make shell     Open another shell in the running container"
	@echo "make test-gpu  Check that the container can see the GPU and render with EGL"
	@echo ""
	@echo "Mounted: $(MOUNTS)"

build:
	docker build -t $(BASE_IMAGE) -f ../.github/docker/Dockerfile ..
	docker build -t $(IMAGE) \
		--build-arg BASE_IMAGE=$(BASE_IMAGE) \
		--build-arg USER_UID=$$(id -u) \
		--build-arg USER_GID=$$(id -g) \
		.

run:
	@xhost +local:docker > /dev/null 2>&1 || true
	docker run -it --rm \
		--name $(CONTAINER_NAME) \
		--gpus all \
		--network host \
		--ipc host \
		--env DISPLAY=$(DISPLAY) \
		--volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
		$(MOUNTS) \
		$(IMAGE)

shell:
	docker exec -it $(CONTAINER_NAME) bash

test-gpu:
	docker run --rm --gpus all $(IMAGE) bash -c \
		"nvidia-smi && python3 -c 'import mujoco; c = mujoco.GLContext(64, 64); c.make_current(); c.free(); print(\"EGL rendering works\")'"
