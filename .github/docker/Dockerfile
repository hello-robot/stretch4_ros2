# Lean Dockerfile for testing stretch4_ros2 with stretch4_install's stretch_update_ros_workspace.sh
# Based on stretch4_install's .github/docker/Dockerfile, but skips the full robot install
# (desktop-full, drivers, udev, networking, etc.) and only sets up what the ROS workspace script needs.
# Build from the root of the stretch4_ros2 repo; the committed HEAD is what gets built in the workspace:
#   docker build -t stretch4-ros2-test -f .github/docker/Dockerfile .
#   docker run -it --rm stretch4-ros2-test

# Ubuntu 24.04 with ROS 2 Jazzy, colcon, vcstool and rosdep (already initialized)
FROM ros:jazzy-ros-base

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

ARG SETUP_FLEET_ID=stretch-se4-0000
ARG STRETCH4_INSTALL_REPO=https://github.com/hello-robot/stretch4_install.git
ARG STRETCH4_INSTALL_REF=main
ENV SETUP_FLEET_ID=${SETUP_FLEET_ID}
ENV USER=hello-robot

# Install the dependencies that stretch_install_system.sh would provide to the workspace script
RUN apt-get update && apt-get install -y \
    sudo \
    git \
    curl \
    wget \
    ca-certificates \
    gnupg \
    zip \
    unzip \
    python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-clean \
    python3-vcstool \
    libnss3-tools \
    build-essential \
    ninja-build \
    && rm -rf /var/lib/apt/lists/*

# Install NodeJS from nodesource, same as stretch_install_system.sh (needed by stretch4_web_teleop)
RUN mkdir -p /etc/apt/keyrings && \
    curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key | gpg --batch --yes --dearmor -o /etc/apt/keyrings/nodesource.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg, arch=amd64] https://deb.nodesource.com/node_24.x nodistro main" > /etc/apt/sources.list.d/nodesource.list && \
    apt-get update && apt-get install -y nodejs && \
    rm -rf /var/lib/apt/lists/*

# Pre-install Playwright's system dependencies, which stretch4_web_teleop's npm postinstall
# (npx playwright install-deps) would otherwise install during the workspace step
RUN npx -y playwright@1 install-deps && \
    rm -rf /var/lib/apt/lists/*

# Mock systemctl, which doesn't work inside Docker (used to install the Zenoh router service)
# Lives in /usr/local/sbin, which is first in both PATH and sudo's secure_path, so that
# rosdep installing the real systemd package (to /usr/bin) doesn't replace the mock
RUN printf '#!/bin/sh\nexit 0\n' > /usr/local/sbin/systemctl && \
    chmod +x /usr/local/sbin/systemctl && \
    grep -q 'secure_path=.*/usr/local/sbin' /etc/sudoers

# Set up /etc/hello-robot, as stretch_initial_setup.sh would
RUN mkdir -p /etc/hello-robot && \
    echo "HELLO_FLEET_ID=${SETUP_FLEET_ID}" > /etc/hello-robot/hello-robot.conf

# Create a non-root user 'hello-robot' with sudo privileges
# The installation scripts expect to run as a regular user with sudo access
RUN useradd -m -s /bin/bash hello-robot && \
    echo "hello-robot ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to the hello-robot user
USER hello-robot
WORKDIR /home/hello-robot

RUN mkdir -p /home/hello-robot/stretch_user/log && \
    mkdir -p /home/hello-robot/.local/bin

# Pre-download Playwright's browsers into ~/.cache/ms-playwright, so the npm postinstall
# (npx playwright install) finds them already present when its Playwright version matches
RUN npx -y playwright@1 install

# Clone stretch4_install, which must be in the home directory as expected by the scripts
RUN git clone --depth 1 --branch ${STRETCH4_INSTALL_REF} ${STRETCH4_INSTALL_REPO} /home/hello-robot/stretch4_install

# Copy the stretch4_ros2 repo under test, and point the workspace's .repos file at it instead of GitHub
COPY --chown=hello-robot:hello-robot . /home/hello-robot/stretch4_ros2_under_test
RUN git -C /home/hello-robot/stretch4_ros2_under_test checkout -B ci-under-test && \
    cd /home/hello-robot/stretch4_install/factory/24.04 && \
    sed -i '/^  stretch4_ros2:/,/version:/{ \
        s|url: .*|url: /home/hello-robot/stretch4_ros2_under_test| ; \
        s|version: .*|version: ci-under-test| \
    }' stretch_ros2_jazzy.repos && \
    grep -A3 '^  stretch4_ros2:' stretch_ros2_jazzy.repos && \
    grep -q 'version: ci-under-test' stretch_ros2_jazzy.repos

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Run the script under test. On failure, print the end of its redirected logs since they hold the details
RUN cd /home/hello-robot/stretch4_install && \
    ./stretch_update_ros_workspace.sh || \
    { for f in /home/hello-robot/stretch_user/log/*_redirected.txt /home/hello-robot/.npm/_logs/*.log; do \
          [ -f "$f" ] && { echo "==== $f"; tail -n 200 "$f"; }; \
      done; exit 1; }

CMD ["/bin/bash"]
