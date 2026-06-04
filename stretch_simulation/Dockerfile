# Stretch Simulation ROS2 Dockerfile with Nvidia GPU Support
# Based on Ubuntu 24.04 with ROS2 Jazzy

FROM nvidia/cuda:12.6.1-base-ubuntu24.04 AS base

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Set up locale
RUN apt-get update && apt-get install -y \
    locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    wget \
    git \
    gnupg2 \
    lsb-release \
    build-essential \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Add universe repository
RUN add-apt-repository universe

# Install ROS2 Jazzy
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    ros-dev-tools \
    ros-jazzy-rviz2 \
    ros-jazzy-rmw-zenoh-cpp \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install NodeJS 22.x and npm
RUN curl -sL https://deb.nodesource.com/setup_22.x | bash - \
    && apt-get install -y nodejs \
    && rm -rf /var/lib/apt/lists/*

# Install additional tools needed for setup
RUN apt-get update && apt-get install -y \
    rpl \
    gnome-terminal \
    terminator \
    gedit \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Remove distutils-installed packages to avoid conflicts
RUN rm -rf /usr/lib/python3/dist-packages/sympy* || true
RUN rm -rf /usr/lib/python3/dist-packages/blinker* || true
RUN rm -rf /usr/lib/python3/dist-packages/transforms3d* || true
RUN rm -rf /usr/lib/python3/dist-packages/psutil* || true
RUN rm -rf /usr/lib/python3/dist-packages/kiwisolver* || true
RUN rm -rf /usr/lib/python3/dist-packages/typing_extensions* || true
RUN rm -rf /usr/lib/python3/dist-packages/numpy* || true
RUN rm -rf /usr/lib/python3/dist-packages/scipy* || true


# Set up environment variables
ENV HELLO_FLEET_PATH=/root/stretch_user
ENV HELLO_FLEET_ID=stretch-se4-local
ENV MUJOCO_GL=egl
ENV PIP_BREAK_SYSTEM_PACKAGES=1

# Create necessary directories
RUN mkdir -p ${HELLO_FLEET_PATH}/${HELLO_FLEET_ID}

# Source ROS2 setup in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# Set up ament_ws workspace
WORKDIR /root

# Copy the ament workspace setup script
COPY stretch_create_ament_workspace.sh /tmp/stretch_create_ament_workspace.sh
COPY stretch_ros2_jazzy.repos /tmp/stretch_ros2_jazzy.repos
RUN chmod +x /tmp/stretch_create_ament_workspace.sh

# Run the ament workspace setup script with DOCKER_BUILD environment variable
# This bypasses sudo checks and interactive prompts
RUN /bin/bash -c "export DOCKER_BUILD=1 && source /opt/ros/jazzy/setup.bash && /tmp/stretch_create_ament_workspace.sh"

# Source ament_ws setup in bashrc
RUN echo "source ~/ament_ws/install/setup.bash" >> /root/.bashrc

# Set up URDF
RUN /bin/bash -c "source ~/ament_ws/install/setup.bash && \
    pip3 install -U hello-robot-stretch-urdf && \
    git clone https://github.com/hello-robot/stretch4_urdf.git --depth 1 /tmp/stretch4_urdf && \
    cd /tmp/stretch4_urdf && \
    pip install -e . "

# Fix PyOpenGL issue before setting up Mujoco
RUN pip3 install PyOpenGL==3.1.4

# Install dependencies for Hello-Helpers that aren't being picked up in the dependencies list in setup.py by ros.
RUN pip3 install pyquaternion transforms3d

# Set up Mujoco
# Note: The setup.sh script is interactive, so we'll need to provide "y" to download kitchen assets
RUN rm -rf /usr/lib/python3/dist-packages/kiwisolver* || true
RUN rm -rf /usr/lib/python3/dist-packages/packaging* || true
RUN pip3 uninstall -y coverage || true
RUN /bin/bash -c "source ~/ament_ws/install/setup.bash && \
    echo 'y' | bash ~/ament_ws/src/stretch4_ros2/stretch_simulation/stretch_mujoco_driver/setup.bash"

    # Force NumPy-2-compatible transforms3d after rosdep/setup scripts
RUN apt-get update && \
    apt-get install -y ros-jazzy-tf-transformations && \
    # apt-get remove -y python3-transforms3d || true && \
    # rm -rf /usr/lib/python3/dist-packages/transforms3d* && \
    pip3 install --no-cache-dir --force-reinstall transforms3d==0.4.2 && \
    python3 -c "import numpy, transforms3d; print('numpy:', numpy.__version__); print('transforms3d:', transforms3d.__version__, transforms3d.__file__)"
    
# Build the workspace
WORKDIR /root/ament_ws
RUN /bin/bash -c "rm -rf build install log && \
                  source /opt/ros/jazzy/setup.bash && \
                  colcon build"

# Set up Stretch Web Teleop URDF export
# WORKDIR /root/ament_ws/src/stretch4_ros2/stretch_description/urdf
# RUN cp ./stretch_uncalibrated.urdf stretch.urdf && \
#     ./export_urdf.sh || true && \
#     mkdir -p ${HELLO_FLEET_PATH}/${HELLO_FLEET_ID}/exported_urdf && \
#     cp -r ./exported_urdf/* ${HELLO_FLEET_PATH}/${HELLO_FLEET_ID}/exported_urdf || true

# Create maps directory
RUN mkdir -p ${HELLO_FLEET_PATH}/maps

# Set up display for GUI applications
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

# Expose any necessary ports (for web teleop)
EXPOSE 8080 3000

# Create entrypoint script
RUN echo '#!/bin/bash\n\
    set -e\n\
    source /opt/ros/jazzy/setup.bash\n\
    source ~/ament_ws/install/setup.bash\n\
    exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

