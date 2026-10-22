# Stretch Simulation with Nvidia GPU support.
# Layered on the image the GitHub Action builds from .github/docker/Dockerfile, which already holds a
# built ~/ament_ws with this repo's committed HEAD. `make build` in this directory builds both:
#   docker build -t stretch4-ros2:latest -f ../.github/docker/Dockerfile ..
#   docker build -t stretch-simulation:latest --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) .

ARG BASE_IMAGE=stretch4-ros2:latest
FROM ${BASE_IMAGE}

# Match the container user to the host user, so checkouts mounted for development stay writable
# (stretch4_mujoco writes its generated MJCF into its own models folder on every launch).
ARG USER_UID=1000
ARG USER_GID=1000

USER root

RUN if id -u ubuntu > /dev/null 2>&1; then userdel -r ubuntu; fi && \
    if [ "$(id -u hello-robot)" != "${USER_UID}" ] || [ "$(id -g hello-robot)" != "${USER_GID}" ]; then \
        groupmod -o -g ${USER_GID} hello-robot && \
        usermod -o -u ${USER_UID} -g ${USER_GID} hello-robot; \
    fi

# MuJoCo renders through EGL. The Nvidia Container Toolkit mounts the driver's EGL library when the
# graphics capability is requested, and glvnd needs this vendor file to find it, since the base image
# is not an Nvidia one.
RUN apt-get update && apt-get install -y libegl1 libgl1 libglvnd0 && \
    rm -rf /var/lib/apt/lists/* && \
    mkdir -p /usr/share/glvnd/egl_vendor.d && \
    echo '{"file_format_version": "1.0.0", "ICD": {"library_path": "libEGL_nvidia.so.0"}}' \
        > /usr/share/glvnd/egl_vendor.d/10_nvidia.json

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV MUJOCO_GL=egl
ENV QT_X11_NO_MITSHM=1

USER hello-robot
WORKDIR /home/hello-robot

# Clone stretch4_mujoco to ~/repos/stretch4_mujoco and install it editable, the same way the launch
# files do on a first run. Copied from the build context so it matches this checkout, not the base.
# INSTALL_MOLMOSPACES=1 also installs MolmoSpaces (and torch), for use_molmospaces:=true.
ARG INSTALL_MOLMOSPACES=0
COPY --chown=hello-robot:hello-robot stretch_mujoco_driver/install_stretch4_mujoco.py /tmp/install_stretch4_mujoco.py
RUN python3 /tmp/install_stretch4_mujoco.py $([ "${INSTALL_MOLMOSPACES}" = "1" ] && echo --molmospaces) && \
    rm /tmp/install_stretch4_mujoco.py

RUN grep -qxF 'source ~/ament_ws/install/setup.bash' ~/.bashrc || \
    echo 'source ~/ament_ws/install/setup.bash' >> ~/.bashrc

WORKDIR /home/hello-robot/ament_ws

CMD ["/bin/bash"]
