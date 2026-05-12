#!/bin/bash

SCRIPT_DIR=$(dirname "$0")
ABS_PATH=$(realpath "$SCRIPT_DIR")

set -e

#Package set-up:
cd "$ABS_PATH"/..
pip install -e "."

if [ ! -d "$ABS_PATH/dependencies" ]; then
    mkdir -p "$ABS_PATH"/dependencies
else
    echo "dependencies directory already exists"
fi

cd "$ABS_PATH"/dependencies

# Install stretch_mujoco:
if [ ! -d "stretch4_mujoco" ]; then
    git clone https://github.com/hello-robot/stretch4_mujoco.git
else
    echo "stretch4_mujoco already cloned"
fi

cd stretch4_mujoco
git submodule update --init

read -p "Install robocasa and robosuite dependencies (Not recommended if using python >= 3.12)? (yN)" response

if [ "$response" = "y" ] || [ "$response" = "Y" ]; then
    # Install robocasa:
    pip install -e ".[robocasa]"
    pip install -e "third_party/robocasa"
    python3 third_party/robocasa/robocasa/scripts/setup_macros.py
    python3 third_party/robocasa/robocasa/scripts/download_kitchen_assets.py

    # Install robosuite:
    pip install "third_party/robosuite"
    python3 third_party/robosuite/robosuite/scripts/setup_macros.py
else
    echo "Skipped robocasa an robotsuite installs"
fi

# Colcon Build:
cd ~/ament_ws
colcon build
source ~/ament_ws/install/setup.bash

echo "Done. You can now use 'ros2 launch stretch_simulation stretch_mujoco_driver.launch.py mode:=navigation'."
