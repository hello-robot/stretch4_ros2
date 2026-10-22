#! /usr/bin/env python3
"""
Install stretch4_mujoco into the Python environment that ROS 2 runs in.

The simulation launch files call ensure_stretch4_mujoco() before importing stretch4_mujoco, so the
first launch on a fresh computer clones and installs it. It can also be run directly:

    python3 -m stretch_mujoco_driver.install_stretch4_mujoco
"""

import importlib.util
import os
import subprocess
import sys
from pathlib import Path

STRETCH4_MUJOCO_URL = "https://github.com/hello-robot/stretch4_mujoco.git"
STRETCH4_MUJOCO_PATH = Path(
    os.environ.get("STRETCH4_MUJOCO_PATH", Path.home() / "repos" / "stretch4_mujoco")
)

# tf_transformations imports transforms3d, and the 0.4.1 that Ubuntu 24.04 ships calls numpy APIs
# that numpy 2 removed. stretch4_mujoco pins numpy 2, so a fixed transforms3d has to come with it.
EXTRA_REQUIREMENTS = ["transforms3d>=0.4.2"]


def is_stretch4_mujoco_installed() -> bool:
    """
    Check whether stretch4_mujoco is importable by this Python interpreter.

    Returns:
        True if stretch4_mujoco can be imported, False otherwise.
    """
    return importlib.util.find_spec("stretch4_mujoco") is not None


def install_stretch4_mujoco() -> None:
    """
    Clone stretch4_mujoco if needed and install it, editable, into this Python interpreter.

    The repository is cloned shallowly to STRETCH4_MUJOCO_PATH (default ~/repos/stretch4_mujoco)
    unless a checkout is already there. It is installed editable so that changes to the checkout
    take effect on the next launch. Outside a virtual environment the install goes to the user
    site-packages, which Ubuntu 24.04 only allows with --break-system-packages.

    Returns:
        None.

    Raises:
        subprocess.CalledProcessError: If git or pip fails.
    """
    if not (STRETCH4_MUJOCO_PATH / "pyproject.toml").exists():
        print(f"Cloning {STRETCH4_MUJOCO_URL} to {STRETCH4_MUJOCO_PATH}...", flush=True)
        STRETCH4_MUJOCO_PATH.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(
            ["git", "clone", "--depth", "1", STRETCH4_MUJOCO_URL, str(STRETCH4_MUJOCO_PATH)],
            check=True,
        )

    pip_install = [sys.executable, "-m", "pip", "install"]
    if sys.prefix == sys.base_prefix:
        pip_install += ["--user", "--break-system-packages"]

    print(f"Installing {STRETCH4_MUJOCO_PATH} with pip (this takes a few minutes)...", flush=True)
    subprocess.run([*pip_install, "-e", str(STRETCH4_MUJOCO_PATH), *EXTRA_REQUIREMENTS], check=True)


def ensure_stretch4_mujoco() -> None:
    """
    Install stretch4_mujoco if it is missing, then restart the current process to load it.

    Meant to be called from a launch file before it imports stretch4_mujoco. Editable installs
    register themselves through .pth files, which Python only reads at startup, so after installing
    this re-executes the running command (e.g. `ros2 launch ...`) with the same arguments. That
    happens while the launch file is being loaded, before any node has been started.

    Returns:
        None. Does not return at all if an install was performed.
    """
    if is_stretch4_mujoco_installed():
        return

    print("stretch4_mujoco is not installed. Installing it now.", flush=True)
    install_stretch4_mujoco()

    print("stretch4_mujoco installed. Restarting the launch...", flush=True)
    os.execv(sys.executable, [sys.executable, *sys.argv])


def main() -> None:
    """
    Install stretch4_mujoco if it is missing, without restarting the process.

    Returns:
        None.
    """
    if is_stretch4_mujoco_installed():
        print("stretch4_mujoco is already installed.")
        return

    install_stretch4_mujoco()


if __name__ == "__main__":
    main()
