#! /usr/bin/env python3
"""
Install stretch4_mujoco into the Python environment that ROS 2 runs in.

The simulation launch files call ensure_stretch4_mujoco() before importing stretch4_mujoco, so the
first launch on a fresh computer clones and installs it, and ensure_molmospaces() when a MolmoSpaces
scene is requested. It can also be run directly:

    python3 -m stretch_mujoco_driver.install_stretch4_mujoco [--molmospaces]
"""

import argparse
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


def is_molmospaces_installed() -> bool:
    """
    Check whether MolmoSpaces (the molmo_spaces package) is importable by this Python interpreter.

    Returns:
        True if molmo_spaces can be imported, False otherwise.
    """
    return importlib.util.find_spec("molmo_spaces") is not None


def pip_install(*requirements: str) -> None:
    """
    Install requirements with pip into this Python interpreter.

    Outside a virtual environment the install goes to the user site-packages, which Ubuntu 24.04
    only allows with --break-system-packages.

    Args:
        requirements: Arguments for `pip install`, e.g. "-e", a path, or requirement specifiers.

    Returns:
        None.

    Raises:
        subprocess.CalledProcessError: If pip fails.
    """
    pip_install_command = [sys.executable, "-m", "pip", "install"]
    if sys.prefix == sys.base_prefix:
        pip_install_command += ["--user", "--break-system-packages"]

    subprocess.run([*pip_install_command, *requirements], check=True)


def install_stretch4_mujoco() -> None:
    """
    Clone stretch4_mujoco if needed and install it, editable, into this Python interpreter.

    The repository is cloned shallowly to STRETCH4_MUJOCO_PATH (default ~/repos/stretch4_mujoco)
    unless a checkout is already there. It is installed editable so that changes to the checkout
    take effect on the next launch.

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

    print(f"Installing {STRETCH4_MUJOCO_PATH} with pip (this takes a few minutes)...", flush=True)
    pip_install("-e", str(STRETCH4_MUJOCO_PATH), *EXTRA_REQUIREMENTS)


def ensure_molmospaces(stretch4_mujoco_path: Path | None = None) -> None:
    """
    Install MolmoSpaces through stretch4_mujoco's `molmo` extra, if it is missing.

    The extra is installed against the stretch4_mujoco checkout that is already installed, wherever
    it is, so that MolmoSpaces gets the versions that checkout pins. It pulls in torch, so it is
    only installed when a MolmoSpaces scene is requested. No restart is needed: only the driver
    node imports molmo_spaces, and it starts in a new process.

    Args:
        stretch4_mujoco_path: The stretch4_mujoco checkout to install the extra from. Defaults to
            the one this interpreter imports stretch4_mujoco from.

    Returns:
        None.

    Raises:
        subprocess.CalledProcessError: If pip fails.
    """
    if is_molmospaces_installed():
        return

    if stretch4_mujoco_path is None:
        # origin is <checkout>/stretch4_mujoco/__init__.py
        stretch4_mujoco_path = Path(importlib.util.find_spec("stretch4_mujoco").origin).parents[1]

    print(
        f"MolmoSpaces is not installed. Installing {stretch4_mujoco_path}[molmo] with pip "
        "(this downloads torch and takes a while)...",
        flush=True,
    )
    pip_install("-e", f"{stretch4_mujoco_path}[molmo]")


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
    Install stretch4_mujoco, and optionally MolmoSpaces, if missing, without restarting the process.

    Returns:
        None.
    """
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument(
        "--molmospaces", action="store_true", help="Also install MolmoSpaces (pulls in torch)."
    )
    args = parser.parse_args()

    stretch4_mujoco_path = None
    if is_stretch4_mujoco_installed():
        print("stretch4_mujoco is already installed.")
    else:
        install_stretch4_mujoco()
        # The fresh editable install is not importable by this interpreter (see
        # ensure_stretch4_mujoco()), so tell ensure_molmospaces() where it is.
        stretch4_mujoco_path = STRETCH4_MUJOCO_PATH

    if args.molmospaces:
        ensure_molmospaces(stretch4_mujoco_path)


if __name__ == "__main__":
    main()
