from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    """GPU autodocking is not implemented yet.
    """
    raise SystemExit(
        'Coming soon... GPU autodocking is not implemented yet.'
        'Use `ros2 launch stretch_nav2 autodocking_cpu.launch.py` instead.'
    )
