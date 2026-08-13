from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Bring up line_sensor_publisher.

    Parameters resolve in two layers, the second overriding the first:

        1. LineSensorConfig defaults
           (stretch4_body/subsystem/line_sensor/filter/config.py) plus this
           node's own _declare_params() defaults
        2. params_file  (config/line_sensors.yaml)
    """
    pkg = FindPackageShare('stretch_core')
    default_params = PathJoinSubstitution([pkg, 'config', 'line_sensors.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='ROS parameter file; overrides the in-code defaults.',
        ),
        Node(
            package='stretch_core',
            executable='line_sensor_publisher',
            name='line_sensor_publisher',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
