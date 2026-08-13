from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    rviz_config = PathJoinSubstitution([pkg, 'rviz', 'line_sensor.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='ROS parameter file; overrides the in-code defaults.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='open RViz on the hazard topics.',
        ),
        Node(
            package='stretch_core',
            executable='line_sensor_publisher',
            name='line_sensor_publisher',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
