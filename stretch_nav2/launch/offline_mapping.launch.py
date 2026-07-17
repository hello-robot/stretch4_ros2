from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    stretch_core_path = FindPackageShare('stretch_core')

    tool_preset_arg = DeclareLaunchArgument(
        'tool_preset',
        default_value='auto',
        description='Mounted tool preset for lidar self-filter: auto, sg4, pg4, tablet, or nil',
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Start RViz with SLAM; set false when using Stretch Nav web UI',
    )

    stretch_driver_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'stretch_driver.launch.py']),
        launch_arguments={'broadcast_odom_tf': 'True', 'mode': 'navigation'}.items()
    )

    lidar_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'dual_hesai.launch.py']),
        launch_arguments={
            'filter_type': 'region',
            'tool_preset': LaunchConfiguration('tool_preset'),
        }.items(),
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare('stretch_nav2'), 'launch', 'include', 'slam_toolbox.launch.py']
        ),
        launch_arguments={'use_rviz': LaunchConfiguration('use_rviz')}.items(),
    )

    return LaunchDescription([
        tool_preset_arg,
        use_rviz_arg,
        stretch_driver_launch,
        lidar_launch,
        slam_toolbox_launch
    ])
