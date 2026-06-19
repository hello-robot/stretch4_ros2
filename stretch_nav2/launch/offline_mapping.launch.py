from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    stretch_core_path = FindPackageShare('stretch_core')

    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([stretch_core_path, 'launch', 'stretch_driver.launch.py']),
            launch_arguments={'broadcast_odom_tf': 'True', 'mode': 'navigation'}.items()
        ),
        
        IncludeLaunchDescription(
            PathJoinSubstitution([stretch_core_path, 'launch', 'dual_hesai.launch.py']),
            launch_arguments={'filter_type': 'region'}.items(),
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('stretch_nav2'), 'launch', 'include', 'slam_toolbox.launch.py']
            ),eclareLaunchArgument(
            'tool_preset',
            default_value='auto',
            description='Mounted tool preset for lidar self-filter: auto, sg4, pg4, tablet, or nil',
        ),
            launch_arguments={'use_rviz': 'true'}.items(),
        ),
    ])

