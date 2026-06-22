from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    stretch_core_path = FindPackageShare('stretch_core')

    # 1. Declare the launch argument
    tool_preset_arg = DeclareLaunchArgument(
        'tool_preset',
        default_value='auto',
        description='Mounted tool preset for lidar self-filter: auto, sg4, pg4, tablet, or nil',
    )

    # 2. Base Driver
    stretch_driver_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'stretch_driver.launch.py']),
        launch_arguments={'broadcast_odom_tf': 'True', 'mode': 'navigation'}.items()
    )
    
    # 3. LiDAR Driver
    lidar_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'dual_hesai.launch.py']),
        launch_arguments={
            'filter_type': 'sor',
            'tool_preset': LaunchConfiguration('tool_preset'),
        }.items(),
    )

    # 4. SLAM Toolbox
    slam_toolbox_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare('stretch_nav2'), 'launch', 'include', 'slam_toolbox.launch.py']
        ),
        launch_arguments={'use_rviz': 'true'}.items(),
    )

    # Return everything to the ROS 2 launch system
    return LaunchDescription([
        tool_preset_arg,
        stretch_driver_launch,
        lidar_launch,
        slam_toolbox_launch
    ])