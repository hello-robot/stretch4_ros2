from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


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

    # Expose nav2's SaveMap service 
    map_saver = LifecycleNode(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        namespace='',
        output='screen',
        parameters=[{
            'save_map_timeout': 5.0,
            'free_thresh_default': 0.25,
            'occupied_thresh_default': 0.65,
        }],
    )

    configure_map_saver = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(map_saver),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_map_saver = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_saver,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[map_saver] configured; activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(map_saver),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    return LaunchDescription([
        tool_preset_arg,
        use_rviz_arg,
        stretch_driver_launch,
        lidar_launch,
        slam_toolbox_launch,
        map_saver,
        activate_map_saver,
        configure_map_saver,
    ])
