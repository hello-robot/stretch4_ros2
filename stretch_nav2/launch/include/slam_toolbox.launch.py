from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (AndSubstitution, LaunchConfiguration,
                                  NotSubstitution)
from launch_ros.actions import LifecycleNode, Node, SetParameter
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from hello_helpers.launch_utils import get_rviz_node

def generate_launch_description():
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )
    use_sim_time_param = SetParameter('use_sim_time', LaunchConfiguration('use_sim_time'))

    stretch_nav2 = FindPackageShare('stretch_nav2')

    rviz_config = PathJoinSubstitution([stretch_nav2, 'rviz', 'slam_rviz.rviz'])

    use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Whether to launch RViz2'
    )

    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the slamtoolbox. '
                    'Ignored when use_lifecycle_manager is true.')
    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='Enable bond connection during node activation')

    start_sync_slam_toolbox_node = LifecycleNode(
        parameters=[
            PathJoinSubstitution([stretch_nav2, 'config/mapper_params_offline.yaml']),
            {'use_lifecycle_manager': use_lifecycle_manager}
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=''
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(start_sync_slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_sync_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_sync_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(use_sim_time_param)
    ld.add_action(use_rviz)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)
    ld.add_action(start_sync_slam_toolbox_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)


    for action in get_rviz_node(str(stretch_nav2 / 'rviz' / 'global.rviz')):
        ld.add_action(action)
        
    return ld
