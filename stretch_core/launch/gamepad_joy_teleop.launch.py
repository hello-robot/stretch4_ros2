"""Launch remote_gamepad + joy_to_cmd_vel_nav for collision_monitor teleop.

  remote_gamepad → /joy → joy_to_cmd_vel_nav
    → cmd_vel_nav (Twist, base) → collision_monitor → /cmd_vel
    → arm via ControlMapping.do_motion (RobotClient, no joint_vel topic)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='cmd_vel_nav',
            description='Twist output topic (collision_monitor cmd_vel_in_topic)',
        ),
        DeclareLaunchArgument(
            'joy_topic',
            default_value='joy',
            description='Joy input topic from remote_gamepad',
        ),
        Node(
            package='stretch_core',
            executable='remote_gamepad',
            name='remote_gamepad',
            output='screen',
        ),
        Node(
            package='stretch_core',
            executable='joy_to_cmd_vel_nav',
            name='joy_to_cmd_vel_nav',
            output='screen',
            parameters=[{
                'joy_topic': LaunchConfiguration('joy_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            }],
        ),
    ])
