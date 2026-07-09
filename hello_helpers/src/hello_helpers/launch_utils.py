from launch.actions import UnsetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def _remove_qt_plugin_env_vars() -> list:
    """
    This resolves the QT plugin that causes rviz to not launch, described in https://github.com/hello-robot/stretch4_ros2/issues/19
    
    Usage:
        return [
            *_remove_qt_plugin_env_vars(),
            rviz_node,
        ]
    """
    return [
        UnsetEnvironmentVariable(name='QT_QPA_PLATFORM_PLUGIN_PATH'),
        UnsetEnvironmentVariable(name='QT_QPA_FONTDIR'),
        UnsetEnvironmentVariable(name='QT_PLUGIN_PATH'),
    ]


def get_rviz_node(rviz_file_path:str|None=None, *, rviz_args:list|None = None, rviz_params:dict|None = None, launch_configration_key:str|None = 'use_rviz') -> list:
    """
    Use this to get the rviz node in your launch file. This automatically adds _remove_qt_plugin_env_vars()

    Only one of rviz_file_path or rviz_args is allowed to be not None.

    If `launch_configration_key` is None, IfCondition will not be checked.

    Usage:
        return [
            your other nodes
        ] + get_rviz_node()

    """
    if rviz_file_path is not None and rviz_args is not None:
        raise ValueError("Only one of rviz_file_path or rviz_args can be used. You may want to add ['-d', rviz_file_path] to your rviz_args instead.")

    arguments = rviz_args
    if rviz_file_path is not None:
        arguments = ["-d", rviz_file_path]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments= arguments,
        parameters=[rviz_params] if rviz_params is not None else [],
        condition=IfCondition(LaunchConfiguration(launch_configration_key)) if launch_configration_key is not None else None,
    )
    return _remove_qt_plugin_env_vars() + [rviz_node]