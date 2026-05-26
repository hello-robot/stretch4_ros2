from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch_ros.actions import Node

import os
import yaml
import tempfile
from pathlib import Path


def generate_launch_description():
    stretch_core = get_package_share_directory('stretch_core')

    # Make a YAML config
    template_file = Path(stretch_core) / 'config' / 'hesai_dual_lidar.yaml'
    with open(template_file, "r") as f:
        cfg = yaml.safe_load(f)
    fleet_dir = os.environ.get("HELLO_FLEET_PATH", "")
    fleet_id = os.environ.get("HELLO_FLEET_ID", "")
    cfg["lidar"][0]["driver"]["correction_file_path"] = (
        f"{fleet_dir}/{fleet_id}/calibration_hesais/left_lidar_calibration.dat"
    )
    cfg["lidar"][1]["driver"]["correction_file_path"] = (
        f"{fleet_dir}/{fleet_id}/calibration_hesais/right_lidar_calibration.dat"
    )
    with tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".yaml", dir="/tmp") as tmp_file:
        yaml.dump(cfg, tmp_file, sort_keys=False)
        temp_yaml_path = tmp_file.name

    hesai_node = Node(
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        output='screen',
        parameters=[{'config_path': temp_yaml_path}]
    )

    # Declare launch argument
    filter_type_arg = DeclareLaunchArgument(
        'filter_type',
        default_value='region',
        description='Choose the filter node: "region" or "sor"'
    )

    filter_type = LaunchConfiguration('filter_type')

    dual_lidar_params = {
        "lidar1_frame": "lidar_right_link",
        "lidar2_frame": "lidar_left_link",
        # optional — only if topics differ from defaults:
        # "lidar1_topic": "/lidar_points_right",
        # "lidar2_topic": "/lidar_points_left",
        # "frame_id": "base_link",
    }

    region_filter_node = Node(
        condition=IfCondition(PythonExpression(["'", filter_type, "' == 'region'"])),
        package='airy_lidar_filter_cpp',
        executable='region_dual_lidar_laserscan',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[dual_lidar_params],
    )

    voxel_filter_node = Node(
        condition=IfCondition(PythonExpression(["'", filter_type, "' == 'sor'"])),
        package='airy_lidar_filter_cpp',
        executable='voxel_dual_lidar_laserscan_RANSAC',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[dual_lidar_params],
    )

    error_log = LogInfo(
        condition=UnlessCondition(PythonExpression(["'", filter_type, "' == 'region' or '", filter_type, "' == 'sor'"])),
        msg="Invalid filter_type! Must be 'region' or 'sor'. No filter node will be launched."
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="If true, launch Rviz2 automatically.",
    )


    rviz_config_path = os.path.join(
        get_package_share_directory("stretch_core"), "rviz", "lidars.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        hesai_node,
        filter_type_arg,
        region_filter_node,
        voxel_filter_node,
        error_log,
        use_rviz_arg,
        rviz_node
    ])
