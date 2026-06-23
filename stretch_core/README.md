# Overview

*stretch_core* provides the drivers to the Stretch mobile manipulator.

## Dual Hesai LiDAR filtering

The `dual_lidar_laserscan` node (`pointcloud_to_laserscan`) fuses the left and right Hesai point clouds, runs a configurable filter pipeline, and publishes `/scan_filtered` as a `LaserScan` in `base_footprint`.

### Filter pipeline

Processing is handled by `DualLidarPipeline`. Choose which stages run with the `filter_type` launch parameter (or the `filter_type` ROS parameter on the node):

| `filter_type` | Stages |
|---------------|--------|
| `region` | Robot self-filter, region (height/range crop) |
| `sor` | Robot self-filter, region, voxel grid + statistical outlier removal (near robot) |
| `sor_ransac` | Same as `sor`, plus floor plane removal via RANSAC |

For each lidar point (in `base_footprint`): apply the region crop, then a cheap spatial gate, then TF-driven robot geometry checks (base, arm, shoulder, wrist chain, tool attachment) only inside that gate. After fusion, an optional speckle filter cleans weak isolated scan bins.

Use `region` for mapping (lighter processing). Use `sor` for navigation (denoise close returns). Use `sor_ransac` when floor points should be removed by plane fit instead of a fixed `z_min`.

### Launch

```bash
# Mapping-style filtering (region + self-filter)
ros2 launch stretch_core dual_hesai.launch.py filter_type:=region tool_preset:=sg4

# Navigation-style filtering (adds voxel/SOR near the robot)
ros2 launch stretch_core dual_hesai.launch.py filter_type:=sor tool_preset:=sg4

# With RViz
ros2 launch stretch_core dual_hesai.launch.py filter_type:=sor use_rviz:=true
```

`tool_preset` selects the mounted-tool attachment box: `sg4`, `pg4`, `tablet`, or `nil`. It must match the hardware on the robot.

Nav2 navigation launch (`stretch_nav2`) typically includes this stack with `filter_type:=sor` and starts `robot_footprint_publisher` for a dynamic costmap footprint.

### Debug self-filter geometry

```bash
ros2 launch stretch_core self_filter_debug.launch.py tool_preset:=sg4 use_rviz:=true
```

RViz markers are published on `/self_filter_markers` when `pub_self_filter_markers` is true.

### Configuration

| File | Purpose |
|------|---------|
| `config/dual_lidar_filter.yaml` | `filter_type`, region limits, voxel/SOR, speckle, floor RANSAC |
| `config/robot_self_filter.yaml` | Base/arm/wrist geometry, spatial gate |
| `config/self_filter_<tool>.yaml` | Tool-specific attachment box |
| `config/robot_footprint.yaml` | Dynamic footprint publisher (topics, base polygon) |

Tuning notes (filter order, gate radius, marker colors): see [config/README.md](config/README.md).

### Head lidar PTP check

Read-only verification of JT128 return mode (Last + Strongest), point-cloud filter (Strong), PTP lock offset (350 µs), locked PTP status (single read), and jitter p95 ≤ 350 µs over 30 s (direct PTC TCP, no ROS topics):

```bash
ros2 run stretch_core stretch_lidar_check
```

Options: `--left`, `--right`, `--duration 30`, `--json`, `--verbose`.

## API

For comprehensive API documentation, please refer to [Coming soon](#TODO).

## Testing

Colcon is used to run the system/perf tests in the */test* folder. The command to run the entire suite of tests is:

```bash
$ cd ~/ament_ws
$ colcon test --packages-select stretch_core
```

You can run individual tests using the following command:

```bash
$ colcon test --packages-select stretch_core --pytest-args -k test_trajectory_server -s --event-handlers console_direct+

Test suites:
  - test_trajectory_server
  - test_pub_topics
  - test_sub_topics
  - test_services
  - test_parameters
```

## Head lidar PTC check

`stretch_lidar_check` verifies JT128 return mode (Last + Strongest), point-cloud filter (Strong), PTP lock offset (350 µs), locked PTP status, and jitter p95 over PTC TCP (port 9347):


## License

Please see the [LICENSE](./LICENSE.md) file.
