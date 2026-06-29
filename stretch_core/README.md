# Overview

*stretch_core* provides the drivers and navigation-facing sensor processing for the Stretch mobile manipulator.

## Dual Hesai point cloud merger

The `dual_lidar_pointcloud_merger` node synchronizes the left and right Hesai `PointCloud2` streams, transforms both clouds into a target frame, preserves the original per-point fields such as `ring` and `timestamp`, and publishes one merged cloud.

It is intended for consumers that need a merged point cloud rather than a projected navigation scan. The node caches the static lidar transforms after they become available, supports an optional pre-transform voxel downsample with `merger_voxel_leaf_size`, and can blank points inside a base-centered cylinder with `cylinder_filter_radius`.

Example:

```bash
ros2 run stretch_core dual_lidar_pointcloud_merger --ros-args \
  -p left_topic:=/lidar_points_left \
  -p right_topic:=/lidar_points_right \
  -p output_topic:=/lidar_points \
  -p target_frame:=base_link
```

## Dual Hesai LaserScan filtering

The `dual_lidar_laserscan` node (`pointcloud_to_laserscan`) fuses the left and right Hesai point clouds, runs a configurable filter pipeline, and always publishes `/scan_filtered` (`LaserScan`) in `base_footprint`.

- `pub_pointcloud` - optional debug output: publish a filtered merged xyz cloud on `pointcloud_topic` (default `/lidar_pointcloud`, off by default)

### Filter presets

Processing is handled by `DualLidarPipeline`. The easiest way to choose behavior is with `filter_type`:

| `filter_type` | What it enables | Typical use |
|---------------|-----------------|-------------|
| `region` | Region crop + robot self-filter | Mapping and general filtered scans |
| `sor` | Region crop + robot self-filter + near-robot SOR | Navigation |
| `sor_ransac` | Region crop + robot self-filter + near-robot SOR + floor RANSAC | Navigation when floor returns need explicit removal |
| `self` | Robot self-filter only | Debugging robot/self-hit removal |
| `none` | No filtering before projection | Baseline/debug comparison |
| `custom` | Uses the individual `enable_*` booleans | Experiments and tuning |

### Available filtering techniques

Each technique is optional except the final LaserScan projection:

| Technique | What it does | User-facing knobs |
|-----------|--------------|-------------------|
| Region crop | Removes points outside the configured height/range limits. This is the simple floor/ceiling/far-range crop. | `z_min`, `z_max`, `range_max` |
| Robot self-filter | Removes returns from the robot itself: base cylinder, arm capsule, and required URDF-derived arm/wrist/gripper/tool boxes. | `base_radius`, `arm_filter_radius`, `self_filter_*_buffer`, `self_filter_box_*` |
| Self-filter spatial gate | Cheap pre-check before robot geometry. Points outside this XY/Z gate skip the expensive robot shape checks. It does not remove points by itself. | `self_filter_spatial_gate_enabled`, `self_filter_gate_radius_m`, `self_filter_gate_z_min_m`, `self_filter_gate_z_max_m` |
| Internal voxel downsample | Near-robot voxel grid reduction used before expensive work. It is not a user-selectable `filter_type`; it runs automatically when robot self-filter or SOR is enabled. | `dist_rob`, `leaf_size` |
| SOR | PCL StatisticalOutlierRemoval for flying/noisy near-robot points. In the `sor` preset, voxel downsample runs first, then SOR runs inside `dist_rob`; far points pass through. | `dist_rob`, `sor_mean_k`, `sor_stddev` |
| Floor RANSAC | Fits the floor plane with RANSAC from the configured floor-detection Z band, then removes points close to that plane. It is only floor removal, not general obstacle detection. | `floor_detect_z_min`, `floor_detect_z_max`, `plane_fitting_threshold`, `angle` |
| Speckle filter | Post-projection cleanup on LaserScan bins. It removes weak isolated bins that do not have angular support from neighboring bins. | `speckle_filter_enabled`, `speckle_min_points`, `speckle_neighbor_window`, `speckle_min_neighbors`, `speckle_range_tolerance` |

The runtime order is:

```text
transform -> region crop -> internal voxel -> robot self-filter -> SOR -> floor RANSAC -> LaserScan projection -> speckle filter
```

Only enabled stages run. Projection always runs because it produces `/scan_filtered`. Use `region` for mapping, `sor` for navigation, and `self` with `pub_pointcloud:=true` to visualize robot/self-hit removal without the region crop.

### Launch

```bash
# Mapping-style filtering: region + self-filter
ros2 launch stretch_core dual_hesai.launch.py filter_type:=region tool_preset:=auto

# Navigation-style filtering: region + self-filter + near-robot SOR
ros2 launch stretch_core dual_hesai.launch.py filter_type:=sor tool_preset:=auto

# With RViz
ros2 launch stretch_core dual_hesai.launch.py filter_type:=sor tool_preset:=auto use_rviz:=true
```

`tool_preset` selects URDF-derived self-filter geometry for the mounted tool. Use `auto` to read Stretch robot params when available, or pass `sg4`, `pg4`, `tablet`, or `nil` explicitly.

Nav2 navigation launch (`stretch_nav2`) typically includes this stack with `filter_type:=sor` and starts `robot_footprint_publisher` for a dynamic costmap footprint.

### Robot self-filter and footprint geometry

Self-filter geometry is generated from URDF collision geometry at launch time. The generated boxes cover the arm/lift details, wrist chain, gripper camera, and the selected SG4/PG4/tablet tool geometry. Empty frames such as grasp frames and ArUco marker frames are not used as collision boxes.

The base cylinder and arm capsule are always active when the self-filter stage runs. The URDF-derived boxes are required; launch through `dual_hesai.launch.py` or `robot_footprint.launch.py` so `self_filter_config.py` can generate the temporary `self_filter_box_*` parameter YAML.

The dynamic footprint publisher receives the same generated geometry as the lidar self-filter. By default, footprint buffers match the self-filter buffers so Nav2 plans conservatively around the same arm/tool volume that lidar filtering treats as robot geometry.

### Debug self-filter geometry

Run the lidar filter and enable marker publication on the node:

```bash
ros2 launch stretch_core dual_hesai.launch.py filter_type:=self tool_preset:=auto pub_pointcloud:=true use_rviz:=true
ros2 param set /pointcloud_to_laserscan pub_self_filter_markers true
ros2 param set /pointcloud_to_laserscan publish_raw_urdf_self_filter_markers true
ros2 param set /pointcloud_to_laserscan publish_buffered_self_filter_markers true
```

RViz markers are published on `/self_filter_markers`. Raw URDF boxes show the collision geometry from the URDF. Buffered boxes show the effective lidar self-filter geometry.

### Configuration

| File | Purpose |
|------|---------|
| `config/dual_lidar_filter.yaml` | `filter_type`, region limits, SOR, speckle, floor RANSAC |
| `config/robot_self_filter.yaml` | Shared self-filter policy: base cylinder, arm capsule, spatial gate, marker controls |
| launch-generated temp YAML | Required URDF-derived `self_filter_box_*` geometry for the selected `tool_preset` |
| `config/robot_footprint.yaml` | Dynamic footprint publisher topics, base polygon, and joint update thresholds |

Tuning notes for filter order, gate radius, URDF box buffers, and markers: see [config/README.md](config/README.md).

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
