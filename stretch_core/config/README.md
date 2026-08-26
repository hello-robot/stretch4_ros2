# stretch_core filter configuration

Shared YAML under this directory configures the dual-lidar pipeline, robot self-filter policy, and Nav2 footprint publisher. Arm, wrist, gripper-camera, and tool boxes are generated from URDF collision geometry at launch time by `launch/self_filter_config.py`.

## Filter presets and techniques

The node always publishes `LaserScan` on `output_topic` (default `/scan_filtered`). Set `pub_pointcloud: true` to also publish a debug xyz `PointCloud2` on `pointcloud_topic`.

`filter_type` selects a preset:

| `filter_type` | Enabled techniques |
|---------------|--------------------|
| `region` | Region crop, robot self-filter |
| `sor` | Region crop, robot self-filter, SOR |
| `sor_ransac` | Region crop, robot self-filter, SOR, floor RANSAC |
| `self` | Robot self-filter only |
| `none` | No filters before LaserScan projection |
| `custom` | Controlled by `enable_self_robot_filter`, `enable_region_filter`, `enable_sor_filter`, `enable_floor_ransac_filter` |

Available techniques:

| Technique | What it does | Main parameters |
|-----------|--------------|-----------------|
| Region crop | Removes points outside the configured height/range limits. | `z_min`, `z_max`, `range_max` |
| Robot self-filter | Removes robot returns using the base cylinder, arm capsule, and required URDF-derived arm/wrist/gripper/tool boxes. | `base_radius`, `arm_filter_radius`, `self_filter_*_buffer`, `self_filter_box_*` |
| Self-filter spatial gate | Cheap XY/Z pre-check before exact robot geometry checks. It reduces computation; it is not a standalone removal filter. | `self_filter_gate_radius_m`, `self_filter_gate_z_min_m`, `self_filter_gate_z_max_m` |
| Internal voxel downsample | Near-robot voxel grid reduction before self-filter or SOR. This is an implementation detail, not a `filter_type`. | `dist_rob`, `leaf_size` |
| SOR | PCL StatisticalOutlierRemoval for noisy/flying points near the robot. In the `sor` preset, voxel downsample runs first, then SOR runs inside `dist_rob`; far points pass through. | `dist_rob`, `sor_mean_k`, `sor_stddev` |
| Floor RANSAC | Fits the floor plane from the floor-detection Z band and removes points close to that plane. It is only floor removal, not general obstacle detection. | `floor_detect_z_min`, `floor_detect_z_max`, `plane_fitting_threshold`, `angle` |
| Speckle filter | Post-projection cleanup that removes weak isolated LaserScan bins. | `speckle_*` |

The implementation order in `dual_lidar_pipeline.cpp` is:

```text
transform -> region crop -> internal voxel -> robot self-filter -> SOR -> floor RANSAC -> LaserScan projection -> speckle filter
```

Only enabled stages run. Projection always runs because it produces `/scan_filtered`. The internal voxel step runs automatically when robot self-filter or SOR is enabled because those are the expensive near-field stages.

## Spatial gate vs other radii

| Parameter | Default | Shape | Purpose |
|-----------|---------|-------|---------|
| `self_filter_gate_radius_m` | 1.5 m | Cylinder in XY | Near-field ROI for robot geometry checks |
| `base_radius` | 0.25 m | Cylinder in XY | Base robot volume removed as self-hit |
| `dist_rob` | 2.5 m | Square in XY | SOR denoise ROI when `filter_type:=sor` |

Tune `self_filter_gate_radius_m` to at least max arm+tool XY reach, usually about 1.0-1.2 m. Increase it if extended-arm returns leak into `/scan_filtered`.

Gate parameters in `robot_self_filter.yaml`:

- `self_filter_spatial_gate_enabled` - master switch
- `self_filter_gate_radius_m` - XY circle radius
- `self_filter_gate_z_min_m` / `self_filter_gate_z_max_m` - optional height band inside the cylinder

## URDF box generation

`self_filter_config.py` generates temporary `self_filter_box_*` parameters from URDF collision geometry. This replaces the old manual `self_filter_<tool>.yaml` files and the old `wrist_chain_*` parameter names.

`tool_preset:=auto` tries Stretch robot params first, then fleet/user YAML, and falls back to `sg4` if the mounted tool cannot be detected. You can also pass `sg4`, `pg4`, `tablet`, or `nil` explicitly.

Generated boxes include physical robot/tool collision links only:

| Group | Typical links |
|-------|---------------|
| `arm` | `arm_l0_link` through `arm_l4_link`, `lift_link` |
| `wrist` | `wrist_link`, `wrist_yaw_link`, `wrist_pitch_link`, `wrist_roll_link` |
| `gripper_camera` | `gripper_camera_link` |
| `tool` | selected SG4, PG4, or tablet tool collision links |

Empty/reference frames such as grasp frames, ArUco marker frames, and attachment-site frames are not used as collision boxes.

The C++ self-filter requires generated URDF boxes. If a node is started without the generated `self_filter_box_*` parameters, configuration will fail instead of silently running with incomplete robot geometry.

## URDF box buffers

Runtime tuning is by group:

| Parameter | Applies to |
|-----------|------------|
| `self_filter_arm_buffer` | `arm` URDF boxes |
| `self_filter_wrist_buffer` | `wrist` URDF boxes |
| `self_filter_gripper_cam_buffer` | `gripper_camera` URDF boxes |
| `self_filter_tool_buffer` | selected SG4/PG4/tablet tool boxes |

Example:

```bash
ros2 param set /pointcloud_to_laserscan self_filter_wrist_buffer 0.06
ros2 param set /pointcloud_to_laserscan self_filter_tool_buffer 0.06
```

`self_filter_box_buffers` remains available as a full per-box override. Leave it empty for group tuning.

By default, the Nav2 footprint uses the same effective buffer as the self-filter so planning remains conservative around the arm and tool. `self_filter_box_footprint_buffers` is available as an expert override when the footprint must intentionally differ from lidar self-filter geometry.

## RViz self-filter markers

Markers are published by `pointcloud_to_laserscan` on `/self_filter_markers`. Start the lidar filter, then enable the marker parameters on that node:

```bash
ros2 launch stretch_core dual_hesai.launch.py filter_type:=self tool_preset:=auto pub_pointcloud:=true use_rviz:=true
ros2 param set /pointcloud_to_laserscan pub_self_filter_markers true
ros2 param set /pointcloud_to_laserscan publish_raw_urdf_self_filter_markers true
ros2 param set /pointcloud_to_laserscan publish_buffered_self_filter_markers true
```

| Namespace | Meaning |
|-----------|---------|
| `self_filter/gate` | Spatial gate volume, which bounds expensive robot geometry checks |
| `self_filter/gate_ring` | Ground circle at gate radius for top-down RViz inspection |
| `self_filter/base` | Base cylinder from `base_radius` |
| `self_filter/arm` | Arm capsule from `arm_l0_link`/`lift_link` to `wrist_link` |
| `self_filter/urdf_raw/<group>/<collision>` | Raw URDF collision bounding box, when `publish_raw_urdf_self_filter_markers` is true |
| `self_filter/urdf_buffered/<group>/<collision>` | Buffered filtering box used for lidar self-hit/artifact removal |

Useful live commands:

```bash
ros2 param list /pointcloud_to_laserscan
ros2 param set /pointcloud_to_laserscan self_filter_gate_radius_m 1.7
ros2 param set /pointcloud_to_laserscan self_filter_wrist_buffer 0.05
ros2 param set /pointcloud_to_laserscan self_filter_tool_buffer 0.05
```
