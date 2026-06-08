# stretch_core filter configuration

Shared YAML under this directory configures the dual-lidar pipeline, robot self-filter geometry, and Nav2 footprint publisher.

## Point filter order (single pass per lidar point)

In `dual_lidar_pipeline.cpp` each point is transformed to `base_footprint`, then:

1. **Region filter** — cheap height/range crop (`z_min`, `z_max`, `range_max` from `dual_lidar_filter.yaml`).
2. **Self-filter spatial gate** — cheap XY cylinder (+ optional Z band) from `robot_self_filter.yaml`.
3. **Robot self-filter geometry** — expensive TF-driven volumes (base, arm, shoulder, wrist, attachment) only for points inside the gate.

Far points skip step 3 and remain as obstacles. Region and gate are different jobs: region shapes the scan; gate limits where robot geometry runs.

## Spatial gate vs other radii

| Parameter | Default | Shape | Purpose |
|-----------|---------|-------|---------|
| `self_filter_gate_radius_m` | 1.5 m | Cylinder in XY | Near-field ROI for robot geometry checks |
| `base_radius` | 0.25 m | Cylinder in XY | Actual base robot volume removed as self-hit |
| `dist_rob` | 2.5 m | Square in XY | VoxelSor denoise ROI when `filter_type:=sor` (navigation) |

Tune `self_filter_gate_radius_m` to at least max arm+tool XY reach (~1.0–1.2 m). Increase (e.g. 1.7 m) if extended-arm returns leak into `/scan_filtered`.

Gate parameters in `robot_self_filter.yaml`:

- `self_filter_spatial_gate_enabled` — master switch
- `self_filter_gate_radius_m` — XY circle radius
- `self_filter_gate_z_min_m` / `self_filter_gate_z_max_m` — optional height band inside the cylinder

## RViz self-filter markers

Enable with `pub_self_filter_markers: true` (on by default in `self_filter_debug.launch.py`). Topic: `/self_filter_markers`.

| Namespace | Meaning |
|-----------|---------|
| `self_filter/gate` | Spatial gate volume (near-field ROI) |
| `self_filter/gate_ring` | Ground circle at gate radius (easy top-down read) |
| `self_filter/base` | Base filter geometry (`base_radius`) |
| `self_filter/arm` | Arm capsule |
| `self_filter/arm_shoulder` | Shoulder OBB |
| `self_filter/wrist/<link>` | Wrist chain OBBs (index order in `wrist_chain_frames`) |
| `self_filter/attachment` | Tool attachment OBB (anchored at `quick_connect_interface_link`; tune via `attachment_box_origin_*`) |

Launch debug view:

```bash
ros2 launch stretch_core self_filter_debug.launch.py tool_preset:=sg4 use_rviz:=true
```

The parameters can be updated. An example to resize the live gate:

```bash
ros2 param set /pointcloud_to_laserscan self_filter_gate_radius_m 1.7
```

Note: I noticed it only takes effect after running another command with ros2 param set, so if you dont see chnages run the same command twice.

You can use ``` ros2 param list /pointcloud_to_laserscan``` to check the parameters that can be updated.

