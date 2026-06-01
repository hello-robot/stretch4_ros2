## Overview

The *stretch_nav2* package provides the standard ROS 2 navigation stack (Nav2) with its launch files. This package utilizes slam_toolbox and Nav2 to drive Stretch around a mapped space. Running this code will require the robot to be untethered. We recommend stowing the arm while running navigation on the robot.

## Quickstart

The first step is to map the space that the robot will navigate in. The `offline_mapping.launch.py` will enable you to do this. First, run:

```bash
ros2 launch stretch_nav2 offline_mapping.launch.py
```

Rviz will show the robot and the map that is being constructed. With the terminal open, use the joystick (see instructions below for using a keyboard) to teleoperate the robot around. Avoid sharp or fast turns and revisit previously visited spots to form loop closures. In Rviz, once you see a map that has reconstructed the space well enough, open a new terminal and run the following commands to save the map to the `stretch_user/` directory.

```bash
mkdir ${HELLO_FLEET_PATH}/maps
ros2 run nav2_map_server map_saver_cli -f ${HELLO_FLEET_PATH}/maps/<map_name>
```

**NOTE**: The `<map_name>` does not include an extension. The map_saver node will save two files as `<map_name>.pgm` and `<map_name>.yaml`.

**Tip**: For a quick sanity check, you can inspect the saved map using a pre-installed tool called Eye of Gnome (eog) by running the following command:

```bash
eog ${HELLO_FLEET_PATH}/maps/<map_name>.pgm
```

Next, with `<map_name>.yaml`, we can navigate the robot around the mapped space. Run:

```bash
ros2 launch stretch_nav2 navigation_mppi.launch.py  map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml
```

A new RViz window should pop up with a `Startup` button in a menu at the bottom left of the window. Press the `Startup` button to kick-start all navigation related lifecycle nodes. Rviz will show the robot in the previously mapped space, however, it's likely that the robot's location on the map does not match the robot's location in the real space. To correct this, from the top bar of Rviz, use `2D Pose Estimate` to lay an arrow down roughly where the robot is located in the real space. This gives an initial estimate of the robot's location to AMCL, the localization package. AMCL will better localize the robot once we pass the robot a `2D Nav Goal`.

In the top bar of Rviz, use `2D Nav Goal` to lay down an arrow where you'd like the robot to navigate. In the terminal, you'll see Nav2 go through the planning phases and then navigate the robot to the goal. If planning fails, the robot will begin a recovery behavior - spinning around 180 degrees in place or backing up.

**Tip**: If navigation fails or the robot becomes unresponsive to subsequent goals through RViz, you can still teleoperate the robot using the Xbox controller.

---

---

## Launch structure + Nav2 params overlay

### Which launch files to use

Top-level launch files live directly under `launch/` — these are the ones you run:

| Launch file | Purpose |
|-------------|---------|
| `offline_mapping.launch.py` | Build a map with SLAM |
| `navigation_mppi.launch.py` | Navigate on a saved map (main entry point) |
| `navigation_mppi_filter.launch.py` | Navigate with binary-filter adaptive params |
| `binary_filter_launch.py` | Start the binary costmap filter servers |
| `global_plan_demo.launch.py` | Standalone global planner demo |

Supporting launch files that are included by the above live under `launch/include/` (`nav_core`, `bringup`, `navigation_launch`, `slam_toolbox`). You normally do not run these directly.

### Launch ordering

The usual entry point is `navigation_mppi.launch.py`, which includes launch files in this order:

`navigation_mppi.launch.py` → `include/nav_core.launch.py` → `include/bringup_launch.py` → `include/navigation_launch.py`

What each file contains:

- **`navigation_mppi.launch.py`**: top-level “run navigation on the robot” launcher. Starts the Stretch driver, starts the dual-lidar filter that publishes `/scan_filtered`, then launches `include/nav_core.launch.py` with the merged Nav2 params.
- **`include/nav_core.launch.py`**: Stretch wrapper around bringup. Validates the map file, includes `include/bringup_launch.py`, and optionally launches RViz.
- **`include/bringup_launch.py`**: Nav2 bringup orchestrator. Loads/rewrites the `params_file`, then includes localization/SLAM (from `nav2_bringup`) and navigation (from `stretch_nav2`).
- **`include/navigation_launch.py`**: Nav2 navigation servers (controller/planner/BT/etc). Runs either as composed components in a container or as separate ROS nodes depending on `use_composition`.

### Nav2 parameter overlay order (`MultiYaml`)

`navigation_mppi.launch.py` passes a `params_file` built with `MultiYaml([...])`. YAML files are merged **in order**; later files override earlier ones at nested keys.

Overlay order:

- `config/original_nav2_params.yaml`: upstream Nav2 baseline
- `config/nav2_params_core.yaml`: Stretch-specific changes (e.g., `/scan_filtered`, omni AMCL, costmap scan topics)
- `config/nav2_params_mppi.yaml`: MPPI controller selection + core controller/costmap changes
- `config/mppi_params.yaml`: MPPI tuning parameters

### Debugging Nav2 components: `use_composition:=False`

By default, Nav2 may run as composable components inside a single container node, which makes per-component logs harder to follow. For debugging, set:

`use_composition:=False`

This runs each Nav2 component as its own ROS node so its logs are visible directly. Note: the `use_composition` launch argument is declared in `include/bringup_launch.py` / `include/navigation_launch.py` and must be passed through from the top-level launch file to take effect.

## Navigation Launch Options:
Different environments often require different navigation strategies. There’s no single setup that works best everywhere. Below are options you can try to adapt navigation performance to your environment.  


**NOTE:** use can use the argument map:=<path to your map>/<map_name>.yaml with all the navigation launch commands
### 1) Handling Noisy Laser Scans
If your LaserScan data contains a lot of noise, use the denoise layer:

```bash
ros2 launch stretch_nav2 navigation_mppi.launch.py \
  params_file:=/home/hello-robot/ament_ws/src/stretch4_ros2/stretch_nav2/config/nav2_params_mppi_denoise.yaml
```

### 2) Preventing Unnecessary Replanning

In environments with many possible paths (e.g., highly connected spaces), Nav2 may constantly switch paths.
You can avoid this by using a behavior tree  (set via the `default_nav_to_pose_bt_xml` parameter)  that only replans when the current path becomes invalid:

```bash
ros2 launch stretch_nav2 navigation_mppi.launch.py params_file:=/home/hello-robot/ament_ws/src/stretch4_ros2/stretch_nav2/config/nav2_params_mppi_bt.yaml
```

### 3) Forcing the Robot to Always Face Forward
You can set the motion_model parameter of the MPPI to diffdrive.
⚠️ Not recommended unless you specifically want the robot to always face forward.
This disables omni-motion, which is usually helpful for obstacle avoidance (the robot can slide sideways without rotating making response faster).

### 4) Using a Binary Filter for Adaptive Navigation

The **binary filter** detects when the robot **enters or exits specific areas** of the map.  
It publishes to the `/binary_state` topic, which outputs `true` or `false` only when the robot transitions into or out of a marked region (not continuously).  

In our setup, the binary filter was used to **dynamically adjust navigation parameters** to improve doorway navigation:

- **Entering a doorway (narrow, cluttered area):** speed is reduced and costmap inflation is lowered.  
- **Exiting the doorway:** normal speed and inflation are restored.  

By lowering inflation in narrow areas, the robot could maneuver without being overly conservative. To maintain safety, speed was also reduced in these areas. In open spaces, higher inflation and normal speed were used to prevent collisions while enabling faster movement. This approach balances safety and efficiency by adjusting both inflation and speed according to the environment.

#### Launching the Binary Filter

1. Start the filter node:

```bash
ros2 launch stretch_nav2 binary_filter_launch.py
```
2. Launch navigation with the filter configuration:

```bash
ros2 launch stretch_nav2 navigation_mppi_filter.launch.py
```

Alternative for running the navigation_mppi_filter:

```bash
ros2 launch stretch_nav2 navigation_mppi.launch.py \
  params_file:=/home/hello-robot/ament_ws/src/stretch4_ros2/stretch_nav2/config/nav2_params_mppi_filter.yaml

ros2 run stretch_nav2 binary_filter_switch.py
```

⚠️ Remember: **binary_filter_launch.py** must always be started first.

You can verify that the filter is active by subscribing to the map topic defined under mask_topic in binary_filter_param.yaml. Ensure the QoS settings match the publisher.

#### How the Filter Works

The binary_filter_switch.py node listens to /binary_state and updates navigation parameters whenever the robot enters or leaves a marked area.

You can modify this node (located in the stretch_nav2 subfolder) to adjust any parameter or trigger additional actions, such as disabling cameras in certain zones.

#### How Set Up Your Own Binary Filter

1. Annotate your map image with the areas where the filter should activate.
<!-- 
video on how to do that. In the video i used map enhancer https://github.com/ali-pahlevani/Map_Enhancer_Wizard to make the obstacles all solid color to make it easy to remove them.

The application used is Gimp https://www.gimp.org/downloads/.

<video src="video/gimp_tutorial.webm" controls width="600">
  Your browser does not support the video tag.
</video>
-->

2. Make sure your global or local costmap includes the filter under the filters: parameter.

Other useful filters include keepout zones and speed limits. For tutorials on these, see:
https://docs.nav2.org/tutorials/docs/navigation2_with_speed_filter.html
https://docs.nav2.org/tutorials/docs/navigation2_with_keepout_filter.html

---

## Things to Note and Design Decisions

- **Filtering Methods:** We use two filters for point cloud processing:  
  1. **voxel_sor filter** – Applies a voxel grid followed by a StatisticalOutlierRemoval within a configurable radius (e.g., ~2 m works well) to clean the point cloud before converting it into a laser scan. For navigation, `voxel_sor` improves localization by reducing outliers, but it performs poorly for mapping because the map builder expects a consistent, full environment.  
  2. **region_filter** – Removes points based on position (e.g., base, ground, ceiling).  

  Mapping launch files use `region_filter`, while navigation uses `voxel_sor` (selected via the `filter_type` parameter). The reason for applying `voxel_sor` only within a certain distance is that farther points have a different distribution than closer ones so they tend to be more distant, and noise from closer points affects motion more significantly so one parameter doesn't fit close and far away points so we focus only on the closer points.  

  Dual-lidar filter nodes live in `stretch_core` (`region_dual_lidar_laserscan`, `voxel_dual_lidar_laserscan`, `voxel_dual_lidar_laserscan_RANSAC`), launched via `stretch_core/launch/dual_hesai.launch.py`.

- **LaserScan Topic:** The filtered laser scan is published to `/scan_filtered` with **Best Effort QoS** (not reliable).

- **Debugging Tips:** If a topic appears inactive in RViz2:
  1. Check that `ros2 topic echo` shows messages.  
  2. Verify that RViz is subscribed to the correct topic.  
  3. Run `ros2 topic info /topic_name -v` to inspect publishers, subscribers, and QoS. Ensure subscriber and publisher QoS match.

---


## License
For license information, please see the LICENSE files.
