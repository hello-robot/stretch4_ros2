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


## Switching Controllers Between MPPI and DWB at Runtime
Nav2 supports multiple controllers, each with different characteristics:  

- **DWB (Dynamic Window Approach):** More accurate in trajectory tracking but treats forward and backward motion the same, which can appear less natural to observers.  
- **MPPI (Model Predictive Path Integral):** Includes a critic that penalizes not facing forward, producing smoother, more natural motion. However, it tends to be slightly less accurate than DWB.

You can also use a **shim_controller** with DWB as the primary controller. This allows rotation behavior while still leveraging DWB, though motion may not be optimal.

**By default, MPPI is used with `yaw_goal_tolerance = 3.145`.**
---

### Launching the Navigation Stack with Multiple Controllers
To start the navigation stack with support for **runtime switching between MPPI (no heading) and DWB**, run:


```bash
ros2 launch stretch_nav2 navigation_multiple_controllers.launch.py map:=<path_to_map>/<map_name>.yaml
```

### Switching Controllers
We provide a helper script, switch_Controller_config.py, which exposes a service to toggle controllers. When set to true, it switches to MPPI and adjusts yaw_goal_tolerance accordingly.


Use the `/switch_controller` service to toggle between MPPI and DWB:
- **Switch to MPPI (no heading):**

  ```bash
  ros2 service call /switch_controller std_srvs/srv/SetBool "{data: true}"
  ```

  Response:\
  `Switched to MPPIController; set yaw_goal_tolerance = 3.145`

- **Switch to DWB:**

  ```bash
  ros2 service call /switch_controller std_srvs/srv/SetBool "{data: false}"
  ```

  Response:\
  `Switched to DWBController; set yaw_goal_tolerance = 0.08`

### Verifying `yaw_goal_tolerance`

To confirm that the `yaw_goal_tolerance` value has been updated:

```bash
ros2 param get /controller_server general_goal_checker.yaw_goal_tolerance
```

## Summary of Different Launch Files
|                                 | Lidar Launch | params                        | Extra                    |
|---------------------------------|--------------|-------------------------------|--------------------------|
| navigation_dwb                  | airy_dual    | nav2_params_dwb               |                          |
| navigation_mppi_dual_hesai      | dual_hesai   | nav2_params_mppi              |                          |
| navigation_mppi_filter          | airy_dual    | nav2_params_mppi_filter       | Binary Filter            |
| navigation_mppi                 | airy_dual    | nav2_params_mppi              |                          |
| navigation_multiple_controllers | airy_dual    | nav2_params_switch_controller | switch_controller_config |

---

<!--### Teleop using a Joystick Controller--> 

## Teleop the Robot
By default teleoperating with keyboard is enabled. You can run 

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/stretch/cmd_vel
```
Make sure you are aware of the speed it is at before moving. If you already moved and wanted to reduce the speed be aware that it will apply the last command you applied so if you don't want the robot to move hit K(button that stops the robot) before reducing the speed.

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

- **Testing Navigation with Loop Plan:** Use `loop_plan` to continuously run the robot through multiple locations. You need to set three different locations, and the planner will direct the robot to visit them repeatedly. Instructions are available at [loop_plan](https://github.com/hello-ola/loop_plan).

---

## Potential Improvements

**Tuning Opportunities:**  
- Adjust the robot footprint.  
- Evaluate how the back of the arm affects occlusion behind the robot with the Calder variation.

**Areas for Enhancement:**  
- **Dynamic Costmap Inflation:** Adjust the costmap inflation radius based on the environment. One approach is to use the costmap to calculate the distance from each cell (or pixel) to the nearest obstacle. By combining this distance information with the robot’s footprint, we can determine an appropriate inflation radius that provides enough clearance for safe navigation. This allows the robot to navigate tightly around obstacles in narrow spaces while maintaining efficiency in open areas, effectively balancing safety and speed. *(Charlie's Idea)*


## License
For license information, please see the LICENSE files.
