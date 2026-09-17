# README

### Overview

This repository holds ROS 2 Jazzy packages for the Stretch 4 mobile manipulator from Hello Robot Inc. On a fresh robot, these packages are built and available through the `~/ament_ws` workspace.

Stretch 4 ROS packages contain breaking changes from Stretch 3 and earlier hardware versions. ROS packages for earlier versions can be found in [stretch\_ros2](https://github.com/hello-robot/stretch_ros2).

### Packages

| Resource                                          | Description                                                                                         |
| ------------------------------------------------- | --------------------------------------------------------------------------------------------------- |
| [stretch\_core](stretch_core/)                    | ROS 2 drivers for Stretch 4                                                                         |
| [stretch\_description](stretch_description/)      | Visualize Stretch 4's URDF                                                                          |
| [stretch\_nav2](stretch_nav2/)                    | Navigation stack built on Nav2                                                                      |
| [stretch\_simulation](stretch_simulation/)        | Simulation of Stretch 4, built on [Stretch4 Mujoco](https://github.com/hello-robot/stretch4_mujoco) |
| [stretch_tag_perception](stretch_python_bridge/README.md)     | Detect aruco tags with Stretch 4 |
| [stretch\_python\_bridge](stretch_python_bridge/) | A high-level Python API that abstracts away rclpy                                                   |
| [hello\_helpers](hello_helpers/)                  | Miscellaneous helper code used across the stretch\_ros2 repository                                  |

### Licenses

For license details for this repository, see the LICENSE files found in the directories. A summary of the licenses follows:

| Directory               | License                                                                               |
| ----------------------- | ------------------------------------------------------------------------------------- |
| stretch\_core           | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)                              |
| stretch\_description    | [BSD 3-Clause Clear License](https://choosealicense.com/licenses/bsd-3-clause-clear/) |
| stretch\_nav2           | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)                              |
| stretch\_simulation     | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)                              |
| stretch_tag_perception   | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)                             |
| stretch\_python\_bridge | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)                              |
| hello\_helpers          | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)                              |

### Common ROS2 commands

Below is an unordered list of common or useful ROS2 commands:

* Start Stretch Driver: `ros2 launch stretch_core stretch_driver.launch.py`
  * Teleop twist keyboard: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
  * Driver parameters (`mode`, `tool_info.*`, `joint_velocity.*`, `joint_acceleration.*`) and joint state diagnostics: see [stretch\_core/README.md](stretch_core/README.md#stretch_driver)
  * List/read them: `ros2 param list /stretch_driver` and `ros2 param get /stretch_driver joint_velocity.lift`
* Start Lidars: `ros2 launch stretch_core dual_hesai.launch.py`
* Start Cameras: `ros2 launch stretch_core luxonis.launch.py use_center:=true use_left:=true use_right:=true` for the head cameras and `ros2 launch stretch_core gripper_camera.launch.py` for the gripper camera.
* Navigation (See [https://docs.hello-robot.com/stretch4\_docs/working-with-stretch/getting-started/demo-mapping-and-navigation#set-a-navigation-goal](https://docs.hello-robot.com/stretch4_docs/working-with-stretch/getting-started/demo-mapping-and-navigation#set-a-navigation-goal)):
  * Start Mapping: `ros2 launch stretch_nav2 offline_mapping.launch.py` and `stretch_gamepad_teleop`
  * Save a map: `ros2 run nav2_map_server map_saver_cli -f ${HELLO_FLEET_PATH}/maps/<map_name>`
  * Start nav2: `ros2 launch stretch_nav2 navigation_mppi.launch.py map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml`
* Web teleop (See [https://docs.hello-robot.com/stretch4\_docs/working-with-stretch/getting-started/demo-web-teleoperation](https://docs.hello-robot.com/stretch4_docs/working-with-stretch/getting-started/demo-web-teleoperation)): `~/ament_ws/src/stretch4_web_teleop/launch_interface.sh` and navigate to `https://localhost/operator`
* Record rosbags/mcap files: `ros2 bag record -a` . Note: `-a` records all the topics, if lidar and cameras are streaming, a few minutes of recording are hundreds of gigabytes large.
  * Play rosbags/mcap files: `ros2 bag play ./path/to/recording`&#x20;
* View topics: `ros2 topic list` and `ros2 topic echo /joint_states`
* View kinematic links: `ros2 run tf2_tools view_frames`&#x20;
  * View transform between two links: `ros2 run tf2_ros tf2_echo base_link head_link`

### Troubleshooting

#### Zenoh middleware not working

Zenoh should start as a linux user service on Stretch 4. Run `systemctl --user status zenoh.service` to see if it has failed to start. Run `systemctl --user restart zenoh.service` to restart it.

If you would like to manually start it, use `ros2 run rmw_zenoh_cpp rmw_zenohd`.

#### rviz2 is not launching with a Launch file

There is a known conflict between opencv and rviz2 in ROS2 Jazzy that is documented and a solution is proposed here: [https://github.com/hello-robot/stretch4\_ros2/issues/19](https://github.com/hello-robot/stretch4_ros2/issues/19)

#### Topics are not appearing

There are a number of reasons for topics to not appear.&#x20;

1. Check that you are running the ros2 node and the client on the same computer. If you are trying to connect from a different computer, you may need to add the following environment variables to your client session:

```
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# Replace ROBOT_IP with your Stretch's IP address
ROBOT_IP=192.168.X.XXX
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/$ROBOT_IP:7447"]'
```

2. Stretch 4 is configured with `export RMW_IMPLEMENTATION=rmw_zenoh_cpp` in the .bashrc file to tell ROS2 to use the Zenoh middleware. Make sure your terminal session with this env variable.
3. Use `ros2 topic list` to list topics. Also, `ros2 topic echo /joint_states` to listen to a topic
4. Check that the TF tree has all the links: `ros2 run tf2_tools view_frames`

<br>
