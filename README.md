## Overview

This repository holds ROS 2 Jazzy packages for the Stretch 4 mobile manipulator from Hello Robot Inc. On a fresh robot, these packages are built and available through the `~/ament_ws` workspace. 

Stretch 4 ROS packages contain breaking changes from Stretch 3 and earlier hardware versions. ROS packages for earlier versions can be found in [stretch_ros2](https://github.com/hello-robot/stretch_ros2).

## Packages

Resource                                                     | Description
-------------------------------------------------------------|---------------------------------------------------------------------------------------------
[stretch_core](stretch_core/README.md)                       | ROS 2 drivers for Stretch 4
[stretch_description](stretch_description/README.md)         | Visualize Stretch 4's URDF
[stretch_nav2](stretch_nav2/README.md)                       | Navigation stack built on Nav2
[stretch_simulation](stretch_simulation/README.md)           | Simulation of Stretch 4, built on [Stretch4 Mujoco](https://github.com/hello-robot/stretch4_mujoco)
[stretch_tag_perception](stretch_python_bridge/README.md)     | Detect aruco tags with Stretch 4 
[stretch_python_bridge](stretch_python_bridge/README.md)     | A high-level Python API that abstracts away rclpy 
[hello_helpers](hello_helpers/README.md)                     | Miscellaneous helper code used across the stretch_ros2 repository

## Licenses

For license details for this repository, see the LICENSE files found in the directories. A summary of the licenses follows: 

Directory               | License
------------------------|--------------------------------------------------------------------------------------
stretch_core            | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_description     | [BSD 3-Clause Clear License](https://choosealicense.com/licenses/bsd-3-clause-clear/)
stretch_nav2            | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_simulation      | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_python_bridge   | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_tag_perception   | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
hello_helpers           | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
