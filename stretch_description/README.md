## Overview

The purpose of *stretch_description* is simply to provide a launch file to visualize the URDF in Rviz. None of the files (`.xacro`, `.urdf`, `.stl`, etc.) live in this repo anymore; they have been moved to [stretch4_urdf](https://github.com/hello-robot/stretch4_urdf).

You can launch the visualization using:

```bash
ros2 launch stretch_description display.launch.py
```

By default, the visualization uses your robot's URDF. To visualize another end-effector, batch, or variant of robot, you can run:

```bash
ros2 launch stretch_description display.launch.py urdf_file:=<path_to_urdf>
```

## Details

`display.launch.py` loads the URDF from [*stretch4_urdf*](https://github.com/hello-robot/stretch4_urdf) using:

```python
from stretch4_urdf import get_robot_params, get_urdf

model_name, batch_name, tool_name = get_robot_params()
robot_description_content = get_urdf(model_name=model_name, batch_name=batch_name, tool_name=tool_name)
robot_state_publisher = Node(package='robot_state_publisher',
                             executable='robot_state_publisher',
                             parameters=[{'robot_description': robot_description_content}])
```

`model` and `tool` is defined for your robot in Stretch's parameters:

```
import stretch4_body.robot
r = stretch4_body.robot.Robot()
model = r.params['model_name']
tool = r.params['tool']
```

To access other URDFs, XACROs, or individual meshes from *stretch4_urdf*, look through [Coming Soon](#TODO) for details on these assets are organized.

## License and Patents

Patents are pending that cover aspects of the Stretch mobile manipulator.

For license information, please see the LICENSE files. 
