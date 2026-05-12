import rerun as rr
import numpy as np
from stretch_python_bridge import *

def lidar_pointcloud():
    rr.init("pointcloud_stream", spawn=True)
    for pc_frame in stream_lidar_points_left():
        if pc_frame is not None:
            
            colors = np.full((len(pc_frame.intensity ), 3), [255, 255, 255], dtype=np.uint8)
            colors[pc_frame.intensity  > 240] = [255, 0, 0]

            rr.log("pointcloud", rr.Points3D(pc_frame.points, colors=colors))

if __name__ == "__main__":
    lidar_pointcloud()