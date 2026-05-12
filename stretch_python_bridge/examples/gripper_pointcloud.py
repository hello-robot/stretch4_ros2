import rerun as rr
import numpy as np
from stretch_python_bridge import *

def gripper_pointcloud():
    rr.init("pointcloud_stream", spawn=True)

    # 1. Tell Rerun the "camera" space uses Right-Down-Forward coordinates
    rr.log("camera", rr.ViewCoordinates.RDF, static=True)

    gripper_camera = stream_gripper_right()
    for pc_frame in stream_gripper_stereo_points():
        if pc_frame is None:
            print("No pointcloud frame received")
            continue
        
        rr.log(
            "camera/pointcloud",
            rr.Points3D(
                positions=pc_frame.points,
                colors=pc_frame.colors
            )
        )

        gripper_rgb = next(gripper_camera)
        if gripper_rgb is not None:
            rr.log(
                "camera/rgb",
                rr.Image(gripper_rgb.image).compress()
            )



if __name__ == "__main__":
    gripper_pointcloud()