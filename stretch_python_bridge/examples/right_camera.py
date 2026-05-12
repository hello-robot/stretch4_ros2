import cv2
from stretch_python_bridge import *

def right_camera():
    for rgb_frame in stream_camera_right_rotated():
        cv2.namedWindow("RGB Stream", cv2.WINDOW_NORMAL)
        if rgb_frame is not None:
            print(f"Got a frame with timestamp: {rgb_frame.timestamp}")
            cv2.imshow("RGB Stream", rgb_frame.image)
        cv2.waitKey(1)

if __name__ == "__main__":
    right_camera()