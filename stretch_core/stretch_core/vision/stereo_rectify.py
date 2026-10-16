"""Stereo rectification for the head camera pair.

Frames must be fed in UNROTATED. All of the mounting roll is already inside R1/R2, so applying the
usual np.rot90 first would double-correct it.
"""

import os

import cv2
import numpy as np
import tf_transformations
import yaml

from stretch4_body.subsystem.cameras.enums.distortion_models import DistortionModels
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras
from stretch4_body.subsystem.cameras.models.camera_calibration import DEFAULT_CALIBRATION_FOLDER_PATH

from stretch_core.vision.vision_topics import VisionFrames

EXTRINSICS_PATH = os.path.join(DEFAULT_CALIBRATION_FOLDER_PATH, "camera_extrinsics.yaml")

# (camera, extrinsics key, physical optical frame) for the stereo left camera then right.
ROS_LEFT = (RGBCameras.head_left, "left_to_center", VisionFrames.camera_frame("left"))
ROS_RIGHT = (RGBCameras.head_right, "right_to_center", VisionFrames.camera_frame("right"))

# A 180 degree roll about the optical axis. stereoRectify leaves the pair upside down, which also
# swaps left and right; folding this into R1/R2 makes the pair upright and correctly ordered.
ROLL_FIX = np.diag([-1.0, -1.0, 1.0])


class StereoRectifier:
    """Builds and applies the rectifying remap for the head stereo pair."""

    def __init__(self, balance: float = 0.0, fov_scale: float = 1.0):
        with open(EXTRINSICS_PATH) as f:
            ext = yaml.safe_load(f)

        def intrinsics(camera_type):
            # load_calibration() also shifts cx/cy when the stream size differs from the
            # calibration size, which reading the YAML directly would miss.
            c = camera_type.load_calibration()
            if c is None:
                raise RuntimeError(f"No calibration is available for {camera_type.name}.")
            if not c.distortion_model.is_fisheye() or c.distortion_model is DistortionModels.omnidir:
                raise RuntimeError(
                    f"{camera_type.name} is calibrated as {c.distortion_model.name}; "
                    "this rectifier assumes the equidistant fisheye model.")
            K = np.asarray(c.camera_matrix, dtype=np.float64)
            D = np.asarray(c.distortion_coefficients, dtype=np.float64).reshape(-1, 1)
            return K, D, (c.width, c.height)

        (lcam, lext, lframe), (rcam, rext, rframe) = ROS_LEFT, ROS_RIGHT
        self.PARENT_FRAME = {"left": lframe, "right": rframe}

        K1, D1, self.size = intrinsics(lcam)
        K2, D2, size2 = intrinsics(rcam)
        if size2 != self.size:
            raise RuntimeError(f"{lcam.name} is {self.size} but {rcam.name} is {size2}; both must match.")
        width, height = self.size

        # Pose of each camera in the head-centre frame, composed into left -> right.
        T_right_left = np.linalg.inv(np.array(ext[rext], dtype=np.float64)) @ np.array(ext[lext], dtype=np.float64)
        R = np.ascontiguousarray(T_right_left[:3, :3])
        T = np.ascontiguousarray(T_right_left[:3, 3]).reshape(3, 1)

        R1, R2, P1, P2, self.Q = cv2.fisheye.stereoRectify(
            K1, D1, K2, D2, self.size, R, T,
            cv2.CALIB_ZERO_DISPARITY, newImageSize=self.size,
            balance=balance, fov_scale=fov_scale)

        # Roll both cameras 180 degrees into an upright, correctly-ordered frame. The principal
        # point follows the rotation, and the baseline term changes sign with the horizontal order.
        self.R1, self.R2 = ROLL_FIX @ np.asarray(R1), ROLL_FIX @ np.asarray(R2)
        fx, fy = P1[0, 0], P1[1, 1]
        cx, cy = width - 1 - P1[0, 2], height - 1 - P1[1, 2]
        baseline = abs(P2[0, 3] / P1[0, 0])
        self.P1 = np.array([[fx, 0, cx, 0.0], [0, fy, cy, 0], [0, 0, 1, 0]])
        self.P2 = np.array([[fx, 0, cx, -fx * baseline], [0, fy, cy, 0], [0, 0, 1, 0]])

        if self.P2[0, 3] >= 0:
            raise RuntimeError(
                f"P2[0,3]={self.P2[0,3]:.3f} should be negative; the camera pair is the wrong way round.")

        self.map1_left, self.map2_left = cv2.fisheye.initUndistortRectifyMap(
            K1, D1, self.R1, self.P1, self.size, cv2.CV_16SC2)
        self.map1_right, self.map2_right = cv2.fisheye.initUndistortRectifyMap(
            K2, D2, self.R2, self.P2, self.size, cv2.CV_16SC2)

    @property
    def baseline_m(self) -> float:
        return -self.P2[0, 3] / self.P1[0, 0]

    def rectify_left(self, image):
        return cv2.remap(image, self.map1_left, self.map2_left, cv2.INTER_LINEAR)

    def rectify_right(self, image):
        return cv2.remap(image, self.map1_right, self.map2_right, cv2.INTER_LINEAR)

    def camera_info(self, side: str):
        """CameraInfo for a rectified image. Rectified frames carry no distortion, so D is zero."""
        from sensor_msgs.msg import CameraInfo
        R, P = (self.R1, self.P1) if side == "left" else (self.R2, self.P2)
        msg = CameraInfo()
        msg.width, msg.height = self.size
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0] * 5
        msg.k = P[:3, :3].flatten().tolist()
        msg.r = np.asarray(R).flatten().tolist()
        msg.p = np.asarray(P).flatten().tolist()
        return msg

    def rect_frame_quaternion(self, side: str):
        """Orientation of a rectified optical frame relative to its physical optical frame.

        Rectification maps a point as x_rect = R @ x_optical, so the frame itself rotates by the
        inverse. Returned as (x, y, z, w), with PARENT_FRAME[side] as the parent.
        """
        T = np.eye(4)
        T[:3, :3] = np.asarray(self.R1 if side == "left" else self.R2, dtype=np.float64).T
        return tuple(float(v) for v in tf_transformations.quaternion_from_matrix(T))
