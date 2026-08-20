"""Stereo rectification for the head camera pair.

The per-camera undistortion in `rectify.py` removes fisheye distortion but leaves each camera with
its own focal length, its own principal point, and its own orientation. Stereo matching needs more
than that: both images must share one focal length and be row-aligned, so a feature in the left
image lies on the same pixel row in the right one. That is what `cv2.fisheye.stereoRectify` gives,
and it needs the extrinsics between the two cameras -- which live in camera_extrinsics.yaml, not in
the per-camera calibration.

The head is mounted rolled ~179 degrees, and stereoRectify resolves that by rotating each camera
~89.5 degrees into a common frame. That frame is a valid rectification but it comes out upside
down, and an upside-down pair also has its left and right swapped, since a 180 degree roll reverses
the horizontal order. Both problems are one problem, so ROLL_FIX folds a further 180 degrees into
both rectification rotations. With it the images are upright AND each physical camera feeds the
stereo side of the same name; without it neither is true.

Frames must be fed in UNROTATED. All of the mounting roll is already inside R1/R2, so applying the
usual np.rot90 first would double-correct it.
"""

import glob

import cv2
import numpy as np
import yaml

CALIBRATION_GLOB = "/home/hello-robot/stretch_user/*/calibration_cameras/calibration_rgb_head_camera.yaml"
EXTRINSICS_GLOB = "/home/hello-robot/stretch_user/*/calibration_cameras/camera_extrinsics.yaml"

# (calibration name, extrinsics key, physical optical frame) for the stereo left camera then right.
ROS_LEFT = ("head_left", "left_to_center", "camera_left_optical_link")
ROS_RIGHT = ("head_right", "right_to_center", "camera_right_optical_link")

# A 180 degree roll about the optical axis. See the module docstring.
ROLL_FIX = np.diag([-1.0, -1.0, 1.0])


def _load(path_glob):
    matches = glob.glob(path_glob)
    if not matches:
        raise RuntimeError(f"No file matching {path_glob}")
    return yaml.safe_load(open(matches[0]))


class StereoRectifier:
    """Builds and applies the rectifying remap for the head stereo pair."""

    def __init__(self, balance: float = 0.0, fov_scale: float = 1.0):
        calib = _load(CALIBRATION_GLOB)
        ext = _load(EXTRINSICS_GLOB)

        def intrinsics(name):
            c = calib[name]
            K = np.array(c["camera_matrix"], dtype=np.float64)
            D = np.array(c["distortion_coefficients"], dtype=np.float64).reshape(-1, 1)
            if c["distortion_model"] != "equidistant":
                raise RuntimeError(
                    f"{name} is calibrated as {c['distortion_model']}; this rectifier assumes fisheye.")
            return K, D, (c["image_size"][1], c["image_size"][0])

        (lname, lext, lframe), (rname, rext, rframe) = ROS_LEFT, ROS_RIGHT
        self.PARENT_FRAME = {"left": lframe, "right": rframe}

        K1, D1, self.size = intrinsics(lname)
        K2, D2, size2 = intrinsics(rname)
        if size2 != self.size:
            raise RuntimeError(f"{lname} is {self.size} but {rname} is {size2}; both must match.")
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
        inverse. Returned as (x, y, z, w) for a static_transform_publisher whose parent is
        PARENT_FRAME[side].
        """
        R = np.asarray(self.R1 if side == "left" else self.R2, dtype=np.float64).T
        trace = np.trace(R)
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        else:
            i = int(np.argmax(np.diag(R)))
            j, k = (i + 1) % 3, (i + 2) % 3
            s = np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1.0) * 2
            q = [0.0, 0.0, 0.0]
            q[i] = 0.25 * s
            q[j] = (R[j, i] + R[i, j]) / s
            q[k] = (R[k, i] + R[i, k]) / s
            w = (R[k, j] - R[j, k]) / s
            x, y, z = q
        return float(x), float(y), float(z), float(w)
