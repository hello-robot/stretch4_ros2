"""This file stores vision-related namespaces and topics, and builds topic names based on camera or lidar name.
This reduces the amount of magic strings across the nodes and launch files.
"""

from enum import Enum
import os

_user_cameras_calibration_file_name = "calibration_rgb_head_camera.yaml"
_left_calibration_file_name = "calibration_ros_camera_info_left.yaml"
_right_calibration_file_name = "calibration_ros_camera_info_right.yaml"
_center_calibration_file_name = "calibration_ros_camera_info_center.yaml"


def _calibration_file_name(camera_name: str) -> str:
    if camera_name == "left":
        return _left_calibration_file_name
    if camera_name == "right":
        return _right_calibration_file_name
    if camera_name == "center":
        return _center_calibration_file_name
    if camera_name == "user":
        return _user_cameras_calibration_file_name
    raise ValueError(f"{camera_name} is not a valid camera name")


def get_camera_calibration_file_path(camera_name: str) -> str:
    """Get the calibration file path for the given camera name from HELLO_FLEET_PATH environment variable."""
    return f"{os.environ.get("HELLO_FLEET_PATH")}/{os.environ.get("HELLO_FLEET_ID")}/calibration_cameras/{_calibration_file_name(camera_name)}"


class VisionFrames(str, Enum):
    """Usage:
    ```
    camera_frame = VisionFrames.camera_frame("left")
    lidar_frame = VisionFrames.lidar_frame("left")
    image_number_of_rotations = VisionFrames.camera_frame_number_of_rotations("left")
    ```
    """

    LEFT = "camera_left_optical_link"
    RIGHT = "camera_right_optical_link"
    CENTER = "camera_center_optical_link"

    LIDAR_LEFT = "lidar_left_link"
    LIDAR_RIGHT = "lidar_right_link"

    @staticmethod
    def lidar_frame(lidar_name: str) -> str:
        if lidar_name == "left":
            return VisionFrames.LIDAR_LEFT.value
        if lidar_name == "right":
            return VisionFrames.LIDAR_RIGHT.value
        raise ValueError(f"{lidar_name} is not a valid lidar name")

    @staticmethod
    def camera_frame(camera_name: str) -> str:
        if camera_name == "left":
            return VisionFrames.LEFT.value
        if camera_name == "right":
            return VisionFrames.RIGHT.value
        if camera_name == "center":
            return VisionFrames.CENTER.value
        raise ValueError(f"{camera_name} is not a valid camera name")

    @staticmethod
    def gripper_camera_frame(camera_name: str) -> str:
        if camera_name == "left":
            return "gripper_left_camera_color_optical_frame"
        if camera_name == "right":
            return "gripper_right_camera_color_optical_frame"
        if camera_name == "stereo":
            return "gripper_stereo_camera_color_optical_frame"
        raise ValueError(f"{camera_name} is not a valid gripper camera name")

    @staticmethod
    def camera_frame_number_of_rotations(camera_name: str) -> int:
        """Number of rotations to apply to the camera image to make it portrait."""
        if camera_name == "left":
            return 1
        if camera_name == "right":
            return -1
        if camera_name == "center":
            return -1
        raise ValueError(f"{camera_name} is not a valid camera name")


class VisionTopics(str, Enum):
    """
    An enum that stores vision-related namespaces and topics, and builds topic names based on camera or lidar name.
    This reduces the amount of magic strings across the nodes and launch files.

    Usage:
    ```
    camera_image_topic = VisionTopics.image_raw("left")
    camera_info_topic = VisionTopics.camera_info("left")

    depth_image_topic = VisionTopics.depth("left")

    lidar_points_topic = VisionTopics.lidar_points("left")
    ```
    """

    CAMERAS_NAMESPACE = "/cameras_head"

    GRIPPER_CAMERA_NAMESPACE = "/cameras_gripper"

    # Base topics
    _IMAGE_RAW = "image_raw"
    _CAMERA_INFO = "camera_info"
    _CAMERA_INFO_LUXONIS = "camera_info_luxonis"

    # Transport/Encoding
    _COMPRESSED = "image_raw/compressed"
    _COMPRESSED_DEPTH = "image_raw/compressedDepth"
    _ZSTD = "image_raw/zstd"
    _THEORA = "image_raw/theora"

    # Processed/Rectified
    _ROTATED_IMAGE = "rotated_image"
    _IMAGE_RECT = "image_rect"

    # Aligned Depth
    _DEPTH = "depth/depth"
    _DEPTH_RECT = "depth/depth_rect"
    _DEPTH_CAMERA_INFO = "depth/camera_info"
    _POINTS = "depth/points"
    _CLOUD_TRANSFORMED = "depth/depth/debug/cloud_transformed"

    # Lidar
    _LIDAR_POINTS_LEFT = "/lidar_points_left"
    _LIDAR_POINTS_RIGHT = "/lidar_points_right"

    @staticmethod
    def cameras_namespace() -> str:
        return VisionTopics.CAMERAS_NAMESPACE.value

    @staticmethod
    def image_raw(camera_name: str) -> str:
        return f"{VisionTopics.CAMERAS_NAMESPACE.value}/{camera_name}/{VisionTopics._IMAGE_RAW.value}"

    @staticmethod
    def camera_info(camera_name: str) -> str:
        return f"{VisionTopics.CAMERAS_NAMESPACE.value}/{camera_name}/{VisionTopics._CAMERA_INFO.value}"

    @staticmethod
    def camera_info_luxonis(camera_name: str) -> str:
        return f"{VisionTopics.CAMERAS_NAMESPACE.value}/{camera_name}/{VisionTopics._CAMERA_INFO_LUXONIS.value}"

    @staticmethod
    def compressed(camera_name: str) -> str:
        return f"{VisionTopics.CAMERAS_NAMESPACE.value}/{camera_name}/{VisionTopics._COMPRESSED.value}"

    @staticmethod
    def compressed_depth(camera_name: str) -> str:
        return f"{VisionTopics.CAMERAS_NAMESPACE.value}/{camera_name}/{VisionTopics._COMPRESSED_DEPTH.value}"

    @staticmethod
    def zstd(camera_name: str) -> str:
        return f"{VisionTopics.CAMERAS_NAMESPACE.value}/{camera_name}/{VisionTopics._ZSTD.value}"

    @staticmethod
    def theora(camera_name: str) -> str:
        return f"{VisionTopics.CAMERAS_NAMESPACE.value}/{camera_name}/{VisionTopics._THEORA.value}"

    @staticmethod
    def rotated_image(camera_name: str) -> str:
        return f"{VisionTopics.CAMERAS_NAMESPACE.value}/{camera_name}/{VisionTopics._ROTATED_IMAGE.value}"

    @staticmethod
    def image_rect(camera_name: str) -> str:
        return f"{VisionTopics.CAMERAS_NAMESPACE.value}/{camera_name}/{VisionTopics._IMAGE_RECT.value}"

    @staticmethod
    def depth(camera_name: str) -> str:
        return f"{VisionTopics.CAMERAS_NAMESPACE.value}/{camera_name}/{VisionTopics._DEPTH.value}"

    @staticmethod
    def depth_rect(camera_name: str) -> str:
        return f"{VisionTopics.CAMERAS_NAMESPACE.value}/{camera_name}/{VisionTopics._DEPTH_RECT.value}"

    @staticmethod
    def depth_camera_info(camera_name: str) -> str:
        return f"{VisionTopics.CAMERAS_NAMESPACE.value}/{camera_name}/{VisionTopics._DEPTH_CAMERA_INFO.value}"

    @staticmethod
    def points(camera_name: str) -> str:
        return f"{VisionTopics.CAMERAS_NAMESPACE.value}/{camera_name}/{VisionTopics._POINTS.value}"

    @staticmethod
    def cloud_transformed(camera_name: str) -> str:
        return f"{VisionTopics.CAMERAS_NAMESPACE.value}/{camera_name}/{VisionTopics._CLOUD_TRANSFORMED.value}"

    @staticmethod
    def lidar_points_left() -> str:
        return f"{VisionTopics._LIDAR_POINTS_LEFT.value}"

    @staticmethod
    def lidar_points_right() -> str:
        return f"{VisionTopics._LIDAR_POINTS_RIGHT.value}"

    @staticmethod
    def lidar_points(lidar_name: str) -> str:
        if lidar_name == "left":
            return VisionTopics._LIDAR_POINTS_LEFT.value
        if lidar_name == "right":
            return VisionTopics._LIDAR_POINTS_RIGHT.value
        raise ValueError(f"{lidar_name} is not a valid lidar name")

    @staticmethod
    def gripper_image_raw(camera_name: str) -> str:
        return f"{VisionTopics.GRIPPER_CAMERA_NAMESPACE.value}/{camera_name}/{VisionTopics._IMAGE_RAW.value}"

    @staticmethod
    def gripper_compressed(camera_name: str) -> str:
        return f"{VisionTopics.GRIPPER_CAMERA_NAMESPACE.value}/{camera_name}/{VisionTopics._COMPRESSED.value}"

    @staticmethod
    def gripper_image_rect(camera_name: str) -> str:
        return f"{VisionTopics.GRIPPER_CAMERA_NAMESPACE.value}/{camera_name}/{VisionTopics._IMAGE_RECT.value}"

    @staticmethod
    def gripper_camera_info(camera_name: str) -> str:
        return f"{VisionTopics.GRIPPER_CAMERA_NAMESPACE.value}/{camera_name}/{VisionTopics._CAMERA_INFO.value}"

    @staticmethod
    def gripper_imu() -> str:
        return f"{VisionTopics.GRIPPER_CAMERA_NAMESPACE.value}/imu/data"

    @staticmethod
    def gripper_stereo_points() -> str:
        return f"{VisionTopics.GRIPPER_CAMERA_NAMESPACE.value}/stereo_left_rgbd/points"