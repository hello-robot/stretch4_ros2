from typing import Generator
from .stream_manager import StreamManager
from .camera_info_bridge import camera_info_stream, CameraInfoFrame
from .image_bridge import image_stream, ImageFrame
from .pointcloud_bridge import pointcloud_stream, LidarPointCloudFrame, StereoPointCloudFrame, RGBDPointCloudFrame, PointCloudFrame
from .imu_bridge import imu_stream, ImuFrame

def stream_camera_center_info(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[CameraInfoFrame | None, None, None]:
    topic = "/cameras_head/center/camera_info"
    if stream_manager is not None:
        stream_manager.add_camera_info_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return camera_info_stream(topic, timeout=timeout, block=blocking)

def stream_camera_center(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    topic = "/cameras_head/center/image_raw"
    if stream_manager is not None:
        stream_manager.add_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return image_stream(topic, timeout=timeout, block=blocking)

def stream_camera_center_rotated(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    topic = "/cameras_head/center/rotated_image"
    if stream_manager is not None:
        stream_manager.add_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return image_stream(topic, timeout=timeout, block=blocking)

def stream_camera_left_info(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[CameraInfoFrame | None, None, None]:
    topic = "/cameras_head/left/camera_info"
    if stream_manager is not None:
        stream_manager.add_camera_info_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return camera_info_stream(topic, timeout=timeout, block=blocking)

def stream_camera_left(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    topic = "/cameras_head/left/image_raw"
    if stream_manager is not None:
        stream_manager.add_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return image_stream(topic, timeout=timeout, block=blocking)

def stream_camera_left_rotated(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    topic = "/cameras_head/left/rotated_image"
    if stream_manager is not None:
        stream_manager.add_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return image_stream(topic, timeout=timeout, block=blocking)

def stream_camera_right_info(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[CameraInfoFrame | None, None, None]:
    topic = "/cameras_head/right/camera_info"
    if stream_manager is not None:
        stream_manager.add_camera_info_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return camera_info_stream(topic, timeout=timeout, block=blocking)

def stream_camera_right(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    topic = "/cameras_head/right/image_raw"
    if stream_manager is not None:
        stream_manager.add_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return image_stream(topic, timeout=timeout, block=blocking)

def stream_camera_right_rotated(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    topic = "/cameras_head/right/rotated_image"
    if stream_manager is not None:
        stream_manager.add_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return image_stream(topic, timeout=timeout, block=blocking)

def stream_lidar_points_left(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[LidarPointCloudFrame | None, None, None]:
    topic = "/lidar_points_left"
    if stream_manager is not None:
        stream_manager.add_pointcloud_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return pointcloud_stream(topic, timeout=timeout, block=blocking)

def stream_lidar_points_right(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[LidarPointCloudFrame | None, None, None]:
    topic = "/lidar_points_right"
    if stream_manager is not None:
        stream_manager.add_pointcloud_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return pointcloud_stream(topic, timeout=timeout, block=blocking)

def stream_gripper_imu(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImuFrame | None, None, None]:
    topic = "/cameras_gripper/imu/data"
    if stream_manager is not None:
        stream_manager.add_imu_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return imu_stream(topic, timeout=timeout, block=blocking)

def stream_gripper_right_info(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[CameraInfoFrame | None, None, None]:
    topic = "/cameras_gripper/right/camera_info"
    if stream_manager is not None:
        stream_manager.add_camera_info_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return camera_info_stream(topic, timeout=timeout, block=blocking)

def stream_gripper_right(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    topic = "/cameras_gripper/right/image_raw"
    if stream_manager is not None:
        stream_manager.add_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return image_stream(topic, timeout=timeout, block=blocking)

def stream_gripper_stereo_info(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[CameraInfoFrame | None, None, None]:
    topic = "/cameras_gripper/stereo/camera_info"
    if stream_manager is not None:
        stream_manager.add_camera_info_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return camera_info_stream(topic, timeout=timeout, block=blocking)

def stream_gripper_stereo(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    topic = "/cameras_gripper/stereo/image_raw"
    if stream_manager is not None:
        stream_manager.add_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return image_stream(topic, timeout=timeout, block=blocking)

def stream_gripper_stereo_points(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[StereoPointCloudFrame | None, None, None]:
    topic = "/cameras_gripper/stereo_left_rgbd/points"
    if stream_manager is not None:
        stream_manager.add_pointcloud_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return pointcloud_stream(topic, timeout=timeout, block=blocking)
