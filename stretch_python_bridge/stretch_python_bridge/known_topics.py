from typing import Generator
from .stream_manager import StreamManager
from .camera_info_bridge import camera_info_stream, CameraInfoFrame
from .image_bridge import compressed_image_stream, image_stream, ImageFrame
from .pointcloud_bridge import pointcloud_stream, LidarPointCloudFrame, StereoPointCloudFrame, RGBDPointCloudFrame, PointCloudFrame
from .imu_bridge import imu_stream, ImuFrame
from stretch_core.vision.vision_topics import VisionTopics

def stream_camera_center_info(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[CameraInfoFrame | None, None, None]:
    topic = VisionTopics.camera_info("center")
    if stream_manager is not None:
        stream_manager.add_camera_info_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return camera_info_stream(topic, timeout=timeout, block=blocking)

def stream_camera_center(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    topic = VisionTopics.image_raw("center")
    if stream_manager is not None:
        stream_manager.add_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return image_stream(topic, timeout=timeout, block=blocking)

def stream_camera_center_rotated(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    topic = VisionTopics.rotated_image("center")
    if stream_manager is not None:
        stream_manager.add_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return image_stream(topic, timeout=timeout, block=blocking)

def stream_camera_left_info(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[CameraInfoFrame | None, None, None]:
    topic = VisionTopics.camera_info("left")
    if stream_manager is not None:
        stream_manager.add_camera_info_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return camera_info_stream(topic, timeout=timeout, block=blocking)

def stream_camera_left(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    topic = VisionTopics.image_raw("left")
    if stream_manager is not None:
        stream_manager.add_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return image_stream(topic, timeout=timeout, block=blocking)

def stream_camera_left_rotated(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    topic = VisionTopics.rotated_image("left")
    if stream_manager is not None:
        stream_manager.add_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return image_stream(topic, timeout=timeout, block=blocking)

def stream_camera_right_info(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[CameraInfoFrame | None, None, None]:
    topic = VisionTopics.camera_info("right")
    if stream_manager is not None:
        stream_manager.add_camera_info_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return camera_info_stream(topic, timeout=timeout, block=blocking)

def stream_camera_right(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    topic = VisionTopics.image_raw("right")
    if stream_manager is not None:
        stream_manager.add_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return image_stream(topic, timeout=timeout, block=blocking)

def stream_camera_right_rotated(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    topic = VisionTopics.rotated_image("right")
    if stream_manager is not None:
        stream_manager.add_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return image_stream(topic, timeout=timeout, block=blocking)

def stream_lidar_points_left(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[LidarPointCloudFrame | None, None, None]:
    topic = VisionTopics.lidar_points("left")
    if stream_manager is not None:
        stream_manager.add_pointcloud_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return pointcloud_stream(topic, timeout=timeout, block=blocking)

def stream_lidar_points_right(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[LidarPointCloudFrame | None, None, None]:
    topic = VisionTopics.lidar_points("right")
    if stream_manager is not None:
        stream_manager.add_pointcloud_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return pointcloud_stream(topic, timeout=timeout, block=blocking)

def stream_gripper_imu(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImuFrame | None, None, None]:
    topic = VisionTopics.gripper_imu()
    if stream_manager is not None:
        stream_manager.add_imu_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return imu_stream(topic, timeout=timeout, block=blocking)

def stream_gripper_left_info(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[CameraInfoFrame | None, None, None]:
    topic = VisionTopics.gripper_camera_info("left")
    if stream_manager is not None:
        stream_manager.add_camera_info_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return camera_info_stream(topic, timeout=timeout, block=blocking)

def stream_gripper_left(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    topic = VisionTopics.gripper_image_raw("left")
    if stream_manager is not None:
        stream_manager.add_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return image_stream(topic, timeout=timeout, block=blocking)

def stream_gripper_right_info(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[CameraInfoFrame | None, None, None]:
    topic = VisionTopics.gripper_camera_info("right")
    if stream_manager is not None:
        stream_manager.add_camera_info_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return camera_info_stream(topic, timeout=timeout, block=blocking)

def stream_gripper_right(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    topic = VisionTopics.gripper_image_raw("right")
    if stream_manager is not None:
        stream_manager.add_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return image_stream(topic, timeout=timeout, block=blocking)

def stream_gripper_stereo_info(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[CameraInfoFrame | None, None, None]:
    topic = VisionTopics.gripper_camera_info("stereo")
    if stream_manager is not None:
        stream_manager.add_camera_info_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return camera_info_stream(topic, timeout=timeout, block=blocking)

def stream_gripper_stereo(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    topic = VisionTopics.gripper_image_raw("stereo")
    if stream_manager is not None:
        stream_manager.add_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return image_stream(topic, timeout=timeout, block=blocking)

def stream_gripper_stereo_points(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[StereoPointCloudFrame | None, None, None]:
    topic = VisionTopics.gripper_stereo_points()
    if stream_manager is not None:
        stream_manager.add_pointcloud_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return pointcloud_stream(topic, timeout=timeout, block=blocking)


# The compressed streams below carry the same images as their raw counterparts, but MJPEG encoded by
# the camera itself. Frames arrive still encoded (`ImageFrame.is_compressed()`), which is a fraction of
# the bytes and lets the publisher skip decoding entirely, so these are the fast path for anything that
# just needs pixels on the host. They also carry the sensor's sequence number in `frame_number`.
# The gripper depth map is 16-bit, so MJPEG cannot carry it; its compressed stream is PNG encoded on
# the compressedDepth topic instead. See stream_gripper_stereo_compressed().

def stream_camera_left_compressed(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    topic = VisionTopics.compressed("left")
    if stream_manager is not None:
        stream_manager.add_compressed_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return compressed_image_stream(topic, timeout=timeout, block=blocking)

def stream_camera_right_compressed(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    topic = VisionTopics.compressed("right")
    if stream_manager is not None:
        stream_manager.add_compressed_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return compressed_image_stream(topic, timeout=timeout, block=blocking)

def stream_camera_center_compressed(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    topic = VisionTopics.compressed("center")
    if stream_manager is not None:
        stream_manager.add_compressed_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return compressed_image_stream(topic, timeout=timeout, block=blocking)

def stream_gripper_left_compressed(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    topic = VisionTopics.gripper_compressed("left")
    if stream_manager is not None:
        stream_manager.add_compressed_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return compressed_image_stream(topic, timeout=timeout, block=blocking)

def stream_gripper_right_compressed(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    topic = VisionTopics.gripper_compressed("right")
    if stream_manager is not None:
        stream_manager.add_compressed_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return compressed_image_stream(topic, timeout=timeout, block=blocking)

def stream_gripper_stereo_compressed(timeout: float | None = 10.0, blocking: bool = True, stream_manager: StreamManager|None = None) -> Generator[ImageFrame | None, None, None]:
    """The gripper depth map, PNG encoded on the compressedDepth topic.

    Frames arrive still encoded, so `ImageFrame.image` is the payload rather than a depth map. Turn it
    into one with decode_compressed_depth().
    """
    topic = VisionTopics.gripper_compressed_depth("stereo")
    if stream_manager is not None:
        stream_manager.add_compressed_image_topic(topic)
        return stream_manager.create_topic_generator(topic)
    return compressed_image_stream(topic, timeout=timeout, block=blocking)
