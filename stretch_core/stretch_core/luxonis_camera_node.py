#!/usr/bin/env python3

from dataclasses import dataclass
from enum import Enum
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
import ros2_numpy
import numpy as np
import cv2
import yaml
import os
from pathlib import Path
import threading
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


# Import stretch4_body camera stream functions
from stretch4_body.subsystem.cameras import (
    stream_left_camera,
    stream_right_camera,
    stream_center_camera,
    stream_left_right_camera,
    stream_left_right_center_camera,
    stream_gripper_camera,
    stream_left_camera_compressed,
    stream_right_camera_compressed,
    stream_center_camera_compressed,
    stream_left_right_camera_compressed,
    stream_left_right_center_camera_compressed,
    stream_gripper_camera_compressed,
)

from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras

from stretch_python_bridge import compressed_format_with_sequence

from stretch_core.vision.ros_messages import DeviceClockOffset, create_timestamp
from stretch_core.vision.vision_topics import (
    VisionTopics,
    VisionFrames,
)

class LuxonisCameraNode(Node):
    def __init__(self):
        super().__init__('luxonis_camera_node')
        
        # Declare parameters
        self.declare_parameter('use_left', True)
        self.declare_parameter('use_right', True)
        self.declare_parameter('use_center', True)
        self.declare_parameter('is_gripper', False)
        self.declare_parameter('publish_rotated', True)
        self.declare_parameter('camera_namespace', 'camera')
        self.declare_parameter(
            'use_compressed',
            True,
            ParameterDescriptor(description=(
                "If true, capture MJPEG straight off the camera and republish it on the compressed "
                "topics. This is the fast path: nothing decodes or re-encodes, and a frame costs a "
                "fraction of the bandwidth of raw BGR. The raw and rotated topics still work, but "
                "they are only published while something is subscribed to them, because serving them "
                "means decoding every frame."
            )),
        )
        self.declare_parameter(
            'use_system_timestamp',
            True,
            ParameterDescriptor(description=(
                "If true, shift the camera's device timestamps onto the system clock before "
                "stamping messages. The device clock's steadiness is kept, it is just offset so "
                "that consumers can compare these stamps against the rest of the system. If "
                "false, the device timestamp is published as-is."
            )),
        )

        # Get parameters
        self.use_left = self.get_parameter('use_left').value
        self.use_right = self.get_parameter('use_right').value
        self.use_center = self.get_parameter('use_center').value
        self.is_gripper = self.get_parameter('is_gripper').value
        self.publish_rotated = self.get_parameter('publish_rotated').value
        self.camera_namespace = self.get_parameter('camera_namespace').value
        self.use_compressed = self.get_parameter('use_compressed').value
        self.use_system_timestamp = self.get_parameter('use_system_timestamp').value

        # The device clock is shared by every camera on the device, so one offset estimator
        # serves them all and sees more samples than a per-camera one would.
        self.clock_offset = DeviceClockOffset() if self.use_system_timestamp else None

        # Publishers and camera info caches
        self.publishers_topics = {}
        self.compressed_publishers = {}
        self.rotated_publishers = {}
        self.info_publishers = {}
        self.camera_info = {}

        # Sensor data (images, camera info) is published Best Effort
        self.sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        if self.is_gripper:
            # Gripper camera topics as requested and verified:
            # left -> /cameras_gripper/left/...
            # right -> /cameras_gripper/right/...
            # depth (stereo) -> /cameras_gripper/stereo/...
            self.get_logger().info("Configuring node for Gripper Camera Mode")
            
            # Left camera
            self.publishers_topics['left'] = self.create_publisher(
                Image, VisionTopics.gripper_image_raw('left'), self.sensor_qos)
            if self.use_compressed:
                self.compressed_publishers['left'] = self.create_publisher(
                    CompressedImage, VisionTopics.gripper_compressed('left'), self.sensor_qos)
            self.info_publishers['left'] = self.create_publisher(
                CameraInfo, VisionTopics.gripper_camera_info('left'), self.sensor_qos)
            self.camera_info['left'] = self.load_camera_info_from_enum(RGBCameras.gripper_left)

            # Right camera
            self.publishers_topics['right'] = self.create_publisher(
                Image, VisionTopics.gripper_image_raw('right'), self.sensor_qos)
            if self.use_compressed:
                self.compressed_publishers['right'] = self.create_publisher(
                    CompressedImage, VisionTopics.gripper_compressed('right'), self.sensor_qos)
            self.info_publishers['right'] = self.create_publisher(
                CameraInfo, VisionTopics.gripper_camera_info('right'), self.sensor_qos)
            self.camera_info['right'] = self.load_camera_info_from_enum(RGBCameras.gripper_right)

            # Depth camera (named stereo)
            self.publishers_topics['stereo'] = self.create_publisher(
                Image, VisionTopics.gripper_image_raw('stereo'), self.sensor_qos)
            self.info_publishers['stereo'] = self.create_publisher(
                CameraInfo, VisionTopics.gripper_camera_info('stereo'), self.sensor_qos)
            self.camera_info['stereo'] = self.camera_info['right'] # share right camera info for depth
                
        else:
            self.get_logger().info("Configuring node for Head Cameras Mode")
            # Head cameras
            head_cameras = {
                'left': (self.use_left, RGBCameras.head_left),
                'right': (self.use_right, RGBCameras.head_right),
                'center': (self.use_center, RGBCameras.head_center),
            }
            for camera_name, (is_enabled, camera_type) in head_cameras.items():
                if not is_enabled:
                    continue

                self.publishers_topics[camera_name] = self.create_publisher(
                    Image, VisionTopics.image_raw(camera_name), self.sensor_qos)
                if self.use_compressed:
                    self.compressed_publishers[camera_name] = self.create_publisher(
                        CompressedImage, VisionTopics.compressed(camera_name), self.sensor_qos)
                if self.publish_rotated:
                    self.rotated_publishers[camera_name] = self.create_publisher(
                        Image, VisionTopics.rotated_image(camera_name), self.sensor_qos)
                self.info_publishers[camera_name] = self.create_publisher(
                    CameraInfo, VisionTopics.camera_info(camera_name), self.sensor_qos)
                self.camera_info[camera_name] = self.load_camera_info_from_enum(camera_type)

        # Initialize thread state
        self.running = True
        self.publish_thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.publish_thread.start()
        
        self.get_logger().info('Luxonis camera node initialized')

    def load_camera_info_from_enum(self, camera_type):
        """Load camera calibration from RGBCameras.load_calibration()"""
        try:
            calib = camera_type.load_calibration()
            if calib is None:
                return None
                
            info = CameraInfo()
            info.width = calib.width
            info.height = calib.height
            info.distortion_model = calib.distortion_model.name.lower()
            
            # distortion coefficients d
            info.d = calib.distortion_coefficients.flatten().tolist()
            
            # camera matrix k (3x3)
            info.k = calib.camera_matrix.flatten().tolist()
            
            # rectification matrix r (3x3, identity by default)
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            
            # projection matrix p (3x4)
            fx = calib.camera_matrix[0, 0]
            fy = calib.camera_matrix[1, 1]
            cx = calib.camera_matrix[0, 2]
            cy = calib.camera_matrix[1, 2]
            info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            
            if info.distortion_model in ["fisheye", "equidistant_with_recompute_extrinsics"]:
                info.distortion_model = "equidistant"
            elif info.distortion_model == "wide_angle":
                info.distortion_model = "plumb_bob"

            return info
        except Exception as e:
            self.get_logger().error(f'Failed to load calibration for {camera_type.name} via RGBCameras enum: {e}')
            return None
        
    def publish_loop(self):
        """Main publishing loop running in separate thread"""
        try:
            if self.is_gripper:
                self.publish_gripper_frames()
            else:
                self.publish_head_frames()
        except Exception as e:
            self.get_logger().error(f'Error in publish loop: {e}')

    def create_stamp(self, device_seconds: float):
        """The frame's timestamp as a ROS Time, on the system clock unless asked otherwise."""
        if self.clock_offset is not None:
            device_seconds = self.clock_offset.to_ros(device_seconds)
        return create_timestamp(device_seconds)

    def decode_if_needed(self, img_frame):
        """The frame as raw BGR pixels, decoding the bitstream first if the camera handed us one.

        Deliberately does not use ImageFrame.uncompress(), which would mutate the frame we are about to
        republish compressed.
        """
        if not img_frame.is_compressed():
            return img_frame.image

        image = cv2.imdecode(img_frame.image, cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().error("Failed to decode a compressed frame; skipping its raw publish.")
        return image

    def publish_camera_frame(self, img_frame, camera_name: str, frame_id: str, rotate_k: int | None = None):
        """Publishes one camera frame plus its CameraInfo.

        An already-encoded frame goes out on the compressed topic as-is, which costs nothing but a
        memcpy. The raw and rotated topics need decoded pixels, so they are only served while someone
        is subscribed - otherwise the whole point of capturing MJPEG would be lost to a decode per frame.
        """
        if img_frame is None or img_frame.image is None:
            return

        stamp = self.create_stamp(img_frame.timestamp)

        compressed_publisher = self.compressed_publishers.get(camera_name)
        if img_frame.is_compressed() and compressed_publisher is not None:
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = stamp
            compressed_msg.header.frame_id = frame_id
            # ROS 2 has no header.seq, so the sensor's sequence number rides along in the format string.
            compressed_msg.format = compressed_format_with_sequence(
                img_frame.compression_format or "jpeg", img_frame.frame_number
            )
            compressed_msg.data = img_frame.image.tobytes()
            compressed_publisher.publish(compressed_msg)

        image = None
        raw_publisher = self.publishers_topics.get(camera_name)
        if raw_publisher is not None and raw_publisher.get_subscription_count() > 0:
            image = self.decode_if_needed(img_frame)
            if image is not None:
                img_msg = ros2_numpy.msgify(Image, image, encoding='bgr8')
                img_msg.header.stamp = stamp
                img_msg.header.frame_id = frame_id
                raw_publisher.publish(img_msg)

        rotated_publisher = self.rotated_publishers.get(camera_name)
        if rotate_k and rotated_publisher is not None and rotated_publisher.get_subscription_count() > 0:
            try:
                if image is None:
                    image = self.decode_if_needed(img_frame)
                if image is not None:
                    rotated_msg = ros2_numpy.msgify(Image, np.rot90(image, k=rotate_k), encoding='bgr8')
                    rotated_msg.header.stamp = stamp
                    rotated_msg.header.frame_id = frame_id
                    rotated_publisher.publish(rotated_msg)
            except Exception as ex:
                self.get_logger().error(f"Failed to rotate and publish image for {camera_name}: {ex}")

        if camera_name not in self.camera_info or self.camera_info[camera_name] is None:
            raise RuntimeError(f"Camera calibration file for {camera_name} is missing or could not be loaded!")
        info_msg = self.camera_info[camera_name]
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = frame_id
        self.info_publishers[camera_name].publish(info_msg)

    def publish_gripper_frames(self):
        self.get_logger().info(f"Starting Gripper Camera Stream (compressed={self.use_compressed})...")

        # is_run_pipeline=False keeps frames exactly as the camera produced them: no decode, no
        # rotation, no extra full-frame copy. Anything that needs raw pixels decodes on demand below.
        stream_gripper = stream_gripper_camera_compressed if self.use_compressed else stream_gripper_camera
        generator = stream_gripper(is_rotate=False, is_run_pipeline=False)

        for synced_frame in generator:
            if not self.running or not rclpy.ok():
                break
            if synced_frame is None:
                continue

            self.publish_camera_frame(synced_frame.left, 'left', VisionFrames.gripper_camera_frame('left'))
            self.publish_camera_frame(synced_frame.right, 'right', VisionFrames.gripper_camera_frame('right'))

            # 16-bit depth is not MJPEG encodable, so it stays on the raw topic.
            if synced_frame.depth is not None:
                frame_id = VisionFrames.gripper_camera_frame('stereo')
                stamp = self.create_stamp(synced_frame.right.timestamp if synced_frame.right is not None else synced_frame.timestamp)

                depth_publisher = self.publishers_topics['stereo']
                if depth_publisher.get_subscription_count() > 0:
                    img_msg = ros2_numpy.msgify(Image, synced_frame.depth, encoding='16UC1')
                    img_msg.header.stamp = stamp
                    img_msg.header.frame_id = frame_id
                    depth_publisher.publish(img_msg)

                if 'stereo' not in self.camera_info or self.camera_info['stereo'] is None:
                    raise RuntimeError("Camera calibration file for gripper stereo is missing or could not be loaded!")
                info_msg = self.camera_info['stereo']
                info_msg.header.stamp = stamp
                info_msg.header.frame_id = frame_id
                self.info_publishers['stereo'].publish(info_msg)

    def publish_head_frames(self):
        self.get_logger().info(f"Starting Head Cameras Stream (compressed={self.use_compressed})...")

        # Decide the correct generator depending on enabled head cameras. See publish_gripper_frames()
        # for why nothing runs the image pipeline here.
        if self.use_compressed:
            single_streams = {
                'left': stream_left_camera_compressed,
                'right': stream_right_camera_compressed,
                'center': stream_center_camera_compressed,
            }
            stream_left_right = stream_left_right_camera_compressed
            stream_left_right_center = stream_left_right_center_camera_compressed
        else:
            single_streams = {
                'left': stream_left_camera,
                'right': stream_right_camera,
                'center': stream_center_camera,
            }
            stream_left_right = stream_left_right_camera
            stream_left_right_center = stream_left_right_center_camera

        enabled = [name for name, is_enabled in (('left', self.use_left), ('right', self.use_right), ('center', self.use_center)) if is_enabled]

        if self.use_left and self.use_right and self.use_center:
            generator = stream_left_right_center(is_rotate=False, is_run_pipeline=False)
        elif self.use_left and self.use_right:
            generator = stream_left_right(is_rotate=False, is_run_pipeline=False)
        elif len(enabled) == 1:
            generator = single_streams[enabled[0]](is_rotate=False, is_run_pipeline=False)
        else:
            self.get_logger().warn(f"Unsupported head camera combination: {enabled or 'none selected'}!")
            return

        for frame in generator:
            if not self.running or not rclpy.ok():
                break
            if frame is None:
                continue

            def publish_head_image_and_info(img_frame, camera_name):
                self.publish_camera_frame(
                    img_frame,
                    camera_name,
                    VisionFrames.camera_frame(camera_name),
                    rotate_k=VisionFrames.camera_frame_number_of_rotations(camera_name) if self.publish_rotated else None,
                )

            # Check if it is a SyncedImageFrame or a single ImageFrame
            if hasattr(frame, 'left') or hasattr(frame, 'right') or hasattr(frame, 'center'):
                # SyncedImageFrame
                if self.use_left and hasattr(frame, 'left'):
                    publish_head_image_and_info(frame.left, 'left')
                if self.use_right and hasattr(frame, 'right'):
                    publish_head_image_and_info(frame.right, 'right')
                if self.use_center and hasattr(frame, 'center'):
                    publish_head_image_and_info(frame.center, 'center')
            else:
                # Single ImageFrame
                publish_head_image_and_info(frame, enabled[0])

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'publish_thread') and self.publish_thread.is_alive():
            self.publish_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LuxonisCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
