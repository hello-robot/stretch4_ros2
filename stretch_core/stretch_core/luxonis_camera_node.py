#!/usr/bin/env python3

from dataclasses import dataclass
from enum import Enum
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import ros2_numpy
import numpy as np
import yaml
import os
from pathlib import Path
import threading
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, DurabilityPolicy

# Import stretch4_body camera stream functions
from stretch4_body.subsystem.cameras import (
    stream_left_camera,
    stream_right_camera,
    stream_center_camera,
    stream_left_right_camera,
    stream_left_right_center_camera,
    stream_gripper_camera,
)

from stretch_core.vision.vision_topics import (
    VisionTopics,
    VisionFrames,
    get_camera_calibration_file_path,
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
        self.declare_parameter('left_calibration_file', '')
        self.declare_parameter('right_calibration_file', '')
        self.declare_parameter('center_calibration_file', '')
        self.declare_parameter('stereo_calibration_file', '') # for depth info if any
        
        # Get parameters
        self.use_left = self.get_parameter('use_left').value
        self.use_right = self.get_parameter('use_right').value
        self.use_center = self.get_parameter('use_center').value
        self.is_gripper = self.get_parameter('is_gripper').value
        self.publish_rotated = self.get_parameter('publish_rotated').value
        self.camera_namespace = self.get_parameter('camera_namespace').value
        
        # Publishers and camera info caches
        self.publishers_topics = {}
        self.rotated_publishers = {}
        self.info_publishers = {}
        self.camera_info = {}
        
        if self.is_gripper:
            # Gripper camera topics as requested and verified:
            # left -> /cameras_gripper/left/...
            # right -> /cameras_gripper/right/...
            # depth (stereo) -> /cameras_gripper/stereo/...
            self.get_logger().info("Configuring node for Gripper Camera Mode")
            
            # Left camera
            self.publishers_topics['left'] = self.create_publisher(
                Image, VisionTopics.gripper_image_raw('left'), 10)
            self.info_publishers['left'] = self.create_publisher(
                CameraInfo, VisionTopics.gripper_camera_info('left'), 10)
            calib_file = self.get_parameter('left_calibration_file').value
            if calib_file:
                self.camera_info['left'] = self.load_camera_info(calib_file)
                
            # Right camera
            self.publishers_topics['right'] = self.create_publisher(
                Image, VisionTopics.gripper_image_raw('right'), 10)
            self.info_publishers['right'] = self.create_publisher(
                CameraInfo, VisionTopics.gripper_camera_info('right'), 10)
            calib_file = self.get_parameter('right_calibration_file').value
            if calib_file:
                self.camera_info['right'] = self.load_camera_info(calib_file)
                
            # Depth camera (named stereo)
            self.publishers_topics['stereo'] = self.create_publisher(
                Image, VisionTopics.gripper_image_raw('stereo'), 10)
            self.info_publishers['stereo'] = self.create_publisher(
                CameraInfo, VisionTopics.gripper_camera_info('stereo'), 10)
            calib_file = self.get_parameter('stereo_calibration_file').value
            if calib_file:
                self.camera_info['stereo'] = self.load_camera_info(calib_file)
                
        else:
            self.get_logger().info("Configuring node for Head Cameras Mode")
            # Head cameras
            if self.use_left:
                self.publishers_topics['left'] = self.create_publisher(
                    Image, VisionTopics.image_raw('left'), 10)
                if self.publish_rotated:
                    self.rotated_publishers['left'] = self.create_publisher(
                        Image, VisionTopics.rotated_image('left'), 10)
                self.info_publishers['left'] = self.create_publisher(
                    CameraInfo, VisionTopics.camera_info('left'), 10)
                calib_file = self.get_parameter('left_calibration_file').value
                if not calib_file:
                    calib_file = get_camera_calibration_file_path('left')
                if calib_file and os.path.exists(calib_file):
                    self.camera_info['left'] = self.load_camera_info(calib_file)
                    
            if self.use_right:
                self.publishers_topics['right'] = self.create_publisher(
                    Image, VisionTopics.image_raw('right'), 10)
                if self.publish_rotated:
                    self.rotated_publishers['right'] = self.create_publisher(
                        Image, VisionTopics.rotated_image('right'), 10)
                self.info_publishers['right'] = self.create_publisher(
                    CameraInfo, VisionTopics.camera_info('right'), 10)
                calib_file = self.get_parameter('right_calibration_file').value
                if not calib_file:
                    calib_file = get_camera_calibration_file_path('right')
                if calib_file and os.path.exists(calib_file):
                    self.camera_info['right'] = self.load_camera_info(calib_file)
                    
            if self.use_center:
                self.publishers_topics['center'] = self.create_publisher(
                    Image, VisionTopics.image_raw('center'), 10)
                if self.publish_rotated:
                    self.rotated_publishers['center'] = self.create_publisher(
                        Image, VisionTopics.rotated_image('center'), 10)
                self.info_publishers['center'] = self.create_publisher(
                    CameraInfo, VisionTopics.camera_info('center'), 10)
                calib_file = self.get_parameter('center_calibration_file').value
                if not calib_file:
                    calib_file = get_camera_calibration_file_path('center')
                if calib_file and os.path.exists(calib_file):
                    self.camera_info['center'] = self.load_camera_info(calib_file)

        # Initialize thread state
        self.running = True
        self.publish_thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.publish_thread.start()
        
        self.get_logger().info('Luxonis camera node initialized')

    def load_camera_info(self, calib_file):
        """Load camera calibration from YAML file"""
        try:
            # Remove file:// prefix if present
            if calib_file.startswith('file://'):
                calib_file = calib_file[7:]

            if not os.path.exists(calib_file):
                self.get_logger().warn(f'Calibration file not found: {calib_file}')
                return None

            with open(calib_file, 'r') as f:
                calib = yaml.safe_load(f)

            if calib is None:
                return None

            # Create CameraInfo message
            info = CameraInfo()
            info.width = calib.get('image_width', 0)
            info.height = calib.get('image_height', 0)
            info.distortion_model = calib.get('distortion_model', '')

            # Load distortion coefficients
            if 'distortion_coefficients' in calib:
                info.d = calib['distortion_coefficients'].get('data', [])

            # Load camera matrix
            if 'camera_matrix' in calib:
                info.k = calib['camera_matrix'].get('data', [0.0] * 9)

            # Load rectification matrix
            if 'rectification_matrix' in calib:
                info.r = calib['rectification_matrix'].get('data', [0.0] * 9)

            # Load projection matrix
            if 'projection_matrix' in calib:
                info.p = calib['projection_matrix'].get('data', [0.0] * 12)

            if info.distortion_model == "fisheye":
                info.distortion_model = "equidistant"

            self.get_logger().info(f'Loaded calibration from {calib_file}')
            return info

        except Exception as e:
            self.get_logger().error(f'Failed to load calibration file {calib_file}: {e}')
            return None

    def get_default_camera_info(self, width, height, frame_id):
        """Return a default/uncalibrated CameraInfo message based on frame dimensions"""
        info = CameraInfo()
        info.width = width
        info.height = height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        fx = float(width)
        fy = float(width)
        cx = width / 2.0
        cy = height / 2.0
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.header.frame_id = frame_id
        return info

    def publish_loop(self):
        """Main publishing loop running in separate thread"""
        try:
            if self.is_gripper:
                self.publish_gripper_frames()
            else:
                self.publish_head_frames()
        except Exception as e:
            self.get_logger().error(f'Error in publish loop: {e}')

    def publish_gripper_frames(self):
        self.get_logger().info("Starting Gripper Camera Stream...")
        # Call stream_gripper_camera with is_rotate=False
        generator = stream_gripper_camera(is_rotate=False)
        for synced_frame in generator:
            if not self.running or not rclpy.ok():
                break
            if synced_frame is None:
                continue
                
            stamp = self.get_clock().now().to_msg()
            
            # 1. Publish Left Camera (image + camera_info)
            if synced_frame.left is not None and synced_frame.left.image is not None:
                left_img = synced_frame.left.image
                h, w = left_img.shape[:2]
                frame_id = VisionFrames.gripper_camera_frame('left')
                
                # Image msg
                img_msg = ros2_numpy.msgify(Image, left_img, encoding='bgr8')
                img_msg.header.stamp = stamp
                img_msg.header.frame_id = frame_id
                self.publishers_topics['left'].publish(img_msg)
                
                # CameraInfo msg
                if 'left' not in self.camera_info or self.camera_info['left'] is None:
                    self.camera_info['left'] = self.get_default_camera_info(w, h, frame_id)
                info_msg = self.camera_info['left']
                info_msg.header.stamp = stamp
                info_msg.header.frame_id = frame_id
                self.info_publishers['left'].publish(info_msg)
                
            # 2. Publish Right Camera (image + camera_info)
            if synced_frame.right is not None and synced_frame.right.image is not None:
                right_img = synced_frame.right.image
                h, w = right_img.shape[:2]
                frame_id = VisionFrames.gripper_camera_frame('right')
                
                # Image msg
                img_msg = ros2_numpy.msgify(Image, right_img, encoding='bgr8')
                img_msg.header.stamp = stamp
                img_msg.header.frame_id = frame_id
                self.publishers_topics['right'].publish(img_msg)
                
                # CameraInfo msg
                if 'right' not in self.camera_info or self.camera_info['right'] is None:
                    self.camera_info['right'] = self.get_default_camera_info(w, h, frame_id)
                info_msg = self.camera_info['right']
                info_msg.header.stamp = stamp
                info_msg.header.frame_id = frame_id
                self.info_publishers['right'].publish(info_msg)
                
            # 3. Publish Depth Camera (named stereo in topic list)
            if synced_frame.depth is not None:
                depth_img = synced_frame.depth
                h, w = depth_img.shape[:2]
                frame_id = VisionFrames.gripper_camera_frame('stereo')
                
                # Image msg
                img_msg = ros2_numpy.msgify(Image, depth_img, encoding='16UC1')
                img_msg.header.stamp = stamp
                img_msg.header.frame_id = frame_id
                self.publishers_topics['stereo'].publish(img_msg)
                
                # CameraInfo msg
                if 'stereo' not in self.camera_info or self.camera_info['stereo'] is None:
                    self.camera_info['stereo'] = self.get_default_camera_info(w, h, frame_id)
                info_msg = self.camera_info['stereo']
                info_msg.header.stamp = stamp
                info_msg.header.frame_id = frame_id
                self.info_publishers['stereo'].publish(info_msg)

    def publish_head_frames(self):
        self.get_logger().info("Starting Head Cameras Stream...")
        
        # Decide the correct generator depending on enabled head cameras
        if self.use_left and self.use_right and self.use_center:
            generator = stream_left_right_center_camera(is_rotate=False)
        elif self.use_left and self.use_right:
            generator = stream_left_right_camera(is_rotate=False)
        elif self.use_left:
            generator = stream_left_camera(is_rotate=False)
        elif self.use_right:
            generator = stream_right_camera(is_rotate=False)
        elif self.use_center:
            generator = stream_center_camera(is_rotate=False)
        else:
            self.get_logger().warn("No head cameras selected!")
            return

        for frame in generator:
            if not self.running or not rclpy.ok():
                break
            if frame is None:
                continue
                
            stamp = self.get_clock().now().to_msg()
            
             # Helper function to publish a single frame's data
            def publish_head_image_and_info(img_frame, camera_name):
                if img_frame is None or img_frame.image is None:
                    return
                img = img_frame.image
                h, w = img.shape[:2]
                frame_id = VisionFrames.camera_frame(camera_name)
                
                # Image msg
                img_msg = ros2_numpy.msgify(Image, img, encoding='bgr8')
                img_msg.header.stamp = stamp
                img_msg.header.frame_id = frame_id
                self.publishers_topics[camera_name].publish(img_msg)
                
                # Rotated Image msg
                if self.publish_rotated:
                    try:
                        pub = self.rotated_publishers.get(camera_name)
                        if pub is not None and pub.get_subscription_count() > 0:
                            rotate_k = VisionFrames.camera_frame_number_of_rotations(camera_name)
                            rotated_img = np.rot90(img, k=rotate_k)
                            rotated_msg = ros2_numpy.msgify(Image, rotated_img, encoding='bgr8')
                            rotated_msg.header.stamp = stamp
                            rotated_msg.header.frame_id = frame_id
                            pub.publish(rotated_msg)
                    except Exception as ex:
                        self.get_logger().error(f"Failed to rotate and publish image for {camera_name}: {ex}")
                
                # CameraInfo msg
                if camera_name not in self.camera_info or self.camera_info[camera_name] is None:
                    self.camera_info[camera_name] = self.get_default_camera_info(w, h, frame_id)
                info_msg = self.camera_info[camera_name]
                info_msg.header.stamp = stamp
                info_msg.header.frame_id = frame_id
                self.info_publishers[camera_name].publish(info_msg)

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
                if self.use_left:
                    publish_head_image_and_info(frame, 'left')
                elif self.use_right:
                    publish_head_image_and_info(frame, 'right')
                elif self.use_center:
                    publish_head_image_and_info(frame, 'center')

    def destroy_node(self):
        """Clean up resources"""
        self.running = False
        if self.publish_thread.is_alive():
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
