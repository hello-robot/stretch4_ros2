#!/usr/bin/env python3

import os
from pathlib import Path
import threading
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

import ros2_numpy

# Import stretch4_body camera stream functions
from stretch4_body.subsystem.cameras import (
    stream_left_rgbd,
    stream_right_rgbd,
    stream_center_rgbd,
    stream_left_right_rgbd,
    stream_left_right_center_rgbd,
    stream_gripper_rgbd,
)

from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras

from stretch_core.vision.vision_topics import (
    VisionTopics,
    VisionFrames,
)

from stretch_core.vision.ros_messages import create_pointcloud_rgb_msg


class RGBDCameraNode(Node):

    def __init__(self):
        super().__init__('rgbd_camera_node')
        
        # Declare parameters
        self.declare_parameter('use_left', True)
        self.declare_parameter('use_right', True)
        self.declare_parameter('use_center', False)
        self.declare_parameter('use_gripper', False)
        self.declare_parameter('publish_rotated', False)
        
        # Read parameters
        self.use_left = self.get_parameter('use_left').value
        self.use_right = self.get_parameter('use_right').value
        self.use_center = self.get_parameter('use_center').value
        self.use_gripper = self.get_parameter('use_gripper').value
        self.publish_rotated = self.get_parameter('publish_rotated').value
        
        # Publishers and camera info caches
        self.publishers_topics = {}
        self.rotated_publishers = {}
        self.info_publishers = {}
        self.depth_publishers = {}
        self.depth_info_publishers = {}
        self.points_publishers = {}
        self.camera_info = {}
        
        self.get_logger().info("Configuring RGBD Node...")
        

        # Configure enabled head cameras
        for cam in ['left', 'right', 'center']:
            if getattr(self, f"use_{cam}"):
                self.publishers_topics[cam] = self.create_publisher(
                    Image, VisionTopics.image_raw(cam), 10)
                
                if self.publish_rotated:
                    self.rotated_publishers[cam] = self.create_publisher(
                        Image, VisionTopics.rotated_image(cam), 10)
                        
                self.info_publishers[cam] = self.create_publisher(
                    CameraInfo, VisionTopics.camera_info(cam), 10)
                    
                self.depth_publishers[cam] = self.create_publisher(
                    Image, VisionTopics.depth(cam), 10)
                    
                self.depth_info_publishers[cam] = self.create_publisher(
                    CameraInfo, VisionTopics.depth_camera_info(cam), 10)
                    
                self.points_publishers[cam] = self.create_publisher(
                    PointCloud2, VisionTopics.points(cam), 10)
                    
                # Load calibration via RGBCameras enum
                if cam == 'left':
                    self.camera_info[cam] = self.load_camera_info_from_enum(RGBCameras.head_left)
                elif cam == 'right':
                    self.camera_info[cam] = self.load_camera_info_from_enum(RGBCameras.head_right)
                elif cam == 'center':
                    self.camera_info[cam] = self.load_camera_info_from_enum(RGBCameras.head_center)
                    
        # Configure gripper camera if enabled
        if self.use_gripper:
            self.publishers_topics['gripper'] = self.create_publisher(
                Image, "/cameras_gripper/right/image_raw", 10)
            self.info_publishers['gripper'] = self.create_publisher(
                CameraInfo, "/cameras_gripper/right/camera_info", 10)
            self.depth_publishers['gripper'] = self.create_publisher(
                Image, "/cameras_gripper/stereo/image_raw", 10)
            self.depth_info_publishers['gripper'] = self.create_publisher(
                CameraInfo, "/cameras_gripper/stereo/camera_info", 10)
            self.points_publishers['gripper'] = self.create_publisher(
                PointCloud2, "/cameras_gripper/stereo_left_rgbd/points", 10)

            # Load calibration via RGBCameras enum
            self.camera_info['gripper'] = self.load_camera_info_from_enum(RGBCameras.gripper_right)

        # Initialize thread states
        self.running = True
        
        # Launch single unified thread for head cameras to guarantee hardware synchronization and avoid device conflicts
        self.head_camera_thread = None
        if self.use_left or self.use_right or self.use_center:
            self.head_camera_thread = threading.Thread(target=self.head_publish_loop, daemon=True)
            self.head_camera_thread.start()
        
        # Gripper camera runs concurrently in its own independent thread
        if self.use_gripper:
            self.gripper_publish_thread = threading.Thread(target=self.gripper_publish_loop, daemon=True)
            self.gripper_publish_thread.start()
        
        self.get_logger().info('RGBD camera node initialized successfully')

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

    def publish_rgbd_frame(self, rgbd_frame, camera_name, stamp):
        if rgbd_frame is None:
            return
        
        # Extract components
        img_frame = rgbd_frame.image_frame
        if img_frame is None or img_frame.image is None:
            return
            
        img = img_frame.image
        h, w = img.shape[:2]
        
        # Get frame id
        if camera_name == 'gripper':
            frame_id = VisionFrames.gripper_camera_frame('right')
        else:
            frame_id = VisionFrames.camera_frame(camera_name)
        
        # 1. Publish Color Image message
        img_msg = ros2_numpy.msgify(Image, img, encoding='bgr8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = frame_id
        self.publishers_topics[camera_name].publish(img_msg)
        
        # 2. Publish Rotated Color Image message (Head only)
        if camera_name != 'gripper' and self.publish_rotated:
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
        
        # 3. Publish CameraInfo message (Color)
        if camera_name not in self.camera_info or self.camera_info[camera_name] is None:
            raise RuntimeError(f"Camera calibration file for {camera_name} is missing or could not be loaded!")
        info_msg = self.camera_info[camera_name]
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = frame_id
        self.info_publishers[camera_name].publish(info_msg)
        
        # 4. Publish Depth Image message
        depth_img = rgbd_frame.depth_image
        if depth_img is not None and depth_img.size > 0:
            try:
                # Handle dynamic depth image formatting (uint16 in mm vs float32 in meters)
                if depth_img.dtype == np.uint16:
                    depth_msg = ros2_numpy.msgify(Image, depth_img, encoding='16UC1')
                else:
                    depth_msg = ros2_numpy.msgify(Image, depth_img, encoding='32FC1')
                depth_msg.header.stamp = stamp
                depth_msg.header.frame_id = frame_id
                self.depth_publishers[camera_name].publish(depth_msg)
            except Exception as ex:
                self.get_logger().error(f"Failed to serialize depth image for {camera_name}: {ex}")
        
        # 5. Publish Depth CameraInfo message
        self.depth_info_publishers[camera_name].publish(info_msg)
        
        # 6. Publish Point Cloud message (PointCloud2)
        points = rgbd_frame.pointcloud
        colors = rgbd_frame.pointcloud_colors
        if points is not None and points.size > 0:
            try:
                # colors has shape (N, 3) and is RGB.
                colors_uint8 = colors.astype(np.uint8)
                # Create pointcloud msg using create_pointcloud_rgb_msg
                cloud_msg = create_pointcloud_rgb_msg(colors_uint8, points)
                cloud_msg.header.stamp = stamp
                cloud_msg.header.frame_id = frame_id
                self.points_publishers[camera_name].publish(cloud_msg)
            except Exception as ex:
                self.get_logger().error(f"Failed to serialize pointcloud for {camera_name}: {ex}")

    def head_publish_loop(self):
        """Concurrently streams and publishes active head cameras using hardware synchronization"""
        self.get_logger().info("Starting Head RGBD Cameras Stream...")
        
        # Select the unified synced generators to ensure camera hardware alignment and avoid board/driver conflict
        if self.use_left and self.use_right and self.use_center:
            generator = stream_left_right_center_rgbd(is_rotate=False, use_ros_for_lidars=True)
        elif self.use_left and self.use_right:
            generator = stream_left_right_rgbd(is_rotate=False, use_ros_for_lidars=True)
        elif self.use_left:
            generator = stream_left_rgbd(is_rotate=False, use_ros_for_lidars=True)
        elif self.use_right:
            generator = stream_right_rgbd(is_rotate=False, use_ros_for_lidars=True)
        elif self.use_center:
            generator = stream_center_rgbd(is_rotate=False, use_ros_for_lidars=True)
        else:
            return

        for frame in generator:
            if not self.running or not rclpy.ok():
                break
            if frame is None:
                continue
                
            stamp = self.get_clock().now().to_msg()

            # Check if synced frame or single RGBDFrame
            if hasattr(frame, 'left') or hasattr(frame, 'right') or hasattr(frame, 'center'):
                # SyncedRGBDFrame
                if self.use_left and hasattr(frame, 'left') and frame.left is not None:
                    self.publish_rgbd_frame(frame.left, 'left', stamp)
                if self.use_right and hasattr(frame, 'right') and frame.right is not None:
                    self.publish_rgbd_frame(frame.right, 'right', stamp)
                if self.use_center and hasattr(frame, 'center') and frame.center is not None:
                    self.publish_rgbd_frame(frame.center, 'center', stamp)
            else:
                # Single RGBDFrame
                if self.use_left:
                    self.publish_rgbd_frame(frame, 'left', stamp)
                elif self.use_right:
                    self.publish_rgbd_frame(frame, 'right', stamp)
                elif self.use_center:
                    self.publish_rgbd_frame(frame, 'center', stamp)

    def gripper_publish_loop(self):
        """Gripper publishing loop running in separate thread"""
        self.get_logger().info("Starting Gripper RGBD Camera Stream...")
        generator = stream_gripper_rgbd(is_rotate=False)
        for frame in generator:
            if not self.running or not rclpy.ok():
                break
            if frame is None:
                continue
                
            stamp = self.get_clock().now().to_msg()
            self.publish_rgbd_frame(frame, 'gripper', stamp)

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'head_camera_thread') and self.head_camera_thread is not None and self.head_camera_thread.is_alive():
            self.head_camera_thread.join(timeout=1.0)
        if self.use_gripper and hasattr(self, 'gripper_publish_thread') and self.gripper_publish_thread is not None and self.gripper_publish_thread.is_alive():
            self.gripper_publish_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RGBDCameraNode()
    # Use MultiThreadedExecutor to concurrently schedule parallelized thread callbacks!
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
