#!/usr/bin/env python3

from dataclasses import dataclass
from enum import Enum
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import depthai as dai
import numpy as np
import yaml
import os
import threading
from rcl_interfaces.msg import ParameterDescriptor



class RGBCameras(Enum):
    luxonis_synced_left = 1
    luxonis_synced_right = 2
    luxonis_synced_center = 3
    @property
    def config(self):
        if self == RGBCameras.luxonis_synced_left:
            rotate_number_of_times = 1
            rotate_number_of_times=0
            return RGBCameraConfig("", image_size=(1200,1920), fps=30, camera_type=self, rotate_number_of_times=rotate_number_of_times, buffer_size=1, is_compressed=False)
        if self == RGBCameras.luxonis_synced_right:
            rotate_number_of_times=-1
            rotate_number_of_times=0
            return RGBCameraConfig("", image_size=(1200,1920), fps=30, camera_type=self, rotate_number_of_times=rotate_number_of_times, buffer_size=1, is_compressed=False)
        if self == RGBCameras.luxonis_synced_center:
            # image_size=(3040,4056) # Full 12MP resolution
            image_size = (3040,4032) # Full 12MP but divisible by 32
            # image_size = (2160,3840) # 4K resolution, the default crop from Luxonis
            rotate_number_of_times=-1
            rotate_number_of_times=0
            return RGBCameraConfig("", image_size=image_size, fps=5, camera_type=self, rotate_number_of_times=rotate_number_of_times, buffer_size=1, is_compressed=True)
        
        raise ValueError(f"{self} does not have a device configuration")


@dataclass
class RGBCameraConfig:
    camera_device:str
    image_size:tuple[int,int]
    fps:int
    camera_type:RGBCameras
    rotate_number_of_times:int = 0
    buffer_size:int = 1
    is_compressed:bool = True
    
    
def get_dai_camera(camera_type:RGBCameras):
    if camera_type == RGBCameras.luxonis_synced_left:
        return dai.CameraBoardSocket.CAM_C
    if camera_type == RGBCameras.luxonis_synced_right:
        return dai.CameraBoardSocket.CAM_B
    if camera_type == RGBCameras.luxonis_synced_center:
        return dai.CameraBoardSocket.CAM_A
    
    raise Exception(f"{camera_type} is not supported as a Luxonis device.")

def create_camera_node(pipeline: dai.Pipeline, camera_config:RGBCameraConfig):
    """
    Takes a dai.Pipeline reference and adds a camera node to it.
    """
    board_socket = get_dai_camera(camera_type=camera_config.camera_type)

    buffer_size = camera_config.buffer_size
    fps = camera_config.fps
    node = pipeline.create(dai.node.Camera)
    node.setNumFramesPools(isp=buffer_size, raw=buffer_size, imgmanip=buffer_size)
    node.setSensorType(dai.CameraSensorType.COLOR)
    node.build(boardSocket=board_socket, sensorFps=fps)

    camera_output = node.requestOutput(size=camera_config.image_size[::-1], fps=fps, type=dai.ImgFrame.Type.NV12, resizeMode=dai.ImgResizeMode.CROP, enableUndistortion=False)
    
    if camera_config.is_compressed:
        videoEncoder = pipeline.create(dai.node.VideoEncoder)
        videoEncoder.setDefaultProfilePreset(fps, dai.VideoEncoderProperties.Profile.MJPEG)
        # videoEncoder.setBitrateKbps(500) # 0.5 Mbps
        videoEncoder.setLossless(True) # Lossless only for MJPEG
        videoEncoder.setNumFramesPool(buffer_size)
        # videoEncoder.setQuality(90)
        videoEncoder.build(
            camera_output,
            frameRate=fps
        )

    return camera_output

def get_frame_from_output_queue(output_queue:dai.MessageQueue, rotate_number_of_times):
    while True:
        message: dai.ImgFrame|None = output_queue.get()
        if message:
            # time_stamp = time.monotonic()
            #https://docs.luxonis.com/hardware/platform/deploy/frame-sync/
            # time_stamp = message.getTimestamp().total_seconds() # Timestamp synced with the host computer clock
            sequence_number = message.getSequenceNum()
            color_image = message.getCvFrame()
            if rotate_number_of_times:
                color_image = np.rot90(color_image, k=rotate_number_of_times)

            # latencyMs = (dai.Clock.now() - message.getTimestamp()).total_seconds() * 1000
            # diffs = np.append(diffs, latencyMs)
            # print(f"Latency: {latencyMs} ms")
                
            # yield color_image, time_stamp
            yield color_image, sequence_number

        # print(f"Dropped frame {output_queue.getName()}")

@staticmethod
def create_pipeline():
    device = dai.Device(maxUsbSpeed=dai.UsbSpeed.HIGH)
    pipeline = dai.Pipeline(defaultDevice=device)
    
    print('DeviceID:',device.getDeviceInfo().getDeviceId())
    print('USB speed:',device.getUsbSpeed())
    print('Connected cameras:',device.getConnectedCameras())

    pipeline.setXLinkChunkSize(0)

    return pipeline

class LuxonisCameraNode(Node):
    def __init__(self):
        super().__init__('luxonis_camera_node')
        
        # Declare parameters
        self.declare_parameter('use_left', True)
        self.declare_parameter('use_right', True)
        self.declare_parameter('use_center', False)
        self.declare_parameter('camera_namespace', 'camera')
        self.declare_parameter('do_sync_frames', True)
        self.declare_parameter('sync_threshold', 4, descriptor=ParameterDescriptor(description="Number of frames to wait for before dropping a syncedframe"))
        self.declare_parameter('left_calibration_file', '')
        self.declare_parameter('right_calibration_file', '')
        self.declare_parameter('center_calibration_file', '')
        
        # Get parameters
        self.use_left = self.get_parameter('use_left').value
        self.use_right = self.get_parameter('use_right').value
        self.use_center = self.get_parameter('use_center').value
        self.camera_namespace = self.get_parameter('camera_namespace').value
        self.do_sync_frames:bool = self.get_parameter('do_sync_frames').value
        self.sync_threshold:int = self.get_parameter('sync_threshold').value

        if (not (not self.use_left and not self.use_right)) and (self.do_sync_frames and not (self.use_left or self.use_right)):
            raise Exception("Cannot sync frames without left and right cameras, you must enable both of them with the use_left and use_right params..")
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.publishers_topics = {}
        self.info_publishers = {}
        self.camera_info = {}

        if self.use_left:
            self.publishers_topics['left'] = self.create_publisher(
                Image, f'{self.camera_namespace}/left/image_raw', 10)
            self.info_publishers['left'] = self.create_publisher(
                CameraInfo, f'{self.camera_namespace}/left/camera_info', 10)
            calib_file = self.get_parameter('left_calibration_file').value
            if calib_file:
                self.camera_info['left'] = self.load_camera_info(calib_file)

        if self.use_right:
            self.publishers_topics['right'] = self.create_publisher(
                Image, f'{self.camera_namespace}/right/image_raw', 10)
            self.info_publishers['right'] = self.create_publisher(
                CameraInfo, f'{self.camera_namespace}/right/camera_info', 10)
            calib_file = self.get_parameter('right_calibration_file').value
            if calib_file:
                self.camera_info['right'] = self.load_camera_info(calib_file)

        if self.use_center:
            self.publishers_topics['center'] = self.create_publisher(
                Image, f'{self.camera_namespace}/center/image_raw', 10)
            self.info_publishers['center'] = self.create_publisher(
                CameraInfo, f'{self.camera_namespace}/center/camera_info', 10)
            calib_file = self.get_parameter('center_calibration_file').value
            if calib_file:
                self.camera_info['center'] = self.load_camera_info(calib_file)
        
        # Initialize pipeline
        self.device = None
        self.running = True
        
        self.create_pipeline()
        
        # Start publishing thread
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

            self.get_logger().info(f'Loaded calibration from {calib_file}')
            return info

        except Exception as e:
            self.get_logger().error(f'Failed to load calibration file {calib_file}: {e}')
            return None

    def create_pipeline(self):
        """Initialize DepthAI pipeline and device"""
        try:
            
            self.pipeline = create_pipeline()
            self.camera = self.pipeline

            self.left_output = None
            self.right_output = None
            self.center_output = None

            self.left = RGBCameras.luxonis_synced_left.config
            self.right = RGBCameras.luxonis_synced_right.config
            self.center = RGBCameras.luxonis_synced_center.config

            if self.use_left:
                node_left = create_camera_node(pipeline=self.pipeline, camera_config=self.left)
                self.left_output = node_left.createOutputQueue(maxSize=1)
            if self.use_right:
                node_right = create_camera_node(pipeline=self.pipeline, camera_config=self.right)
                self.right_output = node_right.createOutputQueue(maxSize=1)
            if self.use_center:
                node_center = create_camera_node(pipeline=self.pipeline, camera_config=self.center)
                self.center_output =  node_center.createOutputQueue(maxSize=1)


            self.pipeline.start()
            
            self.get_logger().info('Pipeline initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize pipeline: {e}')
            raise
    
    
    def publish_loop(self):
        """Main publishing loop running in separate thread"""
        while self.running and rclpy.ok():
            try:
                self.publish_synced_frames()
            except Exception as e:
                self.get_logger().error(f'Error in publish loop: {e}')
    
    def publish_synced_frames(self):
        """Publish synchronized frames from left/right cameras. The center camera is not synced because it uses a different FPS."""

        if self.center_output is not None:
            center_image, center_sequence_number = next(get_frame_from_output_queue(self.center_output, self.center.rotate_number_of_times))
            self.publish_frame_data(center_image, "center")

        if self.do_sync_frames and self.left_output is not None and self.right_output is not None:
            # Do sync
            left_image, left_sequence_number = next(get_frame_from_output_queue(self.left_output, self.left.rotate_number_of_times))
            right_image, right_sequence_number = next(get_frame_from_output_queue(self.right_output, self.right.rotate_number_of_times))

            if abs(left_sequence_number - right_sequence_number) <= self.sync_threshold:
                self.publish_frame_data(left_image, "left")
                self.publish_frame_data(right_image, "right")
            else:
                print("frames are not synced, skipping")
        else:
            # Publish without sync
            if self.left_output is not None:
                left_image, left_sequence_number = next(get_frame_from_output_queue(self.left_output, self.left.rotate_number_of_times))
                self.publish_frame_data(left_image, "left")
            if self.right_output is not None:
                right_image, right_sequence_number = next(get_frame_from_output_queue(self.right_output, self.right.rotate_number_of_times))
                self.publish_frame_data(right_image, "right")


    
    def publish_frame_data(self, frame, camera_name):
        """Convert and publish a single frame"""
        try:

            # Create ROS image message
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = f'{self.camera_namespace}/{camera_name}_optical_frame'

            # Publish image
            self.publishers_topics[camera_name].publish(img_msg)

            # Publish camera info if available
            if camera_name in self.camera_info and self.camera_info[camera_name] is not None:
                info_msg = self.camera_info[camera_name]
                info_msg.header = img_msg.header
                self.info_publishers[camera_name].publish(info_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing frame for {camera_name}: {e}')
    
    def destroy_node(self):
        """Clean up resources"""
        self.running = False
        if self.publish_thread.is_alive():
            self.publish_thread.join(timeout=1.0)
        if self.device:
            self.device.close()
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

