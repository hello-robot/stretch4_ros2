import threading
import types
import queue
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, Imu
import ros2_numpy
import numpy as np
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from .image_bridge import ImageFrame
from .pointcloud_bridge import PointCloudFrame, LidarPointCloudFrame, StereoPointCloudFrame
from .transforms_bridge import TransformsFrame, transform_to_list, to_matrix
from .camera_info_bridge import CameraInfoFrame
from .imu_bridge import ImuFrame

class StreamManager:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
            
        self.node = rclpy.create_node('stream_manager_node')
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        
        # Unified queue for all arriving messages
        # We don't cap the size strictly at 1 here since multiple topics are feeding it,
        # but we do want to avoid stale backlog. We will drain it on fetch.
        self.master_queue = queue.Queue()
        
        # State tracking
        self.latest_frames = {}
        self.tf_streams = {} # key -> (target_frame, base_frame)
        self.tf_last_stamps = {}
        
        # TF Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        
        self._running = True
        
        # Threads
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        
        self.tf_poll_thread = threading.Thread(target=self._tf_polling_loop, daemon=True)
        self.tf_poll_thread.start()

    def add_image_topic(self, topic_name: str):
        self.latest_frames[topic_name] = None
        self.node.create_subscription(
            Image,
            topic_name,
            lambda msg, topic=topic_name: self._handle_image(msg, topic),
            10
        )

    def add_pointcloud_topic(self, topic_name: str):
        self.latest_frames[topic_name] = None
        self.node.create_subscription(
            PointCloud2,
            topic_name,
            lambda msg, topic=topic_name: self._handle_pc(msg, topic),
            10
        )

    def add_camera_info_topic(self, topic_name: str):
        self.latest_frames[topic_name] = None
        self.node.create_subscription(
            CameraInfo,
            topic_name,
            lambda msg, topic=topic_name: self._handle_camera_info(msg, topic),
            10
        )

    def add_imu_topic(self, topic_name: str):
        self.latest_frames[topic_name] = None
        self.node.create_subscription(
            Imu,
            topic_name,
            lambda msg, topic=topic_name: self._handle_imu(msg, topic),
            10
        )

    def add_tf_stream(self, target_frame: str, base_frame: str = "base_link"):
        key = target_frame
        self.latest_frames[key] = None
        self.tf_streams[key] = (target_frame, base_frame)
        self.tf_last_stamps[key] = 0.0

    def _handle_image(self, msg, topic):
        try:
            np_image = ros2_numpy.numpify(msg).copy()
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            frame = ImageFrame(image=np_image, timestamp=stamp)
            self.master_queue.put((topic, frame))
        except Exception as e:
            self.node.get_logger().error(f'Image Convert error: {e}')

    def _handle_pc(self, msg, topic):
        try:
            pc_array = ros2_numpy.numpify(msg)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.master_queue.put((topic, PointCloudFrame.from_pointcloud2(pc_array, timestamp)))
        except Exception as e:
            self.node.get_logger().error(f'PC Convert error: {e}')

    def _handle_camera_info(self, msg, topic):
        try:
            k_matrix = np.array(msg.k).reshape((3, 3))
            d_coeffs = np.array(msg.d)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            frame = CameraInfoFrame(
                camera_matrix=k_matrix,
                distortion_coefficients=d_coeffs,
                distortion_model=msg.distortion_model,
                timestamp=stamp
            )
            self.master_queue.put((topic, frame))
        except Exception as e:
            self.node.get_logger().error(f'CameraInfo Convert error: {e}')

    def _handle_imu(self, msg, topic):
        try:
            orientation = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            linear_acceleration = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            frame = ImuFrame(
                orientation=orientation,
                angular_velocity=angular_velocity,
                linear_acceleration=linear_acceleration,
                timestamp=stamp
            )
            self.master_queue.put((topic, frame))
        except Exception as e:
            self.node.get_logger().error(f'Imu Convert error: {e}')

    def create_topic_generator(self, topic: str):
        while rclpy.ok() and self._running:
            yield self.latest_frames.get(topic)

    def get(self, stream, block: bool = True, timeout: float | None = 10.0):
        start_time = time.time()
        is_gen = isinstance(stream, types.GeneratorType)
        
        def check_val():
            if is_gen:
                return next(stream)
            return self.latest_frames.get(stream)
            
        val = check_val()
        if val is not None:
            return val
            
        if not block:
            return None
            
        while rclpy.ok() and self._running:
            time_left = None
            if timeout is not None:
                elapsed = time.time() - start_time
                time_left = timeout - elapsed
                if time_left <= 0:
                    return None
            
            try:
                fetch_timeout = min(0.1, time_left) if time_left is not None else 0.1
                topic, frame = self.master_queue.get(block=True, timeout=fetch_timeout)
                self.latest_frames[topic] = frame
                
                val = check_val()
                if val is not None:
                    self._drain_queue()
                    return val
            except queue.Empty:
                pass
                
        return None

    def _drain_queue(self):
        while not self.master_queue.empty():
            try:
                topic, frame = self.master_queue.get_nowait()
                self.latest_frames[topic] = frame
            except queue.Empty:
                break

    def _tf_polling_loop(self):
        # We manually sleep to poll TF buffer at roughly 100Hz
        while rclpy.ok() and self._running:
            for key, (target, base) in self.tf_streams.items():
                try:
                    stamped_transform = self.tf_buffer.lookup_transform(
                        base, target, rclpy.time.Time(), rclpy.duration.Duration(seconds=0)
                    )
                    stamp = stamped_transform.header.stamp.sec + stamped_transform.header.stamp.nanosec * 1e-9
                    if stamp > self.tf_last_stamps[key]:
                        self.tf_last_stamps[key] = stamp
                        trans, rot = transform_to_list(stamped_transform)
                        pose_mat = to_matrix(trans, rot)
                        
                        frame = TransformsFrame(transform=pose_mat, timestamp=stamp)
                        self.master_queue.put((key, frame))
                except (LookupException, ConnectivityException, ExtrapolationException):
                    pass
            time.sleep(0.01)

    def stream(self, block: bool = True, timeout: float | None = None):
        """
        Unified generator that yields a dictionary containing the latest frames for all registered streams.
        Keys are topic strings (or target_frame strings for TFs).
        Values are the corresponding Frame dataclass.
        
        If block is True, this will wait until AT LEAST ONE new frame arrives across the registered streams.
        """
        try:
            while rclpy.ok() and self._running:
                
                arrived_any = False
                
                # If we are blocking, we wait for at least one message to arrive in the queue
                if block:
                    try:
                        topic, frame = self.master_queue.get(block=True, timeout=timeout)
                        self.latest_frames[topic] = frame
                        arrived_any = True
                    except queue.Empty:
                        yield None
                        continue
                
                # Drain the rest of the queue to ensure we have the absolute latest for all topics
                while not self.master_queue.empty():
                    try:
                        topic, frame = self.master_queue.get_nowait()
                        self.latest_frames[topic] = frame
                        arrived_any = True
                    except queue.Empty:
                        break
                        
                if not block and not arrived_any:
                    yield None
                    continue
                    
                # Yield a shallow copy of the state dictionary so the user gets a stable snapshot
                yield self.latest_frames.copy()
                
        except GeneratorExit:
            pass

    def close(self):
        self._running = False
        self.node.destroy_node()
        self.executor.shutdown()
        self.executor_thread.join()
        self.tf_poll_thread.join()
