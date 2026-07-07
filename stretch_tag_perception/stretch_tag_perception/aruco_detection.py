#!/usr/bin/env python3
import functools
import os
from typing import List

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
import tf2_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf_transformations import quaternion_from_matrix
from visualization_msgs.msg import Marker, MarkerArray

HEAD_CAMERA_FRAME = "cameras_head_center_camera_optical_frame"
CAMERA_INFO_TOPIC = "/cameras_head/center/camera_info"
CENTER_CAMERA_TOPIC = "/cameras_head/center/image_raw"

logger = rclpy.logging.get_logger('aruco_detection')


class ArucoDetector:
    def __init__(self, aruco_detectors: List[aruco.ArucoDetector]):

        self.detectors = aruco_detectors

    def detect_markers(self, img: np.ndarray):

        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img

        corners = []
        aruco_ids = []
        for detector in self.detectors:
            logger.debug(f"Detecting markers with detector: {detector}")
            detected_corners, detected_ids, aruco_rejected_image_points = (
                detector.detectMarkers(gray_img)
            )
            if len(detected_corners) > 0:
                corners.extend(detected_corners)
                aruco_ids.extend(detected_ids.flatten())
        
        return corners, aruco_ids


class ArucoMarker:
    def __init__(
        self,
        aruco_id: int,
        label: str,
        length_mm: float,
        aruco_detector: aruco.ArucoDetector,
        show_debug_images=False,
    ):

        self.aruco_id = aruco_id
        self.label = label
        self.length_mm = length_mm
        self.aruco_detector = aruco_detector
        self.show_debug_images = show_debug_images

        self.frame_number = None
        self.timestamp = None
        self.ready = False
        self.used_depth_image = False
        self.broadcasted = False
        self.corners = None

        colormap = cv2.COLORMAP_HSV
        offset = 0
        i = (offset + (self.aruco_id * 29)) % 255
        image = np.uint8([[[i]]])
        id_color_image = cv2.applyColorMap(image, colormap)
        bgr = id_color_image[0, 0]
        self.id_color = [bgr[2], bgr[1], bgr[0]]


        self.frame_id = HEAD_CAMERA_FRAME

        self.marker_position = None
        self.marker_quaternion = None

        duration = Duration(seconds=0.2)
        self.marker = Marker()
        self.marker.type = self.marker.CUBE
        self.marker.action = self.marker.ADD
        self.marker.lifetime = duration.to_msg()
        self.marker.text = self.label

    @classmethod
    def from_dict(cls, d: dict, aruco_detection_parameters: aruco.DetectorParameters, show_debug_images: bool = False):

        try:
            aruco_dictionary = aruco.getPredefinedDictionary(
                getattr(aruco, d["tag_dictionary"])
            )
        except AttributeError:
            raise ValueError(f"Invalid ArUco dictionary: {d['tag_dictionary']}")

        aruco_id = int(d["id"])
        label = d["name"] if d["name"] else f"aruco_{aruco_id}"
        return cls(
            aruco_id=aruco_id,
            label=label,
            length_mm=d["length_mm"],
            aruco_detector=aruco.ArucoDetector(aruco_dictionary, aruco_detection_parameters),
            show_debug_images=show_debug_images,
        )

    def update(self, corners, timestamp, frame_number, camera_matrix, camera_dist_coeffs):
        self.ready = True
        self.corners = corners
        self.timestamp = timestamp
        self.frame_number = frame_number
        self.camera_matrix = camera_matrix
        self.camera_dist_coeffs = camera_dist_coeffs

        points_3d = np.array(
            [
                (-self.length_mm / 2, self.length_mm / 2, 0),
                (self.length_mm / 2, self.length_mm / 2, 0),
                (self.length_mm / 2, -self.length_mm / 2, 0),
                (-self.length_mm / 2, -self.length_mm / 2, 0),
            ]
        )

        rvecs = np.zeros((len(corners), 1, 3), dtype=np.float64)
        tvecs = np.zeros((len(corners), 1, 3), dtype=np.float64)
        
        for marker_num in range(len(corners)):
            solved, rvecs_ret, tvecs_ret = cv2.solvePnP(
                objectPoints=points_3d,
                imagePoints=corners[marker_num],
                cameraMatrix=self.camera_matrix,
                distCoeffs=self.camera_dist_coeffs,
            )
            if solved: 
                rvecs[marker_num][:] = np.transpose(rvecs_ret)
                tvecs[marker_num][:] = np.transpose(tvecs_ret)
        
        self.aruco_rotation = rvecs[0][0]

        # Convert ArUco position estimate to be in meters.        
        self.marker_position = tvecs[0][0]/1000.0
        
        T = np.identity(4)
        T[:3,3] = self.marker_position
        T[:3, :3] = cv2.Rodrigues(self.aruco_rotation)[0]
        self.marker_quaternion = quaternion_from_matrix(T)

        self.broadcasted = False
        self.ready = True

    def broadcast_tf(self, tf_broadcaster, force_redundant=False):
        # Create TF frame for the marker. By default, only broadcast a
        # single time after an update.
        if (not self.broadcasted) or force_redundant:
            if self.marker_position is not None and self.marker_quaternion is not None: 
                transform_stamped = TransformStamped()
                transform_stamped.header.stamp = self.timestamp
                transform_stamped.header.frame_id = self.frame_id
                transform_stamped.child_frame_id = self.label
                transform_stamped.transform.translation.x = self.marker_position[0]
                transform_stamped.transform.translation.y = self.marker_position[1]
                transform_stamped.transform.translation.z = self.marker_position[2]
                transform_stamped.transform.rotation.x = self.marker_quaternion[0]
                transform_stamped.transform.rotation.y = self.marker_quaternion[1]
                transform_stamped.transform.rotation.z = self.marker_quaternion[2]
                transform_stamped.transform.rotation.w = self.marker_quaternion[3]
                
                tf_broadcaster.sendTransform(transform_stamped)
                self.broadcasted = True

    def get_ros_marker(self):
        if not self.ready:
            return None

        self.marker.header.frame_id = self.frame_id
        self.marker.header.stamp = self.timestamp
        self.marker.id = self.aruco_id

        # scale of 1,1,1 would result in a 1m x 1m x 1m cube
        self.marker.scale.x = self.length_mm / 1000.0
        self.marker.scale.y = self.length_mm / 1000.0
        self.marker.scale.z = 0.005  # half a centimeter tall

        # make as bright as possible
        den = float(np.max(self.id_color))
        self.marker.color.r = self.id_color[2] / den
        self.marker.color.g = self.id_color[1] / den
        self.marker.color.b = self.id_color[0] / den
        self.marker.color.a = 0.33

        self.marker.pose.position.x = self.marker_position[0]
        self.marker.pose.position.y = self.marker_position[1]
        self.marker.pose.position.z = self.marker_position[2]

        q = self.marker_quaternion
        self.marker.pose.orientation.x = q[0]
        self.marker.pose.orientation.y = q[1]
        self.marker.pose.orientation.z = q[2]
        self.marker.pose.orientation.w = q[3]

        return self.marker


class ArucoMarkerCollection:
    def __init__(self, collection: List[ArucoMarker], aruco_detector: ArucoDetector, show_debug_images=False):

        self.collection: List[ArucoMarker] = collection
        self.show_debug_images = show_debug_images

        self.aruco_detection_parameters = aruco.DetectorParameters()
        # Apparently available in OpenCV 3.4.1, but not OpenCV 3.2.0.
        self.aruco_detection_parameters.cornerRefinementMethod = (
            aruco.CORNER_REFINE_SUBPIX
        )
        self.aruco_detection_parameters.cornerRefinementWinSize = 2

        self.aruco_detector = aruco_detector

        self.frame_number = 0

    @classmethod
    def from_dict(cls, marker_info: dict, show_debug_images=False):

        cls.aruco_detection_parameters = aruco.DetectorParameters()
        # Apparently available in OpenCV 3.4.1, but not OpenCV 3.2.0.
        cls.aruco_detection_parameters.cornerRefinementMethod = (
            aruco.CORNER_REFINE_SUBPIX
        )
        cls.aruco_detection_parameters.cornerRefinementWinSize = 2

        marker_collection = []
        aruco_dictionary_ids = []
        for marker_id, marker_info_dict in marker_info.items():
            aruco_dictionary_ids.append(marker_info_dict["tag_dictionary"])

            marker_info_dict["id"] = int(marker_id)
            marker = ArucoMarker.from_dict(marker_info_dict, cls.aruco_detection_parameters, show_debug_images)
            marker_collection.append(marker)
    
        # Consolidate dictionaries: if multiple sizes of the same grid are used, only use the largest.
        max_dicts = {}
        for dict_id in set(aruco_dictionary_ids):
            parts = dict_id.rsplit('_', 1)
            if len(parts) == 2 and parts[1].isdigit():
                base = parts[0]
                size = int(parts[1])
                if base not in max_dicts or size > max_dicts[base][1]:
                    max_dicts[base] = (dict_id, size)
            else:
                max_dicts[dict_id] = (dict_id, 0)

        consolidated_dictionary_ids = [val[0] for val in max_dicts.values()]
        logger.info(f"Consolidated ArUco dictionaries from {set(aruco_dictionary_ids)} to {consolidated_dictionary_ids}")

        aruco_detector = ArucoDetector([aruco.ArucoDetector(aruco.getPredefinedDictionary(getattr(aruco, dict_id))) for dict_id in consolidated_dictionary_ids])

        return cls(collection=marker_collection,
                   aruco_detector=aruco_detector,
                   show_debug_images=show_debug_images)

    def __iter__(self):
        # iterates through currently visible ArUco markers
        for marker in self.collection:
            if marker.frame_number == self.frame_number:
                yield marker

    @functools.cached_property
    def known_aruco_ids(self):
        return set([marker.aruco_id for marker in self.collection])

    def get_marker_from_id(self, id):
        if id not in self.known_aruco_ids:
            return None

        for marker in self.collection:
            if marker.aruco_id == id:
                return marker

    def broadcast_tf(self, tf_broadcaster):
        # Create TF frames for each of the markers. Only broadcast each
        # marker a single time after it has been updated.
        for marker in self.collection:
            marker.broadcast_tf(tf_broadcaster)

    def update(self, rgb_image, camera_matrix, camera_dist_coeffs, timestamp=None):

        self.frame_number += 1
        self.timestamp = timestamp
        self.rgb_image = rgb_image

        self.aruco_corners, self.aruco_ids = self.aruco_detector.detect_markers(self.rgb_image)
        if self.aruco_ids is None or len(self.aruco_ids) == 0:
            num_detected = 0
        else:
            num_detected = len(self.aruco_ids)

        if num_detected > 0:
            for corners, aruco_id in zip(self.aruco_corners, self.aruco_ids):
                
                marker = self.get_marker_from_id(aruco_id)

                if marker is not None: 
                    marker.update(
                        corners,
                        self.timestamp,
                        self.frame_number,
                        camera_matrix,
                        camera_dist_coeffs,
                    )

    def get_ros_marker_array(self):
        marker_array = MarkerArray()
        for marker in self.collection:
            if marker.frame_number == self.frame_number:
                ros_marker = marker.get_ros_marker()
                marker_array.markers.append(ros_marker)
        return marker_array

    def draw_markers(self, img: np.ndarray): 

        return aruco.drawDetectedMarkers(img, self.aruco_corners, np.array(self.aruco_ids))


class DetectArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_detection_node',
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)
        
        node_name = self.get_name()
        logger.info("{0} started".format(node_name))

        self.cv_bridge = CvBridge()
        self.rgb_image = None
        self.rgb_image_timestamp = None
        self.depth_image = None
        self.depth_image_timestamp = None        
        self.camera_matrix = None
        self.camera_dist_coeffs = None
        self.all_points = []
        self.show_debug_images = False
        self.publish_marker_point_clouds = False

        self.marker_info = {}
        self.load_config_to_paramter_server()

        # Gather the camera info. Assuming it does not change, we only need to wait for the first message, then we can remove the subscription.
        self.center_cam_info_sub = self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, qos_profile=1)
        while self.camera_matrix is None and self.camera_dist_coeffs is None:
            logger.info(f"Waiting for camera info on topic: {CAMERA_INFO_TOPIC}")
            rclpy.spin_once(self)

        self.destroy_subscription(self.center_cam_info_sub)

        logger.info(f"Camera matrix recieved:\n{self.camera_matrix}")
        logger.info(f"Camera distortion coefficients receieved:\n{self.camera_dist_coeffs}")

        # Initialize the aruco marker collection from the gathered marker info
        self.aruco_marker_collection = ArucoMarkerCollection.from_dict(self.marker_info, self.show_debug_images)

        self.center_rgb_sub = self.create_subscription(
            Image, CENTER_CAMERA_TOPIC, self.img_callback, qos_profile=1
        )

        self.visualize_markers_pub = self.create_publisher(MarkerArray, '/aruco/marker_array', 1)

        self.tf_broadcaster = TransformBroadcaster(self)

    def load_config_to_paramter_server(self):

        try:
            pkg_share = get_package_share_directory('stretch_tag_perception')
            stretch_yaml_path = os.path.join(pkg_share, 'config', 'stretch_marker_dict.yaml')
            user_yaml_path = os.path.join(pkg_share, 'config', 'user_aruco_dictionary.yaml')

            for yaml_path in [stretch_yaml_path, user_yaml_path]:
                if os.path.exists(yaml_path):
                    with open(yaml_path, 'r') as f:
                        yaml_data = yaml.safe_load(f)
                        if yaml_data and '/**' in yaml_data and 'ros__parameters' in yaml_data['/**']:
                            params = yaml_data['/**']['ros__parameters'].get('aruco_marker_info', {})
                            self.marker_info.update(params)
                            
            parameters_to_set = []
            for marker_id, info in self.marker_info.items():
                for key, value in info.items():
                    param_name = f'aruco_marker_info.{marker_id}.{key}'
                    if value is None:
                        value = "None"
                    parameters_to_set.append(Parameter(param_name, value=value))
                    if not self.has_parameter(param_name):
                        self.declare_parameter(param_name, value)
            self.set_parameters(parameters_to_set)
        except Exception as e:
            logger.error(f"Failed to load yaml parameters: {e}")

        # Fallback to read any other params passed
        param_dict = self.get_parameters_by_prefix('aruco_marker_info')
        for key in param_dict:
            marker_id, param_key = key.split('.', 1)
            if marker_id not in self.marker_info:
                self.marker_info[marker_id] = {}
            self.marker_info[marker_id][param_key] = self.get_parameter_or(f'aruco_marker_info.{key}').value

        logger.info(f"{param_dict=}")
        logger.info(f"{self.marker_info}")

    def img_callback(self, img_msg):
        try:
            self.rgb_image = self.cv_bridge.imgmsg_to_cv2(img_msg, 'bgr8')
            self.rgb_image_timestamp = img_msg.header.stamp
        except CvBridgeError as error:
            logger.error(error)
        
        self.aruco_marker_collection.update(self.rgb_image, self.camera_matrix, self.camera_dist_coeffs, self.rgb_image_timestamp)

        marker_array = self.aruco_marker_collection.get_ros_marker_array()

        # Create TF frames for each of the markers. Only broadcast
        # each marker a single time after it has been updated.
        self.aruco_marker_collection.broadcast_tf(self.tf_broadcaster)
        self.visualize_markers_pub.publish(marker_array)

        # save rotation for last
        if self.show_debug_images:
            aruco_image = self.aruco_marker_collection.draw_markers(self.rgb_image)
            display_aruco_image = cv2.rotate(aruco_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            cv2.imshow('Detected ArUco Markers', display_aruco_image)
            cv2.waitKey(1) #ms

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.camera_dist_coeffs = np.array(msg.d)


def main(args=None):
    rclpy.init(args=args)
    node = DetectArucoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass    
    finally:
        cv2.destroyAllWindows()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()