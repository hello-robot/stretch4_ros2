#!/usr/bin/env python3
import argparse
import copy
import functools
import os
import sys
from typing import List

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros.transform_broadcaster import TransformBroadcaster
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray


from stretch4_body.subsystem.cameras.cv_utils import solve_pnp

from stretch4_body.subsystem.cameras.enums.distortion_models import DistortionModels

from scipy.spatial.transform import Rotation

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
        show_debug_images:bool,
        frame_id: str,
        camera_name: str,
    ):

        self.aruco_id = aruco_id
        self.label = label
        self.length_mm = length_mm
        self.aruco_detector = aruco_detector
        self.show_debug_images = show_debug_images
        self.frame_id = frame_id
        self.camera_name = camera_name

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

        self.marker_position = None
        self.marker_quaternion = None

        duration = Duration(seconds=0.2)
        self.marker = Marker()
        self.marker.type = self.marker.CUBE
        self.marker.action = self.marker.ADD
        self.marker.lifetime = duration.to_msg()
        self.marker.text = f"{self.camera_name}_{self.label}"

    @classmethod
    def from_dict(cls, d: dict, aruco_detection_parameters: aruco.DetectorParameters, show_debug_images: bool, frame_id: str, camera_name: str):

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
            frame_id=frame_id,
            camera_name=camera_name,
        )

    @property
    def stamp(self):
        return self.timestamp.to_msg() if hasattr(self.timestamp, 'to_msg') else self.timestamp

    def update(self, corners, timestamp, frame_number, camera_matrix, camera_dist_coeffs, distortion_model):
        self.ready = True
        self.corners = corners
        self.timestamp = timestamp
        self.frame_number = frame_number
        self.camera_matrix = camera_matrix
        self.camera_dist_coeffs = camera_dist_coeffs
        self.distortion_model = distortion_model

        length = self.length_mm / 1000

        points_3d = np.array(
            [
                (-length / 2,length / 2, 0),
                (length / 2,length / 2, 0),
                (length / 2, -length / 2, 0),
                (-length / 2, -length / 2, 0),
            ],
            dtype=np.float32,
        )

        success, rvecs, tvecs = solve_pnp(
            object_points=points_3d,
            image_points=corners,
            camera_matrix=self.camera_matrix,
            distortion_coefficients=self.camera_dist_coeffs,
            distortion_model=DistortionModels[distortion_model],
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if not success:
            return None

        R, _ = cv2.Rodrigues(rvecs)

        T_camera_marker = np.eye(4)
        T_camera_marker[:3, :3] = R
        T_camera_marker[:3, 3] = tvecs.flatten()

        self.aruco_rotation = rvecs
     
        self.marker_position = tvecs.flatten()
        self.marker_quaternion = Rotation.from_rotvec(rvecs.flatten()).as_quat()

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
                transform_stamped.child_frame_id = f"aruco_perception_{self.label}_{self.camera_name}"
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
        self.marker.ns = f"aruco_{self.camera_name}"

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

        self.marker.text = f"{self.camera_name}_{self.label}"

        return self.marker

    def to_ros_msg(self) -> Detection3D:
        if not self.ready:
            return None

        detection = Detection3D()
        detection.header.frame_id = self.frame_id
        detection.header.stamp = self.stamp

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = str(self.aruco_id)
        hypothesis.hypothesis.score = 1.0  # Confidence score for ArUco is binary

        hypothesis.pose.pose.position.x = self.marker_position[0]
        hypothesis.pose.pose.position.y = self.marker_position[1]
        hypothesis.pose.pose.position.z = self.marker_position[2]

        q = self.marker_quaternion
        hypothesis.pose.pose.orientation.x = q[0]
        hypothesis.pose.pose.orientation.y = q[1]
        hypothesis.pose.pose.orientation.z = q[2]
        hypothesis.pose.pose.orientation.w = q[3]

        detection.results.append(hypothesis)

        # Bounding box matches marker size
        detection.bbox.size.x = self.length_mm / 1000.0
        detection.bbox.size.y = self.length_mm / 1000.0
        detection.bbox.size.z = 0.005
        detection.bbox.center = hypothesis.pose.pose

        return detection


class ArucoMarkerCollection:
    def __init__(self, collection: List[ArucoMarker], aruco_detector: ArucoDetector, show_debug_images:bool, frame_id: str, camera_name: str):

        self.collection: List[ArucoMarker] = collection
        self.show_debug_images = show_debug_images
        self._frame_id = frame_id
        self.camera_name = camera_name
        self.temp_markers: List[ArucoMarker] = []

        self.aruco_detection_parameters = aruco.DetectorParameters()
        # Apparently available in OpenCV 3.4.1, but not OpenCV 3.2.0.
        self.aruco_detection_parameters.cornerRefinementMethod = (
            aruco.CORNER_REFINE_SUBPIX
        )
        self.aruco_detection_parameters.cornerRefinementWinSize = 2

        self.aruco_detector = aruco_detector

        self.frame_number = 0

    @property
    def frame_id(self):
        return self._frame_id

    @frame_id.setter
    def frame_id(self, val):
        self._frame_id = val
        for marker in self.collection:
            marker.frame_id = val

    @classmethod
    def from_dict(cls, marker_info: dict, show_debug_images:bool, frame_id: str, camera_name: str):

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
            marker = ArucoMarker.from_dict(marker_info_dict, cls.aruco_detection_parameters, show_debug_images, frame_id=frame_id, camera_name=camera_name)
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
                   show_debug_images=show_debug_images,
                   frame_id=frame_id,
                   camera_name=camera_name)

    @property
    def stamp(self):
        if self.timestamp:
            return self.timestamp.to_msg() if hasattr(self.timestamp, 'to_msg') else self.timestamp
        return None

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
        for marker in self.temp_markers:
            marker.broadcast_tf(tf_broadcaster, force_redundant=True)

    def update(self, rgb_image, camera_matrix, camera_dist_coeffs,distortion_model, timestamp=None):

        self.frame_number += 1
        self.timestamp = timestamp
        self.rgb_image = rgb_image
        self.temp_markers = []

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
                        distortion_model
                    )
                else:
                    # Unknown marker - create a temporary one on the fly and update it
                    # Default size to 40.0 mm
                    temp_marker = ArucoMarker(
                        aruco_id=aruco_id,
                        label=f"tag_{aruco_id}",
                        length_mm=40.0,
                        aruco_detector=None,
                        show_debug_images=self.show_debug_images,
                        frame_id=self.frame_id,
                        camera_name=self.camera_name,
                    )
                    temp_marker.update(
                        corners,
                        self.timestamp,
                        self.frame_number,
                        camera_matrix,
                        camera_dist_coeffs,
                        distortion_model
                    )
                    self.temp_markers.append(temp_marker)

    def get_ros_marker_array(self):
        marker_array = MarkerArray()
        for marker in self.collection:
            if marker.frame_number == self.frame_number:
                ros_marker = marker.get_ros_marker()
                marker_array.markers.append(ros_marker)
        for marker in self.temp_markers:
            ros_marker = marker.get_ros_marker()
            if ros_marker:
                marker_array.markers.append(ros_marker)
        return marker_array

    def to_ros_msg(self) -> Detection3DArray:
        detection_array = Detection3DArray()
        detection_array.header.stamp = self.stamp
        detection_array.header.frame_id = self.frame_id
        
        for marker in self.collection:
            if marker.frame_number == self.frame_number:
                ros_detection = marker.to_ros_msg()
                if ros_detection:
                    detection_array.detections.append(ros_detection)
        for marker in self.temp_markers:
            ros_detection = marker.to_ros_msg()
            if ros_detection:
                detection_array.detections.append(ros_detection)
        return detection_array

    def draw_markers(self, img: np.ndarray): 

        return aruco.drawDetectedMarkers(img, self.aruco_corners, np.array(self.aruco_ids))


class DetectArucoNode(Node):
    def __init__(self, cameras='center'):
        super().__init__('aruco_detection_node',
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)
        
        node_name = self.get_name()
        logger.info("{0} started".format(node_name))

        if not self.has_parameter('cameras'):
            self.declare_parameter('cameras', cameras)
        param_val = self.get_parameter('cameras').value
        
        if isinstance(param_val, str):
            val_lower = param_val.strip().lower()
            if val_lower == 'all':
                self.cameras = ['left', 'right', 'center']
            else:
                self.cameras = [c.strip() for c in val_lower.split(',') if c.strip()]
        elif isinstance(param_val, list):
            self.cameras = [str(c).strip().lower() for c in param_val]
        else:
            self.cameras = [cameras]

        valid_cameras = ['left', 'right', 'center']
        self.cameras = [c for c in self.cameras if c in valid_cameras]
        if not self.cameras:
            logger.warn("No valid cameras specified. Defaulting to 'center'.")
            self.cameras = ['center']

        logger.info(f"Using cameras: {self.cameras}")

        self.cv_bridge = CvBridge()
        self.all_points = []
        self.show_debug_images = True
        self.publish_marker_point_clouds = False
        
        self.latest_images = {}

        self.marker_info = {}
        self.load_config_to_paramter_server()

        # Gather the camera info for each enabled camera.
        self.camera_infos = {}
        self.info_subs = []
        for cam in self.cameras:
            topic = f"/cameras_head/{cam}/camera_info"
            sub = self.create_subscription(
                CameraInfo,
                topic,
                functools.partial(self.camera_info_callback, camera_name=cam),
                qos_profile=1
            )
            self.info_subs.append(sub)

        # Wait until we have info for all requested cameras
        while len(self.camera_infos) < len(self.cameras):
            missing = [c for c in self.cameras if c not in self.camera_infos]
            logger.info(f"Waiting for camera info for: {missing}")
            rclpy.spin_once(self)

        for sub in self.info_subs:
            self.destroy_subscription(sub)

        for cam in self.cameras:
            matrix, coeffs, _, distortion_model = self.camera_infos[cam]
            logger.info(f"[{cam}] Camera matrix received:\n{matrix}")
            logger.info(f"[{cam}] Camera distortion coefficients received:\n{coeffs} for {distortion_model=}")

        # Initialize the aruco marker collections for each camera
        self.aruco_marker_collections = {}
        for cam in self.cameras:
            if cam in self.camera_infos:
                frame_id = self.camera_infos[cam][2]
            else:
                if cam == "left":
                    frame_id = "camera_left_optical_link"
                elif cam == "right":
                    frame_id = "camera_right_optical_link"
                else:
                    frame_id = "camera_center_optical_link"

            self.aruco_marker_collections[cam] = ArucoMarkerCollection.from_dict(
                self.marker_info,
                self.show_debug_images,
                frame_id=frame_id,
                camera_name=cam
            )

        self.rgb_subs = []
        for cam in self.cameras:
            topic = f"/cameras_head/{cam}/image_raw"
            sub = self.create_subscription(
                Image,
                topic,
                functools.partial(self.img_callback, camera_name=cam),
                qos_profile=qos_profile_sensor_data
            )
            self.rgb_subs.append(sub)

        if not self.has_parameter('publish_markers'):
            self.declare_parameter('publish_markers', False)
        self.publish_markers = self.get_parameter('publish_markers').value
        self.publish_markers = self.publish_markers == "true" or self.publish_markers == True

        self.visualize_markers_pub = self.create_publisher(MarkerArray, '/aruco/marker_array', 1)
        self.visualize_detections_pub = self.create_publisher(Detection3DArray, '/aruco/detections', 1)

        self.tf_broadcaster = TransformBroadcaster(self)

    def load_config_to_paramter_server(self):

        try:
            pkg_share = get_package_share_directory('stretch_tag_perception')
            stretch_yaml_path = os.path.join(pkg_share, 'config', 'stretch_marker_dict.yaml')
            user_yaml_path = os.path.join(pkg_share, 'config', 'user_aruco_dict.yaml')

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

    def img_callback(self, img_msg, camera_name):
        try:
            rgb_image = self.cv_bridge.imgmsg_to_cv2(img_msg, 'bgr8')
            rgb_image_timestamp =  img_msg.header.stamp
        except CvBridgeError as error:
            logger.error(error)
            return

        logger.info(f"{self.camera_infos[camera_name]=}")
        camera_matrix, camera_dist_coeffs, frame_id,distortion_model = self.camera_infos[camera_name]
        collection = self.aruco_marker_collections[camera_name]

        collection.update(rgb_image, camera_matrix, camera_dist_coeffs, distortion_model, rgb_image_timestamp)

        detection_array = collection.to_ros_msg()
        self.visualize_detections_pub.publish(detection_array)

        # Create TF frames for each of the markers. Only broadcast
        # each marker a single time after it has been updated.
        collection.broadcast_tf(self.tf_broadcaster)
        
        if self.publish_markers:
            marker_array = collection.get_ros_marker_array()
            self.visualize_markers_pub.publish(marker_array)

        # save rotation for last
        if self.show_debug_images:
            aruco_image = collection.draw_markers(rgb_image)
            # Overlay camera label on top-left
            cv2.putText(aruco_image, camera_name.upper(), (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            self.latest_images[camera_name] = aruco_image
            self.display_combined_images()

    def display_combined_images(self):
        ordered_cams = [c for c in ['left', 'center', 'right'] if c in self.cameras]
        
        # Get dimensions from the first available image to format placeholders
        target_height = None
        for cam in ordered_cams:
            if cam in self.latest_images:
                target_height = self.latest_images[cam].shape[0]
                break
                
        if target_height is None:
            return

        images_to_show = []
        for cam in ordered_cams:
            if cam in self.latest_images:
                img = self.latest_images[cam]
                h, w = img.shape[:2]
                if h != target_height:
                    scale = target_height / h
                    new_w = int(w * scale)
                    img = cv2.resize(img, (new_w, target_height))
                images_to_show.append(img)
            else:
                placeholder_width = int(target_height * 1.33)
                placeholder = np.zeros((target_height, placeholder_width, 3), dtype=np.uint8)
                cv2.putText(placeholder, f"Waiting for {cam}...", (placeholder_width // 6, target_height // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                images_to_show.append(placeholder)

        # Concatenate horizontally
        combined_image = np.hstack(images_to_show)
        
        # Resize if width exceeds 1920 pixels
        max_display_width = 1920
        h, w = combined_image.shape[:2]
        if w > max_display_width:
            scale = max_display_width / w
            new_h = int(h * scale)
            combined_image = cv2.resize(combined_image, (max_display_width, new_h))

        cv2.namedWindow('Detected ArUco Markers - Combined', cv2.WINDOW_NORMAL)
        cv2.imshow('Detected ArUco Markers - Combined', combined_image)
        cv2.waitKey(1)

    def camera_info_callback(self, msg, camera_name):
        matrix = np.array(msg.k).reshape((3, 3))
        dist_coeffs = np.array(msg.d)

        self.camera_infos[camera_name] = (matrix, dist_coeffs, msg.header.frame_id, msg.distortion_model)

        if hasattr(self, 'aruco_marker_collections') and camera_name in self.aruco_marker_collections:
            self.aruco_marker_collections[camera_name].frame_id = msg.header.frame_id


def main(args=None):
    parser = argparse.ArgumentParser(description='ArUco marker detection node.')
    parser.add_argument('--cameras', type=str, default='center',
                        help='Camera(s) to use for detection (comma-separated list of: left, right, center, or "all").')
    
    # We should parse only the arguments we know, because ROS 2 passes its own arguments.
    parsed_args, ros_args = parser.parse_known_args()

    # Reconstruct argv for ROS 2 init (keeping the program name)
    new_argv = [sys.argv[0]] + ros_args

    rclpy.init(args=new_argv)
    node = DetectArucoNode(cameras=parsed_args.cameras)

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