#! /usr/bin/env python3
import array
import copy
from functools import cache
import cv2
import numpy as np
import threading

from sensor_msgs.msg._compressed_image import CompressedImage
from stretch4_mujoco import Stretch4MujocoSimulator
from stretch4_mujoco.enums.actuators import Actuators
from stretch4_mujoco.enums.stretch_sensors import StretchSensors
from stretch4_mujoco.enums.stretch_cameras import CameraSettings, StretchCameras
from stretch4_mujoco.utils import get_absolute_path_stretch_xml, models_path
from stretch4_mujoco.pointcloud_utils import depth_to_points

from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSDurabilityPolicy
# from stretch_core.rwlock import RWLock
from stretch_mujoco_driver.joint_trajectory_server import JointTrajectoryAction
import tf2_ros
from tf_transformations import quaternion_from_euler

import rclpy
from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped

from std_srvs.srv import Trigger
from std_srvs.srv import SetBool

from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

from rcl_interfaces.msg import SetParametersResult
from control_msgs.msg import JointJog
from sensor_msgs.msg import BatteryState, JointState, Imu, MagneticField, Joy
from std_msgs.msg import Bool, String, Float64MultiArray

from hello_helpers.joy_conversion import SE4_dw4_sg4_Idx
from hello_helpers.joy_conversion import get_Idx

from hello_helpers.joy_conversion import (
    unpack_joy_to_gamepad_state,
    unpack_gamepad_state_to_joy,
    get_default_joy_msg,
)


# from .joint_trajectory_server import JointTrajectoryAction
from builtin_interfaces.msg import Time as TimeMsg


from rclpy import time as rclpyTime

from stretch_core.stretch4_ros_api import Stretch4ROSDriver


DEFAULT_SIM_TOOL = "eoa_wrist_dw4_tool_sg4"


class StretchMujocoDriver(Stretch4ROSDriver):
    def __init__(self):
        super().__init__('stretch_mujoco_driver')

        #set up any node specific pubs/subs
        #set up any node specific member variables

        self.check_and_load_params()
        self.linear_velocity_mps = 0.0  # m/s ROS SI standard for cmd_vel (REP 103)
        self.linear_velocity_mps_y = 0.0  # m/s ROS SI standard for cmd_vel (REP 103)
        self.angular_velocity_radps = 0.0  # rad/s ROS SI standard for cmd_vel (REP 103)

        self.max_arm_height = 1.1

        self.clock_pub = self.create_publisher(
            msg_type=Clock, topic="/clock", qos_profile=5
        )

        self.imu_mobile_base_pub = self.create_publisher(Imu, "imu_mobile_base", 1)
        self.magnetometer_mobile_base_pub = self.create_publisher(
            MagneticField, "magnetometer_mobile_base", 1
        )
        self.imu_wrist_pub = self.create_publisher(Imu, "imu_wrist", 1)
        self.is_gamepad_dongle_pub = self.create_publisher(Bool, "is_gamepad_dongle", 1)
        self.gamepad_state_pub = self.create_publisher(
            Joy, "stretch_gamepad_state", 1
        )  # decode using gamepad_conversion.unpack_joy_to_gamepad_state() on client side

        self.base_frame_id = "base_footprint"
        self.logger.info(f"base_frame_id = {self.base_frame_id}")
        self.odom_frame_id = "odom"
        self.logger.info(f"odom_frame_id = {self.odom_frame_id}")

        #self.joint_limits_pub = self.create_publisher(JointState, "joint_limits", 1) #needed?

        self.last_twist_time = self.get_clock().now()
        self.last_gamepad_joy_time = self.get_clock().now()

        # Robocasa set-up
        if self.use_robocasa:
            model, xml, objects_info = self.robocasa_setup()
            scene_xml_path = None
        else:
            model = None
            if self.scene_xml:
                scene_xml_path = get_absolute_path_stretch_xml(self.scene_xml)
            else:
                scene_xml_path = None
        
        self.sim = Stretch4MujocoSimulator(
            scene_xml_path=scene_xml_path,
            model=model,
            camera_hz=10,
            cameras_to_use=(
                StretchCameras.all_stretch4() if self.use_cameras else StretchCameras.none()
            ),
        )
        
        self.sim.start(headless=not self.use_mujoco_viewer)
        self.setup_cam_pubs()
        
        self.start()
        
    def setup_cam_pubs(self):
        self.laser_scan_pub = self.create_publisher(
            LaserScan,
            "/scan_filtered",
            qos_profile=QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        self.camera_publishers = {
            camera.name: self.create_publisher(
                Image,
                get_camera_topic_name(camera),
                qos_profile=QoSProfile(
                    depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
                ),
            )
            for camera in self.sim._cameras_to_use
        }
        self.camera_compressed_publishers = {
            camera.name: self.create_publisher(
                CompressedImage,
                f"{get_camera_topic_name(camera)}/compressed",
                qos_profile=QoSProfile(
                    depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
                ),
            )
            for camera in self.sim._cameras_to_use
        }
        self.pointcloud_publishers = {
            camera.name: self.create_publisher(
                PointCloud2,
                get_camera_pointcloud_topic_name(camera),
                qos_profile=QoSProfile(
                    depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
                ),
            )
            for camera in self.sim._cameras_to_use
            if camera.is_depth
        }
        self.camera_info_publishers = {
            camera.name: self.create_publisher(
                CameraInfo,
                get_camera_info_topic_name(camera),
                qos_profile=QoSProfile(
                    depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
                ),
            )
            for camera in self.sim._cameras_to_use
        }


    def declare_node_params(self):
        self.declare_parameter("use_cameras", rclpy.Parameter.Type.BOOL)
        self.declare_parameter("use_mujoco_viewer", rclpy.Parameter.Type.BOOL)
        self.declare_parameter("controller_calibration_file", rclpy.Parameter.Type.STRING)

        self.declare_parameter("use_robocasa", rclpy.Parameter.Type.BOOL)

        if self.get_parameter("use_robocasa").value:
            self.declare_parameter("robocasa_task", rclpy.Parameter.Type.STRING)
            self.declare_parameter("robocasa_layout", rclpy.Parameter.Type.STRING)
            self.declare_parameter("robocasa_style", rclpy.Parameter.Type.STRING)
            
        self.declare_parameter("scene_xml", rclpy.Parameter.Type.STRING)
        self.declare_parameter("scene_name", rclpy.Parameter.Type.STRING)
        self.declare_parameter("fail_out_of_range_goal", rclpy.Parameter.Type.BOOL)
        self.declare_parameter("fail_if_motor_initial_point_is_not_trajectory_first_point", rclpy.Parameter.Type.BOOL)
        self.declare_parameter("action_server_rate", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("timeout", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("default_goal_timeout_s", rclpy.Parameter.Type.DOUBLE)

    
         
    def check_and_load_params(self):
        self.use_cameras: bool = self.get_parameter("use_cameras").value

        
        self.use_mujoco_viewer: bool = self.get_parameter("use_mujoco_viewer").value

        
        self.controller_calibration_file: str = self.get_parameter("controller_calibration_file").value

        self.use_robocasa: bool = self.get_parameter("use_robocasa").value
        if self.use_robocasa:
            
            self.robocasa_task: str = self.get_parameter("robocasa_task").value
            self.robocasa_layout: str = self.get_parameter("robocasa_layout").value

            self.robocasa_style: str = self.get_parameter("robocasa_style").value

        
        self.scene_xml: str = self.get_parameter("scene_xml").value
        scene_name: str = self.get_parameter("scene_name").value

        if self.scene_xml:
            if self.use_robocasa:
                self.logger.warn("Cannot specify scene_xml while use_robocasa is True. "
                                 "Ignoring scene_xml.")
                self.scene_xml = None
            elif scene_name:
                self.logger.warn("Cannot specify scene_xml AND scene_name. "
                                 "Ignoring scene_name.")
        elif scene_name:
            self.scene_xml = models_path / (scene_name + '.xml')

        self.robot_mode: str = self.get_parameter("mode").value
        
        self.broadcast_odom_tf: bool = self.get_parameter("broadcast_odom_tf").value

        
        self.fail_out_of_range_goal: bool = self.get_parameter("fail_out_of_range_goal").value

        self.fail_if_motor_initial_point_is_not_trajectory_first_point: bool = self.get_parameter(
            "fail_if_motor_initial_point_is_not_trajectory_first_point"
        ).value

       
        self.action_server_rate: float = self.get_parameter("action_server_rate").value

        
        self.timeout_s: float = self.get_parameter("timeout").value
        self.timeout: Duration = Duration(seconds=self.timeout_s)

        self.default_goal_timeout_s: float = self.get_parameter("default_goal_timeout_s").value
        self.default_goal_timeout_duration: Duration = Duration(
            seconds=self.default_goal_timeout_s
        )
        
    def robocasa_setup(self):

        from stretch4_mujoco.robocasa_gen import (
            layout_from_str,
            style_from_str,
            model_generation_wizard,
            get_styles,
            layouts,
        )

        if isinstance(self.robocasa_layout, str):
            # Convert robocasa_layout to int
            if self.robocasa_layout.isnumeric():
                self.robocasa_layout = int(self.robocasa_layout)
            elif self.robocasa_layout == "Random":
                self.robocasa_layout = np.random.choice(range(len(layouts)))
            else:
                self.robocasa_layout = layout_from_str(self.robocasa_layout)
        elif self.robocasa_layout is None:
            self.robocasa_layout = -1

        if isinstance(self.robocasa_style, str):
            # Convert robocasa_style to int
            if self.robocasa_style.isnumeric():
                self.robocasa_style = int(self.robocasa_style)
            elif self.robocasa_style == "Random":
                self.robocasa_style = np.random.choice(range(len(get_styles())))
            else:
                self.robocasa_style = style_from_str(self.robocasa_style)
        elif self.robocasa_style is None:
            self.robocasa_style = -1

        model, xml, objects_info = model_generation_wizard(
            task=self.robocasa_task,
            layout=self.robocasa_layout,
            style=self.robocasa_style,
        )
        return model, xml, objects_info

    def joy_callback(self, joy):
        # self.robot_mode_rwlock.acquire_read()
        if self.robot_mode != "gamepad":
            self.logger.error(
                "{0} Stretch Driver must be in gamepad mode to "
                "receive a Joy msg on joy topic. "
                "Current mode = {1}.".format(self.node_name, self.robot_mode)
            )
            # self.robot_mode_rwlock.release_read()
            return
        self.latest_gamepad_joy_msg = joy
        self.last_gamepad_joy_time = self.get_clock().now()
        # self.robot_mode_rwlock.release_read()

    def twist_callback(self, twist:Twist):
        if self.robot_mode != "navigation":
            self.logger.error(
                "{0} action server must be in navigation mode to "
                "receive a twist on cmd_vel. "
                "Current mode = {1}.".format(self.node_name, self.robot_mode)
            )
            return

        self.linear_velocity_mps = twist.linear.x
        self.linear_velocity_mps_y = twist.linear.y
        self.angular_velocity_radps = twist.angular.z
        self.last_twist_time = self.get_clock().now()


    def velocity_callback(self, jointjog_msg: JointJog):
        pass


    def control_loop(self):
        
        # Set new mobile base velocities
        if self.robot_mode == "navigation":
            time_since_last_twist = self.get_clock().now() - self.last_twist_time
            if time_since_last_twist < self.timeout:
                self.sim.set_base_velocity(
                    self.linear_velocity_mps, self.linear_velocity_mps_y, self.angular_velocity_radps
                )
            elif time_since_last_twist < Duration(seconds=self.timeout_s + 1.0):  # type: ignore
                # self.sim.set_base_velocity(0.0, 0.0)
                self.sim.move_by(Actuators.base_translate, 0.0)
            else:
                self.sim.set_base_velocity(0.0, 0.0, 0.0)

        # get copy of the current robot status
        robot_status = self.sim.pull_status()

        self.logger.debug(robot_status.sim_to_real_time_ratio_msg)

        # Publish /clock for ROS to use sim time:
        seconds = int(robot_status.time)
        nanoseconds = int((robot_status.time - seconds) * 1e9)
        sim_time = rclpyTime.Time(seconds=seconds, nanoseconds=nanoseconds).to_msg()
        self.clock_pub.publish(Clock(clock=sim_time))

        # Use node time for other topics, using sim time makes bad things happen.
        current_time = self.get_clock().now().to_msg()

        # obtain odometry
        # assign relevant base status to variables
        base_status = robot_status.base
        x = base_status.x
        y = base_status.y
        theta = base_status.theta
        x_vel = base_status.x_vel
        # y_vel = base_status.y_vel #TODO: implement y_vel in base_status
        y_vel = base_status.x_vel
        theta_vel = base_status.theta_vel

        q = quaternion_from_euler(0.0, 0.0, theta)

        if self.broadcast_odom_tf:
            # publish odometry via TF
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

        # TODO: actually check for homing in sim (this was not implemented in
        # previous driver)
        with self.state_lock:
            self.pubs_state["homed"] = True

        # publish runstop event
        with self.state_lock:
            self.pubs_state["runstop_event"] = self.is_runstopped()

        with self.state_lock:
            self.pubs_state["mode"] = self.robot_mode

        with self.state_lock:
            self.pubs_state["tool"] = DEFAULT_SIM_TOOL

        # publish streaming position status
        '''streaming_position_status = Bool()
        streaming_position_status.data = self.streaming_position_activated
        self.streaming_position_mode_pub.publish(streaming_position_status)'''

        # publish joint state for the arm
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = current_time

        joint_state.name.append("lift_joint")
        joint_state.position.append(robot_status.lift.pos)
        joint_state.velocity.append(robot_status.lift.vel)
        joint_state.effort.append(robot_status.lift.effort)

        for link in ['arm_l4_joint', 'arm_l3_joint', 'arm_l2_joint', 'arm_l1_joint']:
            joint_state.name.append(link)
            joint_state.position.append(robot_status.arm.pos/4)
            joint_state.velocity.append(robot_status.arm.vel/4)
            joint_state.effort.append(robot_status.arm.effort/4)

        joint_state.name.append('wrist_yaw_joint')
        joint_state.position.append(robot_status.wrist_yaw.pos)
        joint_state.velocity.append(robot_status.wrist_yaw.vel)
        joint_state.effort.append(robot_status.wrist_yaw.effort)

        joint_state.name.append('wrist_pitch_joint')
        joint_state.position.append(robot_status.wrist_pitch.pos)
        joint_state.velocity.append(robot_status.wrist_pitch.vel)
        joint_state.effort.append(robot_status.wrist_pitch.effort)

        joint_state.name.append('wrist_roll_joint')
        joint_state.position.append(robot_status.wrist_roll.pos)
        joint_state.velocity.append(robot_status.wrist_roll.vel)
        joint_state.effort.append(robot_status.wrist_roll.effort)
        
        # for link in ['gripper_finger_left_joint', 'gripper_finger_right_joint']:
        joint_state.name.append("gripper_finger_left_joint")
        joint_state.position.append(robot_status.gripper_left_finger.pos)
        joint_state.velocity.append(robot_status.gripper_left_finger.vel)
        joint_state.effort.append(robot_status.gripper_left_finger.effort)
        joint_state.name.append("gripper_finger_right_joint")
        joint_state.position.append(robot_status.gripper_right_finger.pos)
        joint_state.velocity.append(robot_status.gripper_right_finger.vel)
        joint_state.effort.append(robot_status.gripper_right_finger.effort)
        for w in ['wheel_0_joint', 'wheel_1_joint', 'wheel_2_joint']:
            joint_state.name.append(w)
            joint_state.position.append(0.0)
            joint_state.velocity.append(0.0)
            joint_state.effort.append(0.0)

        with self.state_lock:
            self.pubs_state["joint_state"]=joint_state

       
        # publish odometry via the odom topic
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = x_vel
        odom.twist.twist.linear.y = y_vel
        odom.twist.twist.angular.z = theta_vel

        with self.state_lock:
            self.pubs_state["odom"]=odom

        runstop_status = self.is_runstopped()

        if (self.prev_runstop_state is None and runstop_status) or (
            self.prev_runstop_state is not None
            and runstop_status != self.prev_runstop_state
        ):
            self.runstop_the_robot(runstop_status, just_change_mode=True)

        self.prev_runstop_state = runstop_status

        self.publish_camera_and_lidar(current_time=current_time)

    def publish_child_info(self):
        # Use node time for other topics, using sim time makes bad things happen.
        current_time = self.get_clock().now().to_msg()

         ##################################################
        # publish IMU sensor data
        sensor_status = self.sim.pull_sensor_data()

        accel_status = sensor_status.get_data(StretchSensors.base_accel)
        gyro_status = sensor_status.get_data(StretchSensors.base_gyro)
        ax = accel_status[0]
        ay = accel_status[1]
        az = accel_status[2]
        gx = gyro_status[0]
        gy = gyro_status[1]
        # gz = gyro_status[2]
        # qw = gyro_status[3]
        # qx =  gyro_status[4]
        # qy = gyro_status[5]
        # qz = gyro_status[6]
        gz = qw = qx = qy = qz = 0.0  # TODO

        i = Imu()
        i.header.stamp = current_time
        i.header.frame_id = "imu_mobile_base"
        i.angular_velocity.x = gx
        i.angular_velocity.y = gy
        i.angular_velocity.z = gz

        i.orientation.w = qw
        i.orientation.x = qx
        i.orientation.y = qy
        i.orientation.z = qz

        i.linear_acceleration.x = ax
        i.linear_acceleration.y = ay
        i.linear_acceleration.z = az
        self.imu_mobile_base_pub.publish(i)

        m = MagneticField()
        m.header.stamp = current_time
        m.header.frame_id = "imu_mobile_base"
        self.magnetometer_mobile_base_pub.publish(m)

        # accel_status = robot_status.wacc
        # ax = accel_status.ax
        # ay = accel_status.ay
        # az = accel_status.az
        ax = ay = az = 0.0

        i = Imu()
        i.header.stamp = current_time
        i.header.frame_id = "accel_wrist"
        i.linear_acceleration.x = ax
        i.linear_acceleration.y = ay
        i.linear_acceleration.z = az
        self.imu_wrist_pub.publish(i)

        self.publish_camera_and_lidar(current_time=current_time)

        
    def stop_the_robot_callback(self, request, response):
        with self.robot_stop_lock:
            self.sim.move_by(Actuators.base_translate, 0.0)
            self.sim.move_by(Actuators.base_rotate, 0.0)
            self.sim.move_by(Actuators.arm, 0.0)
            self.sim.move_by(Actuators.lift, 0.0)

            self.sim.move_by("wrist_yaw", 0.0)
            self.sim.move_by("gripper", 0.0)

        self.logger.info(
            "Received stop_the_robot service call, so commanded all actuators to stop."
        )
        response.success = True
        response.message = "Stopped the robot."
        return response

    def home_the_robot_callback(self, request, response):
        self.logger.info("Received home_the_robot service call.")
        success, message = self.home_the_robot()
        response.success = success
        response.message = message
        return response

    def stow_the_robot_callback(self, request, response):
        self.logger.info("Received stow_the_robot service call.")
        success, message = self.stow_the_robot()
        response.success = success
        response.message = message
        return response

    def runstop_service_callback(self, request, response):
        self.logger.info("Received runstop_the_robot service call.")
        self.runstop_the_robot(request.data)
        response.success = True
        response.message = f"is_runstopped: {request.data}"
        return response

    def home_the_robot(self):
        # self.robot_mode_rwlock.acquire_read()
        can_home = self.robot_mode in self.control_modes
        last_robot_mode = copy.copy(self.robot_mode)
        # self.robot_mode_rwlock.release_read()
        if not can_home:
            errmsg = f"Cannot home while in mode={last_robot_mode}."
            self.logger.error(errmsg)
            return False, errmsg
        self.change_mode("homing", lambda: None)
        self.sim.home()
        self.change_mode(last_robot_mode, lambda: None)
        return True, "Homed."

    def stow_the_robot(self):
        # self.robot_mode_rwlock.acquire_read()
        can_stow = self.robot_mode in self.control_modes
        last_robot_mode = copy.copy(self.robot_mode)
        # self.robot_mode_rwlock.release_read()
        if not can_stow:
            errmsg = f"Cannot stow while in mode={last_robot_mode}."
            self.logger.error(errmsg)
            return False, errmsg
        self.change_mode("stowing", lambda: None)
        self.sim.stow()
        self.change_mode(last_robot_mode, lambda: None)
        return True, "Stowed."

    def is_runstopped(self):
        return self.robot_mode == "runstopped"
    
    def runstop_the_robot(self, runstopped, just_change_mode=False):
        if runstopped:
            # self.robot_mode_rwlock.acquire_read()
            already_runstopped = self.robot_mode == "runstopped"
            if not already_runstopped:
                self.prerunstop_mode = copy.copy(self.robot_mode)
            # self.robot_mode_rwlock.release_read()
            if already_runstopped:
                return
            self.change_mode("runstopped", lambda: None)
        else:
            # self.robot_mode_rwlock.acquire_read()
            already_not_runstopped = self.robot_mode != "runstopped"
            # self.robot_mode_rwlock.release_read()
            if already_not_runstopped:
                return
            self.change_mode(self.prerunstop_mode, lambda: None)

    def publish_camera_and_lidar(self, current_time: TimeMsg | None = None):

        current_time = current_time or self.get_clock().now().to_msg()

        sensor_status = self.sim.pull_sensor_data()

        try:
            lidar_data = sensor_status.get_data(StretchSensors.base_lidar)

            self.laser_scan_pub.publish(
                create_laser_scan_msg(
                    lidar_data, timestamp=current_time, frame_id="laser"
                )
            )
        except ValueError:
            ...  # Lidar is disabled, get_data() throws a ValueError

        camera_data = self.sim.pull_camera_data()
        for camera, frame in camera_data.get_all(
            auto_rotate=False, auto_correct_rgb=True
        ).items():
            header = Header()
            header.frame_id = get_camera_frame(camera)
            header.stamp = current_time

            ros_image = numpy_to_image(
                frame,
                encoding="bgr8" if not camera.is_depth else "32FC1",
            )
            ros_image.header = header
            self.camera_publishers[camera.name].publish(ros_image)

            settings: CameraSettings = camera.initial_camera_settings
            camera_info = create_camera_info(
                camera_settings=settings,
                frame_id=header.frame_id,
                timestamp=current_time,
            )
            self.camera_info_publishers[camera.name].publish(camera_info)

            if camera.is_depth:
                ros_image_compressed = compress_depth_image(frame)
            else:
                success, encoded_image = cv2.imencode(".png", frame)
                if not success:
                    self.logger.error(f"Failed to encode compressed image for {camera.name}")
                    continue

                ros_image_compressed = CompressedImage()
                ros_image_compressed.format = "png"
                ros_image_compressed.data = encoded_image.tobytes()

            ros_image_compressed.header = header
            self.camera_compressed_publishers[camera.name].publish(ros_image_compressed)

            if camera.is_depth:
                pointcloud_msg = create_pointcloud_msg(camera_info, frame)
                self.pointcloud_publishers[camera.name].publish(pointcloud_msg)

    def set_node_param(self, parameter:Parameter) -> bool:
        updated = False
        match parameter.name:
            case _:
                pass
        return updated

    def change_mode(self, mode):
        if mode not in self.control_modes and mode not in self.priority_modes:
            self.logger.error(f"Mode {mode} not in available control modes, not changing mode.")
            return

        if self.robot_mode in self.priority_modes:
            self.logger.error
        
        match mode:
            case "navigation":       
                self.linear_velocity_mps = 0.0
                self.linear_velocity_mps_y = 0.0
                self.angular_velocity_radps = 0.0
                self.robot_mode = mode
            case _:
                self.robot_mode = mode


name_to_dtypes = {
    "rgb8":    (np.uint8,  3),
    "rgba8":   (np.uint8,  4),
    "rgb16":   (np.uint16, 3),
    "rgba16":  (np.uint16, 4),
    "bgr8":    (np.uint8,  3),
    "bgra8":   (np.uint8,  4),
    "bgr16":   (np.uint16, 3),
    "bgra16":  (np.uint16, 4),
    "mono8":   (np.uint8,  1),
    "mono16":  (np.uint16, 1),

    # for bayer image (based on cv_bridge.cpp)
    "bayer_rggb8":  (np.uint8,  1),
    "bayer_bggr8":  (np.uint8,  1),
    "bayer_gbrg8":  (np.uint8,  1),
    "bayer_grbg8":  (np.uint8,  1),
    "bayer_rggb16":     (np.uint16, 1),
    "bayer_bggr16":     (np.uint16, 1),
    "bayer_gbrg16":     (np.uint16, 1),
    "bayer_grbg16":     (np.uint16, 1),

    # OpenCV CvMat types
    "8UC1":    (np.uint8,   1),
    "8UC2":    (np.uint8,   2),
    "8UC3":    (np.uint8,   3),
    "8UC4":    (np.uint8,   4),
    "8SC1":    (np.int8,    1),
    "8SC2":    (np.int8,    2),
    "8SC3":    (np.int8,    3),
    "8SC4":    (np.int8,    4),
    "16UC1":   (np.uint16,   1),
    "16UC2":   (np.uint16,   2),
    "16UC3":   (np.uint16,   3),
    "16UC4":   (np.uint16,   4),
    "16SC1":   (np.int16,  1),
    "16SC2":   (np.int16,  2),
    "16SC3":   (np.int16,  3),
    "16SC4":   (np.int16,  4),
    "32SC1":   (np.int32,   1),
    "32SC2":   (np.int32,   2),
    "32SC3":   (np.int32,   3),
    "32SC4":   (np.int32,   4),
    "32FC1":   (np.float32, 1),
    "32FC2":   (np.float32, 2),
    "32FC3":   (np.float32, 3),
    "32FC4":   (np.float32, 4),
    "64FC1":   (np.float64, 1),
    "64FC2":   (np.float64, 2),
    "64FC3":   (np.float64, 3),
    "64FC4":   (np.float64, 4)
}


def numpy_to_image(arr, encoding):
    if not encoding in name_to_dtypes:
        raise TypeError('Unrecognized encoding {}'.format(encoding))

    import sys
    im = Image(encoding=encoding)

    # extract width, height, and channels
    dtype_class, exp_channels = name_to_dtypes[encoding]
    dtype = np.dtype(dtype_class)
    if len(arr.shape) == 2:
        im.height, im.width, channels = arr.shape + (1,)
    elif len(arr.shape) == 3:
        im.height, im.width, channels = arr.shape
    else:
        raise TypeError("Array must be two or three dimensional")

    # check type and channels
    if exp_channels != channels:
        raise TypeError("Array has {} channels, {} requires {}".format(
            channels, encoding, exp_channels
        ))
    if dtype_class != arr.dtype.type:
        raise TypeError("Array is {}, {} requires {}".format(
            arr.dtype.type, encoding, dtype_class
        ))

    # make the array contiguous in memory, as mostly required by the format
    contig = np.ascontiguousarray(arr)
    im.data = contig.tobytes()
    im.step = contig.strides[0]
    im.is_bigendian = int(
        arr.dtype.byteorder == '>' or
        arr.dtype.byteorder == '=' and sys.byteorder == 'big'
    )

    return im


def create_laser_scan_msg(lidar_data: np.ndarray, timestamp: TimeMsg, frame_id: str):
    ranges = lidar_data.tolist()

    laser_scan_msg = (
        LaserScan()
    )  # https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/LaserScan.html
    laser_scan_msg.header = Header()
    laser_scan_msg.header.stamp = timestamp
    laser_scan_msg.header.frame_id = frame_id
    laser_scan_msg.angle_min = -np.pi
    laser_scan_msg.angle_max = np.pi
    laser_scan_msg.angle_increment = 2 * np.pi / len(ranges)
    laser_scan_msg.range_min = 0.2
    laser_scan_msg.range_max = 20.0
    laser_scan_msg.ranges = ranges

    return laser_scan_msg


def create_pointcloud_msg(camera_info_msg: CameraInfo, depth_image):

    fx = camera_info_msg.k[0]
    fy = camera_info_msg.k[4]
    cx = camera_info_msg.k[2]
    cy = camera_info_msg.k[5]

    x,y,z = depth_to_points(depth_image, fx, fy, cx, cy)

    points = np.stack((x, y, z), axis=-1)

    cloud_msg = pc2.create_cloud_xyz32(camera_info_msg.header, points)
    return cloud_msg


def create_pointcloud_rgb_msg(
    camera_info_msg: CameraInfo, rgb_image: np.ndarray, depth_image: np.ndarray
):
    fx = camera_info_msg.k[0]
    fy = camera_info_msg.k[4]
    cx = camera_info_msg.k[2]
    cy = camera_info_msg.k[5]
    
    x,y,z = depth_to_points(depth_image, fx, fy, cx, cy)
    
    valid = (depth_image > 0) & np.isfinite(depth_image)
    r = rgb_image[:, :, 2][valid]
    g = rgb_image[:, :, 1][valid]
    b = rgb_image[:, :, 0][valid]
    rgb = (r.astype(np.uint32) << 16) | (g.astype(np.uint32) << 8) | b.astype(np.uint32)

    # Create XYZRGB tuples
    cloud_data = [(x[i], y[i], z[i], rgb[i]) for i in range(len(z))]

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
    ]

    header = Header()
    header.stamp = camera_info_msg.header.stamp
    header.frame_id = (
        camera_info_msg.header.frame_id
    )  # typically "camera_link" or similar

    cloud_msg = pc2.create_cloud(header, fields, cloud_data)
    return cloud_msg


__COMPRESSED_DEPTH_16UC1_HEADER = array.array("B", [0] * 12)


def compress_depth_image(frame: np.ndarray):
    """
    Converts a F32 depth map in meters to a U16 map in millimeters
    """
    normalized_array = (frame * 1000).astype(np.uint16)

    _, encoded_image = cv2.imencode(".png", normalized_array)

    ros_image_compressed = CompressedImage()
    ros_image_compressed.format = "16uc1; compressedDepth"
    ros_image_compressed.data = __COMPRESSED_DEPTH_16UC1_HEADER + array.array(
        "B", encoded_image.tobytes()
    )

    return ros_image_compressed


def create_camera_info(
    camera_settings: CameraSettings, frame_id: str, timestamp: TimeMsg
):
    camera_info_msg = CameraInfo()
    camera_info_msg.header = Header()
    camera_info_msg.header.stamp = timestamp
    camera_info_msg.header.frame_id = frame_id
    camera_info_msg.width = camera_settings.width
    camera_info_msg.height = camera_settings.height
    camera_info_msg.distortion_model = "plumb_bob"

    camera_info_msg.d = camera_settings.get_distortion_params_d()
    camera_info_msg.k = camera_settings.get_intrinsic_params_k()
    camera_info_msg.p = camera_settings.get_projection_matrix_p()

    if camera_settings.crop is not None:
        camera_info_msg.roi.x_offset = camera_settings.crop.x_offset
        camera_info_msg.roi.y_offset = camera_settings.crop.y_offset
        camera_info_msg.roi.width = camera_settings.crop.width
        camera_info_msg.roi.height = camera_settings.crop.height

    return camera_info_msg


@cache
def get_camera_topic_name(camera: StretchCameras):
    """
    Topic names to match the camera topics published by the real Stretch robot.
    """
    if camera == StretchCameras.cam_gripper_se4_left_rgb:
        return "/gripper_camera/left/image_raw"
    if camera == StretchCameras.cam_gripper_se4_right_rgb:
        return "/gripper_camera/right/image_raw"
    if camera == StretchCameras.cam_gripper_se4_stereo_depth:
        return "/gripper_camera/depth/image_rect_raw"
    if camera == StretchCameras.cam_nav_rgb_se4_left:
        return "/cameras_head/left/image_raw"
    if camera == StretchCameras.cam_nav_rgb_se4_right:
        return "/cameras_head/right/image_raw"
    if camera == StretchCameras.cam_nav_rgb_se4_center:
        return "/camera_head/center/image_raw"
    if camera == StretchCameras.cam_hemilidar_left:
        return "/depth/left/depth"
    if camera == StretchCameras.cam_hemilidar_right:
        return "/depth/right/depth"

    raise NotImplementedError(f"Camera {camera} image topic mapping is not implemented")


@cache
def get_camera_info_topic_name(camera: StretchCameras):
    """
    Topic names to match the camera_info topics published by the real Stretch robot.
    """
    if camera == StretchCameras.cam_gripper_se4_left_rgb:
        return "/gripper_camera/left/camera_info"
    if camera == StretchCameras.cam_gripper_se4_right_rgb:
        return "/gripper_camera/right/camera_info"
    if camera == StretchCameras.cam_gripper_se4_stereo_depth:
        return "/gripper_camera/depth/camera_info"
    if camera == StretchCameras.cam_nav_rgb_se4_left:
        return "/cameras_head/left/camera_info"
    if camera == StretchCameras.cam_nav_rgb_se4_right:
        return "/cameras_head/right/camera_info"
    if camera == StretchCameras.cam_nav_rgb_se4_center:
        return "/camera_head/center/camera_info"
    if camera == StretchCameras.cam_hemilidar_left:
        return "/depth/left/camera_info"
    if camera == StretchCameras.cam_hemilidar_right:
        return "/depth/right/camera_info"

    raise NotImplementedError(f"Camera {camera} camera_info topic mapping is not implemented")


@cache
def get_camera_pointcloud_topic_name(camera: StretchCameras):
    """
    Topic names to match the pointcloud2 topics published by the real Stretch robot.
    """
    if camera == StretchCameras.cam_gripper_se4_stereo_depth:
        return "/gripper_camera/depth/color/points"
    if camera == StretchCameras.cam_hemilidar_left:
        return "/lidar_points_left"
    if camera == StretchCameras.cam_hemilidar_right:
        return "/lidar_points_right"

    raise NotImplementedError(f"Camera {camera} pointcloud topic mapping is not implemented")


@cache
def get_camera_frame(camera: StretchCameras):
    """
    Matches the simulation camera with the optical frame on the robot urdf.
    """
    if camera == StretchCameras.cam_gripper_se4_left_rgb:
        return "gripper_left_camera_color_optical_frame"
    if camera == StretchCameras.cam_gripper_se4_right_rgb:
        return "gripper_right_camera_color_optical_frame"
    if camera == StretchCameras.cam_gripper_se4_stereo_depth:
        return "gripper_stereo_camera_color_optical_frame"
    if camera == StretchCameras.cam_nav_rgb_se4_left:
        return "camera_left_optical_link"
    if camera == StretchCameras.cam_nav_rgb_se4_right:
        return "camera_right_optical_link"
    if camera == StretchCameras.cam_nav_rgb_se4_center:
        return "camera_center_optical_link"
    if camera == StretchCameras.cam_hemilidar_left:
        return "lidar_left_link"
    if camera == StretchCameras.cam_hemilidar_right:
        return "lidar_right_link"

    raise NotImplementedError(f"Camera {camera} frame is not implemented")



                
def main():
    rclpy.init()

    node = StretchMujocoDriver()

    try:
        while rclpy.ok() and node.sim.is_running():
            rclpy.spin_once(node)

    except KeyboardInterrupt:
        print("Detecting KeyboardInterrupt")
    finally:
        print("Stopping Stretch Mujoco Driver")
        node.sim.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
