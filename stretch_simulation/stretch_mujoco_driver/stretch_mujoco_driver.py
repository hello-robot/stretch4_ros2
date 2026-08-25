#! /usr/bin/env python3
import array
import copy
import sys
import math
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

from rclpy.qos import QoSProfile, ReliabilityPolicy
# from stretch_core.rwlock import RWLock
#from stretch_mujoco_driver.joint_trajectory_server import JointTrajectoryAction
import tf2_ros
from tf_transformations import quaternion_from_euler

import rclpy
from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped

from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock

from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from sensor_msgs.msg import BatteryState, JointState, Joy
from std_msgs.msg import Bool, String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


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


#from .joint_trajectory_server import JointTrajectoryAction
from builtin_interfaces.msg import Time as TimeMsg


from rclpy import time as rclpyTime

from hello_helpers.stretch4_ros_api import Stretch4ROSDriver


DEFAULT_SIM_TOOL = "eoa_wrist_dw4_tool_sg4"


class StretchMujocoDriver(Stretch4ROSDriver):
    command_joints = ["lift",
                      "arm",
                      "wrist_yaw",
                      "wrist_pitch",
                      "wrist_roll",
                      "stretch_gripper",
                      #"parallel_gripper",
                      #"gripper_right_finger",
                      #"gripper_left_finger",
                      ]
    velocity_joints = ["arm", "lift", "wrist_yaw", "wrist_roll", "wrist_pitch","stretch_gripper"]
    
    def __init__(self):
        super().__init__('stretch_mujoco_driver')

        #set up any node specific pubs/subs
        #set up any node specific member variables

        self.check_and_load_params()
        #self.linear_velocity_mps = 0.0  # m/s ROS SI standard for cmd_vel (REP 103)
        #self.linear_velocity_mps_y = 0.0  # m/s ROS SI standard for cmd_vel (REP 103)
        #self.angular_velocity_radps = 0.0  # rad/s ROS SI standard for cmd_vel (REP 103)

        #self.max_arm_height = 1.1


        # Setup sim-only publishers
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

        #self.last_twist_time = self.get_clock().now()
        #self.last_gamepad_joy_time = self.get_clock().now()

        self.continuous_joints = [
                Actuators.left_wheel_vel,
                Actuators.right_wheel_vel,
                Actuators.back_wheel_vel,
                Actuators.base_rotate,
                Actuators.base_translate,
                Actuators.base_translate_y,]

        '''self.ignored_joints = [
            Actuators.head_pan,
            Actuators.head_tilt,
            Actuators.gripper,]'''

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

        use_cameras = self.get_parameter("use_cameras").value
        self.sim = Stretch4MujocoSimulator(
            scene_xml_path=scene_xml_path,
            model=model,
            camera_hz=10,
            cameras_to_use=(
                StretchCameras.all_stretch4() if use_cameras else StretchCameras.none()
            ),
        )

        self.robot_stop_lock = threading.Lock()

        use_mujoco_viewer = self.get_parameter('use_mujoco_viewer').value
        self.sim.start(headless=not use_mujoco_viewer)

        if self.get_parameter("use_cameras").value:
            self.setup_cam_pubs()

        limits = self.sim.pull_joint_limits()

        # hacky workaround for mismatched representations in sim vs real
        if "stretch_gripper" in self.command_joints and Actuators["gripper"] not in limits:
            if Actuators["gripper_right_finger"] in limits and Actuators["gripper_left_finger"] in limits:
                 (rll, rul) = limits[Actuators["gripper_right_finger"]]
                 (lll, lul) = limits[Actuators["gripper_left_finger"]]
                 limits[Actuators["gripper"]] = (np.float64(rll+lll), np.float64(rul+lul))
            
        for joint in self.command_joints:
            if "stretch_gripper" in joint: #stretch gripper has its fingers modeled separately in sim
                if Actuators["gripper_right_finger"] in limits and Actuators["gripper_left_finger"] in limits:
                    (rll, rul) = limits[Actuators["gripper_right_finger"]]
                    (lll, lul) = limits[Actuators["gripper_left_finger"]]
                    (ll,ul) = (np.float64(rll+lll), np.float64(rul+lul))
                else:
                    (ll,ul)=(None,None)
            elif "parallel_gripper" in joint: #parallel gripper not actually implemented in sim:
                self.logger.warning("Parallel gripper not available in simulation")
                (ll,ul)=(None,None)
            else:
                (ll,ul) = limits[Actuators[joint]]
            if ll is not None and ul is not None:
                results = self.set_parameters([Parameter(f"joint_limit.{joint}.upper", Parameter.Type.DOUBLE, ul),
                                 Parameter(f"joint_limit.{joint}.lower", Parameter.Type.DOUBLE, ll)])
                success = list(map(lambda x: x.successful, results))
                reasons = list(map(lambda x: x.reason, results))
                self.logger.info(f"Setting joint limits for joint {joint} as ({ll},{ul}).  Success: {success}, reasons: {reasons}")
        # Add a check timer to detect when the simulator is closed
        self.sim_check_timer = self.create_timer(0.1, self.check_sim_status, callback_group=self.main_group)
        self.start()
        
    def setup_cam_pubs(self):
        self.laser_scan_pub = self.create_publisher(
            LaserScan,
            "/scan_filtered",
            qos_profile=QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        self.lidar_left_pub = self.create_publisher(
            PointCloud2,
            "/lidar_points_left",
            qos_profile=QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.lidar_right_pub = self.create_publisher(
            PointCloud2,
            "/lidar_points_right",
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
        self.declare_parameter("use_cameras", False)
        self.declare_parameter("use_mujoco_viewer",True)
        self.declare_parameter("controller_calibration_file", rclpy.Parameter.Type.STRING)

        self.declare_parameter("use_robocasa", False)

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

    def startup_robot(self):
        pass
    
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

        def get_button_vel(joint_name, pos_btn, neg_btn):
            max_vel = self.get_parameter(f"gamepad.max_vel.{joint_name}").value
            if state.get(pos_btn, False):
                return max_vel
            elif state.get(neg_btn, False):
                return -max_vel
            return 0.0

        # Define targets dictionary mapping joints to velocities
        targets = {
            "lift": get_val('left_stick_y', 'lift'),
            "arm": get_val('left_stick_x', 'arm'),
            "wrist_yaw": get_val('right_stick_x', 'wrist_yaw'),
            "wrist_pitch": get_val('right_stick_y', 'wrist_pitch'),
            "wrist_roll": get_button_vel('wrist_roll', 'right_shoulder_button_pressed', 'left_shoulder_button_pressed'),
            "stretch_gripper": get_button_vel('stretch_gripper', 'top_button_pressed', 'bottom_button_pressed')
        }

        goal = JointState()
        for joint, vel in targets.items():
            if joint in self.command_joints:
                goal.name.append(joint)
                goal.velocity.append(vel)
                # Compute integrated position target
                curr_pos = self.cmd_joint_position(joint)
                new_pos = curr_pos + vel * dt
                goal.position.append(0.3)#curr_pos)#new_pos)
                self.logger.info(f"joint {joint} in position {curr_pos}. Sending position {new_pos} and velocity {vel}")

        self.logger.info(f"goal: {goal}")
        return goal
        
    def set_base_velocity(self, x, y, theta):
        self.sim.base.set_velocity(x, y, theta)

    def set_joint_position(self, joint, target):  
        self.logger.info(f"Driver setting joint {joint} to position {target}. current position is {self.cmd_joint_position(joint)}.") 
        subsys = None
         
        if hasattr(self.sim, joint):
            subsys = getattr(self.sim, joint)

        if hasattr(self.sim, "end_of_arm"):
            if hasattr(self.sim.end_of_arm, joint):
                subsys = getattr(self.sim.end_of_arm, joint)

        if hasattr(self.sim, "head"):
            if hasattr(self.sim.head, joint):
                subsys = getattr(self.sim.head, joint)

        if subsys is None:
            self.logger.error(f"Joint {joint} not found.  Do you have the name correct? Available joints to command are {self.command_joints}.  If joint is in this list, check that the interface to the Mujoco simulator hasn't changed.")

        subsys.move_to(target)

    def set_joint_velocity(self, joint, target):
        self.logger.info(f"Driver setting joint {joint} to velocity {target}")
        
        subsys = None
        
        if hasattr(self.sim, joint):
            subsys = getattr(self.sim, joint)

        if hasattr(self.sim, "end_of_arm"):
            if hasattr(self.sim.end_of_arm, joint):
                subsys = getattr(self.sim.end_of_arm, joint)

        if hasattr(self.sim, "head"):
            if hasattr(self.sim.head, joint):
                subsys = getattr(self.sim.head, joint)

        if subsys is None:
            self.logger.error(f"Joint {joint} not found.  Do you have the name correct? Available joints to command are {self.command_joints}.  If joint is in this list, check that the interface to the Mujoco simulator hasn't changed.")

        subsys.set_velocity(target)
    

    def publish_child_info(self):
        # > sim only publish streaming position (??)
        # > sim only ?? handle runstop?
        
        current_time = self.get_clock().now().to_msg()
        
        # get copy of the current robot status
        robot_status = self.sim.pull_status()
        self.logger.info(robot_status.sim_to_real_time_ratio_msg,throttle_duration_sec=5.0)

        # Publish /clock for ROS to use sim time:
        seconds = int(robot_status.time)
        nanoseconds = int((robot_status.time - seconds) * 1e9)
        sim_time = rclpyTime.Time(seconds=seconds, nanoseconds=nanoseconds).to_msg()
        self.clock_pub.publish(Clock(clock=sim_time))

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
        gz = gyro_status[2] if len(gyro_status) >= 3 else 0.0

        # Calculate quaternion from ground-truth base yaw (theta)
        theta = robot_status.base.theta
        qw = math.cos(theta / 2.0)
        qx = 0.0
        qy = 0.0
        qz = math.sin(theta / 2.0)

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

        if self.get_parameter("use_cameras").value:
            self.publish_camera_and_lidar(current_time=current_time)

        
    def stop_the_robot_callback(self, request, response):
        with self.robot_stop_lock:
            # 1. Stop base velocity
            self.set_base_velocity(0.0, 0.0, 0.0)
            
            # 2. Stop all commanded joints in their current modes
            for joint in self.command_joints:
                mode = self.get_parameter(f"joint_mode.{joint}").value
                if mode == "velocity":
                    self.set_joint_velocity(joint, 0.0)
                else:
                    # Position or other modes: halt at current position
                    current_pos = self.cmd_joint_position(joint)
                    self.set_joint_position(joint, current_pos)

            # 3. Wait for joints and base to settle if settling.enable is True
            settling_enable = self.get_parameter("settling.enable").value
            stopped = True
            
            if settling_enable:
                stopped = False
                timeout = self.get_parameter("settling.timeout").value
                vel_threshold = self.get_parameter("settling.vel_threshold").value
                
                start_time = self.get_clock().now()
                rate = self.create_rate(100) # 100 Hz
                
                while (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout:
                    robot_status = self.sim.pull_status()
                    current_time = self.get_clock().now().to_msg()
                    joint_state = self.get_joint_state(robot_status, current_time)
                    
                    # Check base velocity (both linear and angular)
                    base_stopped = (
                        abs(robot_status.base.x_vel) < vel_threshold and
                        abs(robot_status.base.theta_vel) < vel_threshold
                    )
                    
                    # Check if all commanded joints are below velocity threshold
                    joints_stopped = True
                    for joint in self.command_joints:
                        vel = self.command_joint_vel_from_joint_state(joint, joint_state, default=0.0)
                        if abs(vel) >= vel_threshold:
                            joints_stopped = False
                            break
                    
                    if base_stopped and joints_stopped:
                        stopped = True
                        break
                    
                    rate.sleep()
                
                try:
                    self.destroy_rate(rate)
                except AttributeError:
                    pass

        if not stopped:
            vel_threshold = self.get_parameter("settling.vel_threshold").value
            timeout = self.get_parameter("settling.timeout").value
            self.logger.warning(
                f"stop_the_robot failed to settle below velocity threshold {vel_threshold} within {timeout}s."
            )
            response.success = False
            response.message = f"Robot failed to settle below velocity threshold {vel_threshold} within {timeout}s."
        else:
            self.logger.info(
                "Received stop_the_robot service call: commanded and verified all actuators stopped and settled successfully."
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
        mode = self.robot_mode()
        can_home = mode in self.control_modes
        last_robot_mode = copy.copy(mode)
        
        if not can_home:
            errmsg = f"Cannot home while in mode={last_robot_mode}."
            self.logger.error(errmsg)
            return False, errmsg

        self.change_mode("homing")
        self.sim.home()
        self.change_mode(last_robot_mode)
        return True, "Homed."

    def stow_the_robot(self):
        mode = self.robot_mode()
        
        can_stow = mode in self.control_modes
        last_robot_mode = copy.copy(mode)

        if not can_stow:
            errmsg = f"Cannot stow while in mode={last_robot_mode}."
            self.logger.error(errmsg)
            return False, errmsg
        self.change_mode("stowing")
        self.sim.stow()
        self.change_mode(last_robot_mode)
        return True, "Stowed."

    def is_runstopped(self):
        return self.robot_mode() == "runstopped"
    
    def runstop_the_robot(self, runstopped, just_change_mode=False):
        if runstopped:
            already_runstopped = self.is_runstopped()
            if not already_runstopped:
                self.prerunstop_mode = copy.copy(self.robot_mode())

            if already_runstopped:
                return
            self.change_mode("runstopped")
        else:
            already_not_runstopped = not self.is_runstopped()
            if already_not_runstopped:
                return
            self.change_mode(self.prerunstop_mode)

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
            if camera.name not in self.camera_publishers:
                self.logger.warning(f"Camera {camera.name} not in cameras_to_use although it was retrieved from the simulation, skipping", throttle_duration_sec=5.0)
                continue
                
            
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
        try:
            hesai_pts = self.sim.pull_lidar_points()
            try:
                left_pts = hesai_pts.get("left")
                right_pts = hesai_pts.get("right")
            except:
                self.logger.warning("Error reading lidar point dictionary. Not publishing.")
                left_pts = None
                right_pts = None
            
            if left_pts is not None and len(left_pts) > 0:
                header = Header()
                header.frame_id = "base_footprint"
                header.stamp = current_time
                cloud_msg_left = pc2.create_cloud_xyz32(header, left_pts)
                self.lidar_left_pub.publish(cloud_msg_left)

            if right_pts is not None and len(right_pts) > 0:
                header = Header()
                header.frame_id = "base_footprint"
                header.stamp = current_time
                cloud_msg_right = pc2.create_cloud_xyz32(header, right_pts)
                self.lidar_right_pub.publish(cloud_msg_right)
        except Exception as e:
            self.logger.error(f"Error publishing Hesai lidar points: {e}")  
            

    def update_child_parameter(self, parameter:Parameter) -> bool:
        match parameter.name:
            case _:
                pass
            
    def check_child_param(self, parameter: Parameter) -> tuple[bool, str]:
        reason = None
        found = False
        
        match parameter.name:
            case "gamepad.dt":
                found = True
                if parameter.value <= 0.0:
                    reason = "gamepad.dt must be positive."
            case n if n.startswith("gamepad.max_vel."):
                joint = n.split(".")[-1]
                if joint in self.command_joints:
                    found = True
                    if parameter.value < 0.0:
                        reason = f"gamepad.max_vel.{joint} cannot be negative."
            case n if n.startswith("gamepad.deadzone."):
                joint = n.split(".")[-1]
                if joint in self.command_joints:
                    found = True
                    if not (0.0 <= parameter.value < 1.0):
                        reason = f"gamepad.deadzone.{joint} must be between 0.0 and 1.0."
                        
        if not found:
            reason = f"Parameter {parameter.name} not mutable or not found."
            
        return found, reason
        
                
    def handle_mode_change(self, mode):
        # called after set parameter check automatically
        pass
        

    def check_sim_status(self):
        if not self.sim.is_running():
            self.logger.info("MuJoCo simulator has stopped. Shutting down node...")
            if rclpy.ok():
                rclpy.shutdown()

    def push_robot_command(self):
        return

    
    def get_robot_status(self):
        robot_status = self.sim.pull_status()
        self.logger.debug(robot_status.sim_to_real_time_ratio_msg)
        return robot_status

    def command_joint_pose_from_joint_state(self, command_joint, joint_state, default=None):

        match command_joint:
            case "arm":
                poses = []
                subjoints = [f"{command_joint}_l{i}_joint" for i in [1,2,3,4]]
                for j in subjoints:
                    try:
                        poses.append(joint_state.position[joint_state.name.index(j)])
                    except ValueError:
                        pass
                return sum(poses) if poses else default
            case "stretch_gripper":
                try:
                    left = joint_state.position[joint_state.name.index("gripper_finger_left_joint")]
                    right = joint_state.position[joint_state.name.index("gripper_finger_right_joint")]
                    return left + right
                except ValueError:
                    return default
            case _:
                joint_name = command_joint+"_joint"
                try:
                    return joint_state.position[joint_state.name.index(joint_name)]
                except (ValueError, IndexError):
                    return default
            

                
    def command_joint_vel_from_joint_state(self, command_joint, joint_state, default=None):
        match command_joint:
            case "arm":
                vels = []
                subjoints = [f"{command_joint}_l{i}_joint" for i in [1,2,3,4]]
                for j in subjoints:
                    try:
                        vels.append(joint_state.velocity[joint_state.name.index(j)])
                    except (ValueError, IndexError):
                        pass
                return sum(vels) if vels else default
            case "stretch_gripper":
                try:
                    left = joint_state.velocity[joint_state.name.index("gripper_finger_left_joint")]
                    right = joint_state.velocity[joint_state.name.index("gripper_finger_right_joint")]
                    return left + right
                except (ValueError, IndexError):
                    return default
            case _:
                joint_name = command_joint+"_joint"
                try:
                    return joint_state.velocity[joint_state.name.index(joint_name)]
                except (ValueError, IndexError):
                    return default

                
    def get_odom(self, robot_status, status_time) -> Odometry:
        # obtain odometry
        # assign relevant base status to variables
        base_status = robot_status.base
        x = base_status.x
        y = base_status.y
        theta = base_status.theta
        x_vel = base_status.x_vel
        # y_vel = base_status.y_vel #TODO: implement y_vel in base_status
        y_vel = 0.0
        theta_vel = base_status.theta_vel

        q = quaternion_from_euler(0.0, 0.0, theta)
        
        odom = Odometry()
        odom.header.stamp = status_time
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

        return odom

    def get_homed(self, robot_status, status_time) -> Bool:
        return True


    def get_mode(self, robot_status, status_time) -> String:
        return self.robot_mode()

                                  
    def get_tool(self, robot_status, status_time) -> String:
        return DEFAULT_SIM_TOOL

                                   
    def get_sensitivity(self, robot_status, status_time) -> String:
        #TODO: return a real value for this (original driver doesn't do anything)
        return "simulation"
              
    def get_runstop(self, robot_status, status_time) -> Bool:
        runstop_status = self.is_runstopped()

        #TODO: figure out what this code is doing and why
        if (self.prev_runstop_state is None and runstop_status) or (
            self.prev_runstop_state is not None
            and runstop_status != self.prev_runstop_state
        ):
            self.runstop_the_robot(runstop_status, just_change_mode=True)

        self.prev_runstop_state = runstop_status

        return runstop_status
    
                                
    def get_joint_state(self, robot_status, status_time) -> JointState:
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = status_time

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

        return joint_state
                                  
    def get_battery(self, robot_status, status_time) -> BatteryState:
        battery = BatteryState()

        battery.header = Header()
        battery.header.stamp = status_time

        battery.voltage = 13.2
        battery.temperature = 25.0
        battery.current = -1.5
        battery.charge = 20.4
        battery.capacity = 24.0
        battery.design_capacity = 24.0
        battery.percentage = 0.85

        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        battery.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO

        battery.present = True

        return battery

    
    def get_diagnostics(self, robot_status, status_time) -> DiagnosticArray:
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = status_time
        
        status = DiagnosticStatus()
        status.name = self.prefix + 'simulation_PC'
        status.level = DiagnosticStatus.OK
        status.message = 'Simulation environment is running normally.'
        status.values.append(KeyValue(key="sim_time", value=f"{robot_status.time:.3f}"))
        status.values.append(KeyValue(key="fps", value=f"{robot_status.fps:.1f}"))
        status.values.append(KeyValue(key="sim_to_real_time_ratio", value=str(robot_status.sim_to_real_time_ratio_msg)))
        status.values.append(KeyValue(key="is_self_colliding", value=str(robot_status.is_self_colliding)))
        diagnostics.status.append(status)
        
        return diagnostics

    
    def get_lease_holder(self, robot_status, status_time) -> DiagnosticStatus:
        lease_holder_msg = DiagnosticStatus()
        lease_holder_msg.name = self.prefix + 'server_lease_holder'
        lease_holder_msg.level = DiagnosticStatus.OK
        lease_holder_msg.values.append(KeyValue(key="lease_holder", value="sim_client"))
        lease_holder_msg.values.append(KeyValue(key="lease_holder_priority", value="0"))
        lease_holder_msg.values.append(KeyValue(key="lease_expiry", value="9999999999.0"))
        lease_holder_msg.values.append(KeyValue(key="lease_expired", value="False"))
        lease_holder_msg.values.append(KeyValue(key="routine_active", value="False"))
        return lease_holder_msg


    def get_joint_state_diagnostics(self, robot_status, status_time) -> DiagnosticArray:
        joint_state_diagnostics = DiagnosticArray()
        joint_state_diagnostics.header.stamp = status_time

        at_limit_msg = DiagnosticStatus(name="at_limit")
        soft_limits_msg = DiagnosticStatus(name="soft_motion_limits")
        braking_distance_msg = DiagnosticStatus(name="braking_distance")
        is_homed_msg = DiagnosticStatus(name="is_homed")
        is_homing_msg = DiagnosticStatus(name="is_homing")
        is_runstopped_msg = DiagnosticStatus(name="is_runstopped")

        current_mode = self.get_parameter('mode').value
        is_runstopped = str(current_mode == 'runstopped')
        is_homing = str(current_mode == 'homing')

        for joint in self.command_joints:
            joint_key = f"{joint}_joint" if not joint.startswith("stretch_") else "stretch_gripper_joint"
            
            # Check limits dynamically
            try:
                lower = self.get_parameter(f"joint_limit.{joint}.lower").value
                upper = self.get_parameter(f"joint_limit.{joint}.upper").value
                pos = self.cmd_joint_position(joint)
                at_limit = str(abs(pos - lower) < 1e-3 or abs(pos - upper) < 1e-3)
            except Exception:
                at_limit = "False"

            at_limit_msg.values.append(KeyValue(key=joint_key, value=at_limit))
            soft_limits_msg.values.append(KeyValue(key=joint_key, value="False"))
            braking_distance_msg.values.append(KeyValue(key=joint_key, value="0.0"))
            is_homed_msg.values.append(KeyValue(key=joint_key, value="True"))
            is_homing_msg.values.append(KeyValue(key=joint_key, value=is_homing))
            is_runstopped_msg.values.append(KeyValue(key=joint_key, value=is_runstopped))

        joint_state_diagnostics.status.append(is_runstopped_msg)
        joint_state_diagnostics.status.append(is_homed_msg)
        joint_state_diagnostics.status.append(is_homing_msg)
        joint_state_diagnostics.status.append(at_limit_msg)
        joint_state_diagnostics.status.append(soft_limits_msg)
        joint_state_diagnostics.status.append(braking_distance_msg)
        
        return joint_state_diagnostics

    def get_safety_diagnostics(self, robot_status, status_time):
        safety_diagnostics = DiagnosticArray()
        safety_diagnostics.header.stamp = status_time
        
        smm_diag = DiagnosticStatus()
        smm_diag.name = self.prefix + 'safety_layer/safe_motion_manager'
        smm_diag.level = DiagnosticStatus.OK
        smm_diag.message = 'OK'
        smm_diag.values.append(KeyValue(key="safe_motions_triggered", value="[]"))
        smm_diag.values.append(KeyValue(key="active/cliff_sentry", value="False"))
        safety_diagnostics.status.append(smm_diag)
        
        sm_diag = DiagnosticStatus()
        sm_diag.name = self.prefix + 'safety_layer/sentry_manager'
        sm_diag.level = DiagnosticStatus.OK
        sm_diag.message = 'OK'
        sm_diag.values.append(KeyValue(key="active/collision_sentry", value="False"))
        safety_diagnostics.status.append(sm_diag)
        
        return safety_diagnostics




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
    if camera == StretchCameras.cam_nav_rgb_se4_center_low_rez:
        return "/camera_head/center/low_res"
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
    if camera == StretchCameras.cam_nav_rgb_se4_center_low_rez:
        return "/camera_head/center/low_res/camera_info"

    raise NotImplementedError(f"Camera {camera} camera_info topic mapping is not implemented")


@cache
def get_camera_pointcloud_topic_name(camera: StretchCameras):
    """
    Topic names to match the pointcloud2 topics published by the real Stretch robot.
    """
    if camera == StretchCameras.cam_gripper_se4_stereo_depth:
        return "/gripper_camera/depth/color/points"
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
    raise NotImplementedError(f"Camera {camera} frame is not implemented")



                
def main():
    try:
        rclpy.init()
        node = StretchMujocoDriver()
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=5)
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            print("Stopping Stretch Mujoco Driver")
            node.sim.stop()
            executor.shutdown()
            node.destroy_node()
    except KeyboardInterrupt:
        print("Detecting KeyboardInterrupt")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
