#! /usr/bin/env python3

from abc import ABC, abstractmethod
from typing import Any
from threading import Lock

import rclpy
from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.publisher import Publisher

from std_srvs.srv import Trigger, SetBool

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from sensor_msgs.msg import BatteryState, JointState, Joy
from std_msgs.msg import Bool, String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

import tf2_ros
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class Stretch4ROSDriver(Node, ABC):
    body_joints = None
    joint_modes = ["position","velocity"]

    def __init__(self,name):
        super().__init__(name)
        self.node_name = self.get_name()
        self.logger = self.get_logger()
        self.logger.info("For use with S T R E T C H (TM) RESEARCH EDITION from Hello Robot Inc.")
        
        self.logger.info("{0} started".format(self.node_name))

        self.control_modes = ['active', 'teleop']
        self.default_mode = 'active'
        self.priority_modes = ['homing', 'stowing', 'runstopped']
        
        self._declare_common_params()
        self.declare_node_params()

        # Runstop management
        self.prev_runstop_state = None
        self.prerunstop_mode = None

        # Callback groups
        self.main_group = ReentrantCallbackGroup()
        self.mutex_group = MutuallyExclusiveCallbackGroup()

        self._setup_common_pubs()
        self._setup_common_subs()
        self._setup_common_srvs()

        self.last_published_value={}
        
        # TF2
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Namespace
        ns = self.get_namespace().strip('/')
        self.prefix = ns + '/' if ns else ''
        if ns:
            self.logger.info(f"namespace = {self.prefix}")

        # Get param values
        mode=self.get_parameter('mode').value
        if mode not in self.control_modes:
            self.logger.warn(f'given invalid mode={mode}, using {self.default_mode} instead')
            mode = self.default_mode
            
        self.logger.info('mode = ' + str(mode))
        
        sensitivity = self.get_parameter('sensitivity').value
        self.logger.info('sensitivity = ' + str(sensitivity))
        
        broadcast_odom_tf = self.get_parameter('broadcast_odom_tf').value
        self.logger.info(f'broadcast_odom_tf = {broadcast_odom_tf}')

        action_timeout = self.get_parameter('action_timeout').value
        self.logger.info(f"action timeout = {action_timeout} s")
        
        velocity_timeout = self.get_parameter('velocity_timeout').value
        #self.velocity_timeout_duration = Duration(seconds=velocity_timeout)
        self.logger.info(f"velocity timeout = {velocity_timeout} s")
        
        self.add_on_set_parameters_callback(self.check_parameter_callback)
        self.add_post_set_parameters_callback(self.parameter_update_callback)

        self.last_position_target = {}
        self.last_known_state = {}
        self.position_tolerance = self.get_parameter('position_tolerance').value

        self.trajectory_server = self.setup_trajectory_action()
        if self.trajectory_server is None:
            self.logger.error("setup_trajectory_action failed to return JointTrajectoryAction. Proceeding without trajectory action server.")
        
    def start(self): #called by subclasses once setup for control loop is done
        self.control_timer = self.create_timer(
            1.0 / self.get_parameter('control_loop_rate').value,
            self.control_loop,
            callback_group=self.mutex_group,
        )
        
    @abstractmethod
    def declare_node_params(self):
        pass

    def _declare_common_params(self):
        self.declare_parameter('mode',self.default_mode)
        self.declare_parameter('sensitivity','default')
        self.declare_parameter('broadcast_odom_tf', False) # based on wheel odometry


        desc = ParameterDescriptor(
            dynamic_typing = True,
            description='Joint properties',
        )
        
        for joint in self.body_joints:
            self.declare_parameter(f"joint_acceleration.{joint}",None, desc)
            self.declare_parameter(f"joint_limit.{joint}.upper",None, desc)
            self.declare_parameter(f"joint_limit.{joint}.lower",None, desc)
            self.declare_parameter(f"joint_mode.{joint}","position", ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description=f"Control mode for individual joint (valid options are: {self.joint_modes})"))
            #default to position control mode            
            
        self.declare_parameter("joint_acceleration.omnibase.linear", None, desc)
        self.declare_parameter("joint_acceleration.omnibase.angular", None, desc)
        
        self.declare_parameter('action_timeout', 3.0, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Default timeout (sec) for execution of joint traj action',
        ))
        self.declare_parameter('velocity_timeout', 0.5, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Default timeout (sec) for velocity control',
        ))
        
        self.declare_parameter('control_loop_rate', 100, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Target rate (hz) for main control loop',
        ))

        self.declare_parameter('jog_duration', 0.1, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Target duration (s) for robot jogging requests',
        ))
        
        self.declare_parameter('position_tolerance', 0.01, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Tolerance on joint position (rad) for position commands',
        ))

    def _setup_common_pubs(self):
        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.latched_publishers = []
        self.unlatched_publishers = []
        
        self.odom_pub = self.create_publisher(Odometry, 'wheel_odom', 1)
        # handle odometry separately; do not add to arrays of regular publishers
        
        self.homed_pub = self.create_publisher(Bool, 'is_homed', latching_qos)
        self.latched_publishers.append((self.homed_pub, self.get_homed))

        self.mode_pub = self.create_publisher(String, 'mode', latching_qos)
        self.latched_publishers.append((self.mode_pub, self.get_mode))
        
        self.tool_pub = self.create_publisher(String, 'tool', latching_qos)
        self.latched_publishers.append((self.tool_pub, self.get_tool))
                
        self.sensitivity_pub = self.create_publisher(String, 'sensitivity', latching_qos)
        self.latched_publishers.append((self.sensitivity_pub, self.get_sensitivity))

        self.runstop_pub = self.create_publisher(Bool, 'is_runstopped', latching_qos)
        self.latched_publishers.append((self.runstop_pub, self.get_runstop))
        
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 1)
        # handle joint state separately; do not add to arrays of regular publishers
        
        self.battery_pub = self.create_publisher(BatteryState, 'battery', 1)
        self.unlatched_publishers.append((self.battery_pub, self.get_battery))
        
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 1) # Diagnostics are centralized, so we publish to a single global /diagnostics topic, but set up separate get functions for regular and safety diagnostics
        self.unlatched_publishers.append((self.diagnostics_pub, self.get_diagnostics))
        self.unlatched_publishers.append((self.diagnostics_pub, self.get_safety_diagnostics))
                
        self.lease_holder_pub = self.create_publisher(DiagnosticStatus, 'server_lease_holder', 1)
        self.unlatched_publishers.append((self.lease_holder_pub, self.get_lease_holder))

        self.joint_state_diagnostics_pub= self.create_publisher(DiagnosticArray, 'joint_states_diagnostics', 1)
        self.unlatched_publishers.append((self.joint_state_diagnostics_pub, self.get_joint_state_diagnostics))
    
        # Saved Message States (for latched topics)
        self.last_published_value = {}

    def _setup_common_subs(self):
        # Subscribers
        self.create_subscription(Twist, "cmd_vel", self.base_twist_callback, 1, callback_group=self.main_group)
        self.create_subscription(JointState, "joint_velocity_cmd", self.velocity_cmd_callback, 1, callback_group=self.main_group)
        
        self.create_subscription(JointState, "joint_position_cmd", self.position_cmd_callback, 1, callback_group=self.main_group)
        
        self.create_subscription(Joy, "joy", self.joy_callback, 1, callback_group=self.main_group)

    def _setup_common_srvs(self):
        # Services
        self.stop_the_robot_service = self.create_service(
            Trigger,
            'stop_the_robot',
            self.stop_the_robot_callback,
            callback_group=self.main_group
        )
        self.home_the_robot_service = self.create_service(
            Trigger,
            'home_the_robot',
            self.home_the_robot_callback,
            callback_group=self.main_group
        )
        self.stow_the_robot_service = self.create_service(
            Trigger,
            'stow_the_robot',
            self.stow_the_robot_callback,
            callback_group=self.main_group
        )
        self.runstop_service = self.create_service(
            SetBool,
            'runstop_the_robot',
            self.runstop_service_callback,
            callback_group=self.main_group
        )    

    def setup_common_actions(self):
        pass
        #self.joint_trajectory_action = JointTrajectoryAction(self)

    @abstractmethod
    def check_child_param(self, parameter: Parameter) -> tuple[bool,String]:
        #ROS expects parameter callback to atomically accept or reject all changes
        #only return true if ALL changes succeed.  Checking should not have side effects!
        pass

    @abstractmethod
    def update_child_parameter(self, parameter: Parameter):
        pass

    @abstractmethod
    def handle_mode_change(self, mode):
        pass

    def update_parameter(self, parameter: Parameter):
        #This function only gets called if all requested parameter updates are
        #validated; no need to check
        match parameter.name:
            case "mode":
                self.handle_mode_change(parameter.value)
            case "action_timeout":
                action_timeout = parameter.value
                self.logger.info(f"Changed to action timeout = {action_timeout} s")
            case "velocity_timeout":
                velocity_timeout = parameter.value
                self.logger.info(f"Changed to velocity timeout = {velocity_timeout} s")
            case "position_tolerance":
                self.logger.info(f"Changed position tolerance to {self.position_tolerance}")
            case n if n in [f"joint_mode.{joint}" for joint in self.body_joints]:
                self.change_joint_mode(n.split(".")[1], parameter.value)

    def change_joint_mode(self, joint, mode):
        #TODO: handle this elegantly (make sure joint is settled before changing modes)
        #e.g., self.set_joint_velocity(joint, 0.0)
        self.logger.warn("Elegant mode changing not implemented, be careful with mode switching!")
        

    def parameter_update_callback(self, parameters: list[Parameter]):
        for p in parameters:
            self.update_parameter(p)
            self.update_child_parameter(p)
                
    # reason = None -> ok to accept parameter
    # found = False -> parameter not settable or does not exist
    def check_common_param(self, parameter: Parameter) -> tuple[bool,str]:
        reason = None
        found = False
     
        #TODO: update all of these, probably add some more to change accelerations on joints
        match parameter.name:
            case "mode":
                found=True
                if parameter.value not in self.control_modes and parameter.value not in self.priority_modes:
                    reason=f"Mode does not exist. (Control modes are {self.control_modes}. Priority modes are {self.priority_modes}.)"
            case "action_timeout":
                found=True
                if parameter.value < 0.0:
                    reason="Timeout cannot be less than 0.0."
            case "default_jog_duration":
                found=True
                if parameter.value < 0.0:
                    reason="Jog duration cannot be less than 0.0."
            case "position_tolerance":
                found=True
                if parameter.value < 0.0:
                    reason="Position tolerance cannot be less than 0.0"
            case n if n in [f"joint_mode.{joint}" for joint in self.body_joints]:
                found = True
                if parameter.value not in self.joint_modes:
                    reason=f"Joint mode must be in {self.joint_modes}." 
            case n if n in [f"joint_limit.{joint}.upper" for joint in self.body_joints]:
                found = True
            case n if n in [f"joint_limit.{joint}.lower" for joint in self.body_joints]:
                found = True
            case _:
                reason=f"Parameter {parameter.name} not mutable or not found."
        return found,reason
        
    def check_parameter_callback(self, parameters: list[Parameter]) -> SetParametersResult:
        #ROS expects parameter callback to atomically accept or reject all changes
        #only return true if ALL changes succeed.  Checking should not have side effects!
        #We only handle parameter changes that pass both parent and child checks
        for p in parameters:
            found_here,reason_here = self.check_common_param(p)
            found_child,reason_child = self.check_child_param(p)
            #self.logger.warning(f"Param: {p.name}, found (here,child): {found_here},{found_child}, reason(here,child): {reason_here},{reason_child}")
            
            if found_here and found_child:
                self.logger.warn(f"Parameter {p.name} exists in Stretch4 API ROS superclass and robot/sim specific subclass.  Subclass should shadow superclass but unexpected behavior may result.")
            if not found_here and not found_child:
                #if not found, both reasons will be not found errors; use the one here for consistency between real and sim.
                return SetParametersResult(successful=False,reason=reason_here)

            if (reason_here and found_here) or (reason_child and found_child):
                reasons = [reason_here,reason_child]
                reasons = filter(lambda x: x is not None, reasons)
                return SetParametersResult(successful=False,reason=f"{' '.join(reasons)} (multiple reasons for rejection are possible)")

        return SetParametersResult(successful=True)

    def base_twist_callback(self, twist:Twist):
        self.logger.info(f"Got request for base twist: x:{twist.linear.x}, y:{twist.linear.y}, theta:{twist.angular.z}") 
        
        mode = self.robot_mode()
        if self.robot_mode() != "active":
            self.logger.warn(f"Cannot send base commands while robot is in mode {mode}.  Must be in mode 'active'")
            return

        self.set_base_velocity(twist.linear.x, twist.linear.y, twist.angular.z)

    @abstractmethod
    def set_base_velocity(self, x, y, theta):
        pass

    def velocity_cmd_callback(self, target: JointState):
        self.logger.info(f"Got velocity command request: names: {target.name} velocities: {target.velocity} position: {target.position} (NB: position not used in velocity command!)")
        current_mode = self.get_parameter('mode').value
        if current_mode != 'active':
            self.logger.warn(f"Cannot send position commands while robot is in mode {current_mode}.  Must be in mode 'active'")
            return
            
        for i in range(len(target.name)):
            self._check_and_set_vel(target.name[i],target.velocity[i])

    @abstractmethod
    def set_joint_velocity(self, joint, target):
        pass

    def _check_and_set_vel(self, joint_name, goal):
        mode = self.get_parameter(f"joint_mode.{joint_name}").value
        if mode != "velocity":
            self.logger.warn(f"Cannot send velocity command to joint {joint_name} while in {mode} mode (must be in 'velocity' mode).")
        else:
            self.set_joint_velocity(joint_name, goal)
        
        
    def _check_and_set_pos(self, joint_name, goal):
        mode = self.get_parameter(f"joint_mode.{joint_name}").value
        if mode != "position":
            self.logger.warn(f"Cannot send position command to joint {joint_name} while in {mode} mode (must be in 'position' mode).")
        else:
            limits = self.get_parameters_by_prefix(f"joint_limit.{joint_name}")
            ul = limits["upper"].value
            ll = limits["lower"].value
                
            if (ul is None or goal <= ul) and (ll is None or goal >= ll):
                # TODO: maybe one last check that the robot's mode hasn't changed
                self.last_position_target[joint_name]=goal
                self.set_joint_position(joint_name, goal)
            else:
                self.logger.warn(f"Cannot send position command to joint {joint_name}: goal pose {goal} outside of joint limits ({ll},{ul}).")
    
    
    def position_cmd_callback(self, target: JointState):
        self.logger.info(f"Got position command request: names: {target.name} velocities: {target.velocity} position: {target.position} (NB: velocity not used in velocity command!)")
        current_mode = self.get_parameter('mode').value
        if current_mode != 'active':
            self.logger.warn(f"Cannot send position commands while robot is in mode {current_mode}.  Must be in mode 'active'")
            return
            
        for i in range(len(target.name)):
            self._check_and_set_pos(target.name[i],target.position[i])

    @abstractmethod
    def set_joint_position(self, joint, target):
        pass

    def joy_callback(self, joy_msg: Joy):
        self.logger.info(f"Got joy message. Buttons: {joy_msg.buttons} Axes: {joy_msg.axes} (this message is throttled to appear at most every 2s)", throttle_duration_sec = 2.0)
        
        current_mode = self.get_parameter('mode').value
        if current_mode != 'teleop':
            self.logger.warn(f"Cannot send joystick commands while robot is in mode {current_mode}.  Must be in mode 'active'")
            return
        
        goal = self.joy_to_joint_cmd(joy_msg)
        for i in range(len(goal.name)):
            joint_name = goal.name[i]
            joint_mode = self.get_parameter(f"joint_mode.{joint_name}").value
            match joint_mode:
                case "position":
                    if len(goal.position) < len(goal.name):
                        self.logger.error(f"Joystick command mapping for position has length {len(goal.position)} (expected length {len(goal.name)} to set target for joint {joint_name} in position control mode)")
                    self._check_and_set_pos(joint_name, goal.position[i])
                case "velocity":
                    if len(goal.velocity) < len(goal.name):
                        self.logger.error(f"Joystick command mapping for velocity has length {len(goal.velocity)} (expected length {len(goal.name)} to set target for joint {joint_name} in veloctiy control mode)")
                    self._check_and_set_vel(joint_name, goal.velocity[i])
                case _:
                    self.logger.warn(f"Joint in unsupported mode {joint_mode}.  Joint must be in position or velocity mode to accept joystick control.  Skipping this joint.")
            
        
        
    @abstractmethod
    def joy_to_joint_cmd(self, joy_msg: Joy) -> JointState:
        #joint state returned must contain ONLY velocity or position
        pass

    @abstractmethod
    def publish_child_info(self):
        pass

    def update_latched_value(self, pub: Publisher, value: Any):
        key = pub.topic_name
        if value == self.last_published_value.get(key):
            return
        msg = pub.msg_type()
        msg.data = value
        pub.publish(msg)
        self.last_published_value[key] = value

    @abstractmethod
    def stop_the_robot_callback(self, request, response):
        pass

    @abstractmethod
    def home_the_robot_callback(self, request, response):
        pass

    @abstractmethod
    def stow_the_robot_callback(self, request, response):
        pass

    @abstractmethod
    def runstop_service_callback(self, request, response):
        pass

    def control_loop(self):
        # Capture driver mode
        current_mode = self.get_parameter('mode').value

        status = self.get_robot_status()
        
        current_time = self.get_clock().now().to_msg()

        # handle odom separately
        odom = self.get_odom(status, current_time)

        broadcast_odom_tf=self.get_parameter("broadcast_odom_tf").value
        if broadcast_odom_tf:
            # publish odometry via TF
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = odom.header.frame_id
            t.child_frame_id = odom.child_frame_id
            t.transform.translation.x = odom.pose.pose.position.x
            t.transform.translation.y = odom.pose.pose.position.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = odom.pose.pose.orientation.x
            t.transform.rotation.y = odom.pose.pose.orientation.y
            t.transform.rotation.z = odom.pose.pose.orientation.z
            t.transform.rotation.w = odom.pose.pose.orientation.w
            self.tf_broadcaster.sendTransform(t)

        self.odom_pub.publish(odom)

        # handle joint state separately to enable internal tracking
        joint_state = self.get_joint_state(status, current_time)
        self.last_known_state = dict(zip(joint_state.name, joint_state.position))
        self.joint_state_pub.publish(joint_state)
        
        for (pub, msg_cb) in self.latched_publishers:
            message = msg_cb(status, current_time)
            if message is not None:
                self.update_latched_value(pub, message)
            else:
                self.logger.warn(f"Function {msg_cb.__name__} for latched publisher returned None; skipping latched value update. (This warning is throttled to appear at most every 2s)", throttle_duration_sec=2)

        for (pub, msg_cb) in self.unlatched_publishers:
            message = msg_cb(status, current_time)
            if message:
                pub.publish(message)
            else:
                self.logger.warn(f"Function {msg_cb.__name__} for unlatched publisher returned None; skipping publishing. (This warning is throttled to appear at most every 2s)", throttle_duration_sec=2)
        
        self.publish_child_info()
        self.push_robot_command()
        
        pass

    @abstractmethod
    def setup_trajectory_action(self):
        pass

    @abstractmethod
    def push_robot_command(self):
        pass

    @abstractmethod
    def get_robot_status(self):
        pass
    
    @abstractmethod
    def get_odom(self, robot_status, status_time) -> Odometry:
        pass

    @abstractmethod
    def get_homed(self, robot_status, status_time) -> Bool:
        pass

    @abstractmethod
    def get_mode(self, robot_status, status_time) -> String:
        pass

    @abstractmethod                                    
    def get_tool(self, robot_status, status_time) -> String:
        pass

    @abstractmethod                                    
    def get_sensitivity(self, robot_status, status_time) -> String:
        pass

    @abstractmethod                                    
    def get_runstop(self, robot_status, status_time) -> Bool:
        pass
    
    @abstractmethod                                    
    def get_joint_state(self, robot_status, status_time) -> JointState:
        pass

    @abstractmethod                                    
    def get_battery(self, robot_status, status_time) -> BatteryState:
        pass

    @abstractmethod                                    
    def get_diagnostics(self, robot_status, status_time) -> DiagnosticArray:
        pass

    @abstractmethod                                    
    def get_lease_holder(self, robot_status, status_time) -> DiagnosticStatus:
        pass

    @abstractmethod
    def get_joint_state_diagnostics(self, robot_status, status_time) -> DiagnosticArray:
        pass
    
    @abstractmethod
    def get_safety_diagnostics(self, robot_status, status_time) -> DiagnosticArray:
        pass
    
