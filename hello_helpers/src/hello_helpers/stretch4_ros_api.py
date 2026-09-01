#! /usr/bin/env python3

from abc import ABC, abstractmethod
from typing import Any
import threading
from threading import Lock
import time
import copy
import numpy as np

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import rclpy
from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.publisher import Publisher
from rclpy.exceptions import ParameterNotDeclaredException

from std_srvs.srv import Trigger, SetBool

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from sensor_msgs.msg import BatteryState, JointState, Joy
from std_msgs.msg import Bool, String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import builtin_interfaces

import tf2_ros
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from hello_helpers.joy_conversion import (
    unpack_joy_to_gamepad_state,
    unpack_gamepad_state_to_joy,
    get_default_joy_msg,
)


class Stretch4ROSDriver(Node, ABC):
    command_joints = None
    joint_modes = ["position", "velocity", "settling"]
    velocity_joints = None
    
    def __init__(self,name):
        super().__init__(name)
        self.startup_robot()
        
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
        self._setup_common_actions()

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
            self.set_parameters([Parameter("mode",Parameter.Type.STRING,self.default_mode)])
            mode = self.get_parameter("mode").value
            
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
        self.last_known_state = None
        self.last_joint_mode = {joint: "position" for joint in self.command_joints}
        self.position_tolerance = self.get_parameter('position_tolerance').value

        
        self.trajectory_command_active = threading.Event()
        
    def start(self): #called by subclasses once setup for control loop is done
        self.control_timer = self.create_timer(
            1.0 / self.get_parameter('control_loop_rate').value,
            self.control_loop,
            callback_group=self.mutex_group,
        )
        
    @abstractmethod
    def declare_node_params(self):
        #happens immediately after declaring superclass (this) params
        pass

    @abstractmethod
    def startup_robot(self):
        #happens after node setup but before everything else
        pass
    
    def _declare_common_params(self):
        self.declare_parameter('mode',self.default_mode)
        self.declare_parameter('sensitivity','default')
        self.declare_parameter('broadcast_odom_tf', False) # based on wheel odometry


        desc = ParameterDescriptor(
            dynamic_typing = True,
            description='Joint properties',
        )
        
        for joint in self.command_joints:
            self.declare_parameter(f"joint_acceleration.{joint}",None, desc)
            self.declare_parameter(f"joint_limit.{joint}.upper",None, desc)
            self.declare_parameter(f"joint_limit.{joint}.lower",None, desc)
            self.declare_parameter(f"joint_limit.{joint}.velocity",None, desc)
            self.declare_parameter(f"joint_limit.{joint}.acceleration",None, desc)
            self.declare_parameter(f"joint_mode.{joint}","position", ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description=f"Control mode for individual joint (valid options are: {self.joint_modes})"))
            #default to position control mode
            
        self.declare_parameter("joint_acceleration.omnibase.linear", None, desc)
        self.declare_parameter("joint_acceleration.omnibase.angular", None, desc)

        self.declare_parameter("joint_limit.omnibase.linear.acceleration", None, desc)
        self.declare_parameter("joint_limit.omnibase.angular.acceleration", None, desc)

        
        self.declare_parameter('action_timeout', 3.0, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Default timeout (sec) for execution of joint traj action',
        ))
        self.declare_parameter('velocity_timeout', 0.5, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Default timeout (sec) for velocity control',
        ))
        
        self.declare_parameter('control_loop_rate', 500, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Target rate (hz) for main control loop',
        ))


        self.declare_parameter('position_tolerance', 0.01, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Tolerance on joint position (rad) for position commands',
        ))

        self.declare_parameter('settling.enable', False, ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description='Whether to settle joint velocities before changing control modes',
        ))
        self.declare_parameter('settling.timeout', 2.0, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Maximum duration (sec) to wait for joint velocity to settle',
        ))
        self.declare_parameter('settling.vel_threshold', 0.01, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Velocity threshold (rad/s or m/s) below which a joint is considered settled',
        ))
        # Gamepad parameters
        self.declare_parameter("gamepad.dt", 0.5)
        for joint in self.command_joints:
            self.declare_parameter(f"gamepad.max_vel.{joint}", 0.5)
            self.declare_parameter(f"gamepad.deadzone.{joint}", 0.0)

    

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
        # handle mode separately; do not add to arrays of regular publishers
        
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

    def _setup_common_actions(self):
        self.trajectory_server = StretchTrajectoryActionServer(self)

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

    # This function is provided as a convenience to access the current mode.
    # Mode must be a parameter so that internal and external access in ROS remain
    # synchronized
    def robot_mode(self):
        return self.get_parameter('mode').value
    
    # This function is provided as a convenience to handle the case
    # where something happens outside the robot and the mode needs to change
    # if require_success = True, this function will throw an error if the
    # mode change fails.  Otherwise it will return a boolean with whether the
    # mode change succeeded.
    def change_mode(self, mode, require_success = False):
        mode_param = Parameter("mode",Parameter.Type.STRING,mode)
        result = self.set_parameters([mode_param])[0]
        if not result.successful and require_success:
            raise RuntimeError(f"Failed to change mode parameter to {mode} when strict success checking is enabled. Reason: {result.reason}")
        return result.successful
    
    def update_parameter(self, parameter: Parameter):
        #This function only gets called if all requested parameter updates are
        #validated; no need to check, and no need to handle any parameters that
        #don't require additional processing. Note that at this point the value of
        # the parameter that will return from get_parameter has already changed.
        match parameter.name:
            case "mode":
                self.handle_mode_change(parameter.value)
            case "velocity_timeout":
                velocity_timeout = parameter.value
                self.logger.info(f"Changed to velocity timeout = {velocity_timeout} s")
            case n if n in [f"joint_mode.{joint}" for joint in self.command_joints]:
                self.change_joint_mode(n.split(".")[1], parameter.value)

    def change_joint_mode(self, joint, mode):
        prev_mode = self.last_joint_mode.get(joint, "position")
        if prev_mode == mode:
            return
        
        # 1. Handle incoming 'settling' mode
        if mode == "settling":
            if prev_mode not in ["position", "velocity"]:
                self.logger.warning(f"Joint {joint} unexpectedly set to 'settling' mode from mode '{prev_mode}'.")
            self.last_joint_mode[joint] = "settling"
            return
            
        # 2. Handle unexpected transitions from 'settling'
        if prev_mode == "settling" and mode not in ["position", "velocity"]:
            self.logger.warning(f"Joint {joint} is transitioning from 'settling' mode to unexpected mode '{mode}'.")

        # 3. Check transition position <-> velocity
        if (mode in ["position", "velocity"]) and (prev_mode in ["position", "velocity"]):
            vel_threshold = self.get_parameter("settling.vel_threshold").value
            
            # Fetch current velocity to check movement status
            current_vel = None
            if self.last_known_state is not None:
                current_vel = self.command_joint_vel_from_joint_state(joint, self.last_known_state)
            
            if self.get_parameter("settling.enable").value:
                self.logger.info(f"Initiating settling transition for {joint} from {prev_mode} to {mode}...")
                
                # Mark as settling to intercept nested calls
                self.last_joint_mode[joint] = "settling"
                
                # Set parameter to "settling"
                settling_param = Parameter(f"joint_mode.{joint}", Parameter.Type.STRING, "settling")
                self.set_parameters([settling_param])
                
                # Command a zero velocity command to halt movement
                self.set_joint_velocity(joint, 0.0)
                
                # Spin/wait for velocity to settle below threshold
                timeout = self.get_parameter("settling.timeout").value
                start_time = self.get_clock().now()
                settled = False
                last_vel = None
                
                while (self.get_clock().now() - start_time).nanoseconds * 1e-9 < timeout:
                    status = self.get_robot_status()
                    current_time = self.get_clock().now().to_msg()
                    joint_state = self.get_joint_state(status, current_time)
                    
                    last_vel = self.command_joint_vel_from_joint_state(joint, joint_state)
                    if last_vel is not None and abs(last_vel) < vel_threshold:
                        settled = True
                        break
                    time.sleep(0.01)
                    
                if settled:
                    self.logger.info(f"Joint {joint} settled successfully (vel: {last_vel:.4f} < {vel_threshold:.4f})")
                else:
                    self.logger.warning(f"Joint {joint} settling timed out after {timeout} seconds! Final velocity: {last_vel} (threshold: {vel_threshold})")
                
                # Maintain 'settling' state until parameter update has finished
                self.last_joint_mode[joint] = "settling"
                final_param = Parameter(f"joint_mode.{joint}", Parameter.Type.STRING, mode)
                self.set_parameters([final_param])
                self.last_joint_mode[joint] = mode
                return
            else:
                # Settling not enabled: warn if switching modes while joint is still moving
                if current_vel is not None and abs(current_vel) > vel_threshold:
                    self.logger.warning(
                        f"Changing joint mode for {joint} from {prev_mode} to {mode} while joint is still moving!"
                        f"Current velocity: {current_vel:.4f} (threshold: {vel_threshold:.4f}) and settling is not enabled."
                    )
                    self.set_joint_velocity(joint, 0.0)
                elif current_vel is None:
                    self.logger.warning(f"Changing joint mode for {joint} from {prev_mode} to {mode} but current joint velocity is unknown.")

        self.last_joint_mode[joint] = mode
        

    def parameter_update_callback(self, parameters: list[Parameter]):
        for p in parameters:
            self.update_parameter(p)
            self.update_child_parameter(p)
                
    # reason = None -> ok to accept parameter
    # found = False -> parameter not settable or does not exist
    def check_common_param(self, parameter: Parameter) -> tuple[bool,str]:
        reason = None
        found = False
     
        match parameter.name:
            case "mode":
                found=True
                if parameter.value not in self.control_modes and parameter.value not in self.priority_modes:
                    reason=f"Mode does not exist. (Control modes are {self.control_modes}. Priority modes are {self.priority_modes}.)"
            case "action_timeout":
                found=True
                if parameter.value < 0.0:
                    reason="Timeout cannot be less than 0.0."
            case "velocity_timeout":
                found = True
                if parameter.value < 0.0:
                    reason = "velocity_timeout cannot be less than 0.0."
            case "position_tolerance":
                found = True
                if parameter.value < 0.0:
                    reason = "position_tolerance cannot be less than 0.0."
            case "settling.enable":
                found = True
                if not isinstance(parameter.value, bool):
                    reason = "settling.enable must be a boolean."
            case "settling.timeout":
                found = True
                if parameter.value < 0.0:
                    reason = "settling.timeout cannot be less than 0.0."
            case "settling.vel_threshold":
                found = True
                if parameter.value < 0.0:
                    reason = "settling.vel_threshold cannot be less than 0.0."
            case n if n in [f"joint_mode.{joint}" for joint in self.command_joints]:
                found = True
                joint_name = n.split(".")[1]
                if parameter.value not in self.joint_modes:
                    reason=f"Joint mode must be in {self.joint_modes}."
                if parameter.value == "velocity" and joint_name not in self.velocity_joints:
                    reason=f"Velocity control not currently available for joint {joint_name}."
            case n if n in [f"joint_limit.{joint}.upper" for joint in self.command_joints]:
                found = True
            case n if n in [f"joint_limit.{joint}.lower" for joint in self.command_joints]:
                found = True
            case n if n in [f"joint_limit.{joint}.velocity" for joint in self.command_joints]:
                found = True
                if parameter.value is not None and parameter.value < 0.0:
                    reason = "Joint velocity limit cannot be less than 0.0."
            case n if "trajectory_server." in n:
                found = True
            # --- Read-Only/Static Parameters ---
            case "control_loop_rate" | "broadcast_odom_tf" | "sensitivity":
                found = True
                reason = f"Parameter '{parameter.name}' is read-only after startup."
            case n if n in [f"joint_acceleration.{joint}" for joint in self.command_joints] or \
                      n in ["joint_acceleration.omnibase.linear", "joint_acceleration.omnibase.angular"]:
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
            self.logger.warn(f"Cannot send velocity commands while robot is in mode {current_mode}.  Must be in mode 'active'")
            return
            
        for i in range(len(target.name)):
            self.check_and_set_vel(target.name[i],target.velocity[i])

    @abstractmethod
    def set_joint_velocity(self, joint, target, v = None, a = None):
        pass

    def check_and_set_vel(self, joint_name, goal, a = None):
        #self.logger.info(f"Setting joint {joint_name} to velocity {goal}")
                
        if (self.trajectory_server is not None and 
            self.trajectory_server.active_joints is not None and 
            joint_name in self.trajectory_server.active_joints and 
            not self.trajectory_command_active.is_set()):
            self.trajectory_server.direct_command_preempted = True

        try:
            mode = self.get_parameter(f"joint_mode.{joint_name}").value
        except ParameterNotDeclaredException:
            self.logger.error(f"Joint name {joint_name} not found in mode parameters.  Joint name is probably incorrect.")
            mode = "<< joint unknown >>"
            
        succeeded = False
        if mode != "velocity":
            self.logger.warn(f"Cannot send velocity command to joint {joint_name} while in {mode} mode (must be in 'velocity' mode).")
        else:
            robot_mode = self.get_parameter('mode').value
            if robot_mode not in ["active", "teleop"]:
                self.logger.warn(f"Cannot send velocity command to joint {joint_name} because robot mode is {robot_mode} (must be 'active' or 'teleop').")
            else:
                try:
                    vel_limit = self.get_parameter(f"joint_limit.{joint_name}.velocity").value
                except Exception:
                    vel_limit = None
                
                if vel_limit is not None and abs(goal) > vel_limit:
                    self.logger.warn(f"Cannot send velocity command to joint {joint_name}: goal velocity {goal} exceeds limit ({vel_limit}).")
                else:
                    self.set_joint_velocity(joint_name, goal, a = a)
                    succeeded = True
        return succeeded
        
        
    def check_and_set_pos(self, joint_name, goal, v = None, a = None):
        #self.logger.info(f"Setting joint {joint_name} to position {goal}.")
        
        if (self.trajectory_server is not None and 
            self.trajectory_server.active_joints is not None and 
            joint_name in self.trajectory_server.active_joints and 
            not self.trajectory_command_active.is_set()):
            self.trajectory_server.direct_command_preempted = True

        try:
            mode = self.get_parameter(f"joint_mode.{joint_name}").value
        except ParameterNotDeclaredException:
            self.logger.error(f"Joint name {joint_name} not found in mode parameters.  Joint name is probably incorrect.")
            mode = "<< joint unknown >>"

        succeeded = False
        if mode != "position":
            self.logger.warn(f"Cannot send position command to joint {joint_name} while in {mode} mode (must be in 'position' mode).")
        else:
            limits = self.get_parameters_by_prefix(f"joint_limit.{joint_name}")
            ul = limits["upper"].value
            ll = limits["lower"].value
                
            if (ul is None or goal <= ul) and (ll is None or goal >= ll):
                robot_mode = self.get_parameter('mode').value
                if robot_mode not in ["active","teleop"]:
                    self.logger.warn(f"Cannot send position command to joint {joint_name} because robot mode is {robot_mode} (must be 'active' or 'teleop').")
                else:
                    self.last_position_target[joint_name]=goal
                    self.set_joint_position(joint_name, goal, v=v, a=a)
                    succeeded = True
            else:
                self.logger.warn(f"Cannot send position command to joint {joint_name}: goal pose {goal} outside of joint limits ({ll},{ul}).")
        return succeeded
    
    def position_cmd_callback(self, target: JointState):
        self.logger.info(f"Got position command request: names: {target.name} velocities: {target.velocity} position: {target.position} (NB: velocity not used in velocity command!)")
        current_mode = self.get_parameter('mode').value
        if current_mode != 'active':
            self.logger.warn(f"Cannot send position commands while robot is in mode {current_mode}.  Must be in mode 'active'")
            return
            
        for i in range(len(target.name)):
            self.check_and_set_pos(target.name[i],target.position[i])

    @abstractmethod
    def set_joint_position(self, joint, target, v, a):
        pass

    def cmd_joint_position(self, command_joint_name):
        if self.last_known_state is None:
            self.logger.warning(f"Last known state is None, unable to find position of command joint {command_joint_name}")
            return None
        val = self.command_joint_pose_from_joint_state(command_joint_name, self.last_known_state, default=None)
        if val is None:
            self.logger.warning(f"Last known state exists but unable to find position of command joint {command_joint_name}, returning None")
        return val

    def cmd_joint_velocity(self, command_joint_name):
        if self.last_known_state is None:
            self.logger.warning(f"Last known state is None, unable to find velocity of command joint {command_joint_name}")
            return None
        val = self.command_joint_vel_from_joint_state(command_joint_name, self.last_known_state, default=None)
        if val is None:
            self.logger.warning(f"Last known state exists but unable to find velocity of command joint {command_joint_name}, returning None")
        return val


    def joy_callback(self, joy_msg: Joy):
        self.logger.info(f"Got joy message. Buttons: {joy_msg.buttons} Axes: {joy_msg.axes} (this message is throttled to appear at most every 2s)", throttle_duration_sec = 2.0)
        
        current_mode = self.get_parameter('mode').value
        if current_mode != 'teleop':
            self.logger.warn(f"Cannot send joystick commands while robot is in mode {current_mode}.  Must be in mode 'teleop'")
            return
        
        goal = self.joy_to_joint_cmd(joy_msg)
        if goal is None:
            self.logger.error(f"Joystick mapping returned None.  Check your code.")
            return

        for i in range(len(goal.name)):
            joint_name = goal.name[i]
            joint_mode = self.get_parameter(f"joint_mode.{joint_name}").value
            match joint_mode:
                case "position":
                    if len(goal.position) < len(goal.name):
                        self.logger.error(f"Joystick command mapping for position has length {len(goal.position)} (expected length {len(goal.name)} to set target for joint {joint_name} in position control mode)")
                    else:
                        self.check_and_set_pos(joint_name, goal.position[i])
                case "velocity":
                    if len(goal.velocity) < len(goal.name):
                        self.logger.error(f"Joystick command mapping for velocity has length {len(goal.velocity)} (expected length {len(goal.name)} to set target for joint {joint_name} in velocity control mode)")
                    else:
                        self.check_and_set_vel(joint_name, goal.velocity[i])
                case _:
                    self.logger.warn(f"Joint in unsupported mode {joint_mode}.  Joint must be in position or velocity mode to accept joystick control.  Skipping this joint.")
            
        
    # optionally re-implement this to get different joy behavior
    def joy_to_joint_cmd(self, joy):
        state = unpack_joy_to_gamepad_state(joy)
        
        # Read parameters dynamically
        dt = self.get_parameter("gamepad.dt").value
        
        def get_val(axis_name, joint_name):
            val = state.get(axis_name, 0.0)
            max_vel = self.get_parameter(f"gamepad.max_vel.{joint_name}").value
            deadzone = self.get_parameter(f"gamepad.deadzone.{joint_name}").value
            
            if val > deadzone:
                effective = (val - deadzone) / (1.0 - deadzone)
                return effective * max_vel
            elif val < -deadzone:
                effective = (val + deadzone) / (1.0 - deadzone)
                return effective * max_vel
            else:
                return 0.0

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
                goal.position.append(new_pos)

        #self.logger.info(f"goal: {goal}")
        return goal

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
        self.logger.info(f"Control loop active. Mode is {self.robot_mode()}", throttle_duration_sec=5.0)
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
        self.last_known_state = joint_state
        self.joint_state_pub.publish(joint_state)

        mode = self.get_mode(status, current_time)
        if mode != self.robot_mode():
            # this will raise a RuntimeError if the mode change fails
            # so the driver will crash rather than running with the mode
            # parameter not matching the true robot mode
            self.change_mode(mode, require_success=True)
            self.mode_pub.publish(mode)
        
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

    #this function is needed because the published joint state splits up the
    #arm and uses different names than the joint names used to send commands
        
    @abstractmethod
    def command_joint_pose_from_joint_state(self, command_joint, joint_state, default=None):
        pass

    @abstractmethod
    def command_joint_vel_from_joint_state(self, command_joint, joint_state, default=None):
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
    

class StretchTrajectoryActionServer:
    def __init__(self, driver: Stretch4ROSDriver):
        # Store the reference to the parent node
        self.driver = driver

        self.active_joints = None
        self.direct_command_preempted = False

        self.param_prefix = "trajectory_server"
        self.declare_params()

        self.modes = ["adaptive_velocity",
                      "target_priority",
                      "time_priority"]
        # Modes are as follows:
        
        # adaptive_velocity = adjust velocity to hit positions at the correct tmies

        # target_priority = hit every position (or velocity) on the list;
        # requires timeout. No guarantee of hitting x(t) at time t,
        # but will not move to next point until x(t) is hit within a threshold

        # time_priority = send position (or veloctiy) request for x(t) at
        # time t, regardless
        # of whether you reach that point.  No guarantee of hitting x(t) at
        # time t, and no guarantee of hitting x(t) at all. takes optional
        # offset parameter that adjusts target for all times by offset 
        

        
        self._action_server = ActionServer(
            self.driver, # Bind the action server to the parent node
            FollowJointTrajectory,
            'follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.driver.main_group
        )

        qos_profile = 10
        
        self.pub_actual_state = self.driver.create_publisher(
            JointState, "/trajectory_server_diagnostics/actual", qos_profile
        )

        # Publisher for desired joint state diagnostics
        self.pub_desired_state = self.driver.create_publisher(
            JointState, "/trajectory_server_diagnostics/desired", qos_profile
        )

        self.pub_commands = self.driver.create_publisher(
            JointState, "/trajectory_server_diagnostics/commands", qos_profile
        )

        
        self.driver.get_logger().info('FollowJointTrajectory Action Server has been initialized.')

    def declare_params(self):
        def add_param(name,value):
            self.driver.declare_parameter(f"{self.param_prefix}.{name}", value)

        add_param("mode", "target_priority")
        add_param("strict_mode", True)
        add_param("timeout", 5.0)
        add_param("kp", 0.1)
        add_param("ki", 0.001)
        add_param("kd", 0.01)
        add_param("threshold", 0.05)
        add_param("loop_rate", 50.0)
        # if velocity not specified, how to do interpolation
        # options are 'zero' (stop between waypoints) or 'smooth'
        # (average slope between prev and next points)
        add_param("velocity_inference", "smooth")

    def get_param(self, short_name):
        return self.driver.get_parameter(f"{self.param_prefix}.{short_name}").value

    def goal_callback(self, goal_request):
        """Accept or reject a new goal request."""
        self.driver.get_logger().info('Received goal request.')
        # You can add logic here to reject invalid trajectories before execution
        return GoalResponse.ACCEPT

    def cancel_callback(self, cancel_request):
        """Accept or reject a request to cancel the current goal."""
        self.driver.get_logger().info('Received cancel request.')
        
        return CancelResponse.ACCEPT


    def _check_for_interrupt(self, goal_handle):
        # 1. Check for interrupts / cancellation requests
        result = FollowJointTrajectory.Result()
        if goal_handle.is_cancel_requested:
            self.active_joints = None
            goal_handle.canceled()
            self.driver.get_logger().warn('Goal was canceled by the client.')
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = "Canceled"
            return result

        # Check robot mode
        if self.driver.robot_mode() != "active":
            self.active_joints = None
            self.driver.get_logger().warn(f"Goal canceled because robot mode is {self.driver.robot_mode()} (must be 'active').")
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = f"Robot mode changed to {self.driver.robot_mode()}"
            return result

        # Check direct joint command preemption
        if self.direct_command_preempted:
            self.active_joints = None
            self.driver.get_logger().warn("Goal canceled because a direct command was sent to one of the commanded joints.")
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = "Direct command preempted trajectory"
            return result

    def _get_feedback(self, point, elapsed_sec, joint_names):
        feedback_msg = FollowJointTrajectory.Feedback()
        feedback_msg.joint_names = joint_names
        feedback_msg.actual = JointTrajectoryPoint()
        feedback_msg.desired = copy.deepcopy(point)

        actual_positions = []
        actual_velocities = []
        for joint_name in joint_names:
            joint_pose = self.driver.cmd_joint_position(joint_name)
            joint_vel = self.driver.cmd_joint_velocity(joint_name)
            if joint_pose is None:
                self.driver.logger.error(f"Joint {joint_name} has unknown position; returning 0.0 for current joint pose.")
                joint_pose = 0.0
            if joint_vel is None:
                self.driver.logger.error(f"Joint {joint_name} has unknown velocity; returning 0.0 for current joint velocity.")
                joint_vel = 0.0

            actual_positions.append(joint_pose)
            actual_velocities.append(joint_vel)
            
        feedback_msg.actual.positions = actual_positions
        feedback_msg.actual.velocities = actual_velocities
        feedback_msg.actual.time_from_start = Duration(seconds=elapsed_sec).to_msg()
        return feedback_msg

    def follow_trajectory(self, next_point_condition, first_loop_command, every_loop_command, trajectory, goal_handle):
        point_i = 0
        start_time = self.driver.get_clock().now()

        prev_state = None

        feedback_msg = self._get_feedback(trajectory.points[0], 0.0, trajectory.joint_names)

        rate = self.driver.create_rate(self.get_param("loop_rate"))
        last_timepoint =self.driver.get_clock().now()
        counter = 0

        
        while point_i < len(trajectory.points):
            active_point = trajectory.points[point_i]
            next_state = trajectory.points[point_i+1] if point_i < len(trajectory.points)-1 else None
            elapsed_time = self.driver.get_clock().now()-start_time
            first = True
            #self.driver.logger.warning(f"On point {point_i}")
        
            while True:
                interrupt_result = self._check_for_interrupt(goal_handle)
                if interrupt_result is not None:
                    return interrupt_result
                
                elapsed_time = self.driver.get_clock().now()-start_time
                
                if first:
                    #self.driver.logger.warning(f"Time: {self.driver.get_clock().now()-last_timepoint}")
                    #last_timepoint =self.driver.get_clock().now()
                    pos, vel, acc = first_loop_command(active_point, prev_state, next_state, feedback_msg)
                    first = False
                else:
                    #self.driver.logger.warning(f"Time: {self.driver.get_clock().now()-last_timepoint}")
                    #last_timepoint =self.driver.get_clock().now()
                    t0 = time.perf_counter_ns()
                    pos, vel, acc = every_loop_command(active_point, prev_state, next_state, feedback_msg)
                    
                #t0 = time.perf_counter_ns()
                command = JointState()
                command.name = trajectory.joint_names
                command.position = pos if pos is not None else []
                command.velocity = vel if vel is not None else []
                command.effort = acc if acc is not None else []

                self.pub_commands.publish(command)
                    
                succeeded = True
                for joint_i, joint_name in enumerate(trajectory.joint_names):
                    if pos is not None:
                        if vel is not None:
                            v = vel[joint_i]
                        else:
                            v = None

                        if acc is not None:
                            a = acc[joint_i]
                        else:
                            a = None
                        
                        succeeded = succeeded and self.driver.check_and_set_pos(joint_name, pos[joint_i], v, a)
                    elif vel is not None:
                        if acc is not None:
                            a = acc[joint_i]
                        else:
                            a = None
                        
                        succeeded = succeeded and self.driver.check_and_set_vel(joint_name, vel[joint_i], a)
                    

                if not succeeded and self.get_param("strict_mode"):
                    self.active_joints = None
                    self.driver.logger.warn("Goal canceled because sending command to a joint failed under strict_mode.")
                    goal_handle.abort()
                    result = FollowJointTrajectory.Result()
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    result.error_string = "Joint command failed in strict mode"
                    return result
                
                feedback_msg = self._get_feedback(active_point, elapsed_time.nanoseconds*1e-9, trajectory.joint_names)
                goal_handle.publish_feedback(feedback_msg)

                feedback_msg.desired.positions = pos if pos is not None and len(pos)>0 else feedback_msg.desired.positions
                feedback_msg.desired.velocities = vel if vel is not None and len(vel)>0 else feedback_msg.desired.velocities
                feedback_msg.desired.accelerations = acc if acc is not None and len(acc)>0 else feedback_msg.desired.accelerations

                actual = JointState()
                desired = JointState()
                
                actual.name = trajectory.joint_names
                actual.position = feedback_msg.actual.positions
                actual.velocity = feedback_msg.actual.velocities

                desired.name = trajectory.joint_names
                desired.position = feedback_msg.desired.positions
                desired.velocity = feedback_msg.desired.velocities

                self.pub_actual_state.publish(actual)
                self.pub_desired_state.publish(desired)

                #if counter % 10 == 0:
                #        print(f"dt: {(time.perf_counter_ns()-t0)/1000.0}")
                        
                if next_point_condition(feedback_msg):
                    #prev_state = copy.deepcopy(feedback_msg.actual)
                    prev_state = copy.deepcopy(feedback_msg.desired)
                    break
                
                rate.sleep()
                            
            while point_i < len(trajectory.points): #zip through points until we get to the first unsatisfied point
                active_point = trajectory.points[point_i]
                elapsed_time = self.driver.get_clock().now()-start_time

                feedback_msg = self._get_feedback(active_point, elapsed_time.nanoseconds*1e-9, trajectory.joint_names)
                interrupt_result = self._check_for_interrupt(goal_handle)
                if interrupt_result is not None:
                    return interrupt_result

                if next_point_condition(feedback_msg):
                    point_i += 1
                else:
                    break
            rate.sleep()
        
                
        # Successful Completion
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Completed trajectory successfully"
        self.driver.get_logger().info('Goal succeeded.')
        return result

    def stop_vel_joints(self, joint_names):
        for joint_name in joint_names:
            try:
                j_mode = self.driver.get_parameter(f"joint_mode.{joint_name}").value
            except Exception:
                j_mode = None
            if j_mode != "velocity":
                self.driver.get_logger().warning(f"Not sending stop command: {joint_name} is in {j_mode} mode (must be in 'velocity' mode to stop after trajectory).")

            else:
                succeeded = self.driver.check_and_set_vel(joint_name, 0.0)
                if not succeeded:
                    self.driver.get_logger().warning(f"Stop command to {joint_name} failed.")

    def at_time(self, feedback_msg):
        actual_time = feedback_msg.actual.time_from_start.sec + feedback_msg.actual.time_from_start.nanosec*1e-9
        desired_time = feedback_msg.desired.time_from_start.sec + feedback_msg.desired.time_from_start.nanosec*1e-9
        
        return actual_time >= desired_time

    def at_target(self, feedback_msg):
        tolerance = self.get_param("threshold")

        all_joints_ok = True
        for i, joint_names in enumerate(feedback_msg.joint_names):
            all_joints_ok = all_joints_ok and abs(feedback_msg.desired.positions[i]-feedback_msg.actual.positions[i])<tolerance

        return all_joints_ok

    def active_point_goal(self, active_point, prev_pt, next_pt, feedback_msg):
        pos = active_point.positions if len(active_point.positions) > 0 else None
        vel = active_point.velocities if len(active_point.velocities) > 0 else None
        acc = active_point.accelerations if len(active_point.accelerations) > 0 else None

        return pos, vel, acc

    def interpolate(self, active_point, prev_state, next_state, feedback_msg):
        now = feedback_msg.actual.time_from_start.sec + feedback_msg.actual.time_from_start.nanosec*1e-9
        end_t = active_point.time_from_start.sec + active_point.time_from_start.nanosec*1e-9
        start_t = prev_state.time_from_start.sec + prev_state.time_from_start.nanosec*1e-9 if prev_state is not None else now
        prop_elapsed = (now-start_t)/(end_t-start_t) if end_t != start_t else 1.0
        secs_remaining = end_t-now
        target_poses = []
        target_vels = []
        target_accs = []

        dt = 1.0/self.get_param("loop_rate")

        '''if end_t - start_t < 3.0*dt:
            #don't do full interpolation if points are very close together
            pos = active_point.positions if len(active_point.positions) > 0 else None
            vel = active_point.velocities if len(active_point.velocities) > 0 else None
            acc = active_point.accelerations if len(active_point.accelerations) > 0 else None

            if vel is None:
                vel = []
                for i, joint in enumerate(feedback_msg.joint_names):
                    if prev_state is not None:
                        vel.append(active_point.positions[i]-prev_state.positions[i]/(end_t-start_t))
                    elif end_t > 0:
                        vel.append(active_point.positions[i]/end_t)
                    else:
                        vel.append(0.0)
            return pos, vel, acc'''
        
        
        for i, goal_pos in enumerate(active_point.positions):
            joint_name = feedback_msg.joint_names[i]
            maxa = self.driver.get_parameter_or(f"joint_limit.{joint_name}.acceleration",None)
            maxv = self.driver.get_parameter_or(f"joint_limit.{joint_name}.velocity",None)

            if maxa is None or maxa.value is None:
                maxa = 12.0
            else:
                maxa = maxa.value

            if maxv is None or maxv.value is None:
                maxv = 12.0
            else:
                maxv = maxv.value
            
            current_pos = feedback_msg.actual.positions[i]
            current_vel = feedback_msg.actual.velocities[i]

            
            if prev_state is None:
                start_vel = 0.0
            elif prev_state.velocities is not None and len(prev_state.velocities) > 0:
                start_vel = prev_state.velocities[i]
            else:
                self.driver.logger.error("Previous state should have velocities; check your code!")
                start_vel=0.0


            slope1 = None
            slope2 = None
            if active_point.velocities is not None and len(active_point.velocities) > 0:
                end_vel = active_point.velocities[i]
            else:
                if self.get_param("velocity_inference") == "zero" or next_state is None:
                    end_vel = 0.0
                elif self.get_param("velocity_inference") == "smooth":
                    d2 = next_state.positions[i] - active_point.positions[i]
                    d1 = active_point.positions[i] - prev_state.positions[i] if prev_state is not None else d2
                    
                    t0 = prev_state.time_from_start.sec + prev_state.time_from_start.nanosec*1e-9 if prev_state is not None else None
                    t1 = active_point.time_from_start.sec + active_point.time_from_start.nanosec*1e-9
                    t2 = next_state.time_from_start.sec + next_state.time_from_start.nanosec*1e-9

                    dt2 = t2-t1
                    dt1 = t1-t0 if t0 is not None else dt2

                    slope1 = (d1)/(dt1) if dt1 != 0.0 else 0.0
                    slope2 = (d2)/(dt2) if dt2 != 0.0 else 0.0

                    print(f"d1: {d1} dt1: {dt1}/d2:{d2} dt2:{dt2}")
                    
                    end_vel = 0.5*(slope1+slope2)                    
                else:
                    self.driver.logger.warning("No end velocity specified and unknown velocity inference method. Setting waypoint target velocity to zero")
                    end_vel = 0.0

                           
            start_pos = prev_state.positions[i] if prev_state is not None else current_pos
            end_pos = active_point.positions[i]
            
            """
            Calculates target velocity and commanded acceleration for the current timestep dt.
            """
            # 1. Determine total segment duration (h) and current normalized time (tau)
            if secs_remaining > 0 and prop_elapsed < 1.0:
                h = secs_remaining / (1.0 - prop_elapsed)
            else:
                h = max(end_t - start_t, 1e-6)

            tau = np.clip(prop_elapsed, 0.0, 1.0)

            # 2. Local Monotonicity Enforcement (Prevents position overshoot between start_pos and end_pos)
            delta = (end_pos - start_pos) / h
            max_monotonic_v = 3.0 * abs(delta)

            v0 = np.clip(start_vel, -max_monotonic_v, max_monotonic_v)
            v1 = np.clip(end_vel, -max_monotonic_v, max_monotonic_v)

            # 3. Cubic Hermite Spline Evaluations
            # Desired Position x_d(tau)
            pos_d = ((2*tau**3 - 3*tau**2 + 1) * start_pos + 
                     (tau**3 - 2*tau**2 + tau) * h * v0 + 
                     (-2*tau**3 + 3*tau**2) * end_pos + 
                     (tau**3 - tau**2) * h * v1)

            # Desired Velocity v_d(tau)
            vel_d = ((6*tau**2 - 6*tau) * start_pos + 
                     (3*tau**2 - 4*tau + 1) * h * v0 + 
                     (-6*tau**2 + 6*tau) * end_pos + 
                     (3*tau**2 - 2*tau) * h * v1) / h

            # Desired Acceleration Feedforward a_d(tau)
            accel_ff = ((12*tau - 6) * start_pos + 
                        (6*tau - 4) * h * v0 + 
                        (-12*tau + 6) * end_pos + 
                        (6*tau - 2) * h * v1) / (h**2)

            # Clamp target velocity to maxv bound
            next_vel = np.clip(vel_d, -maxv, maxv)

            kp = 0.0
            kd = 0.0
            
            # 4. Feedforward + Feedback Command Calculation
            next_acc = accel_ff + kp * (pos_d - current_pos) + kd * (next_vel - current_vel)

            # Enforce maximum acceleration limit maxa
            next_acc = np.clip(next_acc, -maxa, maxa)

            next_pos = current_pos + 0.5 * (current_vel + next_vel) * dt

            target_poses.append(next_pos)
            target_vels.append(next_vel)
            target_accs.append(next_acc)

        return target_poses, target_vels, target_accs

    def interpolate_poses(self, active_point, prev_state, next_state, feedback_msg):
        pos, vel, acc = self.interpolate(active_point, prev_state, next_state, feedback_msg)
        return pos, vel, acc

    def interpolate_velocities(self, active_point, prev_state, next_state, feedback_msg):
        pos, vel, acc = self.interpolate(active_point, prev_state, next_state, feedback_msg)
        return None, vel, acc
    
    
    def execute_callback(self, goal_handle):
        """Execute the trajectory."""
        mode = self.get_param("mode")
        if mode not in self.modes:
            self.driver.get_logger().error(f"Invalid trajectory server mode: {mode}. Must be one of {self.modes}")
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = f"Invalid trajectory server mode: {mode}"
            goal_handle.abort()
            return result

        self.driver.get_logger().info(f'Executing trajectory in {mode} mode (mode cannot be changed during execution)')

        try:
            trajectory = goal_handle.request.trajectory
            match mode:
                case "adaptive_velocity":
                    for joint_name in goal_handle.request.trajectory.joint_names:
                        try:
                            j_mode = self.driver.get_parameter(f"joint_mode.{joint_name}").value
                        except Exception:
                            j_mode = None
                        if j_mode != "velocity":
                            self.driver.get_logger().error(f"Cannot execute trajectory in pid_normal mode because joint {joint_name} is in {j_mode} mode (must be in 'velocity' mode).")
                            result = FollowJointTrajectory.Result()
                            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                            result.error_string = f"Joint {joint_name} is not in velocity mode"
                            goal_handle.abort()
                            return result
                        next_point_condition = self.at_time
                        first_loop_command = self.interpolate_velocities
                        every_loop_command = self.interpolate_velocities
                case "time_priority":
                    next_point_condition = self.at_time
                    first_loop_command = self.active_point_goal
                    every_loop_command = lambda *args, **kwargs: [None, None, None]
                case "target_priority":
                    next_point_condition = self.at_target
                    first_loop_command = self.active_point_goal
                    every_loop_command = lambda *args, **kwargs: [None, None, None]


            result = self.follow_trajectory(next_point_condition, first_loop_command, every_loop_command, trajectory, goal_handle)
        finally:            
            #cleanup:
            self.stop_vel_joints(trajectory.joint_names)
            self.active_joints = None
                
        return result
        
