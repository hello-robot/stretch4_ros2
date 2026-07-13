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

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from sensor_msgs.msg import BatteryState, JointState, Joy
from std_msgs.msg import Bool, String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from control_msgs.msg import JointJog

import tf2_ros
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .joint_trajectory_server import JointTrajectoryAction

class Stretch4ROSDriver(Node, ABC):

    def __init__(self,name):
        super().__init__(name)
        self.node_name = self.get_name()
        self.logger = self.get_logger()
        self.logger.info("For use with S T R E T C H (TM) RESEARCH EDITION from Hello Robot Inc.")
        
        self.logger.info("{0} started".format(self.node_name))

        self.control_modes = ['position', 'velocity', 'navigation', 'teleop']
        #note: sim robot also has "gamepad" mode, but it's not clear if it's used
        self.default_mode = "navigation"
        self.priority_modes = ['homing', 'stowing', 'runstopped']
        
        self.declare_common_params()
        self.declare_node_params()

        # Runstop management
        self.prev_runstop_state = None
        self.prerunstop_mode = None

        # Callback groups
        self.main_group = ReentrantCallbackGroup()
        self.mutex_group = MutuallyExclusiveCallbackGroup()

        self.info_publishers = {}
        self.latched_publishers = []

        self.setup_common_pubs()
        self.setup_common_subs()
        self.setup_common_srvs()


        # Set up robot state & lock for thread safety
        # Keys for common pubs set up here but child
        # classes could add their own keys/publishers
        self.pubs_state = {name: None for name in self.info_publishers}
        self.last_published_value={}
        self.state_lock = Lock()
        
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
        self.driver_mode = mode
        self.driver_mode_lock = Lock()
        self.logger.info('mode = ' + str(mode))
        
        self.sensitivity = self.get_parameter('sensitivity').value
        self.logger.info('sensitivity = ' + str(self.sensitivity))
        
        self.broadcast_odom_tf = self.get_parameter('broadcast_odom_tf').value
        self.logger.info(f'broadcast_odom_tf = {self.broadcast_odom_tf}')

        action_timeout = self.get_parameter('action_timeout').value
        self.action_timeout_duration = Duration(seconds=action_timeout)
        self.logger.info(f"action timeout = {action_timeout} s")
        
        velocity_timeout = self.get_parameter('velocity_timeout').value
        self.velocity_timeout_duration = Duration(seconds=velocity_timeout)
        self.logger.info(f"velocity timeout = {velocity_timeout} s")

        
        self.add_on_set_parameters_callback(self.parameter_callback)


        self.status_rate = self.get_parameter('status_update_rate').value
        self.control_rate = self.get_parameter('control_loop_rate').value


    def start(self): #called by subclasses once setup for control & status loops are done
        # Start timer for common publishers to publish status
        self.status_timer = self.create_timer(
            1.0 / self.status_rate,
            self.status_loop,
            callback_group=self.mutex_group,
        )

        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop,
            callback_group=self.mutex_group,
        )
        
        
    @abstractmethod
    def declare_node_params(self):
        pass

    def declare_common_params(self):
        self.declare_parameter('mode',self.default_mode)
        self.declare_parameter('sensitivity','default')
        self.declare_parameter('broadcast_odom_tf', False) # based on wheel odometry
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

        self.declare_parameter('status_update_rate', 100, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Target rate (hz) for robot status publishers',
        ))

    def setup_common_pubs(self):
        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.info_publishers["odom"] = self.create_publisher(Odometry, 'wheel_odom', 1)
                
        self.info_publishers["homed"] = self.create_publisher(Bool, 'is_homed', latching_qos)
        self.latched_publishers.append("homed")

        self.info_publishers["mode"] = self.create_publisher(String, 'mode', latching_qos)
        self.latched_publishers.append("mode")
        
        self.info_publishers["tool"] = self.create_publisher(String, 'tool', latching_qos)
        self.latched_publishers.append("tool")
                
        self.info_publishers["sensitivity"] = self.create_publisher(String, 'sensitivity', latching_qos)
        self.latched_publishers.append("sensitivity")

        self.info_publishers["runstop_event"] = self.create_publisher(Bool, 'is_runstopped', latching_qos)
        self.latched_publishers.append("runstop_event")
        
        self.info_publishers["joint_state"]= self.create_publisher(JointState, 'joint_states', 1)
        self.info_publishers["battery"] = self.create_publisher(BatteryState, 'battery', 1)
        self.info_publishers["diagnostics"]= self.create_publisher(DiagnosticArray, '/diagnostics', 1) # Diagnostics are centralized, so we publish to a single global /diagnostics topic
        self.info_publishers["lease_holder"]= self.create_publisher(DiagnosticStatus, 'server_lease_holder', 1)
        self.info_publishers["joint_state_diagnostics"] = self.create_publisher(DiagnosticArray, 'joint_states_diagnostics', 1)
    
        # Saved Message States (for latched topics)
        self.last_published_value = {}

    def setup_common_subs(self):
        # Subscribers
        self.create_subscription(Twist, "cmd_vel", self.twist_callback, 1, callback_group=self.main_group)
        self.create_subscription(JointJog, "joint_vel", self.velocity_callback, 1, callback_group=self.main_group)
        self.create_subscription(Joy, "joy", self.joy_callback, 1, callback_group=self.main_group)

    def setup_common_srvs(self):
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
    def set_node_param(self, parameter: Parameter) -> bool:
        pass

    @abstractmethod
    def change_mode(self, mode):
        pass

    def set_common_param(self, parameter: Parameter) -> bool:
        updated = False
        match parameter.name:
            case "mode":
                self.change_mode(parameter.value)
                updated = True
            case "action_timeout":
                action_timeout = parameter.value
                self.action_timeout_duration = Duration(seconds=action_timeout)
                self.logger.info(f"Changed to action timeout = {action_timeout} s")
                updated = True
            case "velocity_timeout":
                velocity_timeout = parameter.value
                self.velocity_timeout_duration = Duration(seconds=velocity_timeout)
                self.logger.info(f"Changed to velocity timeout = {velocity_timeout} s")
                updated = True
        return updated
        
    def parameter_callback(self, parameters: list[Parameter]) -> SetParametersResult:
        for p in parameters:
            set_here = self.set_common_param(p)
            set_in_child = self.set_child_param(p)
            if set_here and set_in_child:
                self.logger.warn(f"Parameter {p.name} set in Stretch4 API ROS superclass and robot/sim specific subclass.  Subclass should shadow superclass but unexpected behavior may result.")

            if not set_here and not set_in_child:
                self.logger.warn(f"Parameter {p.name} not changed because neither the robot/sim specific subclass nor the Stretch4 ROS API superclass handled the change.  This parameter may not be intended to be mutable.")
                return SetParametersResult(successful=False)
            
            return SetParametersResult(successful=True)

    @abstractmethod
    def twist_callback(self, twist:Twist):
        pass

    @abstractmethod
    def velocity_callback(self, jointjog_msg: JointJog):
        pass

    @abstractmethod
    def joy_callback(self, joy_msg: Joy):
        pass

    @abstractmethod
    def publish_child_info(self):
        pass

    def update_latched_value(self, pub: Publisher, value: Any):
        key = pub.topic
        if value == self.last_published_value.get(key):
            return
        msg = pub.msg_type()
        msg.data = value
        pub.publish(msg)
        self.last_published_value[key] = value

    def status_loop(self):       
        with self.state_lock:
            for name,publisher in self.info_publishers.items():                
                if self.pubs_state[name]:
                    if name in self.latched_publishers:
                        self.update_latched_value(publisher, self.pubs_state[name])
                    else:
                        publisher.publish(self.pubs_state[name])

        self.publish_child_info()

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

    @abstractmethod
    def control_loop(self):
        pass
