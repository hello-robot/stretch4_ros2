#! /usr/bin/env python3

from abc import ABC, abstractmethod
from enum import Enum
from typing import Any
import importlib
import threading
from threading import RLock
import time
import copy
import numpy as np
import traceback
from contextlib import contextmanager

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import rclpy
from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, qos_profile_system_default
from rclpy.service_introspection import ServiceIntrospectionState
from rclpy.publisher import Publisher
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.executors import ExternalShutdownException

from std_srvs.srv import Trigger, SetBool

from geometry_msgs.msg import Twist, TwistWithCovarianceStamped, TransformStamped
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


def spin_until_shutdown(executor):
    """Spin `executor` until ROS shuts down (e.g. on SIGINT/SIGTERM), then return.

    Shutdown surfaces as KeyboardInterrupt, ExternalShutdownException, or, if it lands
    while a MultiThreadedExecutor is building its wait set, an RCLError about the
    invalid context; all of these return normally, so a node exits cleanly. Anything
    raised while ROS is still up is a real error and propagates.
    """
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if executor.context.ok():
            raise


class JointMode(str, Enum):
    POSITION = "position"
    VELOCITY = "velocity"
    SETTLING = "settling"

    def __str__(self):
        return self.value


class DriverMode(str, Enum):
    ACTIVE = "active"
    TELEOP = "teleop"
    PUPPET = "puppet"
    HOMING = "homing"
    STOWING = "stowing"
    SEATING = "seating"
    RUNSTOPPED = "runstopped"

    def __str__(self):
        return self.value


class Stretch4ROSDriver(Node, ABC):
    command_joints = None
    joint_modes = [mode.value for mode in JointMode]
    position_mode = JointMode.POSITION
    velocity_mode = JointMode.VELOCITY
    settling_mode = JointMode.SETTLING
    velocity_joints = None

    driver_modes = [mode.value for mode in DriverMode]
    active_mode = DriverMode.ACTIVE
    teleop_mode = DriverMode.TELEOP
    puppet_mode = DriverMode.PUPPET
    homing_mode = DriverMode.HOMING
    stowing_mode = DriverMode.STOWING
    seating_mode = DriverMode.SEATING
    runstopped_mode = DriverMode.RUNSTOPPED

    control_modes = [DriverMode.ACTIVE.value, DriverMode.TELEOP.value, DriverMode.PUPPET.value]
    priority_modes = [DriverMode.HOMING.value, DriverMode.STOWING.value, DriverMode.SEATING.value,
                      DriverMode.RUNSTOPPED.value]
    default_mode = DriverMode.ACTIVE

    DEPRECATED_CONTROL_MODES = {
        "position": (DriverMode.ACTIVE, JointMode.POSITION),
        "navigation": (DriverMode.ACTIVE, JointMode.POSITION),
        "velocity": (DriverMode.ACTIVE, JointMode.VELOCITY),
    }

    # Extra spellings accepted for a command joint on command messages and trajectory
    # goals. Parameters are never aliased -- a parameter name cannot be. The attached
    # tool's own names (its backend name, with and without '_joint', and its URDF
    # joints) are added at startup; see _declare_robot_info_params.
    JOINT_NAME_ALIASES = {
        "stretch_gripper_joint": "gripper_joint",
        "gripper_aperture": "gripper_joint",
    }

    # The command joint for whatever tool is attached. In /joint_states it carries one
    # of the tool's URDF joints (e.g. one finger's angle), in the same units it is
    # commanded in, alongside the tool's own URDF joints.
    TOOL_JOINT = "gripper_joint"

    # Command joints that /joint_states reports as several equal URDF joints, each
    # carrying an equal share of the command joint's position and velocity.
    SPLIT_JOINTS = {
        "arm_joint": ["arm_l4_joint", "arm_l3_joint", "arm_l2_joint", "arm_l1_joint"],
    }

    # Command joints whose stretch_body / stretch_mujoco name is not the command joint
    # with '_joint' removed.
    BACKEND_JOINT_NAMES = {
        "gripper_joint": "stretch_gripper",
    }

    # How the robot may enter puppet mode (puppet.engage): 'require_match' only once its
    # joints are within puppet.match_tolerance of the leader's; 'immediate' at once,
    # jumping to the leader's pose; 'slow' (not implemented) by moving there first.
    PUPPET_ENGAGE_MODES = ("require_match", "immediate", "slow")
    DEFAULT_PUPPET_ENGAGE = "require_match"
    # The leader's joint states must be at most this old (s) for 'require_match'.
    PUPPET_LEADER_MAX_AGE_S = 1.0

    # Default position tolerances (m or rad) the trajectory server uses to decide a
    # joint has reached a waypoint. Joints not listed fall back to
    # trajectory_server.default_tolerance.
    TRAJECTORY_TOLERANCES = {
        "lift_joint": 0.01,
        "arm_joint": 0.01,
    }

    def __init__(self,name):
        super().__init__(name)

        # Outputs (topics, frames) follow the node's namespace; frame ids get this prefix.
        ns = self.get_namespace().strip('/')
        self.prefix = ns + '/' if ns else ''

        self.startup_robot()
        
        self.node_name = self.get_name()
        self.logger = self.get_logger()
        self.logger.info("For use with S T R E T C H (TM) RESEARCH EDITION from Hello Robot Inc.")
        
        self.logger.info("{0} started".format(self.node_name))
        
        self._declare_common_params()
        self.declare_node_params()
        self._declare_robot_info_params()

        # Runstop management
        self.prev_runstop_state = None
        self.prerunstop_mode = None

        # Held while checking and changing the driver mode for a routine or runstop.
        self.driver_mode_lock = RLock()

        # Callback groups
        self.main_group = ReentrantCallbackGroup()
        self.mutex_group = MutuallyExclusiveCallbackGroup()

        # Which trajectory server, if any, is sending the commands on this thread (see
        # StretchTrajectoryActionServer._server_commands and note_joint_command).
        self._command_source = threading.local()
        self.trajectory_servers = []

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
        
        if self.prefix:
            self.logger.info(f"namespace = {self.prefix}")
        self.logger.info(f"command namespace = {self._command_namespace() or '(same as node)'}")

        # Get param values
        mode=self.get_parameter('mode').value
        if mode not in self.control_modes:
            self.logger.warn(f'given invalid mode={mode}, using {self.default_mode} instead')
            self.set_parameters(
                [Parameter("mode", Parameter.Type.STRING, self.default_mode.value)]
            )
            mode = self.get_parameter("mode").value
        mode = self._check_startup_puppet_mode(mode)

        self.logger.info('mode = ' + str(mode))
        self._previous_mode = mode
        
        sensitivity = self.get_parameter('sensitivity').value
        self.logger.info('sensitivity = ' + str(sensitivity))
        
        broadcast_odom_tf = self.get_parameter('broadcast_odom_tf').value
        self.logger.info(f'broadcast_odom_tf = {broadcast_odom_tf}')

        action_timeout = self.get_parameter('action_timeout').value
        self.logger.info(f"action timeout = {action_timeout} s")
        self._apply_launch_action_timeout()
        
        velocity_timeout = self.get_parameter('velocity_timeout').value
        #self.velocity_timeout_duration = Duration(seconds=velocity_timeout)
        self.logger.info(f"velocity timeout = {velocity_timeout} s")
        
        self.add_on_set_parameters_callback(self.check_parameter_callback)
        self.add_post_set_parameters_callback(self.parameter_update_callback)

        self.last_position_target = {}
        self.last_known_state = None
        self.last_odom = None
        self.last_joint_mode = {joint: "position" for joint in self.command_joints}
        self.position_tolerance = self.get_parameter('position_tolerance').value

        
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
        self.declare_parameter("mode", self.default_mode.value)
        self.declare_parameter('sensitivity','default')
        self.declare_parameter('broadcast_odom_tf', False) # based on wheel odometry
        self.declare_parameter('command_namespace', '', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description="Namespace of the command topics (cmd_vel, joint_position_cmd, "
                        "joint_velocity_cmd, joy): '' = the node's namespace, '/' = root, "
                        "'foo' = /foo. Lets several drivers share commands (e.g. a digital "
                        "twin) while their outputs stay in their own namespaces.",
        ))


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
            self.declare_parameter(f"trajectory_server.trajectory_tolerance.{joint}", self.TRAJECTORY_TOLERANCES.get(joint), desc)
            
        self.declare_parameter("joint_acceleration.omnibase.linear", None, desc)
        self.declare_parameter("joint_acceleration.omnibase.angular", None, desc)

        self.declare_parameter("joint_limit.omnibase.linear.acceleration", None, desc)
        self.declare_parameter("joint_limit.omnibase.angular.acceleration", None, desc)

        
        self.declare_parameter('action_timeout', 3.0, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description="Kept for launch files written for the old driver. If set at launch, "
                        "or changed later, it sets trajectory_server.timeout (unless that is "
                        "also set at launch).",
        ))
        self.declare_parameter('velocity_timeout', 0.5, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Default timeout (sec) for velocity control',
        ))
        
        self.declare_parameter('control_loop_rate', 100, ParameterDescriptor(
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
        # Puppet mode: follow another robot's (or any) joint states, and optionally its base
        self.declare_parameter("puppet.joint_states_topic", "", ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description="sensor_msgs/JointState topic (in /joint_states form) whose positions, or velocities "
                        "for joints in velocity mode, this robot follows in puppet mode",
        ))
        self.declare_parameter("puppet.odom_topic", "", ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description="nav_msgs/Odometry topic whose base velocity this robot follows in puppet mode when puppet.follow_base is set",
        ))
        self.declare_parameter("puppet.follow_base", False, ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description="In puppet mode, drive the base at the velocity reported on puppet.odom_topic",
        ))
        self.declare_parameter("puppet.engage", self.DEFAULT_PUPPET_ENGAGE, ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description="How the robot enters puppet mode: 'require_match' (only once every joint in "
                        "position mode is within puppet.match_tolerance of the leader's), 'immediate' "
                        "(at once, jumping to the leader's pose: not safe unless the poses already "
                        "match), or 'slow' (not implemented yet)",
        ))
        self.declare_parameter("puppet.match_tolerance", 0.05, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description="Largest difference (m or rad) between this robot's joints and the leader's "
                        "for entering puppet mode with puppet.engage 'require_match'. The gripper "
                        "uses 10% of the tool's URDF range instead.",
        ))

        # Gamepad parameters
        self.declare_parameter("gamepad.dt", 0.5)
        for joint in self.command_joints:
            self.declare_parameter(f"gamepad.max_vel.{joint}", 0.5)
            self.declare_parameter(f"gamepad.deadzone.{joint}", 0.0)

    

    def _declare_robot_info_params(self):
        """Declare the parameters that describe this robot, from what the concrete driver
        reports: the read-only tool_info.*, the default speeds joint_velocity.*, and the
        joint_limit.<joint>.velocity limits. Also accepts the tool's own names for
        TOOL_JOINT and seeds its trajectory tolerance from the tool."""
        read_only = ParameterDescriptor(read_only=True, description="Attached tool (read-only)")
        info = self.get_tool_info()
        self.tool_is_actuated = bool(info.get("is_actuated", False))
        tool_joints = list(info.get("tool_joints", []))
        self.declare_parameter("tool_info.name", info.get("name") or "unknown", read_only)
        self.declare_parameter("tool_info.is_actuated", self.tool_is_actuated, read_only)
        self.declare_parameter("tool_info.tool_joints", tool_joints, read_only)
        # Ranges in URDF units, so a client can size jog increments and sliders from the tool.
        if self.tool_is_actuated:
            for key in ("urdf_range", "aperture_range"):
                self.declare_parameter(f"tool_info.{key}", [float(v) for v in info[key]], read_only)
            self.declare_parameter("tool_info.position_tolerance", float(info["position_tolerance"]), read_only)

        self.joint_name_aliases = dict(self.JOINT_NAME_ALIASES)
        if self.TOOL_JOINT in self.command_joints:
            backend = self.backend_joint_name(self.TOOL_JOINT)
            for alias in [backend, f"{backend}_joint", *tool_joints]:
                self.joint_name_aliases.setdefault(alias, self.TOOL_JOINT)
            tolerance = f"trajectory_server.trajectory_tolerance.{self.TOOL_JOINT}"
            if self.tool_is_actuated and self.get_parameter(tolerance).value is None:
                self.set_parameters([Parameter(tolerance, Parameter.Type.DOUBLE,
                                               float(info["position_tolerance"]))])

        # Speed used for a position command that doesn't give one; see set_joint_position.
        speed = ParameterDescriptor(description="Default speed (m/s or rad/s) of a position "
                                                "command that doesn't give one")
        for name, vel in self.default_joint_velocity().items():
            self.declare_parameter(f"joint_velocity.{name}", float(vel), speed)

        limits = [
            Parameter(f"joint_limit.{joint}.velocity", Parameter.Type.DOUBLE, float(vel))
            for joint, vel in self.max_joint_velocity().items()
            if self.get_parameter(f"joint_limit.{joint}.velocity").value is None
        ]
        self.set_parameters(limits)

    def _apply_launch_action_timeout(self):
        """Launch files written for the old driver (e.g. navigation's, which sets 30) give
        the trajectory timeout as action_timeout; if it was set at launch and
        trajectory_server.timeout wasn't, it becomes trajectory_server.timeout."""
        overrides = getattr(self, "_parameter_overrides", {})
        if "action_timeout" in overrides and "trajectory_server.timeout" not in overrides:
            self._set_trajectory_timeout(self.get_parameter("action_timeout").value)

    def _set_trajectory_timeout(self, timeout):
        self.set_parameters([Parameter("trajectory_server.timeout", Parameter.Type.DOUBLE, float(timeout))])
        self.logger.info(f"trajectory_server.timeout = {timeout} s (from action_timeout)")

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
        self.latched_publishers.append((self.mode_pub, lambda robot_status, status_time: str(self.robot_mode())))
        
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
        # Command topics follow command_namespace; services and the action stay in the
        # node's namespace, since a service or action name can have only one server.
        self.create_subscription(Twist, self._command_topic("cmd_vel"), self.base_twist_callback, 1, callback_group=self.main_group)

        self.create_subscription(TwistWithCovarianceStamped, self._command_topic("cmd_vel_stiff"), self.stiff_twist_callback, 1, callback_group=self.main_group)
        
        self.create_subscription(JointState, self._command_topic("joint_velocity_cmd"), self.velocity_cmd_callback, 1, callback_group=self.main_group)
        
        self.create_subscription(JointState, self._command_topic("joint_position_cmd"), self.position_cmd_callback, 1, callback_group=self.main_group)
        
        self.create_subscription(Joy, self._command_topic("joy"), self.joy_callback, 1, callback_group=self.main_group)

        # Puppet mode inputs; (re)created when their topic parameters change.
        self.puppet_subscriptions = {}
        self._puppet_leader = None  # (latest leader JointState, time.monotonic() received)
        self._subscribe_puppet_topic("puppet.joint_states_topic")
        self._subscribe_puppet_topic("puppet.odom_topic")
        self.create_timer(3.0, self.check_puppet_topics, callback_group=self.main_group)

    def _command_namespace(self):
        """command_namespace as an absolute namespace ('/' or '/foo'), or '' for the node's own."""
        ns = self.get_parameter('command_namespace').value.strip()
        if not ns:
            return ''
        return '/' + ns.strip('/')

    def _command_topic(self, name):
        """Name of command topic `name` in the command namespace (relative if unset)."""
        ns = self._command_namespace()
        if not ns:
            return name
        return f"{ns.rstrip('/')}/{name}"

    def frame_id(self, name):
        """TF frame `name` for this robot, prefixed with its namespace (matching the
        frame_prefix given to robot_state_publisher)."""
        return self.prefix + name

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
        self.seat_into_dock_service = self.create_service(
            Trigger,
            'seat_into_dock',
            self.seat_into_dock_callback,
            callback_group=self.main_group
        )
        self.freewheel_the_robot_service = self.create_service(
            Trigger,
            'freewheel_the_robot',
            self.freewheel_the_robot_callback,
            callback_group=self.main_group
        )
        self.runstop_service = self.create_service(
            SetBool,
            'runstop_the_robot',
            self.runstop_service_callback,
            callback_group=self.main_group
        )

        # Publish each call on <service>/_service_event, so another node (e.g. the digital
        # twin relay) can follow this robot's service calls without being their server.
        for service in (self.stop_the_robot_service, self.home_the_robot_service,
                        self.stow_the_robot_service, self.seat_into_dock_service,
                        self.freewheel_the_robot_service, self.runstop_service):
            service.configure_introspection(self.get_clock(), qos_profile_system_default,
                                            ServiceIntrospectionState.CONTENTS)

    def _setup_common_actions(self):
        self.trajectory_server = StretchTrajectoryActionServer(self)
        self.trajectory_servers = [self.trajectory_server]
        # More trajectory servers to run alongside the default one, as comma-separated
        # "module:Class" names. Each class takes the driver and must use its own action name
        # and parameter prefix (e.g. stretch_dynamic_trajectory's obstacle-aware server).
        extra = self.declare_parameter("extra_trajectory_servers", "").value
        for name in (n.strip() for n in extra.split(",")):
            if not name:
                continue
            try:
                module_name, class_name = name.split(":")
                server_class = getattr(importlib.import_module(module_name), class_name)
                self.trajectory_servers.append(server_class(self))
                self.logger.info(f"Started trajectory server {name}")
            except Exception as e:
                self.logger.error(f"Couldn't start trajectory server {name}: {e}\n{traceback.format_exc()}")

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
        mode_param = Parameter("mode", Parameter.Type.STRING, str(mode))
        result = self.set_parameters([mode_param])[0]
        if not result.successful and require_success:
            raise RuntimeError(f"Failed to change mode parameter to {mode} when strict success checking is enabled. Reason: {result.reason}")
        return result.successful

    # How often a repeated per-joint mode complaint may be logged. These sit on paths
    # that run per command message or per control loop, so they need a rate limit.
    MODE_WARNING_PERIOD_S = 5.0

    def resolve_joint_name(self, name):
        """Return the command joint `name` addresses, accepting JOINT_NAME_ALIASES.

        A name matching no command joint is returned unchanged, leaving the complaint
        to the caller, which knows what interface it arrived on.
        """
        if self.command_joints and name in self.command_joints:
            return name
        joint = self.joint_name_aliases.get(name)
        if joint is not None and self.command_joints and joint in self.command_joints:
            return joint
        return name

    def backend_joint_name(self, name):
        """Translate a command joint ('lift_joint') into its actuator name ('lift').

        Called by concrete drivers where they dispatch into stretch_body or
        stretch_mujoco.
        """
        backend = self.BACKEND_JOINT_NAMES.get(name)
        if backend is not None:
            return backend
        return name[: -len("_joint")] if name.endswith("_joint") else name

    def replace_deprecated_mode(self, legacy_mode):
        """Set the control mode and joint_mode according to the legacy robot mode."""
        replacement, joint_mode = self.DEPRECATED_CONTROL_MODES[legacy_mode]

        if joint_mode == JointMode.VELOCITY:
            joints = self.velocity_joints or []
        else:
            joints = self.command_joints or []

        self.logger.warn(
            f"mode '{legacy_mode}' is DEPRECATED and is no longer a robot mode. "
            f"Setting control mode to '{replacement}'"
            + (
                f" and joint_mode.<joint> to '{joint_mode}' for {list(joints)}"
                if joints
                else ""
            )
            + f". Valid driver control modes are {self.control_modes} and joint modes are {self.joint_modes}"
        )

        params = [Parameter("mode", Parameter.Type.STRING, str(replacement))]
        params += [
            Parameter(f"joint_mode.{joint}", Parameter.Type.STRING, joint_mode.value)
            for joint in joints
        ]

        for param, result in zip(params, self.set_parameters(params)):
            if not result.successful:
                self.logger.error(
                    f"Failed to apply '{param.name}' while replacing deprecated mode "
                    f"'{legacy_mode}': {result.reason}"
                )
    
    def update_parameter(self, parameter: Parameter):
        #This function only gets called if all requested parameter updates are
        #validated; no need to check, and no need to handle any parameters that
        #don't require additional processing. Note that at this point the value of
        # the parameter that will return from get_parameter has already changed.
        match parameter.name:
            case "mode":
                if parameter.value in self.DEPRECATED_CONTROL_MODES:
                    self.replace_deprecated_mode(parameter.value)
                    return
                if (self._previous_mode == self.puppet_mode and parameter.value != self.puppet_mode
                        and self.get_parameter("puppet.follow_base").value):
                    # stop following the puppet's base
                    self.set_base_velocity(0.0, 0.0, 0.0)
                if (parameter.value == self.puppet_mode and self._previous_mode != self.puppet_mode
                        and self.get_parameter("puppet.engage").value == "immediate"):
                    self.logger.warn("Entering puppet mode with puppet.engage 'immediate': joints jump to "
                                     "the leader's pose, which is not safe unless the poses already match.")
                self._previous_mode = parameter.value
                self.handle_mode_change(parameter.value)
            case "puppet.joint_states_topic" | "puppet.odom_topic":
                self._subscribe_puppet_topic(parameter.name, parameter.value)
            case "velocity_timeout":
                velocity_timeout = parameter.value
                self.logger.info(f"Changed to velocity timeout = {velocity_timeout} s")
            case "action_timeout":
                self._set_trajectory_timeout(parameter.value)
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
                if (
                    parameter.value not in self.control_modes
                    and parameter.value not in self.priority_modes
                    and parameter.value not in self.DEPRECATED_CONTROL_MODES
                ):
                    reason=f"Mode does not exist. (Control modes are {self.control_modes}. Priority modes are {self.priority_modes}.)"
                elif parameter.value == self.puppet_mode and self.robot_mode() != self.puppet_mode:
                    reason = self._puppet_engage_refusal()
            case "puppet.engage":
                found = True
                if parameter.value not in self.PUPPET_ENGAGE_MODES:
                    reason = f"puppet.engage must be one of {list(self.PUPPET_ENGAGE_MODES)}."
            case "puppet.match_tolerance":
                found = True
                if not isinstance(parameter.value, float) or parameter.value <= 0.0:
                    reason = "puppet.match_tolerance must be a positive number."
            case "action_timeout":
                found=True
                if parameter.value < 0.0:
                    reason="Timeout cannot be less than 0.0."
            case "puppet.joint_states_topic" | "puppet.odom_topic":
                found = True
                if not isinstance(parameter.value, str):
                    reason = f"{parameter.name} must be a topic name (string)."
            case "puppet.follow_base":
                found = True
                if not isinstance(parameter.value, bool):
                    reason = "puppet.follow_base must be a boolean."
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
            case n if n.startswith("joint_velocity."):
                found = True
                if not isinstance(parameter.value, (int, float)) or parameter.value <= 0.0:
                    reason = f"{n} must be a positive speed."
            # --- Read-Only/Static Parameters ---
            # (sensitivity is changeable at runtime; the concrete driver checks it.)
            case n if n in ("control_loop_rate", "broadcast_odom_tf", "command_namespace", "extra_trajectory_servers") or n.startswith("tool_info."):
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
            if p.type_ == Parameter.Type.NOT_SET:
                # rclpy would undeclare it, breaking every later read of the parameter
                return SetParametersResult(successful=False, reason=f"Parameter {p.name} can't be unset.")
            found_here,reason_here = self.check_common_param(p)
            found_child,reason_child = self.check_child_param(p)
            #self.logger.warning(f"Param: {p.name}, found (here,child): {found_here},{found_child}, reason(here,child): {reason_here},{reason_child}")
            
            if found_here and found_child:
                self.logger.warn(f"Parameter {p.name} exists in Stretch4 API ROS superclass and robot/sim specific subclass.  Subclass should shadow superclass but unexpected behavior may result.")
            if not found_here and not found_child:
                #if not found, both reasons will be not found errors; use the one here for consistency between real and sim.
                return SetParametersResult(successful=False,reason=reason_here)

            if (reason_here and found_here) or (reason_child and found_child):
                # only the reasons from whichever class handles this parameter; the other's
                # is just "not found"
                reasons = [reason for reason, found in ((reason_here, found_here), (reason_child, found_child))
                           if found and reason]
                return SetParametersResult(successful=False,reason=f"{' '.join(reasons)} (multiple reasons for rejection are possible)")

        return SetParametersResult(successful=True)

    def base_twist_callback(self, twist:Twist):
        self.logger.debug(f"Got request for base twist: x:{twist.linear.x}, y:{twist.linear.y}, theta:{twist.angular.z}") 
        
        mode = self.robot_mode()
        if self.robot_mode() != self.active_mode:
            self.logger.warn(
                f"Cannot send base commands while robot is in mode {mode}.  Must be in mode '{self.active_mode}'"
            )
            return

        self.note_direct_base_command()
        self.set_base_velocity(twist.linear.x, twist.linear.y, twist.angular.z)

    def note_direct_base_command(self):
        """A base command from outside a trajectory server preempts a trajectory that
        rotates the base, as joint commands do for their joints (see note_joint_command)."""
        self.note_joint_command(StretchTrajectoryActionServer.BASE_ROTATION_JOINT)

    def note_joint_command(self, joint_name):
        """A command to a joint preempts every trajectory server running a goal on that joint,
        unless that server sent it (the newest command wins). Servers mark their own commands
        with _server_commands, per thread, so a direct command arriving on another thread
        while a server is mid-command still counts."""
        source = getattr(self._command_source, "server", None)
        for server in self.trajectory_servers:
            if server is not source and server.active_joints is not None and joint_name in server.active_joints:
                server.preempt(joint_name)

    def stiff_twist_callback(self, stiff_twist: TwistWithCovarianceStamped):
        """cmd_vel_stiff: a base velocity together with how stiffly to hold it (the
        docking server drives in compliantly)."""
        mode = self.robot_mode()
        if mode != self.active_mode:
            self.logger.warn(
                f"Cannot send base commands while robot is in mode {mode}.  Must be in mode '{self.active_mode}'"
            )
            return
        twist = stiff_twist.twist.twist
        self.note_direct_base_command()
        self.set_base_velocity(twist.linear.x, twist.linear.y, twist.angular.z,
                               stiffness=self.stiffness_from_covariance(stiff_twist.twist.covariance))

    @staticmethod
    def stiffness_from_covariance(covariance):
        """cmd_vel_stiff carries the base's stiffness (0 to 1) in covariance[0]. A
        covariance whose last element is 0 (e.g. left unset) means full stiffness."""
        return 1.0 if covariance[-1] == 0.0 else float(covariance[0])

    @abstractmethod
    def set_base_velocity(self, x, y, theta, stiffness=1.0):
        pass

    def velocity_cmd_callback(self, target: JointState):
        self.logger.debug(f"Got velocity command request: names: {target.name} velocities: {target.velocity} position: {target.position} (NB: position not used in velocity command!)")
        current_mode = self.get_parameter('mode').value
        if current_mode != self.active_mode:
            self.logger.warn(
                f"Cannot send velocity commands while robot is in mode {current_mode}.  Must be in mode '{self.active_mode}'"
            )
            return
            
        for i in range(len(target.name)):
            self.check_and_set_vel(self.resolve_joint_name(target.name[i]),target.velocity[i])

    @abstractmethod
    def set_joint_velocity(self, joint, target, a = None):
        pass

    def check_and_set_vel(self, joint_name, goal, a = None):
        #self.logger.info(f"Setting joint {joint_name} to velocity {goal}")
        self.note_joint_command(joint_name)

        try:
            mode = self.get_parameter(f"joint_mode.{joint_name}").value
        except ParameterNotDeclaredException:
            self.logger.error(f"Joint name {joint_name} not found in mode parameters.  Joint name is probably incorrect.")
            mode = "<< joint unknown >>"
            return False
            
        succeeded = False
        if mode != self.velocity_mode:
            self.logger.warn(
                f"Cannot send velocity command to joint {joint_name} while in {mode} mode (must be in {self.velocity_mode} mode)."
            )
        else:
            robot_mode = self.get_parameter('mode').value
            if robot_mode not in self.control_modes:
                self.logger.warn(
                    f"Cannot send velocity command to joint {joint_name} because robot mode is {robot_mode} (must be one of {self.control_modes})."
                )
            else:
                try:
                    vel_limit = self.get_parameter(f"joint_limit.{joint_name}.velocity").value
                except Exception:
                    vel_limit = None
                
                if vel_limit is not None and abs(goal) > vel_limit:
                    self.logger.warn(
                        f"Velocity command {goal} for joint {joint_name} exceeds its limit "
                        f"joint_limit.{joint_name}.velocity ({vel_limit}); limiting it.",
                        throttle_duration_sec=self.MODE_WARNING_PERIOD_S,
                    )
                    goal = float(np.clip(goal, -vel_limit, vel_limit))
                self.set_joint_velocity(joint_name, goal, a = a)
                succeeded = True
                    
        return succeeded
        
    def check_and_set_pos(self, joint_name, goal, v = None, a = None):
        #self.logger.info(f"Setting joint {joint_name} to position {goal}.")
        self.note_joint_command(joint_name)

        try:
            mode = self.get_parameter(f"joint_mode.{joint_name}").value
        except ParameterNotDeclaredException:
            self.logger.error(f"Joint name {joint_name} not found in mode parameters.  Joint name is probably incorrect.")
            mode = "<< joint unknown >>"

        succeeded = False
        if mode != self.position_mode:
            self.logger.warn(
                f"Cannot send position command to joint {joint_name} while in {mode} mode (must be in {self.position_mode} mode)."
            )
        else:
            limits = self.get_parameters_by_prefix(f"joint_limit.{joint_name}")
            ul = limits["upper"].value
            ll = limits["lower"].value
                
            if (ul is None or goal <= ul) and (ll is None or goal >= ll):
                robot_mode = self.get_parameter('mode').value
                if robot_mode not in self.control_modes:
                    self.logger.warn(
                        f"Cannot send position command to joint {joint_name} because robot mode is {robot_mode} (must be one of {self.control_modes})."
                    )
                else:
                    self.last_position_target[joint_name]=goal
                    self.set_joint_position(joint_name, goal, v=v, a=a)
                    succeeded = True
            else:
                self.logger.warn(f"Cannot send position command to joint {joint_name}: goal pose {goal} outside of joint limits ({ll},{ul}).")
        return succeeded
    
    def position_cmd_callback(self, target: JointState):
        self.logger.debug(f"Got position command request: names: {target.name} velocities: {target.velocity} position: {target.position} (NB: velocity not used in velocity command!)")
        current_mode = self.get_parameter('mode').value
        if current_mode != self.active_mode:
            self.logger.warn(
                f"Cannot send position commands while robot is in mode {current_mode}.  Must be in mode '{self.active_mode}'"
            )
            return
            
        for i in range(len(target.name)):
            self.check_and_set_pos(self.resolve_joint_name(target.name[i]),target.position[i])

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

    # --- puppet mode ---------------------------------------------------------------

    def _subscribe_puppet_topic(self, param_name, topic=None):
        """(Re)subscribe to the topic named by puppet parameter `param_name`."""
        if topic is None:
            topic = self.get_parameter(param_name).value
        old = self.puppet_subscriptions.pop(param_name, None)
        if old is not None:
            self.destroy_subscription(old)
        if param_name == "puppet.joint_states_topic":
            self._puppet_leader = None
        if not topic:
            return
        msg_type, callback = {
            "puppet.joint_states_topic": (JointState, self.puppet_joint_states_callback),
            "puppet.odom_topic": (Odometry, self.puppet_odom_callback),
        }[param_name]
        self.puppet_subscriptions[param_name] = self.create_subscription(
            msg_type, topic, callback, 1, callback_group=self.main_group)

    def puppet_joint_states_callback(self, joint_state: JointState):
        """In puppet mode, move each command joint to match `joint_state`: its position for
        joints in position mode, its velocity for joints in velocity mode."""
        # kept in every mode, for puppet.engage 'require_match' to compare against
        self._puppet_leader = (joint_state, time.monotonic())
        if self.robot_mode() != self.puppet_mode:
            return
        for joint in self.command_joints:
            match self.get_parameter(f"joint_mode.{joint}").value:
                case self.position_mode:
                    target = self.command_joint_pose_from_joint_state(joint, joint_state, default=None)
                    if target is None:
                        continue
                    # A leader at (or reading slightly past) its own limit puts this joint at its limit.
                    target = self._clamp_to_limits(joint, target)
                    if abs(target - self.last_position_target.get(joint, float("inf"))) > self.PUPPET_DEADBAND:
                        self.check_and_set_pos(joint, target)
                case self.velocity_mode:
                    target = self.command_joint_vel_from_joint_state(joint, joint_state, default=None)
                    if target is not None:
                        self.check_and_set_vel(joint, target)

    # Puppet mode only re-commands a joint whose target moved by more than this (m or rad).
    PUPPET_DEADBAND = 1e-4

    def _puppet_engage_refusal(self):
        """Why the robot can't enter puppet mode now, under puppet.engage, or None if it can.
        Runs while a mode change is being validated, so it only reads state."""
        engage = self.get_parameter("puppet.engage").value
        if engage == "immediate":
            return None
        if engage == "slow":
            return self._puppet_engage_slow()

        # require_match: every joint followed by position must already be at the leader's
        # (limit-clamped) position. Joints in velocity mode follow speeds, so can't jump.
        topic = self.get_parameter("puppet.joint_states_topic").value
        if self._puppet_leader is None or time.monotonic() - self._puppet_leader[1] > self.PUPPET_LEADER_MAX_AGE_S:
            return (f"Can't enter puppet mode: no recent leader joint states on "
                    f"{topic or '(puppet.joint_states_topic is not set)'} to match "
                    f"(puppet.engage is 'require_match').")
        if self.last_known_state is None:
            return "Can't enter puppet mode: this robot's joint states aren't known yet."
        leader = self._puppet_leader[0]
        far = []
        for joint in self.command_joints:
            if self.get_parameter(f"joint_mode.{joint}").value != self.position_mode:
                continue
            target = self.command_joint_pose_from_joint_state(joint, leader)
            current = self.command_joint_pose_from_joint_state(joint, self.last_known_state)
            if target is None or current is None:
                continue
            error = abs(self._clamp_to_limits(joint, target) - current)
            tolerance = self._puppet_match_tolerance(joint)
            if error > tolerance:
                far.append(f"{joint} is {error:.3f} from the leader (tolerance {tolerance:.3f})")
        if far:
            return ("Can't enter puppet mode until this robot matches the leader's pose "
                    "(puppet.engage is 'require_match'): " + "; ".join(far) + ".")
        return None

    def _puppet_match_tolerance(self, joint):
        """puppet.match_tolerance, or for the tool joint 10% of its URDF range, since a tool's
        units (and range) vary."""
        if joint == self.TOOL_JOINT and self.tool_is_actuated:
            low, high = self.get_parameter("tool_info.urdf_range").value
            return 0.1 * abs(high - low)
        return self.get_parameter("puppet.match_tolerance").value

    def _puppet_engage_slow(self):
        """puppet.engage 'slow': move to the leader's pose, then start following.

        Not implemented: moving every joint at once from an arbitrary pose can run the
        robot into itself (as unstowing that way does), so this waits for a motion planner.
        Returns the reason entering puppet mode is refused.
        """
        return "puppet.engage 'slow' is not implemented yet; use 'require_match' or 'immediate'."

    def _check_startup_puppet_mode(self, mode):
        """Apply puppet.engage to a driver launched straight into puppet mode, which the
        mode-change check (registered later) never sees. Under 'require_match' or 'slow'
        it starts in active mode instead, since nothing has been matched yet. Returns the
        mode to start in. Runs before the parameter callbacks are registered."""
        engage = self.get_parameter("puppet.engage").value
        if engage not in self.PUPPET_ENGAGE_MODES:
            self.logger.warn(f"Invalid puppet.engage '{engage}', using '{self.DEFAULT_PUPPET_ENGAGE}' "
                             f"instead (one of {list(self.PUPPET_ENGAGE_MODES)}).")
            self.set_parameters([Parameter("puppet.engage", Parameter.Type.STRING, self.DEFAULT_PUPPET_ENGAGE)])
            engage = self.DEFAULT_PUPPET_ENGAGE
        if mode != self.puppet_mode:
            return mode
        if engage == "immediate":
            self.logger.warn("Starting in puppet mode with puppet.engage 'immediate': joints jump to "
                             "the leader's pose, which is not safe unless the poses already match.")
            return mode
        why = ("isn't implemented yet" if engage == "slow"
               else "only enters puppet mode once this robot matches the leader's pose")
        self.logger.warn(f"Launched in puppet mode, but puppet.engage '{engage}' {why}. Starting in "
                         f"'{self.active_mode}'; set mode to puppet once the poses match.")
        self.set_parameters([Parameter("mode", Parameter.Type.STRING, self.active_mode.value)])
        return self.get_parameter("mode").value

    def return_to_mode(self, mode):
        """Change back to `mode` after a routine or a runstop. If puppet.engage refuses a
        return to puppet mode (the leader moved meanwhile), go to active mode instead."""
        result = self.set_parameters([Parameter("mode", Parameter.Type.STRING, str(mode))])[0]
        if result.successful or mode == self.active_mode:
            return
        self.logger.warn(f"Couldn't return to mode '{mode}' ({result.reason}); switching to "
                         f"'{self.active_mode}' instead.")
        self.change_mode(self.active_mode)

    def _clamp_to_limits(self, joint, target):
        limits = self.get_parameters_by_prefix(f"joint_limit.{joint}")
        lower, upper = limits["lower"].value, limits["upper"].value
        if lower is not None:
            target = max(target, lower)
        if upper is not None:
            target = min(target, upper)
        return target

    def puppet_odom_callback(self, odom: Odometry):
        """In puppet mode with puppet.follow_base, drive the base at the odometry's velocity."""
        if self.robot_mode() != self.puppet_mode or not self.get_parameter("puppet.follow_base").value:
            return
        twist = odom.twist.twist
        self.set_base_velocity(twist.linear.x, twist.linear.y, twist.angular.z)

    def check_puppet_topics(self):
        """In puppet mode, warn (every 3 s, from the timer) about inputs that aren't set or
        aren't being published yet."""
        if self.robot_mode() != self.puppet_mode:
            return
        inputs = ["puppet.joint_states_topic"]
        if self.get_parameter("puppet.follow_base").value:
            inputs.append("puppet.odom_topic")
        for param_name in inputs:
            topic = self.get_parameter(param_name).value
            if not topic:
                self.logger.warn(f"In puppet mode but {param_name} is not set; nothing to follow.")
            elif self.count_publishers(self.puppet_subscriptions[param_name].topic_name) == 0:
                self.logger.warn(f"In puppet mode; waiting for {topic} ({param_name}) to be published.")

    def joy_callback(self, joy_msg: Joy):
        self.logger.debug(f"Got joy message. Buttons: {joy_msg.buttons} Axes: {joy_msg.axes} (this message is throttled to appear at most every 2s)", throttle_duration_sec = 2.0)
        
        current_mode = self.get_parameter('mode').value
        if current_mode != self.teleop_mode:
            self.logger.warn(
                f"Cannot send joystick commands while robot is in mode {current_mode}.  Must be in mode '{self.teleop_mode}'"
            )
            return
        
        goal = self.joy_to_joint_cmd(joy_msg)
        if goal is None:
            self.logger.error(f"Joystick mapping returned None.  Check your code.")
            return

        for i in range(len(goal.name)):
            joint_name = goal.name[i]
            joint_mode = self.get_parameter(f"joint_mode.{joint_name}").value
            match joint_mode:
                case self.position_mode:
                    if len(goal.position) < len(goal.name):
                        self.logger.error(
                            f"Joystick command mapping for position has length {len(goal.position)} (expected length {len(goal.name)} to set target for joint {joint_name} in {self.position_mode} control mode)"
                        )
                    else:
                        self.check_and_set_pos(joint_name, goal.position[i])
                case self.velocity_mode:
                    if len(goal.velocity) < len(goal.name):
                        self.logger.error(
                            f"Joystick command mapping for velocity has length {len(goal.velocity)} (expected length {len(goal.name)} to set target for joint {joint_name} in {self.velocity_mode} control mode)"
                        )
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
            "lift_joint": get_val('left_stick_y', 'lift_joint'),
            "arm_joint": get_val('left_stick_x', 'arm_joint'),
            "wrist_yaw_joint": get_val('right_stick_x', 'wrist_yaw_joint'),
            "wrist_pitch_joint": get_val('right_stick_y', 'wrist_pitch_joint'),
            "wrist_roll_joint": get_button_vel('wrist_roll_joint', 'right_shoulder_button_pressed', 'left_shoulder_button_pressed'),
            "gripper_joint": get_button_vel('gripper_joint', 'top_button_pressed', 'bottom_button_pressed')
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

    def home_the_robot_callback(self, request, response):
        response.success, response.message = self._run_priority_routine(self.homing_mode, "home", self.home_robot)
        return response

    def stow_the_robot_callback(self, request, response):
        response.success, response.message = self._run_priority_routine(self.stowing_mode, "stow", self.stow_robot)
        return response

    def seat_into_dock_callback(self, request, response):
        response.success, response.message = self._run_priority_routine(self.seating_mode, "seat_into_dock", self.seat_robot)
        return response

    def freewheel_the_robot_callback(self, request, response):
        response.success, response.message = self.freewheel_base()
        self.logger.info(f"freewheel_the_robot: {response.message}")
        return response

    def _run_priority_routine(self, mode, action, routine):
        """Run `routine` (a driver hook returning (success, message)) in priority mode
        `mode`, then return to the previous control mode. Refused unless the robot is
        in a control mode."""
        with self.driver_mode_lock:
            last_mode = self.robot_mode()
            if last_mode not in self.control_modes:
                message = f"Cannot {action} while in mode={last_mode}."
                self.logger.error(message)
                return False, message
            self.change_mode(mode)
        self.logger.info(f"Received {action} request; mode = {mode}.")
        try:
            success, message = routine()
        except Exception as e:
            self.logger.error(f"{action} failed: {e}\n{traceback.format_exc()}")
            success, message = False, f"{action} failed: {e}"
        finally:
            with self.driver_mode_lock:
                if self.robot_mode() == mode:
                    self.return_to_mode(last_mode)
                elif self.prerunstop_mode == mode:
                    # Runstopped during the routine: clearing the runstop returns to the
                    # mode from before the routine, not to the routine's.
                    self.prerunstop_mode = last_mode
        # One call site per severity: rclpy raises if a call site changes severity.
        if success:
            self.logger.info(f"{action}: {message}")
        else:
            self.logger.warn(f"{action}: {message}")
        return success, message

    @abstractmethod
    def home_robot(self) -> tuple[bool, str]:
        """Home the robot, blocking until done. Returns (success, message)."""
        pass

    @abstractmethod
    def stow_robot(self) -> tuple[bool, str]:
        """Stow the robot, blocking until done. Returns (success, message)."""
        pass

    @abstractmethod
    def seat_robot(self) -> tuple[bool, str]:
        """Seat the robot into the dock it is lined up with, blocking until done. Returns
        (success, message)."""
        pass

    @abstractmethod
    def freewheel_base(self) -> tuple[bool, str]:
        """Let the base's wheels turn freely. Returns (success, message)."""
        pass

    @abstractmethod
    def get_tool_info(self) -> dict:
        """The attached tool, as a dict with 'name', 'is_actuated' and 'tool_joints'
        (its URDF joints) and, for an actuated tool, 'urdf_range', 'aperture_range' and
        'position_tolerance' in TOOL_JOINT's units. Called during __init__, after
        startup_robot and declare_node_params."""
        pass

    @abstractmethod
    def default_joint_velocity(self) -> dict:
        """Default speed of each command joint ('lift_joint': 0.3, ...) and of the base
        ('omnibase.linear', 'omnibase.angular'); becomes joint_velocity.<name>."""
        pass

    def max_joint_velocity(self) -> dict:
        """Maximum speed of each command joint; becomes joint_limit.<joint>.velocity
        unless that was set at launch. None known by default."""
        return {}

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
        self.last_odom = odom

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

    # A command joint's position or velocity in a /joint_states message (this robot's,
    # or another's in puppet mode): its own entry, or the sum of the URDF joints it is
    # split into (SPLIT_JOINTS).
    def command_joint_pose_from_joint_state(self, command_joint, joint_state, default=None):
        return self._joint_state_value(joint_state.position, command_joint, joint_state, default)

    def command_joint_vel_from_joint_state(self, command_joint, joint_state, default=None):
        return self._joint_state_value(joint_state.velocity, command_joint, joint_state, default)

    def _joint_state_value(self, values, command_joint, joint_state, default):
        found = []
        for name in self.SPLIT_JOINTS.get(command_joint, [command_joint]):
            try:
                found.append(values[joint_state.name.index(name)])
            except (ValueError, IndexError):
                pass
        return sum(found) if found else default

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
    # Virtual joint for turning the base in place. Its positions are radians relative to
    # the base's heading (from odometry) when the trajectory starts; the server turns the
    # base with base velocity commands. Only accepted when trajectory_server.allow_rotation
    # is set.
    BASE_ROTATION_JOINT = "mobile_base_rotation_joint"

    def __init__(self, driver: Stretch4ROSDriver, action_name="follow_joint_trajectory",
                 param_prefix="trajectory_server", diagnostics_namespace="trajectory_server_diagnostics"):
        """action_name, param_prefix and diagnostics_namespace let another server (a
        subclass) run alongside this one."""
        # Store the reference to the parent node
        self.driver = driver

        self.active_joints = None
        self.direct_command_preempted = False
        self.preempted_joints = set()  # joints whose direct command preempted the goal

        self.param_prefix = param_prefix
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
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.driver.main_group
        )

        qos_profile = 10

        # Each accepted goal (with its id), so another node (e.g. the digital twin relay)
        # can follow this robot's trajectories; actions have no introspection in Jazzy.
        self.pub_accepted_goals = self.driver.create_publisher(
            FollowJointTrajectory.Impl.SendGoalService.Request,
            f"{action_name}/accepted_goals", qos_profile
        )
        
        self.pub_actual_state = self.driver.create_publisher(
            JointState, f"{diagnostics_namespace}/actual", qos_profile
        )

        # Publisher for desired joint state diagnostics
        self.pub_desired_state = self.driver.create_publisher(
            JointState, f"{diagnostics_namespace}/desired", qos_profile
        )

        self.pub_commands = self.driver.create_publisher(
            JointState, f"{diagnostics_namespace}/commands", qos_profile
        )

        
        self.driver.get_logger().info(f'FollowJointTrajectory Action Server {action_name} has been initialized.')

    def handle_accepted_callback(self, goal_handle):
        self.pub_accepted_goals.publish(FollowJointTrajectory.Impl.SendGoalService.Request(
            goal_id=goal_handle.goal_id, goal=goal_handle.request))
        goal_handle.execute()

    def declare_params(self):
        def add_param(name,value):
            self.driver.declare_parameter(f"{self.param_prefix}.{name}", value)

        add_param("mode", "target_priority")
        add_param("strict_mode", True)
        add_param("timeout", 5.0)
        # adaptive_velocity: velocity command per unit of position error (1/s)
        add_param("kp", 2.0)
        # adaptive_velocity: after the last waypoint's time, keep correcting toward it until
        # every joint is within tolerance_fraction of its trajectory tolerance (or timeout s)
        add_param("final_convergence.tolerance_fraction", 0.2)
        add_param("final_convergence.timeout", 1.5)
        add_param("ki", 0.001)
        add_param("kd", 0.01)
        add_param("default_tolerance", 0.05)
        add_param("loop_rate", 50.0)
        # if velocity not specified, how to do interpolation
        # options are 'zero' (stop between waypoints) or 'smooth'
        # (average slope between prev and next points)
        add_param("velocity_inference", "smooth")
        # if True, once the final waypoint is reached, joints must stay stopped
        # (below settling.vel_threshold) for settle_duration seconds before the
        # goal succeeds; if False, the goal succeeds as soon as the final
        # waypoint's condition is met, even if joints are still moving
        add_param("settle_before_return", False)
        add_param("settle_duration", 0.25)
        # if False, trajectories that include BASE_ROTATION_JOINT fail
        add_param("allow_rotation", False)
        # the base turns at a feedforward speed plus kp * (heading error to the active
        # waypoint), capped at max_velocity (rad/s); after the last waypoint it keeps
        # correcting until within final_tolerance (rad) or settle_timeout (s) passes
        add_param("base_rotation.kp", 2.0)
        add_param("base_rotation.max_velocity", 0.5)
        add_param("base_rotation.final_tolerance", 0.01)
        add_param("base_rotation.settle_timeout", 2.0)
        # per-joint tolerances (trajectory_tolerance.<joint>) are declared by the driver,
        # except the base rotation's, which isn't a driver joint
        add_param(f"trajectory_tolerance.{self.BASE_ROTATION_JOINT}", 0.02)

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

    def _check_joints_in_mode(self, joint_names, required_mode, goal_handle):
        for joint_name in joint_names:
            if joint_name == self.BASE_ROTATION_JOINT:
                continue  # driven by base velocity in every mode
            try:
                j_mode = self.driver.get_parameter(f"joint_mode.{joint_name}").value
            except Exception:
                j_mode = None
            if j_mode != required_mode:
                self.driver.get_logger().error(
                    f"Cannot execute trajectory because joint {joint_name} is in "
                    f"'{j_mode}' mode (must be in '{required_mode}' mode). It can be set with "
                    f"the parameter joint_mode.{joint_name}"
                )
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                result.error_string = (
                    f"Joint {joint_name} is not in {required_mode} mode"
                )
                goal_handle.abort()
                return result
        return None


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
        if self.driver.robot_mode() != self.driver.active_mode:
            self.active_joints = None
            self.driver.get_logger().warn(
                f"Goal canceled because robot mode is {self.driver.robot_mode()} (must be '{self.driver.active_mode}')."
            )
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
            joint_pose = self._joint_position(joint_name)
            joint_vel = self._joint_velocity(joint_name)
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

    def _joint_position(self, joint_name):
        if joint_name == self.BASE_ROTATION_JOINT:
            return self._base_rotation()
        return self.driver.cmd_joint_position(joint_name)

    def _joint_velocity(self, joint_name):
        if joint_name == self.BASE_ROTATION_JOINT:
            return self.driver.last_odom.twist.twist.angular.z
        return self.driver.cmd_joint_velocity(joint_name)

    def _odom_heading(self):
        q = self.driver.last_odom.pose.pose.orientation
        return np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _start_base_rotation(self):
        """Make the base's current heading BASE_ROTATION_JOINT's zero."""
        self._last_heading = self._odom_heading()
        self._rotation_since_start = 0.0

    def _base_rotation(self):
        """How far the base has turned since _start_base_rotation, without wrapping at pi."""
        heading = self._odom_heading()
        self._rotation_since_start += np.remainder(heading - self._last_heading + np.pi, 2 * np.pi) - np.pi
        self._last_heading = heading
        return self._rotation_since_start

    def _command_base_rotation(self, target, feedforward=0.0):
        """Turn the base toward `target` (BASE_ROTATION_JOINT position) at `feedforward`
        (rad/s) plus base_rotation.kp times the error."""
        self._set_base_rotation_rate(feedforward + self.get_param("base_rotation.kp") * (target - self._base_rotation()))

    def _set_base_rotation_rate(self, velocity):
        """Turn the base at `velocity` (rad/s), capped at base_rotation.max_velocity."""
        max_velocity = self.get_param("base_rotation.max_velocity")
        self.driver.set_base_velocity(0.0, 0.0, float(np.clip(velocity, -max_velocity, max_velocity)))

    def _settle_base_rotation(self, target, goal_handle, rate):
        """Hold the base's heading control on the final waypoint until it's within
        base_rotation.final_tolerance (or base_rotation.settle_timeout passes), then stop.
        Returns an abort result if the goal is interrupted, else None."""
        deadline = self.driver.get_clock().now() + Duration(seconds=self.get_param("base_rotation.settle_timeout"))
        try:
            while abs(target - self._base_rotation()) > self.get_param("base_rotation.final_tolerance"):
                interrupt_result = self._check_for_interrupt(goal_handle)
                if interrupt_result is not None:
                    return interrupt_result
                if self.driver.get_clock().now() > deadline:
                    self.driver.get_logger().warn(
                        f"Base heading is {target - self._base_rotation():.3f} rad from the final waypoint")
                    return None
                with self._server_commands():
                    self._command_base_rotation(target)
                rate.sleep()
            return None
        finally:
            self._stop_base()

    def _stop_base(self):
        with self._server_commands():
            self.driver.set_base_velocity(0.0, 0.0, 0.0)

    def _abort(self, goal_handle, error_code, error_string):
        self.active_joints = None
        self.driver.get_logger().warn(f"Goal aborted: {error_string}")
        goal_handle.abort()
        result = FollowJointTrajectory.Result()
        result.error_code = error_code
        result.error_string = error_string
        return result

    @staticmethod
    def _msg_seconds(duration_msg):
        return duration_msg.sec + duration_msg.nanosec*1e-9

    def _seconds_since(self, start_time):
        return (self.driver.get_clock().now() - start_time).nanoseconds*1e-9

    @contextmanager
    def _server_commands(self):
        """Mark joint commands sent inside this block (on this thread) as coming from this
        server, so the driver doesn't treat them as direct commands that preempt it."""
        source = self.driver._command_source
        previous = getattr(source, "server", None)
        source.server = self
        try:
            yield
        finally:
            source.server = previous

    def preempt(self, joint_name):
        """Called by the driver when another source commands a joint of the running goal."""
        self.preempted_joints.add(joint_name)
        self.direct_command_preempted = True

    def follow_trajectory(self, next_point_condition, first_loop_command, every_loop_command, trajectory, goal_handle, settle_on_position):
        self.driver.logger.info("Starting to follow trajectory")
        start_time = self.driver.get_clock().now()
        rate = self.driver.create_rate(self.get_param("loop_rate"))
        timeout = self.get_param("timeout")
        last_point_i = len(trajectory.points) - 1

        # While the trajectory runs, a command to any of its joints from outside the
        # server (e.g. /joint_position_cmd) preempts it; see note_joint_command.
        self.direct_command_preempted = False
        self.preempted_joints = set()
        self.active_joints = list(trajectory.joint_names)
        base_i = (trajectory.joint_names.index(self.BASE_ROTATION_JOINT)
                  if self.BASE_ROTATION_JOINT in trajectory.joint_names else None)
        if base_i is not None:
            self._start_base_rotation()

        point_i = 0
        feedback_msg = self._get_feedback(trajectory.points[0], 0.0, trajectory.joint_names)

        # The trajectory starts from the robot's current state at t=0, so the
        # segment to the first waypoint is interpolated like any other segment.
        prev_state = JointTrajectoryPoint()
        prev_state.positions = list(feedback_msg.actual.positions)
        prev_state.velocities = list(feedback_msg.actual.velocities)
        prev_state.time_from_start = Duration(seconds=0.0).to_msg()

        while point_i <= last_point_i:
            active_point = trajectory.points[point_i]
            next_state = trajectory.points[point_i+1] if point_i < last_point_i else None
            first = True
            self.driver.logger.debug(f"On point {point_i}")

            # A waypoint times out if it is still unsatisfied `timeout` seconds after
            # its scheduled time (or after we started on it, if that is later).
            deadline = max(self._seconds_since(start_time), self._msg_seconds(active_point.time_from_start)) + timeout

            while True:
                interrupt_result = self._check_for_interrupt(goal_handle)
                if interrupt_result is not None:
                    return interrupt_result

                elapsed_time = self.driver.get_clock().now()-start_time
                if elapsed_time.nanoseconds*1e-9 > deadline:
                    return self._abort(goal_handle, FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                                       f"Timed out after {timeout}s waiting to reach waypoint {point_i}")

                if first:
                    pos, vel, acc = first_loop_command(active_point, prev_state, next_state, feedback_msg)
                    first = False
                else:
                    pos, vel, acc = every_loop_command(active_point, prev_state, next_state, feedback_msg)

                command = JointState()
                command.name = trajectory.joint_names
                command.position = pos if pos is not None else []
                command.velocity = vel if vel is not None else []
                command.effort = acc if acc is not None else []

                self.pub_commands.publish(command)

                succeeded = True
                with self._server_commands():
                    if base_i is not None:
                        # The base is driven every loop. In adaptive_velocity mode the
                        # interpolated velocity already tracks the spline (feedforward plus kp
                        # feedback), so it's sent as is; adding feedback toward the waypoint
                        # too makes the speed sawtooth at every waypoint. In the position modes
                        # it turns toward the waypoint plus the segment's average speed, so it
                        # passes through waypoints at speed instead of slowing to each one.
                        if pos is None and vel is not None:
                            self._set_base_rotation_rate(vel[base_i])
                        else:
                            segment_time = (self._msg_seconds(active_point.time_from_start)
                                            - self._msg_seconds(prev_state.time_from_start))
                            segment_distance = active_point.positions[base_i] - prev_state.positions[base_i]
                            feedforward = segment_distance / segment_time if segment_time > 0.0 else 0.0
                            self._command_base_rotation(active_point.positions[base_i], feedforward)

                    for joint_i, joint_name in enumerate(trajectory.joint_names):
                        if joint_i == base_i:
                            continue
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
                    return self._abort(goal_handle, FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                                       "Joint command failed in strict mode")

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

                if next_point_condition(feedback_msg):
                    prev_state = copy.deepcopy(feedback_msg.desired)
                    break

                rate.sleep()

            # Skip ahead past any intermediate waypoints that are already satisfied.
            # The final waypoint is never skipped: it always gets commanded, so the
            # robot's last command is always the trajectory's actual goal.
            point_i += 1
            while point_i < last_point_i:
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

        if not settle_on_position:
            # In velocity mode the last waypoint is "reached" when its time comes, wherever
            # the joints are; pull them onto it before stopping.
            converge_result = self._converge_on_final_point(trajectory, goal_handle, rate, base_i)
            if converge_result is not None:
                return converge_result

        if base_i is not None:
            # The final waypoint counts as reached within the trajectory tolerance, while
            # the base may still be turning; finish the turn more precisely, then stop.
            settle_result = self._settle_base_rotation(trajectory.points[-1].positions[base_i], goal_handle, rate)
            if settle_result is not None:
                return settle_result

        if self.get_param("settle_before_return"):
            if not settle_on_position:
                # Velocity-controlled joints keep moving at their last commanded
                # velocity, so stop them before waiting for the robot to settle.
                self.stop_vel_joints(trajectory.joint_names)

            settle_result = self._wait_until_settled(trajectory, goal_handle, start_time, rate, settle_on_position)
            if settle_result is not None:
                return settle_result

        # Successful Completion
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Completed trajectory successfully"
        self.driver.get_logger().info('Goal succeeded.')
        return result

    def _converge_on_final_point(self, trajectory, goal_handle, rate, base_i):
        """Velocity modes: command kp * (position error) toward the final waypoint until every
        joint is within final_convergence.tolerance_fraction of its trajectory tolerance, or
        final_convergence.timeout passes. The base rotation is left to _settle_base_rotation.
        Returns an abort result if interrupted, else None."""
        joints = [(joint, target) for i, (joint, target)
                  in enumerate(zip(trajectory.joint_names, trajectory.points[-1].positions)) if i != base_i]
        fraction = self.get_param("final_convergence.tolerance_fraction")
        thresholds = [fraction * self._tolerance(joint) for joint, _ in joints]
        kp = self.get_param("kp")
        deadline = self.driver.get_clock().now() + Duration(seconds=self.get_param("final_convergence.timeout"))

        while True:
            interrupt_result = self._check_for_interrupt(goal_handle)
            if interrupt_result is not None:
                return interrupt_result
            errors = [target - self._joint_position(joint) for joint, target in joints]
            if all(abs(e) < t for e, t in zip(errors, thresholds)):
                return None
            if self.driver.get_clock().now() > deadline:
                self.driver.get_logger().warn(
                    f"Joints still off the final waypoint: "
                    f"{ {joint: round(e, 4) for (joint, _), e in zip(joints, errors)} }")
                return None
            with self._server_commands():
                for (joint, _), error in zip(joints, errors):
                    self.driver.check_and_set_vel(joint, kp * error)
            rate.sleep()

    def _tolerance(self, joint_name):
        tolerance = self.driver.get_parameter_or(f"{self.param_prefix}.trajectory_tolerance.{joint_name}").value
        return self.get_param("default_tolerance") if tolerance is None else tolerance

    def _wait_until_settled(self, trajectory, goal_handle, start_time, rate, check_position):
        """Wait for the robot to stop moving at the end of the trajectory.

        The final waypoint's condition can be met while joints are still moving
        (e.g. in target_priority mode, the moment a moving joint passes within
        tolerance of the goal), so success is only reported once every joint has
        stayed below settling.vel_threshold for trajectory_server.settle_duration
        seconds -- and, if check_position is set, has also stayed within tolerance
        of the final waypoint.

        Requiring the condition to hold for settle_duration (rather than a single
        check) also covers the lag between sending the final command and seeing
        the robot start to move in the joint state.

        Returns None once settled, or an abort result on timeout.
        """
        final_point = trajectory.points[-1]
        vel_threshold = self.driver.get_parameter("settling.vel_threshold").value
        settle_duration = self.get_param("settle_duration")
        timeout = self.get_param("timeout")
        deadline = self._seconds_since(start_time) + timeout
        settled_since = None

        while True:
            interrupt_result = self._check_for_interrupt(goal_handle)
            if interrupt_result is not None:
                return interrupt_result

            elapsed = self._seconds_since(start_time)
            feedback_msg = self._get_feedback(final_point, elapsed, trajectory.joint_names)
            goal_handle.publish_feedback(feedback_msg)

            stopped = all(abs(v) < vel_threshold for v in feedback_msg.actual.velocities)
            in_position = self.at_target(feedback_msg) if check_position else True

            if stopped and in_position:
                if settled_since is None:
                    settled_since = elapsed
                if elapsed - settled_since >= settle_duration:
                    return None
            else:
                settled_since = None

            if elapsed > deadline:
                return self._abort(goal_handle, FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                                   f"Joints did not settle at the final waypoint within {timeout}s "
                                   f"(stopped: {stopped}, in position: {in_position})")

            rate.sleep()

    def stop_vel_joints(self, joint_names):
        """Stop the goal's joints when it ends: velocity-mode joints at zero velocity,
        position-mode joints at their current position. Joints another source took over
        (preempted_joints) are left to it."""
        for joint_name in joint_names:
            if joint_name in self.preempted_joints:
                continue
            if joint_name == self.BASE_ROTATION_JOINT:
                self._stop_base()
                continue
            try:
                j_mode = self.driver.get_parameter(f"joint_mode.{joint_name}").value
            except Exception:
                j_mode = None
            with self._server_commands():
                if j_mode == self.driver.velocity_mode:
                    succeeded = self.driver.check_and_set_vel(joint_name, 0.0)
                elif j_mode == self.driver.position_mode and self._joint_position(joint_name) is not None:
                    succeeded = self.driver.check_and_set_pos(joint_name, self._joint_position(joint_name))
                else:
                    succeeded = False
            if not succeeded:
                self.driver.get_logger().warning(f"Stop command to {joint_name} ({j_mode} mode) failed.")

    def at_time(self, feedback_msg):
        actual_time = feedback_msg.actual.time_from_start.sec + feedback_msg.actual.time_from_start.nanosec*1e-9
        desired_time = feedback_msg.desired.time_from_start.sec + feedback_msg.desired.time_from_start.nanosec*1e-9
        
        return actual_time >= desired_time

    def at_target(self, feedback_msg):
        all_joints_ok = True
        errors = [0.0 for joint in feedback_msg.joint_names]
        tols = [0.0 for joint in feedback_msg.joint_names]

        for i, joint_name in enumerate(feedback_msg.joint_names):
            tolerance = self.driver.get_parameter_or(f"{self.param_prefix}.trajectory_tolerance.{joint_name}").value
            tolerance = self.get_param("default_tolerance") if tolerance is None else tolerance
            all_joints_ok = all_joints_ok and abs(feedback_msg.desired.positions[i]-feedback_msg.actual.positions[i])<tolerance
            errors[i] = abs(feedback_msg.desired.positions[i]-feedback_msg.actual.positions[i])
            tols[i] = tolerance
            
        self.driver.get_logger().info(f"Joint/Error/Tolerance: {list(zip(feedback_msg.joint_names,list(map(lambda x: round(x,4),errors)),list(map(lambda x: round(x,4),tols))))}", throttle_duration_sec=1.0)

        return all_joints_ok

    def active_point_goal(self, active_point, prev_pt, next_pt, feedback_msg):
        pos = active_point.positions if len(active_point.positions) > 0 else None
        vel = active_point.velocities if len(active_point.velocities) > 0 else None
        acc = active_point.accelerations if len(active_point.accelerations) > 0 else None

        return pos, vel, acc

    def interpolate(self, active_point, prev_state, next_state, feedback_msg):
        now = feedback_msg.actual.time_from_start.sec + feedback_msg.actual.time_from_start.nanosec*1e-9
        end_t = active_point.time_from_start.sec + active_point.time_from_start.nanosec*1e-9
        start_t = self._msg_seconds(prev_state.time_from_start)
        prop_elapsed = (now-start_t)/(end_t-start_t) if end_t != start_t else 1.0
        secs_remaining = end_t-now
        target_poses = []
        target_vels = []
        target_accs = []

        dt = 1.0/self.get_param("loop_rate")

        
        
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

            
            if prev_state.velocities is not None and len(prev_state.velocities) > 0:
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
                    d1 = active_point.positions[i] - prev_state.positions[i]

                    t0 = self._msg_seconds(prev_state.time_from_start)
                    t1 = self._msg_seconds(active_point.time_from_start)
                    t2 = self._msg_seconds(next_state.time_from_start)

                    dt2 = t2-t1
                    dt1 = t1-t0

                    slope1 = (d1)/(dt1) if dt1 != 0.0 else 0.0
                    slope2 = (d2)/(dt2) if dt2 != 0.0 else 0.0

                    end_vel = 0.5*(slope1+slope2)                    
                else:
                    self.driver.logger.warning("No end velocity specified and unknown velocity inference method. Setting waypoint target velocity to zero")
                    end_vel = 0.0

                           
            start_pos = prev_state.positions[i]
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

            # 4. Feedforward + Feedback Command Calculation. The velocity command pulls
            # the joint back toward the spline's position, so tracking errors don't build
            # up over a long trajectory; clamp it to the maxv bound.
            kp = self.get_param("kp")
            kd = 0.0
            next_vel = np.clip(vel_d + kp * (pos_d - current_pos), -maxv, maxv)

            next_acc = accel_ff + kd * (next_vel - current_vel)

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

        if self.BASE_ROTATION_JOINT in goal_handle.request.trajectory.joint_names:
            # Checked before the try block so its cleanup doesn't stop a base that
            # someone else is driving.
            if not self.get_param("allow_rotation"):
                return self._abort(goal_handle, FollowJointTrajectory.Result.INVALID_JOINTS,
                                   f"Trajectory includes {self.BASE_ROTATION_JOINT} but "
                                   f"{self.param_prefix}.allow_rotation is false")
            if self.driver.last_odom is None:
                return self._abort(goal_handle, FollowJointTrajectory.Result.INVALID_GOAL,
                                   f"No odometry yet for {self.BASE_ROTATION_JOINT}")

        try:
            trajectory = goal_handle.request.trajectory
            resolved = [
                self.driver.resolve_joint_name(n) for n in trajectory.joint_names
            ]
            if resolved != list(trajectory.joint_names):
                self.driver.get_logger().info(
                    f"follow_joint_trajectory: resolved joint_names "
                    f"{list(trajectory.joint_names)} to {resolved}.",
                    throttle_duration_sec=30.0,
                )
                trajectory.joint_names = resolved
            match mode:
                case "adaptive_velocity":
                    result = self._check_joints_in_mode(
                        goal_handle.request.trajectory.joint_names,
                        self.driver.velocity_mode,
                        goal_handle,
                    )
                    if result is not None:
                        return result
                    next_point_condition = self.at_time
                    first_loop_command = self.interpolate_velocities
                    every_loop_command = self.interpolate_velocities
                    settle_on_position = False
                case "time_priority":
                    result = self._check_joints_in_mode(
                        goal_handle.request.trajectory.joint_names,
                        self.driver.position_mode,
                        goal_handle,
                    )
                    if result is not None:
                        return result
                    next_point_condition = self.at_time
                    first_loop_command = self.active_point_goal
                    every_loop_command = lambda *args, **kwargs: [None, None, None]
                    settle_on_position = True
                case "target_priority":
                    result = self._check_joints_in_mode(
                        goal_handle.request.trajectory.joint_names,
                        self.driver.position_mode,
                        goal_handle,
                    )
                    if result is not None:
                        return result
                    next_point_condition = self.at_target
                    first_loop_command = self.active_point_goal
                    every_loop_command = lambda *args, **kwargs: [None, None, None]
                    settle_on_position = True


            result = self.follow_trajectory(next_point_condition, first_loop_command, every_loop_command, trajectory, goal_handle, settle_on_position)
        except Exception as e:
            self.driver.get_logger().error(
                f"Cannot execute trajectory in {mode} mode because of error {e}.\n"
                f"{traceback.format_exc()}")
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string = f"Error {e} occurred."
            goal_handle.abort()
        finally:            
            #cleanup:
            self.stop_vel_joints(trajectory.joint_names)
            self.active_joints = None
                
        return result
        
