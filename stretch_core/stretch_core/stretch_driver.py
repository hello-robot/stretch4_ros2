#! /usr/bin/env python3

import sys
import copy
import time
from typing import Any
from threading import Lock
from functools import partial

from .joint_trajectory_server import JointTrajectoryAction
import stretch4_body.robot.robot_client as rc

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.publisher import Publisher

from std_srvs.srv import Trigger, SetBool

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from sensor_msgs.msg import BatteryState, JointState, Imu, MagneticField, Joy
from std_msgs.msg import Bool, String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from control_msgs.msg import JointJog

import tf2_ros
from tf_transformations import quaternion_from_euler

import hello_helpers.joy_conversion as jc


class StretchDriver(Node):

    def __init__(self):
        super().__init__('stretch_driver')
        self.get_logger().info("For use with S T R E T C H (TM) RESEARCH EDITION from Hello Robot Inc.")

        # Low level API
        self.robot = rc.RobotClient(client_id="ros2_driver")

        if not self.robot.startup():
            self.robot.logger.fatal('Robot startup failed.')
            rclpy.shutdown()
            sys.exit(1)

        # Warn if robot isn't homed
        if not self.robot.is_homed():
            self.robot.logger.warn('Robot is not homed.')

        # Driver Mode
        self.declare_parameter('mode', "navigation")
        mode = self.get_parameter('mode').value
        self.control_modes = ['position', 'velocity', 'navigation', 'teleop']
        self.priority_modes = ['homing', 'stowing']
        if mode not in self.control_modes:
            self.robot.logger.warn(f'given invalid mode={mode}, using navigation instead')
            mode = 'navigation'
        self.driver_mode = mode
        self.driver_mode_lock = Lock()
        self.robot.logger.info('mode = ' + str(mode))

        # Robot sensitivity (TODO: read sensitivity mode from the robot)
        self.declare_parameter('sensitivity', "default")
        self.sensitivity = self.get_parameter('sensitivity').value
        self.robot.logger.info('sensitivity = ' + str(self.sensitivity))

        # Runstop management
        self.prev_runstop_state = None
        self.prerunstop_mode = None

        # Callback groups
        self.main_group = ReentrantCallbackGroup()
        self.mutex_group = MutuallyExclusiveCallbackGroup()

        # Publishers
        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.odom_pub = self.create_publisher(Odometry, 'wheel_odom', 1)
        self.homed_pub = self.create_publisher(Bool, 'is_homed', latching_qos)
        self.mode_pub = self.create_publisher(String, 'mode', latching_qos)
        self.tool_pub = self.create_publisher(String, 'tool', latching_qos)
        self.sensitivity_pub = self.create_publisher(String, 'sensitivity', latching_qos)
        self.runstop_event_pub = self.create_publisher(Bool, 'is_runstopped', latching_qos)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 1)
        self.battery_pub = self.create_publisher(BatteryState, 'battery', 1)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 1) # Diagnostics are centralized, so we publish to a single global /diagnostics topic
        self.lease_holder_pub = self.create_publisher(DiagnosticStatus, 'server_lease_holder', 1)
        self.joint_state_diagnostics_pub = self.create_publisher(DiagnosticArray, 'joint_states_diagnostics', 1) 
    
        # Saved Message States (for latched topics)
        self.last_published_value = {}

        # Subscribers
        self.create_subscription(Twist, "cmd_vel", self.twist_callback, 1, callback_group=self.main_group)
        self.create_subscription(JointJog, "joint_vel", self.velocity_callback, 1, callback_group=self.main_group)
        self.create_subscription(Joy, "joy", self.joy_callback, 1, callback_group=self.main_group)

        # Velocity Control
        self.set_vel_functions = {}

        if hasattr(self.robot, 'lift'):
            self.set_vel_functions['lift_joint'] = lambda v, a:  self.robot.lift.set_velocity(v, a_m=a)
            self.declare_parameter("joint_acceleration.lift",self.robot.robot_params['lift']['motion']['default']['accel_m'])
        if hasattr(self.robot, 'arm'):
            self.set_vel_functions['arm_joint'] = lambda v, a:  self.robot.arm.set_velocity(v, a_m=a)
            self.declare_parameter("joint_acceleration.arm",self.robot.robot_params['arm']['motion']['default']['accel_m'])
        if hasattr(self.robot, 'end_of_arm') and hasattr(self.robot.end_of_arm, 'joints'):
            for joint in self.robot.end_of_arm.joints: 
                self.set_vel_functions[f'{joint}_joint']= lambda d, a, j=joint: self.robot.end_of_arm.quick_stop(j) if d == 0.0 else self.robot.end_of_arm.move_by(j, d, a_r = a)
                self.declare_parameter(f"joint_acceleration.{joint}",self.robot.robot_params[joint]['motion']['default']['accel'])

        self.declare_parameter("joint_acceleration.omnibase.linear", self.robot.robot_params['omnibase']['motion']['default']['accel_xy_m'])
        self.declare_parameter("joint_acceleration.omnibase.angular", self.robot.robot_params['omnibase']['motion']['default']['accel_w_r'])

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

        # TF2
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Namespace
        ns = self.get_namespace().strip('/')
        self.prefix = ns + '/' if ns else ''
        if ns:
            self.robot.logger.info(f"namespace = {self.prefix}")

        # Parameters (see also 'mode' parameter above)
        self.declare_parameter('broadcast_odom_tf', False) # based on wheel odometry
        self.broadcast_odom_tf = self.get_parameter('broadcast_odom_tf').value
        self.robot.logger.info(f'broadcast_odom_tf = {self.broadcast_odom_tf}')

        self.declare_parameter('action_timeout', 3.0, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Default timeout (sec) for execution of joint traj action',
        ))
        action_timeout = self.get_parameter('action_timeout').value
        self.action_timeout_duration = Duration(seconds=action_timeout)
        self.robot.logger.info(f"action timeout = {action_timeout} s")

        self.declare_parameter('velocity_timeout', 0.5, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Default timeout (sec) for velocity control',
        ))
        velocity_timeout = self.get_parameter('velocity_timeout').value
        self.velocity_timeout_duration = Duration(seconds=velocity_timeout)
        self.robot.logger.info(f"velocity timeout = {velocity_timeout} s")

        self.add_on_set_parameters_callback(self.parameter_callback)

        # Actions
        self.joint_trajectory_action = JointTrajectoryAction(self)

        # Start timer for control loop
        self.timer = self.create_timer(
            1.0 / 100,
            self.control_loop,
            callback_group=self.mutex_group,
        )

    def change_mode(self, new_mode):
        with self.driver_mode_lock:
            self.driver_mode = new_mode
        self.robot.logger.info(f'Changed to mode = {self.driver_mode}')

    def home_the_robot(self):
        with self.driver_mode_lock:
            can_home = self.driver_mode in self.control_modes
            last_driver_mode = copy.copy(self.driver_mode)
        if not can_home:
            errmsg = f'Cannot home while in mode={last_driver_mode}.'
            self.robot.logger.error(errmsg)
            return False, errmsg
        self.change_mode('homing')
        self.robot.home(do_push=False, do_pull=False)
        self.change_mode(last_driver_mode)
        return True, 'Homed.'

    def stow_the_robot(self):
        with self.driver_mode_lock:
            can_stow = self.driver_mode in self.control_modes
            last_driver_mode = copy.copy(self.driver_mode)
        if not can_stow:
            errmsg = f'Cannot stow while in mode={last_driver_mode}.'
            self.robot.logger.error(errmsg)
            return False, errmsg
        self.change_mode('stowing')
        self.robot.stow(do_push=False, do_pull=False)
        self.change_mode(last_driver_mode)
        return True, 'Stowed.'

    def runstop_the_robot(self, runstopped, just_change_mode=False):
        if runstopped:
            with self.driver_mode_lock:
                already_runstopped = self.driver_mode == 'runstopped'
                if not already_runstopped:
                    self.prerunstop_mode = copy.copy(self.driver_mode)
            if already_runstopped:
                return
            self.change_mode('runstopped')
            if not just_change_mode:
                self.robot.power_periph.trigger_runstop()
        else:
            with self.driver_mode_lock:
                already_not_runstopped = self.driver_mode != 'runstopped'
            if already_not_runstopped:
                return
            self.change_mode(self.prerunstop_mode)
            if not just_change_mode:
                self.robot.power_periph.clear_runstop()

    def stop_the_robot_callback(self, request, response):
        with self.driver_mode_lock:
            self.robot.omnibase.rotate_by(0.0)
            self.robot.lift.move_by(0.0)

            if hasattr(self.robot, 'arm'):
                self.robot.arm.move_by(0.0)

            if hasattr(self.robot, 'end_of_arm') and hasattr(self.robot.end_of_arm, 'joints'):
                for joint in self.robot.end_of_arm.joints:
                    self.robot.end_of_arm.move_by(joint, 0.0)

        self.robot.logger.info('Received stop_the_robot service call, so commanded all actuators to stop.')
        response.success = True
        response.message = 'Stopped the robot.'
        return response

    def home_the_robot_callback(self, request, response):
        self.robot.logger.debug('Received home_the_robot service call.')
        did_succeed, msg = self.home_the_robot()
        response.success = did_succeed
        response.message = msg
        return response

    def stow_the_robot_callback(self, request, response):
        self.robot.logger.debug('Received stow_the_robot service call.')
        did_succeed, msg = self.stow_the_robot()
        response.success = did_succeed
        response.message = msg
        return response

    def runstop_service_callback(self, request, response):
        self.runstop_the_robot(request.data)
        response.success = True
        response.message = 'is_runstopped: {0}'.format(request.data)
        return response

    def velocity_callback(self, jointjog_msg: JointJog):
        """
        Velocity control for the ranged joints on Stretch. Velocity control of the mobile base is handled by /cmd_vel.
        Only the JointJog.velocities[] is parsed. JointJog.displacements[] is ignored. JointJog.duration is the timeout
        before the robot stops moving. It cannot exceed velocity_timeout parameter.
        """
        with self.driver_mode_lock:
            if self.driver_mode != 'velocity':
                self.robot.logger.warn(f'Must be in velocity mode to service JointJog msg. Current mode = {self.driver_mode}.')
                return

        # Queue velocity commands
        for i, joint in enumerate(jointjog_msg.joint_names):
            
            if joint not in self.set_vel_functions.keys():
                raise AttributeError(f"Received velocity command for unexpected joint: {joint}")

            acceleration_param = self.get_parameter_or(f"joint_acceleration.{joint.split("_joint")[0]}",None).value

            if "gripper" in joint: 
                jointjog_msg.velocities[i] *= 300

            self.set_vel_functions[joint](jointjog_msg.velocities[i], acceleration_param)

        # Set timeout (TODO)
        self.robot.logger.debug(str(self.robot.cmd_dict))

    def joy_callback(self, joy_msg: Joy):
        """
        During teleop mode, the robot can be controlled with anything that can publish Joy messages on the /joy topic.
        e.g., VR, dex teleop, Rviz joy panel, puppet robot. The Joy message should follow the format described in hello_utils.joy_conversion
        """
        with self.driver_mode_lock:
            if self.driver_mode != 'teleop':
                self.robot.logger.warn(f'Must be in teleop mode to service Joy msg. Current mode = {self.driver_mode}.')
                return

        state = jc.unpack_joy_to_gamepad_state(joy_msg)
        tool_name = self.robot.params['tool']
        Idx = jc.get_Idx(tool_name)

        # Standard Scaling
        MAX_VEL = 0.2
        DEADZONE = 0.05

        def get_val(axis_name, scale):
            val = state.get(axis_name, 0.0)
            return val * scale if abs(val) > DEADZONE else 0.0

        self.robot.lift.set_velocity(get_val('left_stick_y', MAX_VEL))

        if hasattr(self.robot, 'arm'):
            self.robot.arm.set_velocity(get_val('left_stick_x', MAX_VEL))

        if hasattr(Idx, 'WRIST_YAW'):
            self.robot.end_of_arm.set_velocity('wrist_yaw', get_val('right_stick_x', MAX_VEL))        
        if hasattr(Idx, 'WRIST_PITCH'):
            # Dex Wrist or DW4 tools
            self.robot.end_of_arm.set_velocity('wrist_pitch', get_val('right_stick_y', MAX_VEL))
        if hasattr(Idx, 'WRIST_ROLL'):
            roll_vel = 0.0
            if state['right_shoulder_button_pressed']: roll_vel = MAX_VEL
            if state['left_shoulder_button_pressed']: roll_vel = -MAX_VEL
            self.robot.end_of_arm.set_velocity('wrist_roll', roll_vel)
        if hasattr(Idx, 'GRIPPER') and 'stretch_gripper' in self.robot.end_of_arm.joints:
            grip_vel = 0.0
            if state['top_button_pressed']: grip_vel = MAX_VEL   # Open
            if state['bottom_button_pressed']: grip_vel = -MAX_VEL # Close
            self.robot.end_of_arm.set_velocity('stretch_gripper', grip_vel)

        # --- Base Control ---
        # self.robot.base.set_velocity(drive, turn) # TODO

    def twist_callback(self, twist: Twist):
        with self.driver_mode_lock:
            if self.driver_mode not in ['navigation', 'velocity']:
                self.robot.logger.warn(f'Must be in {['navigation', 'velocity']} modes to service Twist msg. Current mode = {self.driver_mode}.')
                return
        linear_acc = self.get_parameter_or("joint_acceleration.omnibase.linear",None).value
        angular_acc = self.get_parameter_or("joint_acceleration.omnibase.angular",None).value
        self.robot.omnibase.set_velocity(vx_m = twist.linear.x, 
                                         vy_m = twist.linear.y, 
                                         w_r = twist.angular.z, 
                                         a_m = linear_acc, 
                                         a_r = angular_acc
                                        )

    def update_latched_value(self, pub: Publisher, value: Any):
        key = pub.topic
        if value == self.last_published_value.get(key):
            return
        msg = pub.msg_type()
        msg.data = value
        pub.publish(msg)
        self.last_published_value[key] = value

    def control_loop(self):
        # Capture driver mode
        current_mode = None
        with self.driver_mode_lock:
            current_mode = self.driver_mode

        ##################################################
        # Publish status
        self.robot.pull_status()
        robot_status = copy.deepcopy(self.robot.status) # get copy of the current robot status
        # use current time as stamp
        current_clock = self.get_clock().now()
        current_time = current_clock.to_msg()

        # publish server lease holder information
        lease_holder_msg = DiagnosticStatus()
        lease_holder_msg.name = self.prefix + 'server_lease_holder'
        lease_holder_msg.level = DiagnosticStatus.OK
        for key in ["lease_holder","lease_holder_priority","lease_expiry"]:
            lease_holder_msg.values.append(KeyValue(key=key, value=str(robot_status['server'][key])))
        lease_holder_msg.values.append(KeyValue(key="lease_expired", value=str(time.monotonic() > robot_status['server']['lease_expiry'])))
        lease_holder_msg.values.append(KeyValue(key="routine_active", value=str(self.robot.routines.status['active_routine'] != 'routine_nop')))
        self.lease_holder_pub.publish(lease_holder_msg)

        # obtain odometry
        x = float(robot_status['omnibase']['x'])
        y = float(robot_status['omnibase']['y'])
        theta = float(robot_status['omnibase']['theta'])
        x_vel = float(robot_status['omnibase']['x_vel'])
        y_vel = float(robot_status['omnibase']['y_vel'])
        theta_vel = float(robot_status['omnibase']['theta_vel'])

        q = quaternion_from_euler(0.0, 0.0, theta)

        if self.broadcast_odom_tf:
            # publish odometry via TF
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = self.prefix + 'wheel_odom'
            t.child_frame_id = self.prefix + 'base_footprint'
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

        # publish odometry via the wheel_odom topic
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.prefix + 'wheel_odom'
        odom.child_frame_id = self.prefix + 'base_footprint'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = x_vel
        odom.twist.twist.linear.y = y_vel
        odom.twist.twist.angular.z = theta_vel
        self.odom_pub.publish(odom)

        # publish homed status
        self.update_latched_value(self.homed_pub, bool(self.robot.is_homed()))

        # publish runstop event
        runstop_event_data = bool(robot_status['power_periph']['runstop_event'])
        self.update_latched_value(self.runstop_event_pub, runstop_event_data)

        # if (the driver starts with the robot already in runstop) or (the user has changed runstop state using the physical button)
        if (self.prev_runstop_state is None and runstop_event_data) or (self.prev_runstop_state is not None and runstop_event_data != self.prev_runstop_state):
            self.runstop_the_robot(runstop_event_data, just_change_mode=True) # just updates the driver's mode w/o calling the power_periph APIs
        self.prev_runstop_state = runstop_event_data

        # publish stretch_driver operation mode
        self.update_latched_value(self.mode_pub, current_mode)

        # publish end of arm tool
        self.update_latched_value(self.tool_pub, self.robot.params['tool'])

        # publish sensitivity
        self.update_latched_value(self.sensitivity_pub, self.sensitivity)

        # publish joint state and joint state diagnostics
        joint_state = JointState()
        joint_state.header.stamp = current_time

        joint_state_diagnostics = DiagnosticArray()
        joint_state_diagnostics.header.stamp = current_time

        at_limit_msg = DiagnosticStatus(name="at_limit")
        soft_limits_msg = DiagnosticStatus(name="soft_motion_limits")
        braking_distance_msg = DiagnosticStatus(name="braking_distance")
        is_homed_msg = DiagnosticStatus(name="is_homed")
        is_runstopped_msg = DiagnosticStatus(name="is_runstopped")

        for cg in self.joint_trajectory_action.command_groups:
            pos, vel, eff = cg.joint_state(robot_status)

            # add telescoping joints to joint state
            if cg.name == "arm_joint":
                for link in ['arm_l4_joint', 'arm_l3_joint', 'arm_l2_joint', 'arm_l1_joint']:
                    joint_state.name.append(link)
                    joint_state.position.append(pos/5.0)
                    joint_state.velocity.append(vel/5.0)
                    joint_state.effort.append(eff)
                # diagnostics for arm
                at_limit_msg.values.append(KeyValue(key=cg.name, value=f"{robot_status['arm']['at_limit']}"))
                soft_limits_msg.values.append(KeyValue(key=cg.name, value=f"{robot_status['arm']['soft_motion_limits']}"))
                braking_distance_msg.values.append(KeyValue(key=cg.name, value=f"{robot_status['arm']['braking_distance']}"))
                continue # arm expands into individual links; do not add joint_arm itself

            # add gripper joints to joint state
            if cg.name == "gripper_joint":
                for link in ['gripper_finger_left_joint', 'gripper_finger_right_joint']:
                    joint_state.name.append(link)
                    joint_state.position.append(pos)
                    joint_state.velocity.append(vel)
                    joint_state.effort.append(eff)
                # diagnostics for gripper
                gripper_status = robot_status['end_of_arm']['stretch_gripper']
                at_limit_msg.values.append(KeyValue(key=cg.name, value=f"{gripper_status['at_limit']}"))
                soft_limits_msg.values.append(KeyValue(key=cg.name, value=f"{gripper_status['soft_motion_limits']}"))
                braking_distance_msg.values.append(KeyValue(key=cg.name, value=f"{gripper_status['braking_distance']}"))
                continue # gripper expands into finger links; do not add joint_gripper itself

            # add wheel joints to joint state
            if cg.name == "translate_mobile_base":
                for w in ['wheel_0_joint', 'wheel_1_joint', 'wheel_2_joint']:
                    joint_state.name.append(w)
                    joint_state.position.append(0.0)
                    joint_state.velocity.append(0.0)
                    joint_state.effort.append(0.0)
                continue # skip adding mobile_base joint

            joint_state.name.append(cg.name)
            joint_state.position.append(pos)
            joint_state.velocity.append(vel)
            joint_state.effort.append(eff)

            joint_status_key = cg.name.replace("_joint","")
            if joint_status_key == "gripper":
                joint_status_key = "stretch_gripper"

            if joint_status_key in ["wrist_roll", "wrist_pitch", "wrist_yaw", "stretch_gripper"]:
                status = robot_status["end_of_arm"]
                is_homed = bool(status[joint_status_key]['pos_calibrated'])
            else: 
                status=robot_status
                is_homed = bool(status[joint_status_key]['motor']['pos_calibrated'])
                is_runstopped = bool(status[joint_status_key]['motor']['runstop_on'])

            at_limit_msg.values.append(KeyValue(key=cg.name, value=f"{status[joint_status_key]["at_limit"]}"))
            soft_limits_msg.values.append(KeyValue(key=cg.name, value=f"{status[joint_status_key]["soft_motion_limits"]}"))
            braking_distance_msg.values.append(KeyValue(key=cg.name, value=f"{status[joint_status_key]["braking_distance"]}"))
            is_homed_msg.values.append(KeyValue(key=cg.name, value=f"{is_homed}"))
            is_runstopped_msg.values.append(KeyValue(key=cg.name, value=f"{is_runstopped}"))

        joint_state_diagnostics.status.append(is_runstopped_msg)
        joint_state_diagnostics.status.append(is_homed_msg)
        joint_state_diagnostics.status.append(at_limit_msg)
        joint_state_diagnostics.status.append(soft_limits_msg)
        joint_state_diagnostics.status.append(braking_distance_msg)

        self.joint_state_pub.publish(joint_state)
        self.joint_state_diagnostics_pub.publish(joint_state_diagnostics)

        # publish battery state
        battery_state = BatteryState()
        battery_state.header.stamp = current_time
        battery_state.voltage = float(robot_status['power_periph']['voltage'])
        battery_state.current = float(robot_status['power_periph']['battery_current'])
        battery_state.temperature = float(robot_status['power_periph']['temp'])
        battery_state.percentage = float(robot_status['power_periph']['battery_soc']) / 100.0

        if robot_status['power_periph']['adapter_connected']:
            if robot_status['power_periph']['charger_is_charging']:
                battery_state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            else:
                battery_state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        else:
            battery_state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        battery_state.present = True
        self.battery_pub.publish(battery_state)

        # publish safety layer diagnostics
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = current_time

        safety_status = robot_status.get('safety_layer', {})
        if safety_status:
            # Safe Motion Manager
            smm_status = safety_status.get('safe_motion_manager', {})
            smm_diag = DiagnosticStatus()
            smm_diag.name = self.prefix + 'safety_layer/safe_motion_manager'
            smm_diag.level = DiagnosticStatus.OK
            smm_diag.message = 'OK'

            smm_active = smm_status.get('active', {})
            for motion_name, is_active in smm_active.items():
                smm_diag.values.append(KeyValue(key=f"active/{motion_name}", value=str(is_active)))

                motion_data = smm_status.get(motion_name, {})
                for k, v in motion_data.items():
                    smm_diag.values.append(KeyValue(key=f"{motion_name}/{k}", value=str(v)))

            triggered = smm_status.get('safe_motions_triggered', [])
            smm_diag.values.append(KeyValue(key="safe_motions_triggered", value=str(triggered)))
            if triggered:
                smm_diag.level = DiagnosticStatus.WARN
                smm_diag.message = f"Triggered: {', '.join(triggered)}"
            diag_msg.status.append(smm_diag)

            # Sentry Manager
            sm_status = safety_status.get('sentry_manager', {})
            sm_diag = DiagnosticStatus()
            sm_diag.name = self.prefix + 'safety_layer/sentry_manager'
            sm_diag.level = DiagnosticStatus.OK
            sm_diag.message = 'OK'

            sm_active = sm_status.get('active', {})
            for sentry_name, is_active in sm_active.items():
                sm_diag.values.append(KeyValue(key=f"active/{sentry_name}", value=str(is_active)))

                sentry_data = sm_status.get(sentry_name, {})
                for k, v in sentry_data.items():
                    sm_diag.values.append(KeyValue(key=f"{sentry_name}/{k}", value=str(v)))
            diag_msg.status.append(sm_diag)

        self.diagnostics_pub.publish(diag_msg)

        # ##################################################
        # Push commands
        self.robot.push_command()

    def parameter_callback(self, parameters: list[Parameter]) -> SetParametersResult:
        """
        Update the parameters that allow for dynamic updates.
        """
        for parameter in parameters:
            if parameter.name == "mode":
                self.change_mode(parameter.value)
            if parameter.name == "sensitivity":
                # (e.g. 'off', 'default','high_sensitivity_nav', 'high_sensitivity_manipulation')
                if parameter.value not in self.robot.get_guarded_contact_modes():
                    return SetParametersResult(successful=False)
                self.sensitivity = parameter.value
                self.robot.set_guarded_contact_sensitivity(self.sensitivity)
                self.robot.logger.info(f'Changed to sensitivity = {self.sensitivity}')
            if parameter.name == "action_timeout":
                action_timeout = parameter.value
                self.action_timeout_duration = Duration(seconds=action_timeout)
                self.robot.logger.info(f"Changed to action timeout = {action_timeout} s")
            if parameter.name == "velocity_timeout":
                velocity_timeout = parameter.value
                self.velocity_timeout_duration = Duration(seconds=velocity_timeout)
                self.robot.logger.info(f"Changed to velocity timeout = {velocity_timeout} s")

        return SetParametersResult(successful=True)


def main():
    try:
        rclpy.init()
        executor = MultiThreadedExecutor(num_threads=5)
        node = StretchDriver()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            node.robot.stop()
            executor.shutdown()
            node.destroy_node()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
