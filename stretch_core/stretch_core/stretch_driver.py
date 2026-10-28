#! /usr/bin/env python3
from threading import Lock
import copy
import time
from math import copysign
import pprint
import sys

import stretch4_body.robot.robot_client as rc
from stretch4_body.utils.tool_metadata import ToolConfigurationError, get_tool_metadata, is_tool_joint
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.publisher import Publisher
from rclpy.exceptions import ParameterNotDeclaredException

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
from trajectory_msgs.msg import JointTrajectoryPoint

from hello_helpers.stretch4_ros_api import Stretch4ROSDriver, spin_until_shutdown


import tf2_ros
from tf_transformations import quaternion_from_euler

class StretchDriver(Stretch4ROSDriver):

    # Command joints are discovered from stretch4_body rather than listed here: each
    # joint's ROS name is its stretch4_body name + "_joint", except the attached tool,
    # which is commanded as TOOL_JOINT ("gripper_joint") when it is actuated. Its
    # value is one of the tool's URDF joints (ToolMetadata.tool_joints, e.g. one finger
    # of the Stretch gripper), in URDF units, reported and commanded alike. Assumes a
    # single-DOF tool (ToolMetadata is scalar).

    def __init__(self):
        super().__init__('stretch_driver')
        self.get_logger().info("For use with S T R E T C H (TM) RESEARCH EDITION from Hello Robot Inc.")

        
        #TODO: pull joint limits on real robot    
        '''limits = self.robot.pull_joint_limits()

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
                self.logger.info(f"Setting joint limits for joint {joint} as ({ll},{ul}).  Success: {success}, reasons: {reasons}")'''

        self.velocity_commands = {joint: None for joint in self.command_joints}

        self.command_publishers = {}
        qos_profile = 10
        
        for joint in self.command_joints:
            self.command_publishers[joint] = self.create_publisher(
                JointState, f"joint_command_diagnostics/{joint}/command", qos_profile
        )

        #pprint.pprint(self.robot.robot_params)

        # Velocity Control
        self.set_vel_functions = {}

        accel_params = []
        eoa_deadband = self.get_parameter("eoa_velocity_deadband").value
        for joint, backend in self.joint_backends.items():
            motion = self.robot.robot_params[backend]['motion']
            if self._is_end_of_arm(backend):
                # each velocity command moves the joint by 0.05 s worth of its max speed
                vel_increment = motion["max"]["vel"]*0.05
                if joint == self.TOOL_JOINT:
                    vel_increment = self._tool_actuator_to_command_delta(vel_increment)
                    to_actuator_vel = lambda v: self._tool_velocity_to_actuator(v, self._tool_urdf_position())
                else:
                    to_actuator_vel = lambda v: v
                self.set_vel_functions[joint] = (
                    lambda v, a, j=backend, eps=eoa_deadband, step=vel_increment, to_act=to_actuator_vel:
                        self.robot.end_of_arm.quick_stop(j) if abs(v) < eps
                        else self.robot.end_of_arm.move_by(j, copysign(step, v), v_r=to_act(v), a_r=a))
            else:
                self.set_vel_functions[joint] = (
                    lambda v, a, subsystem=getattr(self.robot, backend): subsystem.set_velocity(v, a_m=a))
            accel_params.append(Parameter(f"joint_acceleration.{joint}", Parameter.Type.DOUBLE, self._accel(motion['default'])))
            accel_params.append(Parameter(f"joint_limit.{joint}.acceleration", Parameter.Type.DOUBLE, self._accel(motion['max'])))


        accel_params.append(Parameter("joint_acceleration.omnibase.linear", Parameter.Type.DOUBLE,self.robot.robot_params['omnibase']['motion']['default']['accel_xy_m']))
        accel_params.append(Parameter("joint_acceleration.omnibase.angular", Parameter.Type.DOUBLE,self.robot.robot_params['omnibase']['motion']['default']['accel_w_r']))

        accel_params.append(Parameter("joint_limit.omnibase.linear.acceleration", Parameter.Type.DOUBLE,self.robot.robot_params['omnibase']['motion']['max']['accel_xy_m']))
        accel_params.append(Parameter("joint_limit.omnibase.angular.acceleration", Parameter.Type.DOUBLE,self.robot.robot_params['omnibase']['motion']['max']['accel_w_r']))

        if self.TOOL_JOINT in self.joint_backends:
            low, high = self._tool_metadata.urdf_range
            accel_params.append(Parameter(f"joint_limit.{self.TOOL_JOINT}.lower", Parameter.Type.DOUBLE, float(min(low, high))))
            accel_params.append(Parameter(f"joint_limit.{self.TOOL_JOINT}.upper", Parameter.Type.DOUBLE, float(max(low, high))))

        
        #succeeded = self.set_parameters(accel_params)
        
        results = self.set_parameters(accel_params)
        for param, result in zip(accel_params, results):
            if not result.successful:
                self.logger.warning(f"Unable to set parameter {param.name}.  Reason: {result.reason}")

        
        self.robot.set_guarded_contact_sensitivity(self.get_parameter("sensitivity").value)

        self.start()

    def startup_robot(self):
        # Low level API
        self.robot = rc.RobotClient(client_id="ros2_driver")

        if not self.robot.startup():
            self.robot.logger.fatal('Robot startup failed.')
            rclpy.shutdown()
            sys.exit(1)

        # Warn if robot isn't homed
        if not self.robot.is_homed():
            self.robot.logger.warn('Robot is not homed.')

        # Unit conversions for the attached tool (gripper); None if there is no tool, or
        # it is passive (e.g. a tablet) and so has no ToolMetadata.
        self._tool_name = self.robot.params.get('tool')
        try:
            self._tool_metadata = get_tool_metadata(self._tool_name)
        except ToolConfigurationError as e:
            self._tool_metadata = None
            self.get_logger().info(f"No actuated tool ({e}).")
        self._tool_is_actuated = bool(self._tool_metadata and self._tool_metadata.actuated_joints)

        # Discover the command joints, as {ROS command joint: stretch4_body joint name}:
        # top-level subsystems whose status is a joint's (the lift and arm), then the
        # end of arm's joints (wrists and tool).
        self.robot.pull_status()
        self._status_paths = {}
        self.joint_backends = {}
        for name, status in self.robot.status.items():
            if self._is_joint_status(status) and hasattr(self.robot, name):
                self.joint_backends[f"{name}_joint"] = name
        for name in getattr(getattr(self.robot, 'end_of_arm', None), 'joints', []):
            if not is_tool_joint(name):
                self.joint_backends[f"{name}_joint"] = name
            elif self._tool_is_actuated:
                self.joint_backends[self.TOOL_JOINT] = name
        self.command_joints = list(self.joint_backends)
        self.velocity_joints = list(self.joint_backends)
        self.get_logger().info(f"Command joints: {self.joint_backends}")

    def backend_joint_name(self, name):
        return self.joint_backends.get(name, name)

    def _is_end_of_arm(self, backend):
        return backend in getattr(getattr(self.robot, 'end_of_arm', None), 'joints', [])

    @staticmethod
    def _accel(motion_limits):
        """A joint's acceleration from its stretch4_body motion params (m/s^2 or rad/s^2)."""
        for key in ('accel_m', 'accel_r', 'accel'):
            if key in motion_limits:
                return motion_limits[key]
        return None

    # --- reading joints out of stretch4_body's status --------------------------

    @staticmethod
    def _is_joint_status(status):
        return isinstance(status, dict) and 'pos' in status and 'vel' in status

    def _find_status_path(self, robot_status, backend):
        """Path to the shallowest dict keyed `backend` that is a joint's status (has its
        own 'pos' and 'vel'). Searching shallowest-first matters: safety_layer data is
        copied into each subsystem's status and contains deeper dicts keyed by joint
        names (e.g. the self-collision sentry's per-joint state) that aren't joints."""
        level = [((), robot_status)]
        seen = set()
        while level:
            next_level = []
            for path, d in level:
                for key, value in d.items():
                    if not isinstance(value, dict) or id(value) in seen:
                        continue
                    seen.add(id(value))
                    if key == backend and self._is_joint_status(value):
                        return path + (key,)
                    next_level.append((path + (key,), value))
            level = next_level
        return None

    def _joint_status(self, robot_status, backend):
        """The status dict for stretch4_body joint `backend`, or None if it isn't there."""
        path = self._status_paths.get(backend)
        if path is None:
            path = self._status_paths[backend] = self._find_status_path(robot_status, backend)
            if path is None:
                return None
        status = robot_status
        for key in path:
            status = status[key]
        return status

    @staticmethod
    def _status_field(status, *keys, default=None):
        """First of `keys` found in a joint's status, or in its 'motor' sub-dict (where
        stepper-driven joints such as the lift and arm keep their motor-level values)."""
        for d in (status, status.get('motor', {})):
            for key in keys:
                if key in d:
                    return d[key]
        return default

    def _ros_joint_states(self, joint, pos, vel):
        """(name, position, velocity) for each /joint_states entry of a command joint."""
        if joint == self.TOOL_JOINT:
            # The tool's URDF joints, and the command joint itself in the same units.
            meta = self._tool_metadata
            urdf_pos, urdf_vel = meta.actuator_to_urdf(pos), meta.actuator_to_urdf_velocity(vel, pos)
            return [(name, urdf_pos, urdf_vel) for name in [*meta.tool_joints, joint]]
        if joint in self.SPLIT_JOINTS:
            links = self.SPLIT_JOINTS[joint]
            return [(link, pos/len(links), vel/len(links)) for link in links]
        return [(joint, pos, vel)]
            
    def _get_push_interval(self):
        return 0.2
            
    def push_robot_command(self):
        for joint in self.velocity_commands:
            try:
                mode = self.get_parameter(f"joint_mode.{joint}").value
            except ParameterNotDeclaredException:
                self.logger.error(
                    f"Joint name {joint} not found in mode parameters while pushing command to robot.  Make sure you're calling check_and_set_joint_vel not set_joint_velocity.",
                    throttle_duration_sec=5.0,
                )
                continue

            if mode == "velocity" and self.velocity_commands[joint] is not None:
                last_sent = self.velocity_commands[joint]["last_sent"]
                target,a = self.velocity_commands[joint]["target"]
                if self.get_clock().now() - last_sent > Duration(seconds=self._get_push_interval()):
                    #self.logger.warn(f"Sending velocity {target} to joint {joint}")
                    self.set_joint_velocity(joint,target,a)
        
        self.robot.push_command()

    def get_robot_status(self):
        self.robot.pull_status()
        return copy.deepcopy(self.robot.status)
    
    def get_odom(self, robot_status, status_time) -> Odometry:
        x = float(robot_status['omnibase']['x'])
        y = float(robot_status['omnibase']['y'])
        theta = float(robot_status['omnibase']['theta'])
        x_vel = float(robot_status['omnibase']['x_vel'])
        y_vel = float(robot_status['omnibase']['y_vel'])
        theta_vel = float(robot_status['omnibase']['theta_vel'])

        q = quaternion_from_euler(0.0, 0.0, theta)
        
        odom = Odometry()
        odom.header.stamp = status_time
        odom.header.frame_id = self.frame_id('wheel_odom')
        odom.child_frame_id = self.frame_id('base_footprint')
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
        return bool(self.robot.is_homed())

    def get_mode(self, robot_status, status_time) -> String:
        return self.get_parameter("mode").value

    def get_tool(self, robot_status, status_time) -> String:
        return self.robot.params['tool']
                                   
    def get_sensitivity(self, robot_status, status_time) -> String:
        return self.get_parameter("sensitivity").value
    
    def get_runstop(self, robot_status, status_time) -> Bool:
        is_runstopped = bool(robot_status['power_periph']['runstop_event'])
        if self.robot_mode()!=self.runstopped_mode and is_runstopped or self.robot_mode()==self.runstopped_mode and not is_runstopped:
            self.runstop_the_robot(runstopped=is_runstopped,just_change_mode=True)
        return self.robot_mode()==self.runstopped_mode
                                      
    def get_joint_state(self, robot_status, status_time) -> JointState:
        joint_state = JointState()
        joint_state.header.stamp = status_time

        for joint, backend in self.joint_backends.items():
            status = self._joint_status(robot_status, backend)
            if status is None:
                continue
            effort = self._status_field(status, 'effort', 'effort_pct')
            for name, pos, vel in self._ros_joint_states(joint, status['pos'], status['vel']):
                joint_state.name.append(name)
                joint_state.position.append(pos)
                joint_state.velocity.append(vel)
                joint_state.effort.append(effort)

        # The wheels aren't command joints, but the URDF needs them; their status is in
        # motor units, so publish zeros until that is converted (see the to-do list).
        for name, status in robot_status['omnibase'].items():
            if self._is_joint_status(status):
                joint_state.name.append(f"{name}_joint")
                joint_state.position.append(0.0)
                joint_state.velocity.append(0.0)
                joint_state.effort.append(0.0)

        return joint_state

    def declare_node_params(self):
        # Below this magnitude, a wrist velocity-mode command is treated as an explicit stop
        # rather than forwarded to move_by(). Necessary because a tiny nonzero float velocity
        # silently truncates to an integer register value of 0 in the Feetech SDK's
        # set_goal_vel and writing 0 to that register means "no speed limit" rather than "stop"
        self.declare_parameter("eoa_velocity_deadband", 0.005)  # rad/s -- ~6.5x the truncation threshold
        

    
    def check_child_param(self, parameter: Parameter) -> tuple[bool,String]:
        #ROS expects parameter callback to atomically accept or reject all changes
        #only return true if ALL changes succeed.  Checking should not have side effects!
        reason = None
        found = False
     
        match parameter.name:
            case "sensitivity":
                found = True
                if parameter.value not in self.robot.get_guarded_contact_modes():
                    reason = f"set_guarded_contact_sensitivity: Invalid mode name: {parameter.value}"
            case _:
                reason=f"Parameter {parameter.name} not mutable or not found."
        return found,reason
    
    def update_child_parameter(self, parameter: Parameter):
        match parameter.name:
            case "sensitivity":
                self.robot.set_guarded_contact_sensitivity(parameter.value)

    def handle_mode_change(self, mode):
        pass

    def set_base_velocity(self, x, y, theta, stiffness=1.0):
        linear_acc = self.get_parameter_or("joint_acceleration.omnibase.linear",None).value
        angular_acc = self.get_parameter_or("joint_acceleration.omnibase.angular",None).value
        self.robot.omnibase.set_velocity(vx_m = x, 
                                         vy_m = y, 
                                         w_r = theta, 
                                         a_m = linear_acc, 
                                         a_r = angular_acc,
                                         stiffness = stiffness,
                                        )

    def set_joint_velocity(self, joint, target, a=None):
        if a is None:
            a = self.get_parameter_or(f"joint_acceleration.{joint}",None).value
        self.velocity_commands[joint]={"target":(target, a),
                                       "last_sent":self.get_clock().now()}
        
        command = JointState()
        command.name = [joint]
        command.position = []
        command.velocity = [target] if target is not None else []
        command.effort = [a] if a is not None else []
        
        self.command_publishers[joint].publish(command)
        self.set_vel_functions[joint](target, a)

    def set_joint_position(self, joint, target, v=None, a=None):
        self.velocity_commands[joint] = None
        if a is None:
            a = self.get_parameter_or(f"joint_acceleration.{joint}",None).value
        if v is None:
            v = self.get_parameter_or(f"joint_velocity.{joint}",None).value

        command = JointState()
        command.name = [joint]
        command.position = [target] if target is not None else []
        command.velocity = [v] if v is not None else []
        command.effort = [a] if a is not None else []
        
        self.command_publishers[joint].publish(command)
            
        backend = self.joint_backends.get(joint)
        if backend is None:
            self.logger.warn(f"Unable to set position for unknown joint {joint}.")
        elif joint == self.TOOL_JOINT:
            command = self._tool_metadata.urdf_to_command(target)
            v_r = self._tool_velocity_to_actuator(v, target) if v is not None else None
            self.robot.end_of_arm.move_to(backend, command, v_r, a)
        elif self._is_end_of_arm(backend):
            self.robot.end_of_arm.move_to(backend, target, v, a)
        else:
            getattr(self.robot, backend).move_to(target, v_m=v, a_m=a)

    def _tool_velocity_to_actuator(self, v, at_urdf):
        """Tool speed in URDF units at URDF position `at_urdf` -> actuator speed (rad/s),
        for move_to/move_by's v_r."""
        return abs(self._tool_metadata.urdf_to_actuator_velocity(v, at_urdf))

    def _tool_urdf_position(self):
        """The tool's current position in URDF units, or mid-range if not known yet."""
        pos = None
        if self.last_known_state is not None:
            pos = self.command_joint_pose_from_joint_state(self.TOOL_JOINT, self.last_known_state)
        if pos is None:
            low, high = self._tool_metadata.urdf_range
            pos = (low + high) / 2.0
        return pos

    def _tool_actuator_to_command_delta(self, delta):
        """Actuator displacement (rad) -> tool command units (percent on SG4, meters on PG4)."""
        low, high = self._tool_metadata.actuator_range
        return abs(self._tool_metadata.convert_delta(delta, "actuator", "command", (low + high) / 2.0))
        
    def publish_child_info(self):
        #real robot driver doesn't have additional publishers
        pass
  

    def stop_the_robot_callback(self, request, response):
        with self.driver_mode_lock:
            self.robot.omnibase.rotate_by(0.0)
            for backend in self.joint_backends.values():
                if self._is_end_of_arm(backend):
                    self.robot.end_of_arm.move_by(backend, 0.0)
                else:
                    getattr(self.robot, backend).move_by(0.0)
        response.success = True
        response.message = "Stopped the robot."
        return response


    def runstop_service_callback(self, request, response):
        self.runstop_the_robot(request.data)
        response.success = True
        response.message = 'is_runstopped: {0}'.format(request.data)
        return response

    
    # Routines run with do_pull=False: the control loop keeps pulling status meanwhile.
    # A routine that times out keeps running on the robot; the call just reports failure.
    def home_robot(self):
        finished, _rid = self.robot.routines.routine_robot_home(
            do_push=False, wait_on_completion=True, timeout=60, do_pull=False)
        return (True, 'Homed.') if finished else (False, 'Homing routine did not complete.')

    def stow_robot(self):
        finished, _rid = self.robot.routines.routine_robot_stow(
            do_push=False, wait_on_completion=True, timeout=30, do_pull=False)
        return (True, 'Stowed.') if finished else (False, 'Stowing routine did not complete.')

    def seat_robot(self):
        succeeded, _rid = self.robot.routines.routine_blind_dock(
            do_push=False, wait_on_completion=True, do_pull=False)
        return (True, 'Seated into dock.') if succeeded else (False, 'Seating into dock failed.')

    def freewheel_base(self):
        with self.driver_mode_lock:
            self.robot.omnibase.enable_freewheel_mode()
        return True, 'Freewheeled the robot.'

    def get_tool_info(self):
        info = {"name": self._tool_name, "is_actuated": self._tool_is_actuated,
                "tool_joints": list(self._tool_metadata.tool_joints) if self._tool_metadata else []}
        if self._tool_is_actuated:
            meta = self._tool_metadata
            info.update(urdf_range=meta.urdf_range, aperture_range=meta.aperture_range,
                        position_tolerance=meta.position_tolerance)
        return info

    def default_joint_velocity(self):
        return self._joint_velocities('default')

    def max_joint_velocity(self):
        velocities = self._joint_velocities('max')
        velocities.pop("omnibase.linear", None)
        velocities.pop("omnibase.angular", None)
        return velocities

    def _joint_velocities(self, profile):
        """Each command joint's and the base's speed in stretch4_body motion `profile`."""
        velocities = {}
        for joint, backend in self.joint_backends.items():
            if joint == self.TOOL_JOINT:
                try:
                    velocities[joint] = self._tool_metadata.position_independent_velocity_limit("urdf", profile)
                except ToolConfigurationError as e:
                    self.get_logger().warn(f"No {profile} speed for {joint}: {e}")
                continue
            motion = self.robot.robot_params[backend]['motion'][profile]
            velocities[joint] = motion['vel_m'] if 'vel_m' in motion else motion['vel']
        base = self.robot.robot_params['omnibase']['motion'][profile]
        velocities["omnibase.linear"] = base['vel_xy_m']
        velocities["omnibase.angular"] = base['vel_w_r']
        return velocities

    def runstop_the_robot(self, runstopped, just_change_mode=False):
        if runstopped:
            with self.driver_mode_lock:
                already_runstopped = self.robot_mode() == 'runstopped'
                if not already_runstopped:
                    self.prerunstop_mode = copy.copy(self.robot_mode())
            if already_runstopped:
                return
            self.change_mode('runstopped')
            if not just_change_mode:
                self.robot.power_periph.trigger_runstop()
        else:
            with self.driver_mode_lock:
                already_not_runstopped = self.robot_mode() != 'runstopped'
            if already_not_runstopped:
                return
            self.return_to_mode(self.prerunstop_mode)
            if not just_change_mode:
                self.robot.power_periph.clear_runstop()

    def get_battery(self, robot_status, status_time) -> BatteryState:
        battery_state = BatteryState()
        battery_state.header.stamp = status_time
        battery_state.voltage = float(robot_status['power_periph']['voltage'])
        battery_state.current = float(robot_status['power_periph']['battery_current'])
        battery_state.temperature = float(robot_status['power_periph']['temp'])
        battery_state.percentage = float(robot_status['power_periph']['battery_soc']) / 100.0

        if robot_status['power_periph']['adapter_voltage_present']:
            if robot_status['power_periph']['charger_is_charging']:
                battery_state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            else:
                battery_state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        else:
            battery_state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        battery_state.present = True

        return battery_state

                                 
    def get_diagnostics(self, robot_status, status_time) -> DiagnosticArray:
        # publish safety layer diagnostics
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = status_time

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
        return diag_msg
                              
    def get_lease_holder(self, robot_status, status_time) -> DiagnosticStatus:
        lease_holder_msg = DiagnosticStatus()
        lease_holder_msg.name = self.prefix + 'server_lease_holder'
        lease_holder_msg.level = DiagnosticStatus.OK
        for key in ["lease_holder","lease_holder_priority","lease_expiry"]:
            lease_holder_msg.values.append(KeyValue(key=key, value=str(robot_status['server'][key])))
        lease_holder_msg.values.append(KeyValue(key="lease_expired", value=str(time.monotonic() > robot_status['server']['lease_expiry'])))
        lease_holder_msg.values.append(KeyValue(key="routine_active", value=str(self.robot.routines.status['active_routine'] != 'routine_nop')))
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
        current_msg = DiagnosticStatus(name="current")
        in_collision_msg = DiagnosticStatus(name="in_collision")


        robot_runstopped = bool(robot_status['power_periph']['runstop_event'])
        for joint, backend in self.joint_backends.items():
            status = self._joint_status(robot_status, backend)
            if status is None:
                continue
            is_homed = bool(self._status_field(status, 'pos_calibrated', default=False))
            is_homing = bool(self._status_field(status, 'is_homing', default=False))
            # stepper joints report their own runstop; servo joints only have the robot's
            is_runstopped = bool(self._status_field(status, 'runstop_on', default=robot_runstopped))
            current = self._status_field(status, 'current_mA', 'current')

            at_limit_msg.values.append(KeyValue(key=joint, value=f"{status['at_limit']}"))
            soft_limits_msg.values.append(KeyValue(key=joint, value=f"{status['soft_motion_limits']}"))
            braking_distance_msg.values.append(KeyValue(key=joint, value=f"{status['braking_distance']}"))
            current_msg.values.append(KeyValue(key=joint, value=f"{current}"))
            is_homed_msg.values.append(KeyValue(key=joint, value=f"{is_homed}"))
            is_homing_msg.values.append(KeyValue(key=joint, value=f"{is_homing}"))
            is_runstopped_msg.values.append(KeyValue(key=joint, value=f"{is_runstopped}"))
            in_collision_msg.values.append(KeyValue(key=joint, value=f"{status['in_collision_stop']}"))

        joint_state_diagnostics.status.append(is_runstopped_msg)
        joint_state_diagnostics.status.append(is_homed_msg)
        joint_state_diagnostics.status.append(is_homing_msg)
        joint_state_diagnostics.status.append(at_limit_msg)
        joint_state_diagnostics.status.append(soft_limits_msg)
        joint_state_diagnostics.status.append(braking_distance_msg)
        joint_state_diagnostics.status.append(current_msg)
        joint_state_diagnostics.status.append(in_collision_msg)

        return joint_state_diagnostics
    

def main():
    rclpy.init()
    node = None
    executor = MultiThreadedExecutor(num_threads=5)
    try:
        node = StretchDriver()
        executor.add_node(node)
        spin_until_shutdown(executor)
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
