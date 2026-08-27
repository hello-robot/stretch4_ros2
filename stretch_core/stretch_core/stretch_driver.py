#! /usr/bin/env python3
from threading import Lock
import importlib
import copy
import time
from math import copysign

import stretch4_body.robot.robot_client as rc
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

from hello_helpers.stretch4_ros_api import Stretch4ROSDriver


import tf2_ros
from tf_transformations import quaternion_from_euler

class StretchDriver(Stretch4ROSDriver):
    
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
            

        # Velocity Control
        self.set_vel_functions = {}

        self.vel_increment = self.robot.robot_params["wrist_roll"]["motion"]["max"]["vel"]*self._get_push_interval()*0.5 #assume it takes half of the distance to reach steady vel

        accel_params = []
        if hasattr(self.robot, 'lift'):
            self.set_vel_functions['lift'] = lambda v, a:  self.robot.lift.set_velocity(v, a_m=a)
            accel_params.append(Parameter("joint_acceleration.lift",Parameter.Type.DOUBLE,self.robot.robot_params['lift']['motion']['default']['accel_m']))
        if hasattr(self.robot, 'arm'):
            self.set_vel_functions['arm'] = lambda v, a:  self.robot.arm.set_velocity(v, a_m=a)
            accel_params.append(Parameter("joint_acceleration.arm",Parameter.Type.DOUBLE, self.robot.robot_params['arm']['motion']['default']['accel_m']))
        if hasattr(self.robot, 'end_of_arm') and hasattr(self.robot.end_of_arm, 'joints'):
            for joint in self.robot.end_of_arm.joints: 

                # velocity control of gripper is unsafe; for other joints velocity control is ok
                if "gripper" in f'{joint}':
                    self.set_vel_functions[f'{joint}']= lambda d, a, j=joint: self.robot.end_of_arm.quick_stop(j) if d == 0.0 else self.robot.end_of_arm.move_by(j, d, a_r = a)
                else:
                    self.set_vel_functions[f'{joint}']= lambda v, a, j=joint: self.robot.end_of_arm.quick_stop(j) if v == 0.0 else self.robot.end_of_arm.move_by(j, copysign(self.vel_increment,v), v_r=v, a_r = a)
                accel_params.append(Parameter(f"joint_acceleration.{joint}",Parameter.Type.DOUBLE,self.robot.robot_params[joint]['motion']['default']['accel']))

        accel_params.append(Parameter("joint_acceleration.omnibase.linear", Parameter.Type.DOUBLE,self.robot.robot_params['omnibase']['motion']['default']['accel_xy_m']))
        accel_params.append(Parameter("joint_acceleration.omnibase.angular", Parameter.Type.DOUBLE,self.robot.robot_params['omnibase']['motion']['default']['accel_w_r']))

        #succeeded = self.set_parameters(accel_params)
        
        results = self.set_parameters(accel_params)
        for i in range(len(results)):
            param = accel_params[i]
            result = results[0]
            reason = result.reason
            check = result.successful
            
            if not check:
                self.logger.warning(f"Unable to set parameter {param.name}.  Reason: {reason}")

        
        self.robot.set_guarded_contact_sensitivity(self.get_parameter("sensitivity").value)

        self.driver_mode_lock = Lock()
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

        self.joint_command_groups: list[BaseCommandGroup] = []
        self.command_joints = []
        for joint_params in self.robot.robot_params['ros']['joints']:
            module_name = joint_params['py_module_name']
            class_name = joint_params['py_class_name']
            module = importlib.import_module(module_name)
            class_obj = getattr(module, class_name)
            cg = class_obj()
            self.joint_command_groups.append(cg)
            joint_name = cg.name.split("_joint")[0]
            if joint_name == "gripper":
                joint_name = "stretch_gripper"
            self.command_joints.append(joint_name)
            self.get_logger().debug(f"Discovered {class_name}")
        self.velocity_joints = ["arm", "lift", "wrist_yaw", "wrist_roll", "wrist_pitch"]
            
    def _get_push_interval(self):
        return 0.1
            
    def push_robot_command(self):
        for joint in self.velocity_commands:
            try:
                mode = self.get_parameter(f"joint_mode.{joint}").value
            except ParameterNotDeclaredException:
                self.logger.error(f"Joint name {joint} not found in mode parameters while pushing command to robot.  Make sure you're calling check_and_set_joint_vel not set_joint_velocity.")
                mode = "<< joint unknown >>"

            if mode == "velocity" and self.velocity_commands[joint] is not None:
                last_sent = self.velocity_commands[joint]["last_sent"]
                target = self.velocity_commands[joint]["target"]
                if self.get_clock().now() - last_sent > Duration(seconds=self._get_push_interval()):
                    self.logger.warn(f"Sending velocity {target} to joint {joint}")
                    self.set_joint_velocity(joint,target)                
        
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
        if self.robot_mode()!="runstopped" and is_runstopped or self.robot_mode()=="runstopped" and not is_runstopped:
            self.runstop_the_robot(runstopped=is_runstopped,just_change_mode=True)
        return self.robot_mode()=="runstopped"
                                      
    def get_joint_state(self, robot_status, status_time) -> JointState:
        joint_state = JointState()
        joint_state.header.stamp = status_time

        for cg in self.joint_command_groups:
            pos, vel, eff = cg.joint_state(robot_status)

            if cg.name == "arm_joint":
                for link in ['arm_l4_joint', 'arm_l3_joint', 'arm_l2_joint', 'arm_l1_joint']:
                    joint_state.name.append(link)
                    joint_state.position.append(pos/5.0)
                    joint_state.velocity.append(vel/5.0)
                    joint_state.effort.append(eff)
            elif cg.name == "gripper_joint":
                for link in ['gripper_finger_left_joint', 'gripper_finger_right_joint']:
                    joint_state.name.append(link)
                    joint_state.position.append(pos)
                    joint_state.velocity.append(vel)
                    joint_state.effort.append(eff)
            elif cg.name == "parallel_gripper_joint":
                finger_pos = -pos / 2.0
                for link in ['finger_left_joint', 'finger_right_joint']:
                    joint_state.name.append(link)
                    joint_state.position.append(finger_pos)
                    joint_state.velocity.append(vel)
                    joint_state.effort.append(eff)
            elif cg.name == "translate_mobile_base":
                for w in ['wheel_0_joint', 'wheel_1_joint', 'wheel_2_joint']:
                    joint_state.name.append(w)
                    joint_state.position.append(0.0)
                    joint_state.velocity.append(0.0)
                    joint_state.effort.append(0.0)
            else:
                joint_state.name.append(cg.name)
                joint_state.position.append(pos)
                joint_state.velocity.append(vel)
                joint_state.effort.append(eff)

        return joint_state


    #this function is needed because the published joint state splits up the
    #arm and uses different names than the joint names used to send commands
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

                
    def declare_node_params(self):
        pass

    
    def check_child_param(self, parameter: Parameter) -> tuple[bool,String]:
        #ROS expects parameter callback to atomically accept or reject all changes
        #only return true if ALL changes succeed.  Checking should not have side effects!
        reason = None
        found = False
     
        match parameter.name:
            case "joint_mode.stretch_gripper":
                found = True
                if parameter.value == "velocity":
                    reason = f"Velocity mode results in unsafe behavior from the gripper and is not allowed."
            case "joint_mode.parallel_gripper":
                found = True
                if parameter.value == "velocity":
                    reason = f"Velocity mode results in unsafe behavior from the gripper and is not allowed."
            case "sensitivity":
                found = True
                if parameter.value not in self.robot.get_guarded_modes():
                    f"set_guarded_contact_sensitivity: Invalid mode name: {parameter.value}"
            case _:
                reason=f"Parameter {parameter.name} not mutable or not found."
        return found,reason
    
    def update_child_parameter(self, parameter: Parameter):
        match parameter.name:
            case "sensitivity":
                self.robot.set_guarded_contact_sensitivity(parameter.value)

    def handle_mode_change(self, mode):
        pass

    def set_base_velocity(self, x, y, theta):
        linear_acc = self.get_parameter_or("joint_acceleration.omnibase.linear",None).value
        angular_acc = self.get_parameter_or("joint_acceleration.omnibase.angular",None).value
        self.robot.omnibase.set_velocity(vx_m = x, 
                                         vy_m = y, 
                                         w_r = theta, 
                                         a_m = linear_acc, 
                                         a_r = angular_acc
                                        )

    def set_joint_velocity(self, joint, target):
        self.velocity_commands[joint]={"target":target,
                                       "last_sent":self.get_clock().now()}
        acceleration_param = self.get_parameter_or(f"joint_acceleration.{joint}",None).value
        self.set_vel_functions[joint](target, acceleration_param)

    def set_joint_position(self, joint, target):
        self.velocity_commands[joint] = None
        a = self.get_parameter_or(f"joint_acceleration.{joint}",None).value
        v = self.get_parameter_or(f"joint_default_velocity.{joint}",None).value
        
        match joint:
            case "lift":
                self.robot.lift.move_to(target,v_m = v, a_m = a)
            case "arm":
                self.robot.arm.move_to(target,v_m = v, a_m = a)
            case "wrist_pitch"|"wrist_roll"|"wrist_yaw"|"stretch_gripper"|"parallel_gripper":
                self.robot.end_of_arm.move_to(joint, target, v, a)
            case _:
                self.logger.warn(f"Unable to set position for unknown joint {joint}.")
        

    def publish_child_info(self):
        #real robot driver doesn't have additional publishers
        pass
  

    def stop_the_robot_callback(self, request, response):
        with self.driver_mode_lock:
            self.robot.omnibase.rotate_by(0.0)
            self.robot.lift.move_by(0.0)

            if hasattr(self.robot, 'arm'):
                self.robot.arm.move_by(0.0)

            if hasattr(self.robot, 'end_of_arm') and hasattr(self.robot.end_of_arm, 'joints'):
                for joint in self.robot.end_of_arm.joints:
                    self.robot.end_of_arm.move_by(joint, 0.0)

 
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

    
    def home_the_robot(self):
        with self.driver_mode_lock:
            can_home = self.robot_mode() in self.control_modes
            last_driver_mode = copy.copy(self.robot_mode())
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
            can_stow = self.robot_mode() in self.control_modes
            last_driver_mode = copy.copy(self.robot_mode())
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
            self.change_mode(self.prerunstop_mode)
            if not just_change_mode:
                self.robot.power_periph.clear_runstop()

    def get_battery(self, robot_status, status_time) -> BatteryState:
        battery_state = BatteryState()
        battery_state.header.stamp = status_time
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

        for cg in self.joint_command_groups:
            pos, vel, eff = cg.joint_state(robot_status)
            if cg.name == "translate_mobile_base":
                continue

            joint_status_key = cg.name.replace("_joint","")
            if joint_status_key == "gripper":
                joint_status_key = "stretch_gripper"

            if joint_status_key in ["wrist_roll", "wrist_pitch", "wrist_yaw", "stretch_gripper", "parallel_gripper"]:
                status_dict = robot_status["end_of_arm"][joint_status_key]
                is_homed = bool(status_dict.get('pos_calibrated', False))
                is_homing = bool(status_dict.get('is_homing', False))
            else: 
                status_dict = robot_status[joint_status_key]
                is_homed = bool(status_dict['motor'].get('pos_calibrated', False))
                is_homing = bool(status_dict['motor'].get('is_homing', False))
                is_runstopped = bool(status_dict['motor'].get('runstop_on', False))

            at_limit_msg.values.append(KeyValue(key=cg.name, value=f"{status_dict['at_limit']}"))
            soft_limits_msg.values.append(KeyValue(key=cg.name, value=f"{status_dict['soft_motion_limits']}"))
            braking_distance_msg.values.append(KeyValue(key=cg.name, value=f"{status_dict['braking_distance']}"))
            is_homed_msg.values.append(KeyValue(key=cg.name, value=f"{is_homed}"))
            is_homing_msg.values.append(KeyValue(key=cg.name, value=f"{is_homing}"))
            is_runstopped_msg.values.append(KeyValue(key=cg.name, value=f"{is_runstopped}"))

        joint_state_diagnostics.status.append(is_runstopped_msg)
        joint_state_diagnostics.status.append(is_homed_msg)
        joint_state_diagnostics.status.append(is_homing_msg)
        joint_state_diagnostics.status.append(at_limit_msg)
        joint_state_diagnostics.status.append(soft_limits_msg)
        joint_state_diagnostics.status.append(braking_distance_msg)

        return joint_state_diagnostics
    

def main():
    try:
        rclpy.init()
        node = StretchDriver()
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=5)
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    except KeyboardInterrupt:
        print("Detecting KeyboardInterrupt")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
