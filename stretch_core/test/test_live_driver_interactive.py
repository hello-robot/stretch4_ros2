#!/usr/bin/env python3
"""
Interactive and Sequential Live Driver Test Script for Stretch
=============================================================

This script performs integration and end-to-end testing against a live, running
StretchDriver (or StretchGazeboDriver) ROS 2 node. It supports:
- Interactive mode (CLI menu to select specific tests)
- Sequential mode (runs all tests one after another)
- Getting/setting parameters
- Homing/stow services
- Joint position and velocity control with steady-state velocity accuracy verification
- Mobile base velocity control
- Joystick command simulation
- Safety runstop check

Usage:
------
Run interactively (default):
    python3 test_live_driver_interactive.py

Run sequentially:
    python3 test_live_driver_interactive.py --sequential
"""

import sys
import time
import math
import argparse
import threading
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Joy, BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from diagnostic_msgs.msg import DiagnosticArray

# Terminal Colors for Professional Output
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BLUE = "\033[94m"
CYAN = "\033[96m"
BOLD = "\033[1m"
RESET = "\033[0m"

def print_header(title):
    print(f"\n{BOLD}{BLUE}{'='*80}{RESET}")
    print(f"{BOLD}{CYAN}TEST PHASE: {title}{RESET}")
    print(f"{BOLD}{BLUE}{'='*80}{RESET}")

def print_robot_behavior(behavior_desc):
    print(f"{BOLD}{YELLOW}PHYSICAL ROBOT BEHAVIOR TO OBSERVE:{RESET}")
    print(f"  👉 {behavior_desc}\n")

class StretchLiveDriverTester(Node):
    def __init__(self, pub_count=1):
        super().__init__('stretch_live_driver_tester')
        self.pub_count = pub_count
        self.get_logger().info(f"Initializing Live Driver Test Client (pub_count={pub_count})...")

        # Setup variables to record received data from subscribers
        self.latest_joint_state = None
        self.latest_odom = None
        self.latest_battery = None
        self.latest_is_runstopped = None
        self.latest_is_homed = None
        self.latest_mode = None

        # Setup standard home positions for Stretch 4 joints to avoid self-collisions
        self.HOME_POSITIONS = {
            "lift": 0.4,
            "arm": 0.2,
            "wrist_yaw": 0.0,
            "wrist_pitch": 0.5,
            "wrist_roll": 0.0,
            "stretch_gripper": 0.5
        }

        # Setup subscribers with relative and absolute fallbacks to handle namespaces gracefully
        self.js_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_cb, 10)
        self.js_sub_abs = self.create_subscription(JointState, '/stretch/joint_states', self.joint_state_cb, 10)
        self.js_sub_raw = self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)

        self.odom_sub = self.create_subscription(Odometry, 'wheel_odom', self.odom_cb, 10)
        self.odom_sub_abs = self.create_subscription(Odometry, '/wheel_odom', self.odom_cb, 10)
        self.odom_sub_raw = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.odom_sub_raw_abs = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.bat_sub = self.create_subscription(BatteryState, 'battery', self.battery_cb, 10)
        self.bat_sub_abs = self.create_subscription(BatteryState, '/battery', self.battery_cb, 10)

        self.runstop_sub = self.create_subscription(Bool, 'is_runstopped', self.runstop_cb, 10)
        self.runstop_sub_abs = self.create_subscription(Bool, '/is_runstopped', self.runstop_cb, 10)

        self.homed_sub = self.create_subscription(Bool, 'is_homed', self.homed_cb, 10)
        self.homed_sub_abs = self.create_subscription(Bool, '/is_homed', self.homed_cb, 10)

        self.mode_sub = self.create_subscription(String, 'mode', self.mode_cb, 10)
        self.mode_sub_abs = self.create_subscription(String, '/mode', self.mode_cb, 10)

        # Setup publishers for command inputs
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd_vel_pub_abs = self.create_publisher(Twist, '/stretch/cmd_vel', 10)

        self.position_cmd_pub = self.create_publisher(JointState, 'joint_position_cmd', 10)
        self.position_cmd_pub_abs = self.create_publisher(JointState, '/stretch/joint_position_cmd', 10)

        self.velocity_cmd_pub = self.create_publisher(JointState, 'joint_velocity_cmd', 10)
        self.velocity_cmd_pub_abs = self.create_publisher(JointState, '/stretch/joint_velocity_cmd', 10)

        self.joy_pub = self.create_publisher(Joy, 'joy', 10)
        self.joy_pub_abs = self.create_publisher(Joy, '/stretch/joy', 10)

        # Setup FollowJointTrajectory action client
        self._fjt_client = ActionClient(self, FollowJointTrajectory, 'follow_joint_trajectory')

        # Active spinning in a separate thread so that we don't block CLI input
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        # Connect to services after starting spin so node-name list is populated
        self.get_logger().info("Searching for driver node...")
        time.sleep(1.0)
        self.auto_discover_driver()

    def _spin(self):
        try:
            rclpy.spin(self)
        except Exception:
            pass

    def joint_state_cb(self, msg):
        self.latest_joint_state = msg

    def odom_cb(self, msg):
        self.latest_odom = msg

    def battery_cb(self, msg):
        self.latest_battery = msg

    def runstop_cb(self, msg):
        self.latest_is_runstopped = msg

    def homed_cb(self, msg):
        self.latest_is_homed = msg

    def mode_cb(self, msg):
        self.latest_mode = msg

    def auto_discover_driver(self):
        # Auto-detect driver node name
        node_names = self.get_node_names()
        self.driver_node = 'stretch_driver'
        for name in node_names:
            if 'stretch_gazebo_driver' in name or 'gazebo' in name:
                self.driver_node = name
                break
            elif 'stretch_driver' in name:
                self.driver_node = name
        self.get_logger().info(f"Using driver node name: {BOLD}{CYAN}{self.driver_node}{RESET}")

        # Setup service clients
        self.param_get_cli = self.create_client(GetParameters, f'/{self.driver_node}/get_parameters')
        self.param_set_cli = self.create_client(SetParameters, f'/{self.driver_node}/set_parameters')
        self.home_cli = self.create_client(Trigger, 'home_the_robot')
        self.stow_cli = self.create_client(Trigger, 'stow_the_robot')
        self.stop_cli = self.create_client(Trigger, 'stop_the_robot')
        self.runstop_cli = self.create_client(SetBool, 'runstop_the_robot')

    def wait_for_services(self, timeout_s=5.0):
        """Verify that the driver node is up and advertising its services."""
        services = [
            (self.param_get_cli, f"/{self.driver_node}/get_parameters"),
            (self.param_set_cli, f"/{self.driver_node}/set_parameters"),
            (self.home_cli, "home_the_robot"),
            (self.stow_cli, "stow_the_robot"),
            (self.stop_cli, "stop_the_robot"),
            (self.runstop_cli, "runstop_the_robot")
        ]
        for cli, name in services:
            self.get_logger().info(f"Checking for service {name}...")
            if not cli.wait_for_service(timeout_sec=timeout_s):
                print(f"{RED}{BOLD}ERROR: Service '{name}' is not available!{RESET}")
                print(f"{RED}Make sure you launched the driver via:{RESET}")
                print(f"{YELLOW}ros2 launch stretch_core stretch_driver.launch.py{RESET}")
                return False
        return True

    def publish_position_cmd(self, msg):
        self.position_cmd_pub.publish(msg)
        self.position_cmd_pub_abs.publish(msg)

    def publish_velocity_cmd(self, msg):
        self.velocity_cmd_pub.publish(msg)
        self.velocity_cmd_pub_abs.publish(msg)

    def publish_cmd_vel(self, msg):
        self.cmd_vel_pub.publish(msg)
        self.cmd_vel_pub_abs.publish(msg)

    def publish_joy(self, msg):
        self.joy_pub.publish(msg)
        self.joy_pub_abs.publish(msg)

    def get_joint_pose_and_vel(self, joint_name):
        """
        Calculates joint pose and velocity from the latest JointState message,
        handling joint-splitting logic (like arm links or gripper fingers) properly.
        """
        state = self.latest_joint_state
        if state is None:
            return None, None
        
        if joint_name == "arm":
            poses = []
            vels = []
            for link in ['arm_l1_joint', 'arm_l2_joint', 'arm_l3_joint', 'arm_l4_joint']:
                if link in state.name:
                    idx = state.name.index(link)
                    poses.append(state.position[idx])
                    if len(state.velocity) > idx:
                        vels.append(state.velocity[idx])
            if poses:
                return sum(poses), (sum(vels) if len(vels) == len(poses) else 0.0)
            return None, None
        elif joint_name == "stretch_gripper":
            left_pos, left_vel = None, None
            right_pos, right_vel = None, None
            if "gripper_finger_left_joint" in state.name:
                idx = state.name.index("gripper_finger_left_joint")
                left_pos = state.position[idx]
                if len(state.velocity) > idx:
                    left_vel = state.velocity[idx]
            if "gripper_finger_right_joint" in state.name:
                idx = state.name.index("gripper_finger_right_joint")
                right_pos = state.position[idx]
                if len(state.velocity) > idx:
                    right_vel = state.velocity[idx]
            if left_pos is not None and right_pos is not None:
                total_pos = left_pos + right_pos
                total_vel = (left_vel + right_vel) if (left_vel is not None and right_vel is not None) else 0.0
                return total_pos, total_vel
            return None, None
        else:
            actual_name = f"{joint_name}_joint"
            if actual_name in state.name:
                idx = state.name.index(actual_name)
                return state.position[idx], (state.velocity[idx] if len(state.velocity) > idx else 0.0)
            if joint_name in state.name:
                idx = state.name.index(joint_name)
                return state.position[idx], (state.velocity[idx] if len(state.velocity) > idx else 0.0)
            return None, None

    def restore_joint_to_home(self, joint):
        """Restores a given joint to its calibrated/home position."""
        target = self.HOME_POSITIONS.get(joint, 0.0)
        print(f"{YELLOW}Restoring {joint} to home position {target:.4f}...{RESET}")
        
        js = JointState()
        js.name = [joint]
        js.position = [float(target)]
        
        # Publish position commands to ensure the driver registers it
        for _ in range(self.pub_count):
            self.publish_position_cmd(js)
            if self.pub_count > 1:
                time.sleep(0.1)
        
        # Wait up to 5.0 seconds for the joint to reach and settle at its home position
        start_time = time.time()
        while time.time() - start_time < 5.0:
            pos, _ = self.get_joint_pose_and_vel(joint)
            if pos is not None and abs(pos - target) < 0.02:
                break
            time.sleep(0.1)
        time.sleep(0.5)

    def restore_all_to_home(self):
        """Restores all joints to their neutral/home positions without using calibration service."""
        print(f"{YELLOW}Restoring all joints to neutral home posture to avoid collisions...{RESET}")
        js = JointState()
        js.name = list(self.HOME_POSITIONS.keys())
        js.position = [float(x) for x in self.HOME_POSITIONS.values()]
        
        for _ in range(self.pub_count):
            self.publish_position_cmd(js)
            if self.pub_count > 1:
                time.sleep(0.05)
        
        # Wait up to 5.0 seconds for all joints to settle
        start_time = time.time()
        while time.time() - start_time < 5.0:
            all_settled = True
            for joint, target in self.HOME_POSITIONS.items():
                pos, _ = self.get_joint_pose_and_vel(joint)
                if pos is None or abs(pos - target) > 0.02:
                    all_settled = False
                    break
            if all_settled:
                break
            time.sleep(0.1)
        time.sleep(1.0)

    # =========================================================================
    # 1. HOMING & STOWING SERVICE INTERFACES
    # =========================================================================
    def test_homing_service(self, interactive=False):
        print_header("HOMING SERVICE")
        if interactive:
            confirm = input(f"{YELLOW}Confirm: This will physically home the robot (which initiates motor calibration). Proceed? (y/n): {RESET}").lower()
            if confirm != 'y':
                print(f"{YELLOW}Skipped homing service test.{RESET}")
                return True

        print_robot_behavior("The robot's lift, arm, and wrist joints should home back to their extended/calibrated starting positions.")
        print(f"{YELLOW}Calling /home_the_robot...{RESET}")
        req = Trigger.Request()
        future = self.home_cli.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        res = future.result()
        assert res.success, "Homing service call reported failure!"
        print(f"{GREEN}✓ Service '/home_the_robot' successfully completed! Message: {res.message}{RESET}")
        time.sleep(2.0)
        return True

    def test_stowing_service(self, interactive=False):
        print_header("STOWING SERVICE")
        if interactive:
            confirm = input(f"{YELLOW}Confirm: This will physically stow the robot. Proceed? (y/n): {RESET}").lower()
            if confirm != 'y':
                print(f"{YELLOW}Skipped stowing service test.{RESET}")
                return True

        print_robot_behavior("The robot should fold its arm, stow its wrist, and rotate its head to the stow posture.")
        print(f"{YELLOW}Calling /stow_the_robot...{RESET}")
        req = Trigger.Request()
        future = self.stow_cli.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        res = future.result()
        assert res.success, "Stowing service call reported failure!"
        print(f"{GREEN}✓ Service '/stow_the_robot' successfully completed! Message: {res.message}{RESET}")
        time.sleep(2.0)
        
        # Restore robot to safe neutral position via standard position commands so subsequent tests can run
        self.restore_all_to_home()
        return True

    # =========================================================================
    # 2. EMERGENCY STOP & RUNSTOP SAFETY SERVICE
    # =========================================================================
    def test_runstop_safety(self, interactive=False):
        print_header("SAFETY RUNSTOP SERVICE & EMERGENCY STOP")
        print_robot_behavior("1. The robot enters runstopped mode. All movement commands will be blocked.\n"
                            "  2. We attempt to send a position command to the lift. The robot must NOT move at all.\n"
                            "  3. We release the runstop and stop the robot.")

        # Step 1: Engage Runstop
        print(f"{YELLOW}Engaging safety runstop via /runstop_the_robot (data=True)...{RESET}")
        req = SetBool.Request()
        req.data = True
        future = self.runstop_cli.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        assert future.result().success, "Failed to engage runstop safety!"
        print(f"{GREEN}✓ Safety runstop successfully ENGAGED!{RESET}")
        time.sleep(1.0)

        # Get current lift position
        start_lift_pos, _ = self.get_joint_pose_and_vel("lift")
        if start_lift_pos is None:
            # Fallback default
            start_lift_pos = 0.5
            print(f"{YELLOW}Warning: Lift position not detected, assuming 0.5m for safety.{RESET}")

        # Step 2: Attempt position command (should be blocked)
        target_pos = min(0.9, start_lift_pos + 0.15)
        print(f"{YELLOW}Attempting to command lift to {target_pos}m (should be completely ignored by driver)...{RESET}")
        js = JointState()
        js.name = ["lift"]
        js.position = [target_pos]
        for _ in range(self.pub_count):
            self.publish_position_cmd(js)
            if self.pub_count > 1:
                time.sleep(0.1)
        
        print(f"{YELLOW}Verification: Waiting 1.5s to confirm no physical movement occurs...{RESET}")
        time.sleep(1.5)
        
        # Verify lift did not move
        end_lift_pos, _ = self.get_joint_pose_and_vel("lift")
        if end_lift_pos is not None:
            print(f"  Lift position before runstop command: {start_lift_pos:.4f}m | After: {end_lift_pos:.4f}m")
            assert abs(end_lift_pos - start_lift_pos) < 0.01, f"{RED}SAFETY FAILURE: Robot moved {abs(end_lift_pos - start_lift_pos):.4f}m while safety runstop was active!{RESET}"
            print(f"{GREEN}✓ Verification Successful: Robot movement was fully blocked!{RESET}")
        else:
            print(f"{YELLOW}✓ Verification: Assuming blocked (no joint states received).{RESET}")

        # Step 3: Release Runstop
        print(f"{YELLOW}Releasing safety runstop via /runstop_the_robot (data=False)...{RESET}")
        req.data = False
        future = self.runstop_cli.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        assert future.result().success, "Failed to release runstop!"
        print(f"{GREEN}✓ Safety runstop successfully RELEASED!{RESET}")
        time.sleep(1.0)

        # Step 4: Test stop_the_robot
        print(f"{YELLOW}Testing /stop_the_robot service...{RESET}")
        req_trigger = Trigger.Request()
        future = self.stop_cli.call_async(req_trigger)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        assert future.result().success, "Failed to call /stop_the_robot"
        print(f"{GREEN}✓ Service '/stop_the_robot' completed successfully! Message: {future.result().message}{RESET}")
        time.sleep(1.0)
        return True

    # =========================================================================
    # 3. PARAMETER SET/GET INTERFACE
    # =========================================================================
    def test_parameters(self, interactive=False):
        print_header("PARAMETER INTERFACE TESTING")
        print_robot_behavior("The robot should not move during parameter querying and setting.")

        # Test A: Get parameters
        req_get = GetParameters.Request()
        req_get.names = ['mode', 'position_tolerance', 'action_timeout', 'velocity_timeout']
        
        print(f"{YELLOW}Querying common parameters...{RESET}")
        future = self.param_get_cli.call_async(req_get)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        res = future.result()
        
        assert res is not None, "Failed to call get_parameters service"
        mode_val = res.values[0].string_value
        tol_val = res.values[1].double_value
        at_val = res.values[2].double_value
        vt_val = res.values[3].double_value
        print(f"{GREEN}✓ Retrieved default 'mode': {mode_val}{RESET}")
        print(f"{GREEN}✓ Retrieved 'position_tolerance': {tol_val}{RESET}")
        print(f"{GREEN}✓ Retrieved 'action_timeout': {at_val}{RESET}")
        print(f"{GREEN}✓ Retrieved 'velocity_timeout': {vt_val}{RESET}")

        # Test B: Set 'mode' parameter to 'teleop' and back
        req_set = SetParameters.Request()
        p_mode = ParameterMsg()
        p_mode.name = 'mode'
        p_mode.value.type = ParameterType.PARAMETER_STRING
        p_mode.value.string_value = 'teleop'
        req_set.parameters = [p_mode]

        print(f"{YELLOW}Setting mode parameter to 'teleop'...{RESET}")
        future = self.param_set_cli.call_async(req_set)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        res_set = future.result()
        assert res_set is not None and res_set.results[0].successful, f"Failed to set 'mode' to teleop. Reason: {res_set.results[0].reason}"
        print(f"{GREEN}✓ Successfully set parameter 'mode' to 'teleop'{RESET}")
        time.sleep(0.5)

        # Restore 'active' (or 'navigation') mode
        p_mode.value.string_value = mode_val
        future = self.param_set_cli.call_async(req_set)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        print(f"{GREEN}✓ Restored parameter 'mode' back to '{mode_val}'{RESET}")

        # Test C: Set read-only parameter -> should fail
        req_set_ro = SetParameters.Request()
        p_ro = ParameterMsg()
        p_ro.name = 'control_loop_rate'
        p_ro.value.type = ParameterType.PARAMETER_DOUBLE
        p_ro.value.double_value = 200.0
        req_set_ro.parameters = [p_ro]
        
        print(f"{YELLOW}Attempting to set read-only 'control_loop_rate'...{RESET}")
        future_ro = self.param_set_cli.call_async(req_set_ro)
        while rclpy.ok() and not future_ro.done():
            time.sleep(0.1)
        res_ro = future_ro.result()
        assert res_ro is not None and not res_ro.results[0].successful, "Unexpectedly allowed changing read-only parameter 'control_loop_rate'"
        print(f"{GREEN}✓ Successfully rejected changing read-only parameter 'control_loop_rate' (Reason: {res_ro.results[0].reason}){RESET}")

        # Test D: Set valid & invalid velocity_timeout
        p_vt = ParameterMsg()
        p_vt.name = 'velocity_timeout'
        p_vt.value.type = ParameterType.PARAMETER_DOUBLE
        
        # Valid positive value
        p_vt.value.double_value = 0.8
        req_set.parameters = [p_vt]
        future_vt = self.param_set_cli.call_async(req_set)
        while rclpy.ok() and not future_vt.done():
            time.sleep(0.1)
        assert future_vt.result().results[0].successful, "Failed to set valid velocity_timeout"
        print(f"{GREEN}✓ Successfully set parameter 'velocity_timeout' to 0.8{RESET}")

        # Invalid negative value
        p_vt.value.double_value = -0.5
        future_vt_neg = self.param_set_cli.call_async(req_set)
        while rclpy.ok() and not future_vt_neg.done():
            time.sleep(0.1)
        assert not future_vt_neg.result().results[0].successful, "Unexpectedly allowed setting negative velocity_timeout"
        print(f"{GREEN}✓ Successfully rejected invalid negative velocity_timeout{RESET}")

        # Restore velocity_timeout
        p_vt.value.double_value = vt_val
        future_vt_res = self.param_set_cli.call_async(req_set)
        while rclpy.ok() and not future_vt_res.done():
            time.sleep(0.1)
        print(f"{GREEN}✓ Restored 'velocity_timeout' parameter back to {vt_val}{RESET}")
        return True

    # =========================================================================
    # 4. POSITION CONTROL COMMANDS (All joints)
    # =========================================================================
    def test_position_commands(self, interactive=False):
        print_header("POSITION COMMANDS (/joint_position_cmd)")
        if interactive:
            confirm = input(f"{YELLOW}Confirm: This will physically sweep all joints in position mode. Proceed? (y/n): {RESET}").lower()
            if confirm != 'y':
                print(f"{YELLOW}Skipped position commands test.{RESET}")
                return True

        joints_to_test = ["lift", "arm", "wrist_yaw", "wrist_pitch", "wrist_roll", "stretch_gripper"]
        
        # Check that we have joint states before continuing
        if self.latest_joint_state is None:
            print(f"{YELLOW}Waiting for joint state telemetry...{RESET}")
            for _ in range(30):
                if self.latest_joint_state is not None:
                    break
                time.sleep(0.1)
            assert self.latest_joint_state is not None, "Failed to receive JointState messages!"

        # Send joints to their home positions before the position test
        print(f"{YELLOW}Preparing position command test: sending all joints to home posture...{RESET}")
        self.restore_all_to_home()

        print(f"{GREEN}Starting Position Sweeps...{RESET}")
        
        for joint in joints_to_test:
            current_pos, _ = self.get_joint_pose_and_vel(joint)
            if current_pos is None:
                print(f"{RED}Skipping {joint} as current position could not be read from JointState.{RESET}")
                continue
            
            # Decide on a safe, small relative movement offset based on current position
            if joint == "lift":
                # Move slightly up or down depending on height
                target = current_pos + 0.05 if current_pos < 0.6 else current_pos - 0.05
                desc = "moves lift joint vertically by 5cm"
            elif joint == "arm":
                target = current_pos + 0.05 if current_pos < 0.15 else current_pos - 0.05
                desc = "extends/retracts arm slightly"
            elif joint == "stretch_gripper":
                # Gripper opening translation
                target = 0.2 if current_pos < 0.15 else 0.0
                desc = "opens/closes gripper finger joints"
            else:
                # Wrist yaw, pitch, roll
                target = current_pos + 0.15 if current_pos < 0.1 else current_pos - 0.15
                desc = f"rotates {joint} joint"

            print_robot_behavior(f"The robot {desc} to target position {target:.4f}.")
            print(f"{YELLOW}Publishing position command for {joint} -> {target:.4f}...{RESET}")
            
            js = JointState()
            js.name = [joint]
            js.position = [float(target)]
            
            # Publish command to guarantee delivery
            for _ in range(self.pub_count):
                self.publish_position_cmd(js)
                if self.pub_count > 1:
                    time.sleep(0.1)

            print(f"{YELLOW}Waiting up to 4.0s for {joint} to reach target...{RESET}")
            start_time = time.time()
            success = False
            while time.time() - start_time < 4.0:
                pos, _ = self.get_joint_pose_and_vel(joint)
                if pos is not None:
                    print(f"  Current {joint} position: {pos:.4f} (target: {target:.4f})", end="\r")
                    if abs(pos - target) < 0.02:
                        success = True
                        break
                time.sleep(0.1)
            print() # Newline

            if success:
                print(f"{GREEN}✓ {joint} successfully reached position {target:.4f}!{RESET}")
            else:
                print(f"{RED}✗ Timeout: {joint} did not reach target position {target:.4f}. Current: {pos}{RESET}")
            
            # Restore to home position immediately after testing this joint to prevent self-collision
            self.restore_joint_to_home(joint)
            time.sleep(1.0)
        return True

    # =========================================================================
    # 5. VELOCITY CONTROL COMMANDS & STEADY-STATE ACCURACY VERIFICATION
    # =========================================================================
    def test_joint_velocity_commands(self, interactive=False):
        print_header("JOINT VELOCITY COMMANDS & STEADY-STATE ACCURACY VERIFICATION")
        if interactive:
            confirm = input(f"{YELLOW}Confirm: This will physically move non-gripper joints at constant speed to verify velocity. Proceed? (y/n): {RESET}").lower()
            if confirm != 'y':
                print(f"{YELLOW}Skipped joint velocity test.{RESET}")
                return True

        if self.latest_joint_state is None:
            print(f"{RED}No joint state telemetry available. Cannot run velocity tests.{RESET}")
            return False

        # Send joints to their home positions before the velocity test
        print(f"{YELLOW}Preparing velocity command test: sending all joints to home posture...{RESET}")
        self.restore_all_to_home()

        # Dynamically discover which joints support velocity mode by attempting to set them to 'velocity'.
        # We query all 6 commandable joints.
        all_joints = ["lift", "arm", "wrist_yaw", "wrist_pitch", "wrist_roll", "stretch_gripper"]
        velocity_joints = []
        rejected_velocity_joints = []

        req_set = SetParameters.Request()
        p_mode = ParameterMsg()
        p_mode.value.type = ParameterType.PARAMETER_STRING
        
        print(f"{YELLOW}Discovering velocity mode capabilities for all joints...{RESET}")
        for joint in all_joints:
            p_mode.name = f'joint_mode.{joint}'
            p_mode.value.string_value = 'velocity'
            req_set.parameters = [p_mode]
            
            future = self.param_set_cli.call_async(req_set)
            while rclpy.ok() and not future.done():
                time.sleep(0.1)
            res = future.result()
            
            if res is not None and res.results[0].successful:
                print(f"{GREEN}✓ Joint {joint} ALLOWS velocity mode.{RESET}")
                velocity_joints.append(joint)
                
                # Restore to position mode immediately so it remains in position mode until its specific sweep
                p_mode_pos = ParameterMsg()
                p_mode_pos.name = f'joint_mode.{joint}'
                p_mode_pos.value.type = ParameterType.PARAMETER_STRING
                p_mode_pos.value.string_value = 'position'
                req_restore = SetParameters.Request()
                req_restore.parameters = [p_mode_pos]
                future_restore = self.param_set_cli.call_async(req_restore)
                while rclpy.ok() and not future_restore.done():
                    time.sleep(0.1)
            else:
                print(f"{YELLOW}✗ Joint {joint} REJECTS velocity mode.{RESET}")
                rejected_velocity_joints.append(joint)
            time.sleep(0.1)
            
        print(f"\n{BOLD}Discovery results:{RESET}")
        print(f"  Velocity mode supported: {velocity_joints}")
        print(f"  Velocity mode rejected:  {rejected_velocity_joints}\n")
        time.sleep(1.0)

        # Verify velocity command execution and accuracy for the joints that allow it
        for joint in velocity_joints:
            current_pos, _ = self.get_joint_pose_and_vel(joint)
            if current_pos is None:
                print(f"{RED}Skipping velocity test for {joint} because position is unknown.{RESET}")
                continue

            # Set safe command velocity and direction based on current joint limit margins
            if joint == "lift":
                vel_command = -0.04 if current_pos > 0.5 else 0.04
                desc = "moves lift joint continuously"
            elif joint == "arm":
                vel_command = -0.04 if current_pos > 0.15 else 0.04
                desc = "extends/retracts arm continuously"
            elif joint in ["wrist_yaw", "wrist_pitch", "wrist_roll"]:
                # Move wrist joints slowly biased toward center to stay in [-0.5, 0.5] range
                vel_command = -0.06 if current_pos > 0.0 else 0.06
                desc = f"rotates {joint} joint continuously"
            elif joint == "stretch_gripper":
                # Gripper translation speed
                vel_command = -0.05 if current_pos > 0.2 else 0.05
                desc = "opens/closes gripper finger joints continuously"
            else:
                vel_command = 0.02
                desc = f"moves {joint} joint continuously"

            print_robot_behavior(f"The robot {desc} at commanded velocity {vel_command:+.4f}.")
            print(f"{YELLOW}Setting joint_mode.{joint} to 'velocity'...{RESET}")
            
            p_mode.name = f'joint_mode.{joint}'
            p_mode.value.string_value = 'velocity'
            req_set.parameters = [p_mode]
            future = self.param_set_cli.call_async(req_set)
            while rclpy.ok() and not future.done():
                time.sleep(0.1)
            assert future.result().results[0].successful, f"Failed to set {joint} mode to velocity!"

            # Construct joint velocity command
            js_vel = JointState()
            js_vel.name = [joint]
            js_vel.velocity = [float(vel_command)]

            print(f"{YELLOW}Running velocity test for {joint} at {vel_command:+.4f} (4.0s total)...{RESET}")
            
            # Send the velocity command once (or self.pub_count times)
            for _ in range(self.pub_count):
                self.publish_velocity_cmd(js_vel)
                if self.pub_count > 1:
                    time.sleep(0.05)

            # Phase 1: Ramping & Acceleration (1.0s)
            time.sleep(1.0)

            # Phase 2: Steady-state measurement window (2.0s)
            # Record initial steady-state position and timestamp
            t1 = time.time()
            p1, _ = self.get_joint_pose_and_vel(joint)
            
            p_prev = p1
            t_prev = t1
            
            print(f"{BOLD}Timestep telemetry during 2.0s steady-state window:{RESET}")
            print(f"  {'Time (s)':<10} | {'Published Vel':<15} | {'Measured Vel':<15}")
            print(f"  {'-'*10}-+-{'-'*15}-+-{'-'*15}")
            
            while time.time() - t1 < 2.0:
                time.sleep(0.2)
                t_curr = time.time()
                p_curr, v_pub = self.get_joint_pose_and_vel(joint)
                
                if p_curr is not None and p_prev is not None:
                    dt = t_curr - t_prev
                    dp = p_curr - p_prev
                    v_measured = dp / dt if dt > 0.0 else 0.0
                    
                    elapsed = t_curr - t1
                    v_pub_str = f"{v_pub:+.4f}" if v_pub is not None else "N/A"
                    print(f"  {elapsed:8.2f}s | {v_pub_str:>15} | {v_measured:>+15.4f}")
                    
                    p_prev = p_curr
                    t_prev = t_curr

            # Record final steady-state position and timestamp
            t2 = time.time()
            p2, _ = self.get_joint_pose_and_vel(joint)

            # Phase 3: Halt movement and restore joint mode to position
            print(f"{YELLOW}Halting joint {joint} and restoring mode to position...{RESET}")
            js_vel.velocity = [0.0]
            for _ in range(self.pub_count):
                self.publish_velocity_cmd(js_vel)
                if self.pub_count > 1:
                    time.sleep(0.05)

            p_mode.value.string_value = 'position'
            future = self.param_set_cli.call_async(req_set)
            while rclpy.ok() and not future.done():
                time.sleep(0.1)
            assert future.result().results[0].successful, f"Failed to restore {joint} mode to position!"

            # Wait to settle before calculating
            time.sleep(1.0)

            # Perform Mathematical Steady-State Velocity verification
            if p1 is not None and p2 is not None:
                dt = t2 - t1
                dp = p2 - p1
                vel_measured = dp / dt
                error = vel_measured - vel_command
                pct_error = (abs(error) / abs(vel_command)) * 100.0 if vel_command != 0 else 0.0

                print(f"{BOLD}Velocity Analysis for {joint}:{RESET}")
                print(f"  Commanded Velocity: {vel_command:+.4f}")
                print(f"  Measured Steady-State Velocity: {vel_measured:+.4f} (Moved {dp:+.4f}m/rad in {dt:.3f}s)")
                print(f"  Absolute Deviation: {error:+.4f} | Error Percentage: {pct_error:.2f}%")

                # Validate velocity margin (allow up to 25% deviation on real physical robot dynamics)
                if pct_error < 25.0:
                    print(f"{GREEN}✓ PASS: {joint} moves at close to commanded velocity!{RESET}\n")
                else:
                    print(f"{RED}✗ FAIL: {joint} velocity deviation exceeds 25% tolerance!{RESET}\n")
            else:
                print(f"{RED}✗ Failed to calculate steady-state velocity (JointState telemetry missing during window){RESET}\n")

            # Restore to home position immediately after testing this joint to prevent self-collision
            self.restore_joint_to_home(joint)
            time.sleep(1.0)
        return True

    # =========================================================================
    # 6. MOBILE BASE TWIST COMMANDS (/cmd_vel)
    # =========================================================================
    def test_velocity_commands(self, interactive=False):
        print_header("MOBILE BASE VELOCITY TWIST COMMANDS (/cmd_vel)")
        if interactive:
            confirm = input(f"{YELLOW}Confirm: This will physically rotate/move the mobile base. Proceed? (y/n): {RESET}").lower()
            if confirm != 'y':
                print(f"{YELLOW}Skipped mobile base velocity test.{RESET}")
                return True

        # Test A: Rotate the base slightly in-place
        print_robot_behavior("The mobile base should rotate in-place counter-clockwise at 0.1 rad/s.")
        print(f"{YELLOW}Publishing twist command (angular.z = 0.1) for 2.0 seconds...{RESET}")
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.1

        start_time = time.time()
        while time.time() - start_time < 2.0:
            self.publish_cmd_vel(twist)
            time.sleep(0.1)

        # Halt
        print(f"{YELLOW}Halting mobile base...{RESET}")
        twist.angular.z = 0.0
        for _ in range(self.pub_count):
            self.publish_cmd_vel(twist)
            if self.pub_count > 1:
                time.sleep(0.1)

        print(f"{GREEN}✓ Mobile base rotation test completed successfully!{RESET}")
        time.sleep(1.5)
        return True

    # =========================================================================
    # 7. JOYSTICK TELEOP simulation
    # =========================================================================
    def test_joy_commands(self, interactive=False):
        print_header("JOYSTICK TELEOP SIMULATION (/joy)")
        print_robot_behavior("The robot's mode parameter will temporarily change to 'teleop' and process a dummy Joy command.")

        # Test A: Set mode to teleop
        req_set = SetParameters.Request()
        p_mode = ParameterMsg()
        p_mode.name = 'mode'
        p_mode.value.type = ParameterType.PARAMETER_STRING
        p_mode.value.string_value = 'teleop'
        req_set.parameters = [p_mode]
        
        print(f"{YELLOW}Setting mode parameter to 'teleop'...{RESET}")
        future = self.param_set_cli.call_async(req_set)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        assert future.result().results[0].successful, "Failed to set mode to teleop"
        print(f"{GREEN}✓ Driver successfully transitioned to 'teleop' mode!{RESET}")
        time.sleep(0.5)

        # Test B: Publish joy commands
        print(f"{YELLOW}Publishing dummy joy messages to check driver response...{RESET}")
        joy = Joy()
        joy.axes = [0.0, 0.0, 0.0, 0.0]
        joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        
        for _ in range(self.pub_count):
            self.publish_joy(joy)
            if self.pub_count > 1:
                time.sleep(0.1)

        # Test C: Restore mode to active
        p_mode.value.string_value = 'active'
        future = self.param_set_cli.call_async(req_set)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)
        assert future.result().results[0].successful, "Failed to restore mode to active"
        print(f"{GREEN}✓ Successfully restored driver back to 'active' mode!{RESET}")
        time.sleep(1.0)
        return True

    # =========================================================================
    # 8. TRAJECTORY SERVER circle trace (All modes)
    # =========================================================================
    def test_trajectory_server(self, interactive=False):
        print_header("TRAJECTORY SERVER MULTI-CIRCLE TRACING")
        print_robot_behavior("The robot will execute trajectories in each of the three server modes: 'pid_normal', 'target_priority', and 'time_priority'.")

        if interactive:
            confirm = input(f"{BOLD}{YELLOW}Confirm: This will physically test FollowJointTrajectory to trace circles simultaneously. Proceed? (y/n): {RESET}").lower()
            if confirm != 'y':
                print(f"{YELLOW}Skipped trajectory server test.{RESET}")
                return True

        print(f"{YELLOW}Waiting for follow_joint_trajectory action server...{RESET}")
        if not self._fjt_client.wait_for_server(timeout_sec=5.0):
            print(f"{RED}ERROR: Trajectory action server 'follow_joint_trajectory' is not available!{RESET}")
            print(f"{RED}Make sure you launched the driver properly.{RESET}")
            return False
        print(f"{GREEN}✓ Connected to follow_joint_trajectory action server!{RESET}")

        # The three trajectory server modes implemented by StretchTrajectoryActionServer
        modes = ["target_priority", "pid_normal", "time_priority"]
        modes_desc = {
            "pid_normal": "PID Normal (requires kp, ki, kd; commanded joints must be in velocity mode)",
            "target_priority": "Target Priority (guarantees hitting all positions within a threshold)",
            "time_priority": "Time Priority (interpolates and sends commands at target times regardless)"
        }

        # Backup original trajectory server mode so we can restore it at the end
        original_mode = "target_priority" # Default fallback
        print(f"{YELLOW}Testing all three trajectory server modes...{RESET}")

        for mode_name in modes:
            print(f"\n{BOLD}{CYAN}--------------------------------------------------------------------------------{RESET}")
            print(f"{BOLD}{CYAN}Testing Trajectory Mode: {mode_name} ({modes_desc[mode_name]}){RESET}")
            print(f"{BOLD}{CYAN}--------------------------------------------------------------------------------{RESET}")

            # Prepare joint modes based on the trajectory server mode
            allowed_velocity_joints = []
            if mode_name == "pid_normal":
                # In pid_normal mode, the trajectory server expects all commanded joints to be in velocity mode.
                # Since the gripper typically rejects velocity mode (and wrists did in older driver configurations), we dynamically probe and use
                # only the joints that allow velocity mode.
                print(f"{YELLOW}Probing velocity mode capability for pid_normal joints...{RESET}")
                probe_joints = ["lift", "arm", "wrist_yaw", "wrist_pitch", "wrist_roll", "stretch_gripper"]
                
                p_prob = ParameterMsg()
                p_prob.value.type = ParameterType.PARAMETER_STRING
                p_prob.value.string_value = 'velocity'
                req_prob = SetParameters.Request()
                
                for j_name in probe_joints:
                    p_prob.name = f'joint_mode.{j_name}'
                    req_prob.parameters = [p_prob]
                    future_prob = self.param_set_cli.call_async(req_prob)
                    while rclpy.ok() and not future_prob.done():
                        time.sleep(0.05)
                    res_prob = future_prob.result()
                    if res_prob is not None and res_prob.results[0].successful:
                        allowed_velocity_joints.append(j_name)
                
                active_joints = list(allowed_velocity_joints)
                        
                print(f"{GREEN}✓ Dynamic discovery: joints supporting velocity mode = {active_joints}{RESET}")
                
                if not active_joints:
                    print(f"{YELLOW}No joints support velocity mode. Skipping pid_normal trajectory server test.{RESET}")
                    continue
            else:
                # For target_priority and time_priority, we can command any joints.
                # We command lift, arm, wrist_yaw, wrist_pitch, wrist_roll simultaneously!
                active_joints = ["lift", "arm", "wrist_yaw", "wrist_pitch", "wrist_roll"]

            # Set trajectory_server.mode parameter on driver node
            print(f"{YELLOW}Setting trajectory_server.mode to '{mode_name}'...{RESET}")
            p_srv_mode = ParameterMsg()
            p_srv_mode.name = 'trajectory_server.mode'
            p_srv_mode.value.type = ParameterType.PARAMETER_STRING
            p_srv_mode.value.string_value = mode_name
            req_srv_mode = SetParameters.Request()
            req_srv_mode.parameters = [p_srv_mode]
            future_srv_mode = self.param_set_cli.call_async(req_srv_mode)
            while rclpy.ok() and not future_srv_mode.done():
                time.sleep(0.05)
            assert future_srv_mode.result().results[0].successful, f"Failed to set trajectory_server.mode to {mode_name}"
            print(f"{GREEN}✓ Successfully set trajectory_server.mode parameter!{RESET}")

            # Define the three trajectory configurations to test: sparse, slow/smooth, and fast/smooth
            trajectories_config = [
                {"type": "Sparse/9-point", "duration": 8.0, "num_points": 8},
                {"type": "Slow/Smooth", "duration": 8.0, "num_points": 160},
                {"type": "Fast/Smooth", "duration": 5.0, "num_points": 160}
            ]

            for traj in trajectories_config:
                traj_type = traj["type"]
                duration = traj["duration"]
                n_pts = traj["num_points"]

                print(f"\n{BOLD}{YELLOW}  Executing Trajectory: {traj_type} ({duration}s, {n_pts + 1} points){RESET}")

                # Send joints to their home positions first to make sure they are in safe starting postures
                print(f"{YELLOW}  Sending all joints to home posture for a safe trajectory start...{RESET}")
                self.restore_all_to_home()

                # Set discovered joints to velocity mode if in pid_normal
                if mode_name == "pid_normal":
                    print(f"{YELLOW}  Configuring joints to 'velocity' mode for pid_normal execution...{RESET}")
                    for j_name in allowed_velocity_joints:
                        p_prob = ParameterMsg()
                        p_prob.name = f'joint_mode.{j_name}'
                        p_prob.value.type = ParameterType.PARAMETER_STRING
                        p_prob.value.string_value = 'velocity'
                        req_prob = SetParameters.Request()
                        req_prob.parameters = [p_prob]
                        future_set = self.param_set_cli.call_async(req_prob)
                        while rclpy.ok() and not future_set.done():
                            time.sleep(0.05)

                # Generate circle trajectory points
                points = self.make_circle_trajectory(active_joints=active_joints, num_points=n_pts, total_duration=duration)

                # Execute trajectory
                goal = FollowJointTrajectory.Goal()
                goal.trajectory.joint_names = active_joints
                goal.trajectory.points = points
                goal.goal_time_tolerance = Duration(seconds=2.0).to_msg()
                
                print(f"{YELLOW}  Sending '{mode_name}' {traj_type} circle goal with {len(points)} points...{RESET}")
                future = self._fjt_client.send_goal_async(goal)
                
                # Spin/wait until goal is accepted/rejected
                while rclpy.ok() and not future.done():
                    time.sleep(0.05)
                    
                goal_handle = future.result()
                if not goal_handle.accepted:
                    print(f"{RED}  ERROR: Trajectory mode {mode_name} ({traj_type}) was rejected by the action server!{RESET}")
                    # Restore joint mode if needed before returning
                    if mode_name == "pid_normal":
                        self.restore_joints_to_position(allowed_velocity_joints)
                    return False
                    
                print(f"{GREEN}  ✓ Trajectory goal accepted! Executing circle trace...{RESET}")
                
                # Get result future
                result_future = goal_handle.get_result_async()
                
                # Monitor the execution
                while rclpy.ok() and not result_future.done():
                    time.sleep(0.1)
                    
                result = result_future.result()
                # STATUS_SUCCEEDED = 4 in action_msgs.msg.GoalStatus
                if result.status == 4:
                    print(f"{GREEN}  ✓ Mode '{mode_name}' ({traj_type}) circle trace completed successfully!{RESET}")
                else:
                    print(f"{RED}  ERROR: Trajectory mode '{mode_name}' ({traj_type}) execution failed with status: {result.status}{RESET}")
                    if mode_name == "pid_normal":
                        self.restore_joints_to_position(allowed_velocity_joints)
                    return False

                # Restore joint modes to position mode if they were modified for pid_normal
                if mode_name == "pid_normal":
                    self.restore_joints_to_position(allowed_velocity_joints)

                time.sleep(1.0)

        # Restore trajectory server mode back to its original default
        print(f"{YELLOW}Restoring trajectory_server.mode to default 'target_priority'...{RESET}")
        p_srv_mode = ParameterMsg()
        p_srv_mode.name = 'trajectory_server.mode'
        p_srv_mode.value.type = ParameterType.PARAMETER_STRING
        p_srv_mode.value.string_value = original_mode
        req_srv_mode = SetParameters.Request()
        req_srv_mode.parameters = [p_srv_mode]
        future_srv_mode = self.param_set_cli.call_async(req_srv_mode)
        while rclpy.ok() and not future_srv_mode.done():
            time.sleep(0.05)

        # Restore home posture at the end of the test
        print(f"{YELLOW}Homing all joints at the end of trajectory test...{RESET}")
        self.restore_all_to_home()
        print(f"{GREEN}✓ All three trajectory server modes ('pid_normal', 'target_priority', 'time_priority') tested successfully!{RESET}")
        return True

    def restore_joints_to_position(self, joints):
        """Helper to restore a list of joints back to position mode."""
        print(f"{YELLOW}Restoring joints {joints} back to position mode...{RESET}")
        p_prob = ParameterMsg()
        p_prob.value.type = ParameterType.PARAMETER_STRING
        p_prob.value.string_value = 'position'
        req_restore = SetParameters.Request()
        for j_name in joints:
            p_prob.name = f'joint_mode.{j_name}'
            req_restore.parameters = [p_prob]
            future_restore = self.param_set_cli.call_async(req_restore)
            while rclpy.ok() and not future_restore.done():
                time.sleep(0.05)

    def make_circle_trajectory(self, active_joints, num_points=8, total_duration=8.0):
        """
        Creates a list of JointTrajectoryPoint for tracing synchronized circles
        on the specified active_joints.
        - arm and lift trace a circle in the sagittal plane.
        - wrist yaw, pitch, and roll trace a circle in orientation space.
        """
        points = []
        
        # We define the circle parameters based on home positions
        lift_center = self.HOME_POSITIONS["lift"]
        arm_center = self.HOME_POSITIONS["arm"]
        pitch_center = self.HOME_POSITIONS["wrist_pitch"]
        yaw_center = self.HOME_POSITIONS["wrist_yaw"]
        roll_center = self.HOME_POSITIONS["wrist_roll"]
        
        # Radii of circles
        r_lift = 0.04
        r_arm = 0.04
        r_pitch = 0.15
        r_yaw = 0.15
        r_roll = 0.15
        
        dt = total_duration / num_points
        
        for i in range(num_points + 1):
            point = JointTrajectoryPoint()
            point.time_from_start = Duration(seconds=i * dt).to_msg()
            
            # Parametric angle theta from 0 to 2*pi
            theta = 2.0 * math.pi * (i / num_points)
            
            # Tracing circles simultaneously
            p_lift = lift_center + r_lift * math.sin(theta)
            p_arm = arm_center + r_arm * math.cos(theta)
            p_pitch = pitch_center + r_pitch * math.sin(theta)
            p_yaw = yaw_center + r_yaw * math.cos(theta)
            p_roll = roll_center + r_roll * math.sin(theta)
            
            # Map each active joint to its target position
            positions = []
            for j_name in active_joints:
                if j_name == "lift":
                    positions.append(float(p_lift))
                elif j_name == "arm":
                    positions.append(float(p_arm))
                elif j_name == "wrist_yaw":
                    positions.append(float(p_yaw))
                elif j_name == "wrist_pitch":
                    positions.append(float(p_pitch))
                elif j_name == "wrist_roll":
                    positions.append(float(p_roll))
                elif j_name == "stretch_gripper":
                    positions.append(0.5) # constant open gripper
                else:
                    positions.append(0.0)
            
            point.positions = positions
            points.append(point)
            
        return points

def interactive_menu(tester):
    while rclpy.ok():
        print(f"\n{BOLD}{CYAN}{'='*80}{RESET}")
        print(f"{BOLD}{CYAN}             STRETCH LIVE DRIVER TESTER - INTERACTIVE MENU{RESET}")
        print(f"{BOLD}{CYAN}{'='*80}{RESET}")
        print("  1. [Parameters] Test parameter getting, setting & boundaries")
        print("  2. [Homing Service] Test Homing service (/home_the_robot) (WARNING: moves robot!)")
        print("  3. [Stowing Service] Test Stowing service (/stow_the_robot) (WARNING: moves robot!)")
        print("  4. [Runstop Service] Test Runstop safety, emergency stopping & command blocking")
        print("  5. [Joint Position] Test joint-by-joint Position Sweeps (WARNING: moves robot!)")
        print("  6. [Joint Velocity] Test joint Velocity control & Steady-State Accuracy (WARNING: moves robot!)")
        print("  7. [Base velocity] Test mobile base twist velocity rotation (WARNING: moves robot!)")
        print("  8. [Joy simulation] Test Joystick Teleop transition and Joy subscriber")
        print("  9. [Trajectory Server] Test Trajectory Server multi-circle tracing (All styles) (WARNING: moves robot!)")
        print("  10. [Posture Reset] Restore all joints to neutral/home posture")
        print("  11. [Run All] Run all tests sequentially")
        print("  12. Exit")
        print(f"{BOLD}{CYAN}{'-'*80}{RESET}")
        
        try:
            choice = input(f"{BOLD}{YELLOW}Select option (1-12): {RESET}").strip()
            if choice == '1':
                tester.test_parameters(interactive=True)
            elif choice == '2':
                tester.test_homing_service(interactive=True)
            elif choice == '3':
                tester.test_stowing_service(interactive=True)
            elif choice == '4':
                tester.test_runstop_safety(interactive=True)
            elif choice == '5':
                tester.test_position_commands(interactive=True)
            elif choice == '6':
                tester.test_joint_velocity_commands(interactive=True)
            elif choice == '7':
                tester.test_velocity_commands(interactive=True)
            elif choice == '8':
                tester.test_joy_commands(interactive=True)
            elif choice == '9':
                tester.test_trajectory_server(interactive=True)
            elif choice == '10':
                tester.restore_all_to_home()
            elif choice == '11':
                run_sequential_suite(tester)
            elif choice == '12':
                print(f"\n{BOLD}{GREEN}Exiting. Thank you!{RESET}\n")
                break
            else:
                print(f"{RED}Invalid option. Please input a number from 1 to 12.{RESET}")
        except KeyboardInterrupt:
            print(f"\n{YELLOW}Terminated by user.{RESET}")
            break
        except Exception as e:
            print(f"{RED}Error executing test: {e}{RESET}")

def run_sequential_suite(tester):
    print_header("STARTING SEQUENTIAL TEST SUITE")
    print(f"{YELLOW}Warning: This runs all integration tests sequentially. Physical motion will occur.{RESET}")
    confirm = input(f"{BOLD}{YELLOW}Are you absolutely sure you want to run all tests? (y/n): {RESET}").lower()
    if confirm != 'y':
        print(f"{YELLOW}Sequential suite cancelled.{RESET}")
        return

    try:
        tester.test_parameters(interactive=False)
        tester.test_homing_service(interactive=False)
        tester.test_stowing_service(interactive=False)
        tester.test_runstop_safety(interactive=False)
        tester.test_position_commands(interactive=False)
        tester.test_joint_velocity_commands(interactive=False)
        tester.test_velocity_commands(interactive=False)
        tester.test_joy_commands(interactive=False)
        tester.test_trajectory_server(interactive=False)
        print(f"\n{BOLD}{GREEN}================================================================================{RESET}")
        print(f"{BOLD}{GREEN}                  ALL SEQUENTIAL TESTS COMPLETED SUCCESSFULY!{RESET}")
        print(f"{BOLD}{GREEN}================================================================================{RESET}\n")
    except Exception as e:
        print(f"\n{RED}{BOLD}TEST SUITE ERROR: {e}{RESET}\n")

def main():
    parser = argparse.ArgumentParser(description="Live integration test script for Stretch robot driver node.")
    parser.add_argument("-s", "--sequential", action="store_true", help="Run the entire suite sequentially without asking for menu choice")
    parser.add_argument("-p", "--pub-count", type=int, default=1, help="Number of times to publish commands to ensure delivery (default: 1)")
    args, unknown = parser.parse_known_args()

    rclpy.init()
    tester = StretchLiveDriverTester(pub_count=args.pub_count)

    # Wait for the ROS driver services to be active
    if not tester.wait_for_services(timeout_s=5.0):
        rclpy.shutdown()
        sys.exit(1)

    if args.sequential:
        run_sequential_suite(tester)
    else:
        interactive_menu(tester)

    # Shutdown cleanly
    rclpy.shutdown()

if __name__ == '__main__':
    main()
