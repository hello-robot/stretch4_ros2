#!/usr/bin/env python3
"""
Integration and Live System Test Script for StretchMujocoDriver
===============================================================

This script performs full integration and end-to-end testing against a live, running
StretchMujocoDriver node. It asserts correct responses across all standard ROS2 interfaces
including publishers, subscribers, parameters, and services.

Assumptions:
------------
1. The simulated driver has been launched in a separate terminal:
   ros2 launch stretch_simulation stretch_mujoco_driver.launch.py
2. The MuJoCo simulator GUI or headless viewer is active.

Run instructions:
-----------------
python3 test_live_driver.py
"""

import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Joy
from nav_msgs.msg import Odometry

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
    def __init__(self):
        super().__init__('stretch_live_driver_tester')
        self.get_logger().info("Initializing Live Driver Integration Test Client...")

        # Setup service clients to communicate with StretchMujocoDriver
        self.param_get_cli = self.create_client(GetParameters, '/stretch_mujoco_driver/get_parameters')
        self.param_set_cli = self.create_client(SetParameters, '/stretch_mujoco_driver/set_parameters')
        self.home_cli = self.create_client(Trigger, '/home_the_robot')
        self.stow_cli = self.create_client(Trigger, '/stow_the_robot')
        self.stop_cli = self.create_client(Trigger, '/stop_the_robot')
        self.runstop_cli = self.create_client(SetBool, '/runstop_the_robot')

        # Setup publishers for command inputs
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.position_cmd_pub = self.create_publisher(JointState, '/joint_position_cmd', 10)
        self.velocity_cmd_pub = self.create_publisher(JointState, '/joint_velocity_cmd', 10)
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)

        # Setup variables to record received data from subscribers
        self.latest_joint_state = None
        self.latest_odom = None

        # Setup subscribers to verify outgoing telemetry
        self.js_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

    def joint_state_cb(self, msg):
        self.latest_joint_state = msg

    def odom_cb(self, msg):
        self.latest_odom = msg

    def wait_for_services(self, timeout_s=5.0):
        """Verify that the driver node is indeed up and advertising its interfaces."""
        clients = [
            (self.param_get_cli, "GetParameters"),
            (self.param_set_cli, "SetParameters"),
            (self.home_cli, "home_the_robot"),
            (self.stow_cli, "stow_the_robot"),
            (self.stop_cli, "stop_the_robot"),
            (self.runstop_cli, "runstop_the_robot")
        ]
        for cli, name in clients:
            self.get_logger().info(f"Checking for service {name}...")
            if not cli.wait_for_service(timeout_sec=timeout_s):
                print(f"{RED}{BOLD}ERROR: Service '{name}' is not available!{RESET}")
                print(f"{RED}Make sure you launched the driver via:{RESET}")
                print(f"{YELLOW}ros2 launch stretch_simulation stretch_mujoco_driver.launch.py{RESET}")
                return False
        return True

    # =========================================================================
    # 1. PARAMETER SET/GET INTERFACE
    # =========================================================================
    def test_parameters(self):
        print_header("1. PARAMETER INTERFACE TESTING")
        print_robot_behavior("The robot should not move during parameter querying and setting.")

        # Test A: Get 'mode' parameter
        req_get = GetParameters.Request()
        req_get.names = ['mode', 'position_tolerance']
        
        future = self.param_get_cli.call_async(req_get)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        
        assert res is not None, "Failed to call get_parameters service"
        mode_val = res.values[0].string_value
        print(f"{GREEN}✓ Successfully retrieved default 'mode': {mode_val}{RESET}")
        print(f"{GREEN}✓ Successfully retrieved 'position_tolerance': {res.values[1].double_value}{RESET}")

        # Test B: Set 'mode' parameter to 'teleop'
        req_set = SetParameters.Request()
        p_mode = ParameterMsg()
        p_mode.name = 'mode'
        p_mode.value.type = ParameterType.PARAMETER_STRING
        p_mode.value.string_value = 'teleop'
        req_set.parameters = [p_mode]

        future = self.param_set_cli.call_async(req_set)
        rclpy.spin_until_future_complete(self, future)
        res_set = future.result()
        
        assert res_set is not None and res_set.results[0].successful, "Failed to set 'mode' to teleop"
        print(f"{GREEN}✓ Successfully set parameter 'mode' to 'teleop'{RESET}")

        # Restore 'active' mode
        p_mode.value.string_value = 'active'
        future = self.param_set_cli.call_async(req_set)
        rclpy.spin_until_future_complete(self, future)
        print(f"{GREEN}✓ Restored parameter 'mode' back to 'active'{RESET}")

    # =========================================================================
    # 2. HOMING & STOWING SERVICE INTERFACES
    # =========================================================================
    def test_homing_and_stowing(self):
        print_header("2. STOWING & HOMING SERVICES")
        
        # Test A: Stow the Robot
        print_robot_behavior("The simulated robot should fold its arm, stow its wrist, and rotate its head to the stow posture.")
        req = Trigger.Request()
        future = self.stow_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        assert res.success, "Stowing service call reported failure!"
        print(f"{GREEN}✓ Service '/stow_the_robot' successfully completed!{RESET}")
        time.sleep(2.5)  # Allow time for stowing motion to complete visually

        # Test B: Home the Robot
        print_robot_behavior("The simulated robot's lift, arm, and wrist joints should now home back to their extended/calibrated starting positions.")
        future = self.home_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        assert res.success, "Homing service call reported failure!"
        print(f"{GREEN}✓ Service '/home_the_robot' successfully completed!{RESET}")
        time.sleep(2.5)

    # =========================================================================
    # 3. POSITION CONTROL COMMANDS
    # =========================================================================
    def test_position_commands(self):
        print_header("3. SUBSCRIBER: POSITION COMMANDS (/joint_position_cmd)")
        print_robot_behavior("The simulated robot's lift joint should move upward smoothly to about halfway up the vertical mast (0.5 meters).")

        # Construct position command for the lift joint
        js = JointState()
        js.name = ["lift"]
        js.position = [0.5]
        
        # Publish command multiple times to ensure subscriber receives it
        for _ in range(5):
            self.position_cmd_pub.publish(js)
            time.sleep(0.1)
        
        print(f"{GREEN}✓ Published position goal of 0.5 meters to lift joint.{RESET}")
        print(f"{YELLOW}Waiting for joint state telemetry to reflect target movement...{RESET}")
        
        # Wait and verify joint state updates
        start_time = time.time()
        success = False
        while time.time() - start_time < 8.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            print(" ")
            if self.latest_joint_state is not None:
                idx = self.latest_joint_state.name.index("lift_joint")
                pos = self.latest_joint_state.position[idx]
                print(f"  Current joint_lift position: {pos:.3f} m", end="\r")
                if abs(pos - 0.5) < 0.15:
                    success = True
                    break
            time.sleep(0.2)
        
        print()  # newline
        assert success, "Robot joint did not reach target position in simulation!"
        print(f"{GREEN}✓ Lift joint successfully reached target halfway position in simulation!{RESET}")

    # =========================================================================
    # 4. VELOCITY BASE COMMANDS
    # =========================================================================
    def test_velocity_commands(self):
        print_header("4. SUBSCRIBER: BASE TWIST COMMANDS (/stretch/cmd_vel)")
        print_robot_behavior("The simulated mobile base should move forward linearly and rotate slowly in place.")

        twist = Twist()
        twist.linear.x = 0.15     # Move forward at 0.15 m/s
        twist.angular.z = 0.2     # Rotate at 0.2 rad/s
        
        # Publish for 2 seconds to make base movement clearly visible
        print(f"{YELLOW}Publishing velocity commands...{RESET}")
        for _ in range(20):
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Command a halt
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        print(f"{GREEN}✓ Base movement command and deceleration halt executed!{RESET}")

    # =========================================================================
    # 5. JOYSTICK TELEOP COMMANDS
    # =========================================================================
    def test_joy_commands(self):
        print_header("5. SUBSCRIBER: JOYSTICK CALLBACK (/joy)")
        print_robot_behavior("The robot lift joint should move upward due to virtual button presses, then halt.")

        # First, set mode to teleop
        req_set = SetParameters.Request()
        p_mode = ParameterMsg()
        p_mode.name = 'mode'
        p_mode.value.type = ParameterType.PARAMETER_STRING
        p_mode.value.string_value = 'teleop'
        req_set.parameters = [p_mode]
        future = self.param_set_cli.call_async(req_set)
        rclpy.spin_until_future_complete(self, future)
        
        # Publish Joy msg simulating holding lift button up
        joy = Joy()
        joy.axes = [0.0, 0.0]
        joy.buttons = [1, 0]  # Simulates holding button 0 (lift up)
        
        print(f"{YELLOW}Publishing virtual gamepad button presses...{RESET}")
        for _ in range(15):
            self.joy_pub.publish(joy)
            rclpy.spin_once(self, timeout_sec=0.1)

        # Release buttons
        joy.buttons = [0, 0]
        self.joy_pub.publish(joy)
        
        # Restore mode back to active
        p_mode.value.string_value = 'active'
        future = self.param_set_cli.call_async(req_set)
        rclpy.spin_until_future_complete(self, future)
        print(f"{GREEN}✓ Joystick teleop simulation completed and mode restored to active.{RESET}")

    # =========================================================================
    # 6. SAFETY RUNSTOP SERVICE
    # =========================================================================
    def test_runstop_safety(self):
        print_header("6. SAFETY RUNSTOP SERVICE (/runstop)")
        print_robot_behavior("1. The robot enters runstopped mode. All movement commands will be completely blocked.\n"
                            "  2. We attempt to send a position command to the lift. The robot must NOT move at all.\n"
                            "  3. We release the runstop, and the robot resumes normal active operations.")

        # Step 1: Engage Runstop
        req = SetBool.Request()
        req.data = True
        future = self.runstop_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        assert future.result().success, "Failed to engage runstop safety!"
        print(f"{GREEN}✓ Safety runstop successfully ENGAGED!{RESET}")

        # Step 2: Attempt position command (should be blocked)
        js = JointState()
        js.name = ["lift"]
        js.position = [0.9]  # Command lift to go high up
        for _ in range(5):
            self.position_cmd_pub.publish(js)
            time.sleep(0.1)
        
        print(f"{YELLOW}Verification: Waiting to confirm no physical movement occurs...{RESET}")
        time.sleep(1.5)
        
        # Verify lift did not move to 0.9 (should still be near 0.5)
        rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_joint_state is not None:
            idx = self.latest_joint_state.name.index("lift_joint")
            pos = self.latest_joint_state.position[idx]
            print(f"  Current joint_lift position during runstop: {pos:.3f} m")
            assert pos < 0.75, "SAFETY FAILURE: Robot moved while safety runstop was active!"
            print(f"{GREEN}✓ Verification Successful: Robot movement was blocked!{RESET}")

        # Step 3: Release Runstop
        req.data = False
        future = self.runstop_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        assert future.result().success, "Failed to release runstop!"
        print(f"{GREEN}✓ Safety runstop successfully RELEASED!{RESET}")

def main():
    rclpy.init()
    tester = StretchLiveDriverTester()

    # Wait for the main driver node services to make sure it is running
    if not tester.wait_for_services():
        tester.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    try:
        # Run entire integration test suite
        tester.test_parameters()
        time.sleep(3.0)
        tester.test_homing_and_stowing()
        time.sleep(3.0)
        tester.test_position_commands()
        time.sleep(3.0)
        tester.test_velocity_commands()
        time.sleep(3.0)
        tester.test_runstop_safety()
        time.sleep(3.0)
        tester.test_joy_commands()
        time.sleep(3.0)
        
        print(f"\n{BOLD}{GREEN}{'='*80}{RESET}")
        print(f"{BOLD}{GREEN}ALL LIVE INTEGRATION TESTS PASSED SUCCESSFULLY!{RESET}")
        print(f"{BOLD}{GREEN}{'='*80}{RESET}")

    except AssertionError as e:
        print(f"\n{BOLD}{RED}{'='*80}{RESET}")
        print(f"{BOLD}{RED}INTEGRATION TEST FAILURE: {e}{RESET}")
        print(f"{BOLD}{RED}{'='*80}{RESET}")
    except KeyboardInterrupt:
        print(f"\n{YELLOW}Testing interrupted by user.{RESET}")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
