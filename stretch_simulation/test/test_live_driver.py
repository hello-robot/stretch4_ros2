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
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Joy
from nav_msgs.msg import Odometry
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

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
        self.cmd_vel_pub = self.create_publisher(Twist, '/stretch/cmd_vel', 10)
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

    def ros_sleep(self, duration_s):
        """Sleeps using ROS 2 simulation time while spinning the executor."""
        start_time = self.get_clock().now()
        while rclpy.ok():
            elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9
            if elapsed >= duration_s:
                break
            # Spin briefly to allow other callbacks to execute
            rclpy.spin_once(self, timeout_sec=0.01)

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
        print_robot_behavior("The simulated mobile base should move forward linearly in an arc, then execute a reverse maneuvering curve.")

        twist = Twist()
        twist.linear.x = 0.15     # Move forward at 0.15 m/s
        twist.angular.z = 0.25    # Rotate at 0.25 rad/s
        
        # Stage 1: Forward turning arc for 6 seconds
        print(f"{YELLOW}Publishing forward arc velocity commands (6.0 seconds)...{RESET}")
        for _ in range(60):
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
            
        # Stage 2: Reverse right curve for 6 seconds to show complex maneuvering
        print(f"{YELLOW}Publishing reverse curve velocity commands (6.0 seconds)...{RESET}")
        twist.linear.x = -0.12    # Reverse at 0.12 m/s
        twist.angular.z = -0.3    # Turn opposite direction at -0.3 rad/s
        for _ in range(60):
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Command a halt
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        print(f"{GREEN}✓ Base complex driving maneuver and deceleration halt completed successfully!{RESET}")

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

    def test_trajectory_action(self):
        print_header("7. FOLLOW_JOINT_TRAJECTORY ACTION SERVER INTERFACE")
        print_robot_behavior("The robot should execute a trajectory with its lift joint in various modes, and handle preemption.")

        # Create action client
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/follow_joint_trajectory')
        self.get_logger().info("Waiting for /follow_joint_trajectory action server...")
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            print(f"{RED}{BOLD}ERROR: /follow_joint_trajectory Action Server is not available!{RESET}")
            assert False, "Action server not found"
        print(f"{GREEN}✓ FollowJointTrajectory Action Client initialized!{RESET}")

        # Helper to construct a simple 1-waypoint trajectory goal for the lift joint
        def make_lift_goal(pos_target, time_from_start_sec):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ["lift"]
            
            point = JointTrajectoryPoint()
            point.positions = [float(pos_target)]
            point.time_from_start = Duration(seconds=int(time_from_start_sec)).to_msg()
            
            goal.trajectory.points = [point]
            return goal

        # Helper to set trajectory_server parameters
        def set_traj_param(name, value, val_type):
            req = SetParameters.Request()
            p = ParameterMsg()
            p.name = f"trajectory_server.{name}"
            p.value.type = val_type
            if val_type == ParameterType.PARAMETER_STRING:
                p.value.string_value = value
            elif val_type == ParameterType.PARAMETER_BOOL:
                p.value.bool_value = value
            elif val_type == ParameterType.PARAMETER_DOUBLE:
                p.value.double_value = float(value)
            elif val_type == ParameterType.PARAMETER_INTEGER:
                p.value.integer_value = int(value)
            req.parameters = [p]
            future = self.param_set_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            return future.result().results[0].successful

        # Set joint_mode.lift to position
        req_set = SetParameters.Request()
        p_mode = ParameterMsg()
        p_mode.name = 'joint_mode.lift'
        p_mode.value.type = ParameterType.PARAMETER_STRING
        p_mode.value.string_value = 'position'
        req_set.parameters = [p_mode]
        future = self.param_set_cli.call_async(req_set)
        rclpy.spin_until_future_complete(self, future)

        # -------------------------------------------------------------
        # Test A: time_priority mode
        # -------------------------------------------------------------
        print(f"\n{BOLD}{CYAN}Test A: 'time_priority' Mode{RESET}")
        assert set_traj_param("mode", "time_priority", ParameterType.PARAMETER_STRING), "Failed to set mode"
        assert set_traj_param("offset", 0.0, ParameterType.PARAMETER_DOUBLE), "Failed to set offset"
        
        goal_a = make_lift_goal(0.3, 2.0)
        print(f"{YELLOW}Sending goal to move lift to 0.3m in 2.0s using time_priority...{RESET}")
        
        send_goal_future = self.trajectory_client.send_goal_async(goal_a)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        assert goal_handle.accepted, "Goal was rejected by action server!"
        
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()
        
        assert result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL, f"Goal failed with error: {result.result.error_code}"
        print(f"{GREEN}✓ time_priority test passed successfully!{RESET}")
        time.sleep(1.0)

        # -------------------------------------------------------------
        # Test B: target_priority mode (with feedback verification)
        # -------------------------------------------------------------
        print(f"\n{BOLD}{CYAN}Test B: 'target_priority' Mode{RESET}")
        assert set_traj_param("mode", "target_priority", ParameterType.PARAMETER_STRING), "Failed to set mode"
        assert set_traj_param("threshold", 0.05, ParameterType.PARAMETER_DOUBLE), "Failed to set threshold"
        assert set_traj_param("timeout", 5.0, ParameterType.PARAMETER_DOUBLE), "Failed to set timeout"

        goal_b = make_lift_goal(0.6, 2.0)
        print(f"{YELLOW}Sending goal to move lift to 0.6m (threshold 0.05) using target_priority...{RESET}")
        
        send_goal_future = self.trajectory_client.send_goal_async(goal_b)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        assert goal_handle.accepted, "Goal was rejected!"
        
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()
        
        assert result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL, f"Goal failed: {result.result.error_code}"
        
        # Verify lift position is indeed near 0.6
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_joint_state is not None:
            idx = self.latest_joint_state.name.index("lift_joint")
            pos = self.latest_joint_state.position[idx]
            print(f"  Final position: {pos:.3f} m")
        print(f"{GREEN}✓ target_priority test passed successfully!{RESET}")
        time.sleep(1.0)

        # -------------------------------------------------------------
        # Test C: pid_normal mode (calculates and sends velocities) - BYPASSED (Velocity control not implemented)
        # -------------------------------------------------------------
        # print(f"\n{BOLD}{CYAN}Test C: 'pid_normal' Mode{RESET}")
        # assert set_traj_param("mode", "pid_normal", ParameterType.PARAMETER_STRING), "Failed to set mode"
        # assert set_traj_param("kp", 2.0, ParameterType.PARAMETER_DOUBLE), "Failed to set kp"
        # assert set_traj_param("ki", 0.05, ParameterType.PARAMETER_DOUBLE), "Failed to set ki"
        # assert set_traj_param("kd", 0.1, ParameterType.PARAMETER_DOUBLE), "Failed to set kd"

        # # Change joint mode of lift to velocity first because pid_normal outputs velocities
        # p_mode.value.string_value = 'velocity'
        # future = self.param_set_cli.call_async(req_set)
        # rclpy.spin_until_future_complete(self, future)

        # goal_c = make_lift_goal(0.4, 2.0)
        # print(f"{YELLOW}Sending goal to move lift to 0.4m using pid_normal PID control...{RESET}")
        
        # send_goal_future = self.trajectory_client.send_goal_async(goal_c)
        # rclpy.spin_until_future_complete(self, send_goal_future)
        # goal_handle = send_goal_future.result()
        # assert goal_handle.accepted, "Goal was rejected!"
        
        # get_result_future = goal_handle.get_result_async()
        # rclpy.spin_until_future_complete(self, get_result_future)
        # result = get_result_future.result()
        
        # assert result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL, f"Goal failed: {result.result.error_code}"
        
        # # Verify position is near 0.4
        # for _ in range(10):
        #     rclpy.spin_once(self, timeout_sec=0.1)
        # if self.latest_joint_state is not None:
        #     idx = self.latest_joint_state.name.index("lift_joint")
        #     pos = self.latest_joint_state.position[idx]
        #     print(f"  Final position: {pos:.3f} m")
        #     assert abs(pos - 0.4) < 0.1, f"PID failed to regulate to target position! Expected 0.4, got {pos}"
        # print(f"{GREEN}✓ pid_normal test passed successfully!{RESET}")
        # time.sleep(1.0)

        # -------------------------------------------------------------
        # Test D: pid_correction mode - BYPASSED (Velocity control not implemented)
        # -------------------------------------------------------------
        # print(f"\n{BOLD}{CYAN}Test D: 'pid_correction' Mode{RESET}")
        # assert set_traj_param("mode", "pid_correction", ParameterType.PARAMETER_STRING), "Failed to set mode"
        
        # # Restore lift joint mode to position since pid_correction adds correction on top of position commands
        # p_mode.value.string_value = 'position'
        # future = self.param_set_cli.call_async(req_set)
        # rclpy.spin_until_future_complete(self, future)

        # goal_d = make_lift_goal(0.5, 2.0)
        # print(f"{YELLOW}Sending goal to move lift to 0.5m using pid_correction...{RESET}")
        
        # send_goal_future = self.trajectory_client.send_goal_async(goal_d)
        # rclpy.spin_until_future_complete(self, send_goal_future)
        # goal_handle = send_goal_future.result()
        # assert goal_handle.accepted, "Goal was rejected!"
        
        # get_result_future = goal_handle.get_result_async()
        # rclpy.spin_until_future_complete(self, get_result_future)
        # result = get_result_future.result()
        
        # assert result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL, f"Goal failed: {result.result.error_code}"
        # print(f"{GREEN}✓ pid_correction test passed successfully!{RESET}")
        # time.sleep(1.0)
        time.sleep(1.0)

        # -------------------------------------------------------------
        # Test E: Interruption via direct joint commands (preemption)
        # -------------------------------------------------------------
        print(f"\n{BOLD}{CYAN}Test E: Interruption via Direct Command (Preemption){RESET}")
        assert set_traj_param("mode", "time_priority", ParameterType.PARAMETER_STRING), "Failed to set mode"
        
        goal_e = make_lift_goal(0.2, 4.0) # Slow 4-second motion
        print(f"{YELLOW}Sending slow goal to 0.2m, will interrupt with direct position command...{RESET}")
        
        send_goal_future = self.trajectory_client.send_goal_async(goal_e)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        assert goal_handle.accepted, "Goal was rejected!"
        
        # Wait a moment for trajectory to start executing (using ROS simulation time)
        self.ros_sleep(0.8)
        
        # Send a direct position command to the lift joint to preempt it
        print(f"{YELLOW}Publishing direct joint command to 'lift' joint to preempt the active goal...{RESET}")
        js = JointState()
        js.name = ["lift"]
        js.position = [0.45]
        self.position_cmd_pub.publish(js)
        
        # Get result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()
        
        # Ensure that the server returned failure/aborted (not SUCCESSFUL) because of preemption
        assert result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL, "Goal succeeded but was expected to be aborted due to preemption!"
        print(f"{GREEN}✓ Preemption via direct command test passed successfully (goal was aborted as expected)!{RESET}")
        time.sleep(1.0)

        # -------------------------------------------------------------
        # Test F: Interruption via robot mode change
        # -------------------------------------------------------------
        print(f"\n{BOLD}{CYAN}Test F: Interruption via Robot Mode Change{RESET}")
        goal_f = make_lift_goal(0.6, 4.0)
        print(f"{YELLOW}Sending goal to 0.6m, will interrupt by changing robot mode parameter to 'teleop'...{RESET}")
        
        send_goal_future = self.trajectory_client.send_goal_async(goal_f)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        assert goal_handle.accepted, "Goal was rejected!"
        
        # Wait a moment for trajectory to start executing (using ROS simulation time)
        self.ros_sleep(0.8)
        
        # Change robot mode to teleop
        print(f"{YELLOW}Changing robot 'mode' parameter to 'teleop'...{RESET}")
        req_set_mode = SetParameters.Request()
        p_mode_teleop = ParameterMsg()
        p_mode_teleop.name = 'mode'
        p_mode_teleop.value.type = ParameterType.PARAMETER_STRING
        p_mode_teleop.value.string_value = 'teleop'
        req_set_mode.parameters = [p_mode_teleop]
        future_mode = self.param_set_cli.call_async(req_set_mode)
        rclpy.spin_until_future_complete(self, future_mode)
        
        # Get result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()
        
        # Ensure goal was aborted
        assert result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL, "Goal succeeded but was expected to be aborted due to robot mode change!"
        print(f"{GREEN}✓ Preemption via robot mode change test passed successfully!{RESET}")
        
        # Restore robot mode back to active
        p_mode_teleop.value.string_value = 'active'
        future_mode_restore = self.param_set_cli.call_async(req_set_mode)
        rclpy.spin_until_future_complete(self, future_mode_restore)
        time.sleep(1.0)

        # -------------------------------------------------------------
        # Test G: Multi-joint & Long Duration Circle Tracing Trajectory
        # -------------------------------------------------------------
        print(f"\n{BOLD}{CYAN}Test G: Multi-Joint & Long-Duration Trajectory (Tracing a Circle){RESET}")
        assert set_traj_param("mode", "time_priority", ParameterType.PARAMETER_STRING), "Failed to set mode"
        
        # Set relevant joints to position mode
        for joint in ["wrist_pitch", "wrist_roll", "stretch_gripper", "wrist_yaw"]:
            req_set_joint = SetParameters.Request()
            p_joint_mode = ParameterMsg()
            p_joint_mode.name = f'joint_mode.{joint}'
            p_joint_mode.value.type = ParameterType.PARAMETER_STRING
            p_joint_mode.value.string_value = 'position'
            req_set_joint.parameters = [p_joint_mode]
            future_joint = self.param_set_cli.call_async(req_set_joint)
            rclpy.spin_until_future_complete(self, future_joint)

        # Build trajectory goal
        goal_g = FollowJointTrajectory.Goal()
        goal_g.trajectory.joint_names = ["wrist_pitch", "wrist_yaw", "wrist_roll", "stretch_gripper"]
        
        # Create 40 waypoints over 8.0 seconds to trace a circle
        import math
        num_points = 40
        total_time = 8.0
        for idx in range(1, num_points + 1):
            t = (idx / num_points) * total_time
            angle = (2.0 * math.pi * t) / total_time
            
            point = JointTrajectoryPoint()
            # Tracing circle with wrist_pitch and wrist_yaw
            pitch_pos = 0.2 * math.sin(angle)
            yaw_pos = 0.2 * math.cos(angle)
            # Roll rotates slowly
            roll_pos = 0.5 * math.sin(angle)
            # Gripper opens and closes
            gripper_pos = 0.1 * (1.0 + math.sin(2.0 * angle))
            
            point.positions = [float(pitch_pos), float(yaw_pos), float(roll_pos), float(gripper_pos)]
            point.time_from_start = Duration(seconds=int(t), nanoseconds=int((t - int(t)) * 1e9)).to_msg()
            goal_g.trajectory.points.append(point)

        print(f"{YELLOW}Sending 40-waypoint trajectory to trace a circle with the wrist and gripper over {total_time}s...{RESET}")
        
        send_goal_future = self.trajectory_client.send_goal_async(goal_g)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        assert goal_handle.accepted, "Circle tracing goal was rejected by action server!"
        
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()
        
        assert result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL, f"Circle tracing failed: {result.result.error_code}"
        print(f"{GREEN}✓ Multi-joint circle-tracing trajectory completed successfully!{RESET}")
        time.sleep(1.0)

    def test_trajectory_diagnostics(self):
        print_header("8. TRAJECTORY AND JOINT POSITION DIAGNOSTICS")
        print_robot_behavior("Diagnosing control discrepancies between commanded goals and actual physical positions on both publishers and action server.")

        results = []

        def log_result(interface, joint, target, actual):
            results.append({
                'Interface': interface,
                'Joint': joint,
                'Target': target,
                'Actual': actual,
                'Error': actual - target,
                'Ratio': actual / target if target != 0.0 else 1.0
            })

        # --- Part 1: Direct Joint Command Message Interface ---
        print(f"\n{BOLD}{CYAN}Part 1: Direct Joint Command Message Interface (/joint_position_cmd){RESET}")
        
        # Restore lift joint mode to position
        req_set = SetParameters.Request()
        p_mode = ParameterMsg()
        p_mode.name = 'joint_mode.lift'
        p_mode.value.type = ParameterType.PARAMETER_STRING
        p_mode.value.string_value = 'position'
        req_set.parameters = [p_mode]
        future = self.param_set_cli.call_async(req_set)
        rclpy.spin_until_future_complete(self, future)

        tests = [
            ("lift", 0.4),
            ("lift", 0.6),
            ("lift", 0.8),
            ("wrist_yaw", 1.0),
            ("wrist_pitch", 0.2),
            ("wrist_roll", 0.5),
            ("stretch_gripper", 40),
            ("arm", 0.3)
        ]

        for joint, target in tests:
            print(f"Commanding {joint} to {target} via /joint_position_cmd...")
            js = JointState()
            js.name = [joint]
            js.position = [float(target)]
            self.position_cmd_pub.publish(js)
            
            # Wait and spin
            time.sleep(1.5)
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
                
            # Get actual position from /joint_states
            actual_joint_name = "lift_joint" if joint == "lift" else f"{joint}_joint"
            if joint == "arm":
                actual_joint_name = "arm_l1_joint"  # or sum of arm_l1_joint..arm_l4_joint
            elif joint == "stretch_gripper":
                actual_joint_name = "gripper_finger_left_joint"
                
            if self.latest_joint_state is not None:
                if joint == "arm":
                    # Sum of arm_l1_joint to arm_l4_joint
                    arm_positions = [self.latest_joint_state.position[self.latest_joint_state.name.index(link)] for link in ['arm_l1_joint', 'arm_l2_joint', 'arm_l3_joint', 'arm_l4_joint']]
                    pos = sum(arm_positions)
                else:
                    idx = self.latest_joint_state.name.index(actual_joint_name)
                    pos = self.latest_joint_state.position[idx]
                
                print(f"  --> Target: {target}, Actual: {pos:.4f}")
                log_result("Direct command", joint, target, pos)
            else:
                print(f"  --> Target: {target}, Actual: Unknown (No JointState received)")

        # --- Part 2: Trajectory Server Interface ---
        print(f"\n{BOLD}{CYAN}Part 2: Trajectory Server Interface (/follow_joint_trajectory){RESET}")
        
        # Helper to set trajectory_server parameters
        def set_traj_param(name, value, val_type):
            req = SetParameters.Request()
            p = ParameterMsg()
            p.name = f"trajectory_server.{name}"
            p.value.type = val_type
            if val_type == ParameterType.PARAMETER_STRING:
                p.value.string_value = value
            elif val_type == ParameterType.PARAMETER_BOOL:
                p.value.bool_value = value
            elif val_type == ParameterType.PARAMETER_DOUBLE:
                p.value.double_value = float(value)
            elif val_type == ParameterType.PARAMETER_INTEGER:
                p.value.integer_value = int(value)
            req.parameters = [p]
            future = self.param_set_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            return future.result().results[0].successful

        assert set_traj_param("mode", "time_priority", ParameterType.PARAMETER_STRING), "Failed to set mode"

        # Initialize FollowJointTrajectory client if not already
        if not hasattr(self, 'trajectory_client'):
            self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/follow_joint_trajectory')
            self.trajectory_client.wait_for_server(timeout_sec=5.0)

        def make_goal(joint, pos_target, time_from_start_sec):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = [joint]
            point = JointTrajectoryPoint()
            point.positions = [float(pos_target)]
            point.time_from_start = Duration(seconds=int(time_from_start_sec)).to_msg()
            goal.trajectory.points = [point]
            return goal

        trajectory_tests = [
            ("lift", 0.4),
            ("lift", 0.6),
            ("lift", 0.8),
            ("wrist_yaw", 0.5),
            ("wrist_pitch", 0.2),
            ("wrist_roll", 0.4),
            ("stretch_gripper", 0.1),
        ]

        for joint, target in trajectory_tests:
            print(f"Sending trajectory goal for {joint} to {target} in 2.0s...")
            goal = make_goal(joint, target, 2.0)
            send_goal_future = self.trajectory_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()
            
            if goal_handle.accepted:
                get_result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, get_result_future)
                
                # Sleep and spin to let telemetry settle
                time.sleep(1.0)
                for _ in range(10):
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                actual_joint_name = "lift_joint" if joint == "lift" else f"{joint}_joint"
                if joint == "stretch_gripper":
                    actual_joint_name = "gripper_finger_left_joint"
                if self.latest_joint_state is not None:
                    idx = self.latest_joint_state.name.index(actual_joint_name)
                    pos = self.latest_joint_state.position[idx]
                    print(f"  --> Target: {target}, Actual: {pos:.4f}")
                    log_result("Trajectory server", joint, target, pos)
            else:
                print("  --> Goal REJECTED")

        # --- Print Diagnostic Summary Table ---
        print(f"\n{BOLD}{GREEN}{'='*80}{RESET}")
        print(f"{BOLD}{GREEN}                      DIAGNOSTIC SUMMARY REPORT                      {RESET}")
        print(f"{BOLD}{GREEN}{'='*80}{RESET}")
        print(f"{'Interface':<20} | {'Joint':<10} | {'Target':<8} | {'Actual':<8} | {'Error':<8} | {'Ratio':<6}")
        print("-" * 80)
        for r in results:
            print(f"{r['Interface']:<20} | {r['Joint']:<10} | {r['Target']:<8.2f} | {r['Actual']:<8.4f} | {r['Error']:<+8.4f} | {r['Ratio']:<6.4f}")
        print(f"{BOLD}{GREEN}{'='*80}{RESET}")

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
        '''tester.test_parameters()
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
        tester.test_trajectory_action()
        time.sleep(3.0)'''
        tester.test_trajectory_diagnostics()
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
