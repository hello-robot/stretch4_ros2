#!/usr/bin/env python3

import sys
import select
import tty
import termios
import threading
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from control_msgs.msg import JointJog
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from tf_transformations import euler_from_quaternion

# Help text and keyboard mapping
HELP_TEXT = """
Stretch 4 ROS2 Keyboard Teleop CLI Tool
=======================================
Control all joints of Stretch 4 using your keyboard.

Movement Controls:
------------------
Base Movement (WASD):
  [w] / [s] : Translate Forward / Backward
  [a] / [d] : Strafe Left / Right
  [q] / [e] : Rotate Left / Right (CCW / CW)

Joint Movement (Positive / Negative):
  [r] / [f] : Lift Up / Down
  [t] / [g] : Arm Extend / Retract
  [y] / [h] : Wrist Yaw Left / Right (CCW / CW)
  [u] / [j] : Wrist Pitch Up / Down
  [i] / [k] : Wrist Roll Left / Right (CCW / CW)
  [o] / [l] : Gripper Open / Close

Utility Controls:
  [space]   : Emergency Stop (zero all velocities instantly)
  [v]       : Set driver mode to 'velocity' (automatic on startup)
  [p]       : Enter 'position' mode (prompts for coordinate movement)
  [h]       : Print this help menu again
  [esc] / [enter] : Exit the tool safely

* Note: Actuators stop automatically and instantly when keys are released.
"""

class KBHit:
    """
    Context manager to handle non-blocking keyboard input in the terminal.
    """
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)

    def __enter__(self):
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def kbhit(self):
        dr, dw, de = select.select([sys.stdin], [], [], 0)
        return dr != []

    def getch(self):
        return sys.stdin.read(1)


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.get_logger().info("Initializing Keyboard Teleop Node...")

        self.callback_group = ReentrantCallbackGroup()

        # Command states
        self.cmd_lock = threading.Lock()
        self.last_keypress_time = 0.0
        self.key_timeout = 0.25  # seconds

        # Target velocities
        self.base_linear_x = 0.0
        self.base_linear_y = 0.0
        self.base_angular_z = 0.0

        self.joint_vels = {
            'lift_joint': 0.0,
            'arm_joint': 0.0,
            'wrist_yaw_joint': 0.0,
            'wrist_pitch_joint': 0.0,
            'wrist_roll_joint': 0.0,
            'gripper_joint': 0.0,
            'parallel_gripper_joint': 0.0
        }

        # Joint positions & base odometry state
        self.state_lock = threading.Lock()
        self.joint_positions = {}
        self.base_x = 0.0
        self.base_y = 0.0
        self.base_theta = 0.0

        # Dynamically discovered gripper joint type
        self.gripper_joint_type = 'gripper_joint'

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_vel_pub = self.create_publisher(JointJog, 'joint_vel', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10,
            callback_group=self.callback_group
        )
        self.odom_sub = self.create_subscription(
            Odometry, 'wheel_odom', self.odom_callback, 10,
            callback_group=self.callback_group
        )

        # Action Client for joint trajectory tracking
        self.action_client = ActionClient(
            self, FollowJointTrajectory, 'follow_joint_trajectory',
            callback_group=self.callback_group
        )

        # Service Client for switching driver mode
        self.set_param_client = self.create_client(
            SetParameters, '/stretch_driver/set_parameters',
            callback_group=self.callback_group
        )

        # Timer to periodically publish commands at 15 Hz
        self.timer = self.create_timer(
            1.0 / 15.0, self.publish_commands,
            callback_group=self.callback_group
        )

        # Mode tracking
        self.driver_mode = "unknown"

        # Clear terminal screen completely on start
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()

    def set_driver_mode(self, mode_str: str):
        """
        Calls the /stretch_driver/set_parameters service to switch driver mode.
        """
        self.get_logger().info(f"Requesting driver mode change to: {mode_str}...")
        if not self.set_param_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("SetParameters service not available. Is stretch_driver running?")
            return False

        req = SetParameters.Request()
        param = Parameter()
        param.name = 'mode'
        param.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=mode_str)
        req.parameters = [param]

        future = self.set_param_client.call_async(req)
        
        # Simple spin helper to wait for the future
        start_t = time.time()
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
            if time.time() - start_t > 3.0:
                self.get_logger().error("Mode change request timed out.")
                return False

        try:
            res = future.result()
            if res and len(res.results) > 0 and res.results[0].successful:
                self.get_logger().info(f"Driver successfully switched to {mode_str} mode.")
                self.driver_mode = mode_str
                return True
            else:
                msg = res.results[0].reason if res else "unknown error"
                self.get_logger().error(f"Failed to switch driver mode: {msg}")
                return False
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return False

    def send_position_goal(self, joint_name: str, target_pos: float):
        """
        Sends a FollowJointTrajectory goal to move the selected joint to target_pos.
        """
        self.get_logger().info(f"Preparing position goal: {joint_name} -> {target_pos}")
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            print("\nError: follow_joint_trajectory action server not available!")
            return False

        # Map to specific gripper names
        if joint_name == 'gripper':
            joint_name = self.gripper_joint_type

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [joint_name]

        point = JointTrajectoryPoint()
        point.positions = [target_pos]
        point.time_from_start.sec = 3
        point.time_from_start.nanosec = 0
        goal_msg.trajectory.points = [point]

        future = self.action_client.send_goal_async(goal_msg)
        
        # Wait for acceptance
        start_t = time.time()
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
            if time.time() - start_t > 3.0:
                print("\nError: Send goal timed out.")
                return False
                
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("\nGoal rejected by server!")
            return False

        print("Goal accepted, executing movement...")
        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.1)

        print("Movement completed successfully!")
        return True

    def odom_callback(self, msg: Odometry):
        with self.state_lock:
            self.base_x = msg.pose.pose.position.x
            self.base_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.base_theta = euler[2]

    def joint_state_callback(self, msg: JointState):
        with self.state_lock:
            for name, pos in zip(msg.name, msg.position):
                self.joint_positions[name] = pos

            # Check arm_l1_joint to arm_l4_joint to compute full arm extension
            arm_links = ['arm_l1_joint', 'arm_l2_joint', 'arm_l3_joint', 'arm_l4_joint']
            if all(link in self.joint_positions for link in arm_links):
                # Total extension is the sum of the link positions (or pos_motor)
                self.joint_positions['arm_joint'] = sum(self.joint_positions[link] for link in arm_links)
            elif 'arm_l4_joint' in self.joint_positions:
                # Fallback: if only arm_l4_joint is updated, total is 4 times that
                self.joint_positions['arm_joint'] = self.joint_positions['arm_l4_joint'] * 4.0

            # Dynamically determine if parallel gripper or regular stretch gripper is used
            if 'parallel_gripper_joint' in self.joint_positions or 'finger_left_joint' in self.joint_positions:
                self.gripper_joint_type = 'parallel_gripper_joint'
                # For parallel gripper, pos is finger_left_joint (which is absolute position or pos_mm)
                # or we can read parallel_gripper_joint directly
                if 'parallel_gripper_joint' not in self.joint_positions and 'finger_left_joint' in self.joint_positions:
                    self.joint_positions['parallel_gripper_joint'] = self.joint_positions['finger_left_joint']
            else:
                self.gripper_joint_type = 'gripper_joint'
                if 'gripper_joint' not in self.joint_positions and 'gripper_finger_left_joint' in self.joint_positions:
                    self.joint_positions['gripper_joint'] = self.gripper_finger_left_joint

    def publish_commands(self):
        with self.cmd_lock:
            now = self.get_clock().now().nanoseconds / 1e9
            # If the user has not pressed a key recently, reset all velocities to 0 (auto-stop)
            if now - self.last_keypress_time > self.key_timeout:
                self.base_linear_x = 0.0
                self.base_linear_y = 0.0
                self.base_angular_z = 0.0
                for joint in self.joint_vels:
                    self.joint_vels[joint] = 0.0

            # Publish base movement (Twist)
            twist = Twist()
            twist.linear.x = self.base_linear_x
            twist.linear.y = self.base_linear_y
            twist.angular.z = self.base_angular_z
            self.cmd_vel_pub.publish(twist)

            # Publish joint movement (JointJog)
            jog = JointJog()
            jog.header.stamp = self.get_clock().now().to_msg()
            jog.duration = 0.1  # Matches the publishing period

            # Command active/inactive joints
            # To ensure INSTANT stops, we always publish explicit target velocities (even 0.0)
            # for all known joints instead of sending empty lists.
            for joint_name, vel in self.joint_vels.items():
                target_joint = joint_name
                if joint_name in ['gripper_joint', 'parallel_gripper_joint']:
                    target_joint = self.gripper_joint_type
                
                jog.joint_names.append(target_joint)
                jog.velocities.append(vel)

            self.joint_vel_pub.publish(jog)

    def process_key(self, key: str):
        """
        Maps a keyboard character to target robot joint/base velocities.
        """
        with self.cmd_lock:
            self.last_keypress_time = self.get_clock().now().nanoseconds / 1e9

            # Base controls (WASD + QE)
            if key == 'w':
                self.base_linear_x = 0.10
            elif key == 's':
                self.base_linear_x = -0.10
            elif key == 'a':
                self.base_linear_y = 0.10
            elif key == 'd':
                self.base_linear_y = -0.10
            elif key == 'q':
                self.base_angular_z = 0.35
            elif key == 'e':
                self.base_angular_z = -0.35

            # Lift controls (RF)
            elif key == 'r':
                self.joint_vels['lift_joint'] = 0.05
            elif key == 'f':
                self.joint_vels['lift_joint'] = -0.05

            # Arm controls (TG)
            elif key == 't':
                self.joint_vels['arm_joint'] = 0.05
            elif key == 'g':
                self.joint_vels['arm_joint'] = -0.05

            # Wrist Yaw controls (YH)
            elif key == 'y':
                self.joint_vels['wrist_yaw_joint'] = 0.20
            elif key == 'h':
                self.joint_vels['wrist_yaw_joint'] = -0.20

            # Wrist Pitch controls (UJ)
            elif key == 'u':
                self.joint_vels['wrist_pitch_joint'] = 0.20
            elif key == 'j':
                self.joint_vels['wrist_pitch_joint'] = -0.20

            # Wrist Roll controls (IK)
            elif key == 'i':
                self.joint_vels['wrist_roll_joint'] = 0.25
            elif key == 'k':
                self.joint_vels['wrist_roll_joint'] = -0.25

            # Gripper controls (OL)
            elif key == 'o':
                self.joint_vels['gripper_joint'] = 0.15
                self.joint_vels['parallel_gripper_joint'] = 0.02
            elif key == 'l':
                self.joint_vels['gripper_joint'] = -0.15
                self.joint_vels['parallel_gripper_joint'] = -0.02

            # Utilities
            elif key == ' ':
                # Emergency Stop (Instantaneous stop command)
                self.base_linear_x = 0.0
                self.base_linear_y = 0.0
                self.base_angular_z = 0.0
                for joint in self.joint_vels:
                    self.joint_vels[joint] = 0.0

    def print_status(self):
        """
        Prints the current positions of the joints and base coordinates dynamically on the terminal
        without flickering (moves cursor to home and overwrites each line).
        """
        with self.state_lock:
            # Extract joint positions safely with defaults
            lift_pos = self.joint_positions.get('lift_joint', 0.0)
            arm_pos = self.joint_positions.get('arm_joint', 0.0)
            wyaw_pos = self.joint_positions.get('wrist_yaw_joint', 0.0)
            wpitch_pos = self.joint_positions.get('wrist_pitch_joint', 0.0)
            wroll_pos = self.joint_positions.get('wrist_roll_joint', 0.0)
            g_pos = self.joint_positions.get(self.gripper_joint_type, 0.0)

            # Move cursor to home (top left)
            sys.stdout.write("\033[H")

            # Format print lines with clear-to-end-of-line escape code (\033[K) to avoid trails
            lines = [
                "Stretch 4 ROS2 Keyboard Teleop CLI Tool",
                "=======================================",
                "Control all joints of Stretch 4 using your keyboard.",
                "",
                "Movement Controls:",
                "------------------",
                "Base Movement (WASD):",
                "  [w] / [s] : Translate Forward / Backward",
                "  [a] / [d] : Strafe Left / Right",
                "  [q] / [e] : Rotate Left / Right (CCW / CW)",
                "",
                "Joint Movement (Positive / Negative):",
                "  [r] / [f] : Lift Up / Down",
                "  [t] / [g] : Arm Extend / Retract",
                "  [y] / [h] : Wrist Yaw Left / Right (CCW / CW)",
                "  [u] / [j] : Wrist Pitch Up / Down",
                "  [i] / [k] : Wrist Roll Left / Right (CCW / CW)",
                "  [o] / [l] : Gripper Open / Close",
                "",
                "Utility Controls:",
                "  [space]   : Emergency Stop (zero all velocities instantly)",
                "  [v]       : Set driver mode to 'velocity' (automatic on startup)",
                "  [p]       : Enter 'position' mode (prompts for coordinate movement)",
                "  [h]       : Print this help menu again",
                "  [esc] / [enter] : Exit the tool safely",
                "",
                "* Note: Actuators stop automatically and instantly when keys are released.",
                "",
                "Current Robot Status & Positions:",
                "=================================",
                f"  Omnibase X      (wheel.x)     : {self.base_x:6.3f} m",
                f"  Omnibase Y      (wheel.y)     : {self.base_y:6.3f} m",
                f"  Omnibase Theta  (wheel.theta) : {self.base_theta:6.3f} rad ({math.degrees(self.base_theta):5.1f} deg)",
                f"  Lift Position   (lift_joint)  : {lift_pos:6.3f} m",
                f"  Arm Extension   (arm_joint)   : {arm_pos:6.3f} m",
                f"  Wrist Yaw       (wrist_yaw)   : {wyaw_pos:6.3f} rad ({math.degrees(wyaw_pos):5.1f} deg)",
                f"  Wrist Pitch     (wrist_pitch) : {wpitch_pos:6.3f} rad ({math.degrees(wpitch_pos):5.1f} deg)",
                f"  Wrist Roll      (wrist_roll)  : {wroll_pos:6.3f} rad ({math.degrees(wroll_pos):5.1f} deg)",
                f"  Gripper         ({self.gripper_joint_type.split('_joint')[0]})      : {g_pos:6.3f}",
                "=================================",
                f"Last command mode: {self.driver_mode.upper()}  (Press keys to command robot, ctrl+c / enter to exit)"
            ]

            for line in lines:
                sys.stdout.write(line + "\033[K\n")
            sys.stdout.flush()


def run_interactive_position_mode(node: KeyboardTeleop):
    """
    Suspends raw keyboard input and prompts the user to move a selected joint to a target position.
    """
    node.set_driver_mode("position")
    
    while rclpy.ok():
        sys.stdout.write("\033[2J\033[H")  # Clear terminal
        print("==================================================")
        print("Interactive Position Mode Control")
        print("==================================================")
        print("Enter a target joint and position to move the robot.")
        print("")
        
        with node.state_lock:
            lift_pos = node.joint_positions.get('lift_joint', 0.0)
            arm_pos = node.joint_positions.get('arm_joint', 0.0)
            wyaw_pos = node.joint_positions.get('wrist_yaw_joint', 0.0)
            wpitch_pos = node.joint_positions.get('wrist_pitch_joint', 0.0)
            wroll_pos = node.joint_positions.get('wrist_roll_joint', 0.0)
            g_pos = node.joint_positions.get(node.gripper_joint_type, 0.0)

        print("Available Joints:")
        print(f"  1. lift_joint       (current: {lift_pos:6.3f} m, range: [0.0, 1.1])")
        print(f"  2. arm_joint        (current: {arm_pos:6.3f} m, range: [0.0, 0.51])")
        print(f"  3. wrist_yaw_joint  (current: {wyaw_pos:6.3f} rad, range: [-1.75, 4.0])")
        print(f"  4. wrist_pitch_joint(current: {wpitch_pos:6.3f} rad, range: [-1.57, 0.56])")
        print(f"  5. wrist_roll_joint (current: {wroll_pos:6.3f} rad, range: [-2.61, 2.61])")
        print(f"  6. gripper          (current: {g_pos:6.3f})")
        print("==================================================")
        print("  Type 'v' to return to Keyboard Velocity Jogging")
        print("  Type 'q' or 'exit' to quit the tool")
        print("==================================================")
        
        try:
            choice = input("\nSelect a joint (1-6, name) or command: ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            break

        if choice in ['q', 'exit']:
            sys.exit(0)
        if choice == 'v':
            node.set_driver_mode("velocity")
            break

        # Map choice to joint name
        joint_map = {
            '1': 'lift_joint',
            '2': 'arm_joint',
            '3': 'wrist_yaw_joint',
            '4': 'wrist_pitch_joint',
            '5': 'wrist_roll_joint',
            '6': 'gripper'
        }
        
        target_joint = joint_map.get(choice, choice)
        if target_joint not in ['lift_joint', 'arm_joint', 'wrist_yaw_joint', 'wrist_pitch_joint', 'wrist_roll_joint', 'gripper']:
            print("\nInvalid selection. Press Enter to try again.")
            input()
            continue

        try:
            pos_str = input(f"Enter target position for {target_joint}: ").strip()
            if pos_str.lower() in ['q', 'exit']:
                continue
            target_pos = float(pos_str)
        except ValueError:
            print("\nInvalid position value. Must be a float. Press Enter to try again.")
            input()
            continue

        # Execute position movement
        node.send_position_goal(target_joint, target_pos)
        print("\nPress Enter to continue...")
        input()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    # Start ROS2 spinning in a separate thread so callbacks can update states asynchronously
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Automatically set driver mode to velocity
    node.set_driver_mode("velocity")

    try:
        while rclpy.ok():
            # Velocity Jogging Mode
            with KBHit() as kb:
                in_position_mode = False
                while rclpy.ok() and not in_position_mode:
                    node.print_status()

                    # Poll for key press
                    if kb.kbhit():
                        c = kb.getch()
                        
                        # Exit conditions
                        if c in ['\n', '\r', '\x1b']:  # Enter or Esc
                            raise KeyboardInterrupt
                        
                        # Manual Mode switching
                        if c == 'v':
                            node.set_driver_mode("velocity")
                        elif c == 'p':
                            in_position_mode = True
                        elif c == 'h':
                            # Print help menu
                            sys.stdout.write("\033[2J\033[H")
                            print(HELP_TEXT)
                            time.sleep(2.0)
                        else:
                            node.process_key(c)

                    time.sleep(0.05)

            # If the user requested position mode, we run the prompt outside raw terminal mode
            if in_position_mode:
                run_interactive_position_mode(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down Keyboard Teleop...")
        # Reset driver mode back to navigation/position and ensure robot stops
        node.set_driver_mode("navigation")
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        print("\nKeyboard Teleop closed safely.")

if __name__ == '__main__':
    main()
