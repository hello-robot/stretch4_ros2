# Import the Robot Client Library (RCL) for Python
import click
import rclpy
import numpy as np

from rclpy.node import Node
from typing import List, Optional

# Import the different ROS interface types that we will need
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from stretch_core.keyboard import KBHit

# Constants for max velocities and joint limits
MAX_VX = 0.12 #m/s
MAX_VY = 0.12 #m/s
MAX_VRZ = 0.50 #rad/s

EXTENSION_LIMITS = (0.00, 0.60) #meters
LIFT_LIMITS = (0.10, 1.21) #meters

APERTURE_CLOSED = 55.0 # degrees
APERTURE_OPEN = 0.0 #degrees

class Stretch4Teleop(Node):

    def __init__(self):
        super().__init__('stretch4_teleop')

        self.keyboard = KBHit()
        self.keyactive = False

        # Create a new message of type Twist to store base movement commands
        self.twist = Twist()

        self.arm_extended = False
        self.arm_raised = False
        self.gripper_closed = False

        # Create a new message of type Float64MultiArray to store joint state commands
        self.joint_states = Float64MultiArray()
        self.joint_states.data = [0.0] * 8 # Stow position
        self.update_robot_commands([0,0,0],[0,0,0,0])

        # Create the two publishers with their message type and topic name
        self.twist_publisher = self.create_publisher(Twist, '/stretch/cmd_vel', 0)
        self.joint_publisher = self.create_publisher(Float64MultiArray, '/joint_pose_cmd', 0)

        # Create the subscription using the message type, topic name, 
        # and the function to call when there's a new message.
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 0)

        # Code to automatically call the service which activates streaming
        self.active_srv = self.create_client(Trigger, '/activate_streaming_position')
        while not self.active_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service not available, waiting again...')
        self.active_srv.call_async(Trigger.Request())
        self.get_logger().info('Ready')

    def get_keypress(self):
        if self.keyboard.kbhit(): 
            key = self.keyboard.getch()
            self.keypress_callback(key)
            self.keyactive = True
        elif self.keyactive:
            self.keypress_callback(None)
            self.keyactive = False
        else:
            pass


    def joy_callback(self, msg: Joy):
        '''This is the code that is called each time there is a new Joy message'''

        # Logging the key message components to the terminal. Feel free to remove.
        self.get_logger().info(f'Axes: {list(msg.axes)}')
        self.get_logger().info(f'Buttons: {list(msg.buttons)}')

        self.update_robot_commands(list(msg.axes),list(msg.buttons))

    def keypress_callback(self, key: Optional[str]): 
        
        if key is None: 
            axes = [0, 0, 0]
            buttons = [0, 0, 0, 0]
        else:
            x_cmd = int(key=='a') - int(key=='d')
            y_cmd = int(key=='w') - int(key=='s')
            rz_cmd = int(key=='A') - int(key=='D')

            button1 = int(key=='1')
            button2 = int(key=='2')
            button3 = int(key=='3')
            button4 = int(key=='4')

            axes = [x_cmd, y_cmd, rz_cmd]
            buttons = [button1, button2, button3, button4]

        self.update_robot_commands(axes,buttons)

    def update_robot_commands(self, axes: List, buttons: List):


        #TODO: Fill in logic for what should happen with each button press

        if buttons[0]:
            self.arm_raised = not self.arm_raised
        if buttons[1]:
            self.arm_extended = not self.arm_extended
        if buttons[2]: 
            self.gripper_closed = not self.gripper_closed
        if buttons[3]:
            #reset to stow position
            self.arn_raise = False
            self.arm_extended = False
            self.gripper_closed = False 

        # A twist message will send velocity commands to the robot base
        self.twist.linear.x = MAX_VX * axes[0]
        self.twist.linear.y = MAX_VY * axes[1]
        self.twist.angular.z = MAX_VRZ * axes[2]

        # A joint state message will send robot pose commands to the robot
        self.joint_states.data[0] = EXTENSION_LIMITS[int(self.arm_extended)]
        self.joint_states.data[1] = LIFT_LIMITS[int(self.arm_raised)]
        self.joint_states.data[-1] = APERTURE_CLOSED if self.gripper_closed else APERTURE_OPEN


    def update_robot_state(self): 
        self.twist_publisher.publish(self.twist)
        self.joint_publisher.publish(self.joint_states)


def main(args=None):

    # Initialize the node
    rclpy.init(args=args)
    node = Stretch4Teleop()
    node.create_rate(5)

    click.secho("\n       Keyboard Controls:", fg="yellow")
    click.secho("=====================================", fg="yellow")
    print("a / w / s / d: translate BASE")
    print("A / D: Rotate BASE")
    print("1: Raise/Lower LIFT")
    print("2: Extend/Retract ARM")
    print("3: Open/Close GRIPPER")
    print("4: Reset")
    click.secho("=====================================", fg="yellow")

    # Start a loop that will run until the node is killed or ROS shutsdown
    while rclpy.ok():
        
        # Check for keypresses
        node.get_keypress()

        # Trigger the function that will publish the latest robot commands 
        node.update_robot_state()

        # Sleep for the period specified by the set node rate. 
        # During this time, the subscribers can recieve messages and trigger their callback functions.  
        rclpy.spin_once(node)

if __name__ == '__main__':
    main()