#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import hello_helpers.joy_conversion as jc
from pprint import pprint
from stretch4_body.core.gamepad_controller import GamePadController
from stretch4_body.core.hello_utils import ThreadServiceExit

PRINT_DEBUG = False

class StretchRemoteGamepad(Node):
    """
    A ROS2 node that publishes a joy message from the stretch4_body gamepad input
    """
    def __init__(self):
        """
        Intialize a timer that will periodcally publish the joy message
        """
        super().__init__('stretch_remote_gamepad')
        print("Starting Stretch Remote Gamepad....")
        self.joy_publisher = self.create_publisher(Joy, 'joy', 10)  
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_message)
        self.gamepad_controller = GamePadController()
        self.gamepad_controller.startup()
    
    def gamepad_state_to_joy(self,gamepad_state):
        """
        Converts a gamepad state dictionary into a timestamped sensor_msgs.msg.Joy message.

        Args:
            gamepad_state (dict): The current state of the gamepad from the controller.

        Returns:
            sensor_msgs.msg.Joy: The populated joy message.
        """
        current_clock = self.get_clock().now()
        current_time = current_clock.to_msg()
        out_msg = jc.unpack_gamepad_state_to_joy(gamepad_state)
        out_msg.header.stamp = current_time
        if PRINT_DEBUG:
            print(f"------------ Gamepad State -------------")
            pprint(jc.unpack_joy_to_gamepad_state(out_msg))
        return out_msg
    
    def publish_message(self):
        """
        Timer callback that retrieves the current gamepad state and publishes it if active.
        """
        state = self.gamepad_controller.get_state()
        if state is not None:
            msg = self.gamepad_state_to_joy(state)
            self.joy_publisher.publish(msg)

def main():
    """
    Start the ROS2 node and handles graceful shutdown of the controller.
    """
    try:
        rclpy.init()
        node = StretchRemoteGamepad()
        try:
            rclpy.spin(node)
        finally:
            node.gamepad_controller.stop()
            node.destroy_node()
    except (KeyboardInterrupt, ThreadServiceExit):
        node.gamepad_controller.stop()
        node.destroy_node()

if __name__ == '__main__':
    main()