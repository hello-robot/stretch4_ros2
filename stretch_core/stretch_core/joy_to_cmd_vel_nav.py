#! /usr/bin/env python3
"""Bridge Stretch /joy → cmd_vel_nav (base) + ControlMapping arm motion.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

import hello_helpers.joy_conversion as jc
from stretch4_body.core.gamepad_control_mappings import ControlMapping
from stretch4_body.core.gamepad_teleop import GamePadTeleop
from stretch4_body.robot.robot_client import RobotClient

from stretch_core.gamepad_cmd_mapping import gamepad_state_to_twist


class JoyToCmdVelNav(Node):
    """Subscribes to Joy; publishes base Twist; drives arm via ControlMapping."""

    def __init__(self):
        super().__init__('joy_to_cmd_vel_nav')

        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_nav')

        joy_topic = self.get_parameter('joy_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.robot = RobotClient(client_id='joy_to_cmd_vel_nav')
        if not self.robot.startup():
            raise RuntimeError('RobotClient startup failed')

        self.gamepad_teleop = GamePadTeleop(robot=self.robot, use_server=True)
        # Base must go through cmd_vel_nav → collision_monitor, not do_motion.
        self.gamepad_teleop.use_devices['base'] = False

        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(Joy, joy_topic, self.joy_callback, 10)

        self.get_logger().info(
            f'{joy_topic} → {cmd_vel_topic} (Twist); arm via ControlMapping.JOINT_SPACE'
        )

    def joy_callback(self, joy_msg: Joy):
        state = jc.unpack_joy_to_gamepad_state(joy_msg)
        self.cmd_vel_publisher.publish(gamepad_state_to_twist(state))

        self.gamepad_teleop.controller_state = state
        self.gamepad_teleop.precision_mode = float(state.get('left_trigger_pulled', 0.0))
        self.gamepad_teleop._update_modes()
        ControlMapping.JOINT_SPACE.do_motion(self.robot, self.gamepad_teleop)
        self.robot.push_command(ignore_control_lock=True, priority=1)

    def publish_stop(self):
        self.cmd_vel_publisher.publish(Twist())

    def destroy_node(self) -> bool:
        try:
            self.publish_stop()
        except Exception:
            pass
        robot = getattr(self, 'robot', None)
        if robot is not None:
            try:
                robot.stop()
            except Exception as exc:
                self.get_logger().warn(f'RobotClient stop failed: {exc}')
        return super().destroy_node()


def main():
    node = None
    try:
        rclpy.init()
        node = JoyToCmdVelNav()
        try:
            rclpy.spin(node)
        finally:
            if node is not None:
                node.destroy_node()
    except KeyboardInterrupt:
        if node is not None:
            node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
