#! /usr/bin/env python3
"""Bridge Stretch /joy → cmd_vel_nav (base) + joint_vel (arm). """

import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

import hello_helpers.joy_conversion as jc
from stretch4_body.core.gamepad_control_mappings import ControlMapping
from stretch4_body.core.gamepad_teleop import GamePadTeleop

from stretch_core.gamepad_cmd_mapping import gamepad_state_to_twist


# To undo the rescaling in stretch_driver.velocity_callback
GRIPPER_VELOCITY_SCALE = 300.0


class _JointRecorder:
    """Stands in for the robot in ControlMapping: records commands."""

    def __init__(self):
        self.base = self._Base(self)
        self.lift = self._Ranged(self, 'lift_joint')
        self.arm = self._Ranged(self, 'arm_joint')
        self.end_of_arm = self._EndOfArm(self)
        self.commands = {}

    subsystems = ('arm', 'lift', 'end_of_arm', 'omnibase')

    def set_guarded_contact_sensitivity(self, _name):
        """No-op: contact sensitivity lives on the driver's robot, not here."""

    def clear(self):
        self.commands = {}

    class _Base:
        def __init__(self, outer):
            self._outer = outer

        def set_velocity(self, *_args, **_kwargs):
            """Base goes out as a Twist via cmd_vel_nav so collision_monitor can gate it."""

    class _Ranged:
        def __init__(self, outer, joint_name):
            self._outer = outer
            self._joint_name = joint_name

        def set_velocity(self, v_m, a_m=None):
            self._outer.commands[self._joint_name] = float(v_m)

    class _EndOfArm:
        def __init__(self, outer):
            self._outer = outer

        def move_by(self, name, dx, *_args, **_kwargs):
            self._outer.commands[f'{name}_joint'] = float(dx)

        def quick_stop(self, name):
            self._outer.commands[f'{name}_joint'] = 0.0


class JoyToCmdVelNav(Node):
    """Subscribes to Joy; publishes base Twist and arm JointJog. """

    def __init__(self):
        super().__init__('joy_to_cmd_vel_nav')

        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_nav')
        self.declare_parameter('joint_vel_topic', 'joint_vel')
        self.declare_parameter('command_duration', 0.1)

        joy_topic = self.get_parameter('joy_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        joint_vel_topic = self.get_parameter('joint_vel_topic').value
        self.command_duration = float(self.get_parameter('command_duration').value)

        self.recorder = _JointRecorder()
        self.gamepad_teleop = GamePadTeleop(robot=self.recorder, use_server=False)
        # Base must go through cmd_vel_nav → collision_monitor, not the mapping.
        self.gamepad_teleop.use_devices['base'] = False

        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.joint_vel_publisher = self.create_publisher(JointJog, joint_vel_topic, 10)
        self.create_subscription(Joy, joy_topic, self.joy_callback, 10)

        self.get_logger().info(
            f'{joy_topic} → {cmd_vel_topic} (base Twist) + {joint_vel_topic} (arm JointJog); '
            f'arm mapped by ControlMapping.JOINT_SPACE. stretch_driver must be in velocity mode.'
        )

    def joy_callback(self, joy_msg: Joy):
        state = jc.unpack_joy_to_gamepad_state(joy_msg)
        self.cmd_vel_publisher.publish(gamepad_state_to_twist(state))

        self.gamepad_teleop.controller_state = state
        self.gamepad_teleop.precision_mode = float(state.get('left_trigger_pulled', 0.0))
        self.gamepad_teleop._update_modes()

        self.recorder.clear()
        ControlMapping.JOINT_SPACE.do_motion(self.recorder, self.gamepad_teleop)
        self.publish_joint_vel(self.recorder.commands)

    def publish_joint_vel(self, commands: dict):
        if not commands:
            return
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.duration = self.command_duration
        for joint, value in commands.items():
            msg.joint_names.append(joint)
            msg.velocities.append(self.to_jointjog_velocity(joint, value))
        self.joint_vel_publisher.publish(msg)

    def to_jointjog_velocity(self, joint: str, value: float) -> float:
        """Pre-divide so stretch_driver's own rescaling reproduces the mapping's command."""
        if value == 0.0:
            return 0.0
        if 'gripper' in joint:
            return value / GRIPPER_VELOCITY_SCALE
        if 'wrist' in joint:
            return value / self.command_duration
        return value

    def publish_stop(self):
        self.cmd_vel_publisher.publish(Twist())

    def destroy_node(self) -> bool:
        try:
            self.publish_stop()
        except Exception:
            pass
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
