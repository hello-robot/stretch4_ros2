#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState

from stretch4_kinematics.kinematic_models.tool_frame_kinematics import ToolFrameKinematics
from stretch4_kinematics.state import StretchJointPositions, StretchJointVelocities


class EndEffectoryVelocitySafetyFilterNode(Node):
    """
    ROS 2 Safety Filter Node that intercepts velocity commands (/teleop/cmd_vel, /teleop/cmd_vel_nav, /teleop/joint_vel),
    uses StretchKinematics.forward_velocity() to calculate resultant end-effector (EE) speed,
    and scales down velocity vectors that exceed max_ee_speed before publishing to driver/nav topics.
    """
    def __init__(self):
        super().__init__('ee_velocity_safety_filter')

        # Declare ROS Parameters
        self.declare_parameter('max_ee_speed', 0.20)  # Max absolute EE speed in m/s
        self.declare_parameter('target_frame', 'tool_attachment_site_link')
        self.declare_parameter('input_cmd_vel_topic', '/teleop/cmd_vel')
        self.declare_parameter('output_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('input_cmd_vel_nav_topic', '/teleop/cmd_vel_nav')
        self.declare_parameter('output_cmd_vel_nav_topic', '/cmd_vel_nav')
        self.declare_parameter('input_joint_vel_topic', '/teleop/joint_vel')
        self.declare_parameter('output_joint_vel_topic', '/joint_vel')
        self.declare_parameter('joint_states_topic', '/joint_states')

        self.max_ee_speed = float(self.get_parameter('max_ee_speed').value)
        self.target_frame = str(self.get_parameter('target_frame').value)

        input_cmd_vel_topic = str(self.get_parameter('input_cmd_vel_topic').value)
        output_cmd_vel_topic = str(self.get_parameter('output_cmd_vel_topic').value)
        input_cmd_vel_nav_topic = str(self.get_parameter('input_cmd_vel_nav_topic').value)
        output_cmd_vel_nav_topic = str(self.get_parameter('output_cmd_vel_nav_topic').value)
        input_joint_vel_topic = str(self.get_parameter('input_joint_vel_topic').value)
        output_joint_vel_topic = str(self.get_parameter('output_joint_vel_topic').value)
        joint_states_topic = str(self.get_parameter('joint_states_topic').value)

        # Instantiate kinematics model from stretch4_kinematics
        self.kinematics = ToolFrameKinematics()
        self.latest_q = StretchJointPositions()
        self.has_received_joint_states = False

        # Subscribers (use qos_profile_sensor_data for Best Effort ROS 2 JointState compatibility)
        self.sub_joint_states = self.create_subscription(
            JointState,
            joint_states_topic,
            self.joint_states_callback,
            qos_profile_sensor_data
        )
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            input_cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )
        self.sub_cmd_vel_nav = self.create_subscription(
            Twist,
            input_cmd_vel_nav_topic,
            self.cmd_vel_nav_callback,
            10
        )
        self.sub_joint_vel = self.create_subscription(
            JointJog,
            input_joint_vel_topic,
            self.joint_vel_callback,
            10
        )

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, output_cmd_vel_topic, 10)
        self.pub_cmd_vel_nav = self.create_publisher(Twist, output_cmd_vel_nav_topic, 10)
        self.pub_joint_vel = self.create_publisher(JointJog, output_joint_vel_topic, 10)

        self.get_logger().info(
            f"EeVelocitySafetyFilterNode initialized. "
            f"max_ee_speed={self.max_ee_speed:.3f} m/s, target_frame='{self.target_frame}', "
            f"joint_states_topic='{joint_states_topic}'"
        )

    def joint_states_callback(self, msg: JointState):
        """Updates internal joint position state from /joint_states."""
        if not self.has_received_joint_states:
            self.has_received_joint_states = True
            self.get_logger().info(
                f"Received first JointState on '{self.get_parameter('joint_states_topic').value}'"
            )

        name_pos = dict(zip(msg.name, msg.position))
        
        lift = name_pos.get('lift_joint', name_pos.get('joint_lift', 0.5))

        if 'arm_l4_joint' in name_pos:
            arm = name_pos['arm_l4_joint'] * 5.0
        elif 'arm_joint' in name_pos:
            arm = name_pos['arm_joint']
        elif 'joint_arm' in name_pos:
            arm = name_pos['joint_arm']
        else:
            arm = sum(name_pos.get(f'joint_arm_l{i}', name_pos.get(f'arm_l{i}', 0.0)) for i in range(4))

        wrist_yaw = name_pos.get('wrist_yaw_joint', name_pos.get('joint_wrist_yaw', 0.0))
        wrist_pitch = name_pos.get('wrist_pitch_joint', name_pos.get('joint_wrist_pitch', 0.0))
        wrist_roll = name_pos.get('wrist_roll_joint', name_pos.get('joint_wrist_roll', 0.0))

        self.latest_q = StretchJointPositions(
            base_x=0.0,
            base_y=0.0,
            base_theta=0.0,
            lift=lift,
            arm=arm,
            wrist_yaw=wrist_yaw,
            wrist_pitch=wrist_pitch,
            wrist_roll=wrist_roll
        )

    def compute_ee_speed_and_gain(self, q_dot_state: StretchJointVelocities) -> tuple[float, float]:
        """
        Computes the resultant 3D linear EE speed (m/s) using StretchKinematics.forward_velocity(),
        and returns (ee_speed, scale_gain).
        """
        target_frame = str(self.get_parameter('target_frame').value)
        max_ee_speed = float(self.get_parameter('max_ee_speed').value)
        joint_states_topic = str(self.get_parameter('joint_states_topic').value)

        if not self.has_received_joint_states:
            self.get_logger().warn(
                f"No JointState message received yet on '{joint_states_topic}'. "
                f"Computing forward_velocity with default state (lift={self.latest_q.lift}, arm={self.latest_q.arm}).",
                throttle_duration_sec=3.0
            )

        try:
            # Uses forward_velocity on StretchKinematics without importing Pinocchio directly in node
            v_spatial = self.kinematics.forward_velocity(self.latest_q, q_dot_state, target_frame)
            v_linear = v_spatial[:3]
            ee_speed = float(np.linalg.norm(v_linear))
        except Exception as e:
            self.get_logger().error(f"Error computing forward_velocity for frame '{target_frame}': {e}")
            return 0.0, 1.0

        if ee_speed > max_ee_speed and ee_speed > 1e-6:
            gain = max_ee_speed / ee_speed
        else:
            gain = 1.0

        return ee_speed, gain

    def _process_twist_cmd(self, msg: Twist, publisher, tag: str):
        """Helper to scale Twist velocity command and publish to the target publisher."""
        q_dot = StretchJointVelocities(
            base_x=msg.linear.x,
            base_y=msg.linear.y,
            base_theta=msg.angular.z,
            lift=0.0,
            arm=0.0,
            wrist_yaw=0.0,
            wrist_pitch=0.0,
            wrist_roll=0.0
        )

        ee_speed, gain = self.compute_ee_speed_and_gain(q_dot)

        scaled_msg = Twist()
        scaled_msg.linear.x = msg.linear.x * gain
        scaled_msg.linear.y = msg.linear.y * gain
        scaled_msg.linear.z = msg.linear.z * gain
        scaled_msg.angular.x = msg.angular.x * gain
        scaled_msg.angular.y = msg.angular.y * gain
        scaled_msg.angular.z = msg.angular.z * gain

        # if gain < 1.0:
        #     self.get_logger().warn(
        #         f"[{tag}] EE speed {ee_speed:.3f} m/s exceeds max {self.get_parameter('max_ee_speed').value:.3f} m/s "
        #         f"(arm={self.latest_q.arm:.2f}m). Scaling by {gain:.3f}"
        #     )

        publisher.publish(scaled_msg)

    def cmd_vel_callback(self, msg: Twist):
        """Callback for /teleop/cmd_vel. Scales and publishes to /cmd_vel."""
        self._process_twist_cmd(msg, self.pub_cmd_vel, 'cmd_vel')

    def cmd_vel_nav_callback(self, msg: Twist):
        """Callback for /teleop/cmd_vel_nav. Scales and publishes to /cmd_vel_nav."""
        self._process_twist_cmd(msg, self.pub_cmd_vel_nav, 'cmd_vel_nav')

    def joint_vel_callback(self, msg: JointJog):
        """Callback for /teleop/joint_vel. Scales and publishes to /joint_vel."""
        name_vel = dict(zip(msg.joint_names, msg.velocities))

        lift = name_vel.get('lift_joint', name_vel.get('joint_lift', 0.0))

        if 'arm_l4_joint' in name_vel:
            arm = name_vel['arm_l4_joint'] * 5.0
        elif 'arm_joint' in name_vel:
            arm = name_vel['arm_joint']
        elif 'joint_arm' in name_vel:
            arm = name_vel['joint_arm']
        elif any(f'joint_arm_l{i}' in name_vel or f'arm_l{i}' in name_vel for i in range(4)):
            arm = sum(name_vel.get(f'joint_arm_l{i}', name_vel.get(f'arm_l{i}', 0.0)) for i in range(4))
        else:
            arm = name_vel.get('arm', 0.0)

        wrist_yaw = name_vel.get('wrist_yaw_joint', name_vel.get('joint_wrist_yaw', name_vel.get('wrist_yaw', 0.0)))
        wrist_pitch = name_vel.get('wrist_pitch_joint', name_vel.get('joint_wrist_pitch', name_vel.get('wrist_pitch', 0.0)))
        wrist_roll = name_vel.get('wrist_roll_joint', name_vel.get('joint_wrist_roll', name_vel.get('wrist_roll', 0.0)))

        base_x = name_vel.get('translate_mobile_base', name_vel.get('base_x', 0.0))
        base_theta = name_vel.get('rotate_mobile_base', name_vel.get('base_theta', 0.0))

        q_dot = StretchJointVelocities(
            base_x=base_x,
            base_y=0.0,
            base_theta=base_theta,
            lift=lift,
            arm=arm,
            wrist_yaw=wrist_yaw,
            wrist_pitch=wrist_pitch,
            wrist_roll=wrist_roll
        )

        ee_speed, gain = self.compute_ee_speed_and_gain(q_dot)

        scaled_msg = JointJog()
        scaled_msg.header = msg.header
        scaled_msg.joint_names = msg.joint_names
        scaled_msg.velocities = [v * gain for v in msg.velocities]
        scaled_msg.duration = msg.duration

        # if gain < 1.0:
        #     self.get_logger().warn(
        #         f"[joint_vel] EE speed {ee_speed:.3f} m/s exceeds max {self.get_parameter('max_ee_speed').value:.3f} m/s "
        #         f"(arm={self.latest_q.arm:.2f}m). Scaling by {gain:.3f}"
        #     )

        self.pub_joint_vel.publish(scaled_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EndEffectoryVelocitySafetyFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
