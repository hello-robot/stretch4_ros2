#!/usr/bin/env python3

"""
Task Space Controller node for Stretch 4.

Subscribes to desired task-space end-effector velocity commands (geometry_msgs/Twist on /ee_cmd_vel),
joint states, and odometry, solves differential IK using stretch4_kinematics,
and publishes joint velocity commands (control_msgs/JointJog) and base twist commands (geometry_msgs/Twist).
"""

import time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from control_msgs.msg import JointJog

from stretch4_kinematics.state.joint_positions import StretchJointPositions
from stretch4_kinematics.kinematic_models.tool_frame_kinematics import ToolFrameKinematics


class TaskSpaceController(Node):
    """
    ROS 2 node that converts 6D task-space end-effector velocity commands into joint velocities.
    """

    def __init__(self) -> None:
        super().__init__('task_space_controller')

        # Parameters
        self.declare_parameter('target_frame', 'tool_attachment_site_link')
        self.declare_parameter('control_rate', 15.0)  # Control loop frequency in Hz
        self.declare_parameter('watchdog_timeout', 0.4)  # Safety timeout in seconds

        self.target_frame = self.get_parameter('target_frame').value
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.watchdog_timeout = float(self.get_parameter('watchdog_timeout').value)

        # Kinematic model and state
        self.kinematic_model = ToolFrameKinematics()
        self.stretch_joint_position = StretchJointPositions()
        self.latest_v_desired = np.zeros(6)
        self.last_cmd_time = 0.0

        # Publishers
        self.pub_joint_vel = self.create_publisher(JointJog, 'joint_vel', 10)
        self.pub_base_twist = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        self.sub_ee_cmd_vel = self.create_subscription(
            Twist, 'ee_cmd_vel', self.ee_cmd_vel_callback, 10
        )
        self.sub_joint_state = self.create_subscription(
            JointState, 'joint_states', self.joint_states_callback, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, 'wheel_odom', self.odom_callback, 10
        )

        # Timer loop for smooth differential IK evaluation
        dt = 1.0 / self.control_rate
        self.timer = self.create_timer(dt, self.control_loop)

        self.get_logger().info(
            f"Task Space Controller node initialized. Target frame: '{self.target_frame}', "
            f"Rate: {self.control_rate} Hz, Watchdog: {self.watchdog_timeout} s."
        )

    def ee_cmd_vel_callback(self, msg: Twist) -> None:
        """Receive desired 6D task-space velocity command."""
        self.latest_v_desired = np.array([
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        ], dtype=float)
        self.last_cmd_time = time.time()

    def odom_callback(self, msg: Odometry) -> None:
        """Update mobile base heading and position from wheel odometry."""
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.stretch_joint_position.base_theta = np.arctan2(siny_cosp, cosy_cosp)
        self.stretch_joint_position.base_x = msg.pose.pose.position.x
        self.stretch_joint_position.base_y = msg.pose.pose.position.y

    def joint_states_callback(self, joint_state: JointState) -> None:
        """Update joint positions from joint state feedback."""
        joint_dict = dict(zip(joint_state.name, joint_state.position))

        self.stretch_joint_position.lift = joint_dict.get('lift_joint', 0.5)

        if 'arm_l4_joint' in joint_dict:
            self.stretch_joint_position.arm = joint_dict['arm_l4_joint'] * 5.0
        elif 'arm_joint' in joint_dict:
            self.stretch_joint_position.arm = joint_dict['arm_joint']
        else:
            self.stretch_joint_position.arm = 0.0

        self.stretch_joint_position.wrist_yaw = joint_dict.get('wrist_yaw_joint', 0.0)
        self.stretch_joint_position.wrist_pitch = joint_dict.get('wrist_pitch_joint', 0.0)
        self.stretch_joint_position.wrist_roll = joint_dict.get('wrist_roll_joint', 0.0)

    def control_loop(self) -> None:
        """Evaluate differential IK and publish velocity commands."""
        now = time.time()
        dt_step = 1.0 / self.control_rate

        # Enforce watchdog timeout
        if (now - self.last_cmd_time) > self.watchdog_timeout:
            v_task = np.zeros(6)
        else:
            v_task = self.latest_v_desired

        # Compute differential IK
        q_dot = self.kinematic_model.differential_ik(
            q=self.stretch_joint_position,
            target_frame=self.target_frame,
            v_desired=v_task
        )

        # Prepare JointJog message
        joint_jog = JointJog()
        joint_jog.joint_names = [
            'lift_joint',
            'arm_joint',
            'wrist_yaw_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint',
        ]
        joint_jog.velocities = [
            q_dot.lift,
            q_dot.arm,
            q_dot.wrist_yaw,
            q_dot.wrist_pitch,
            q_dot.wrist_roll,
        ]
        joint_jog.duration = dt_step

        # Prepare base Twist message
        base_twist = Twist()
        base_twist.linear.x = q_dot.base_x
        base_twist.linear.y = q_dot.base_y
        base_twist.angular.z = q_dot.base_theta

        # Publish commands
        self.pub_joint_vel.publish(joint_jog)
        self.pub_base_twist.publish(base_twist)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TaskSpaceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
