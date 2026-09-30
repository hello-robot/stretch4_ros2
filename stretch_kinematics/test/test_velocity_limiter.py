"""Unit and functionality tests for EndEffectoryVelocitySafetyFilterNode."""

import unittest

import rclpy
from sensor_msgs.msg import JointState

from stretch4_kinematics.state import StretchJointVelocities
from stretch_kinematics.nodes.velocity_limiter import EndEffectoryVelocitySafetyFilterNode


class TestVelocityLimiter(unittest.TestCase):
    """Test suite for velocity limiter safety scaling logic."""

    @classmethod
    def setUpClass(cls) -> None:
        """Initialize ROS 2 context once for testing node instances."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        """Shutdown ROS 2 context after tests complete."""
        rclpy.shutdown()

    def setUp(self) -> None:
        """Instantiate velocity limiter node before each test."""
        self.node = EndEffectoryVelocitySafetyFilterNode()

    def tearDown(self) -> None:
        """Destroy velocity limiter node after each test."""
        self.node.destroy_node()

    def test_zero_velocity_gain(self) -> None:
        """Verify that zero velocity input yields a gain of 1.0 without error."""
        q_dot = StretchJointVelocities(
            base_x=0.0,
            base_y=0.0,
            base_theta=0.0,
            lift=0.0,
            arm=0.0,
            wrist_yaw=0.0,
            wrist_pitch=0.0,
            wrist_roll=0.0,
        )
        ee_speed, gain = self.node.compute_ee_speed_and_gain(q_dot)
        self.assertAlmostEqual(ee_speed, 0.0, places=5)
        self.assertAlmostEqual(gain, 1.0, places=5)

    def test_speed_below_max_limit_gain(self) -> None:
        """Verify gain is 1.0 when velocity is below the max_ee_speed threshold."""
        # Slow base translation along X (0.05 m/s < 0.20 m/s default limit)
        q_dot = StretchJointVelocities(
            base_x=0.05,
            base_y=0.0,
            base_theta=0.0,
            lift=0.0,
            arm=0.0,
            wrist_yaw=0.0,
            wrist_pitch=0.0,
            wrist_roll=0.0,
        )
        ee_speed, gain = self.node.compute_ee_speed_and_gain(q_dot)
        self.assertLessEqual(ee_speed, self.node.max_ee_speed)
        self.assertAlmostEqual(gain, 1.0, places=5)

    def test_overspeed_scaling_gain(self) -> None:
        """Verify velocity scaling gain reduces overspeed commands down to max_ee_speed."""
        # Fast base translation along X (0.80 m/s > 0.20 m/s limit)
        q_dot = StretchJointVelocities(
            base_x=0.80,
            base_y=0.0,
            base_theta=0.0,
            lift=0.0,
            arm=0.0,
            wrist_yaw=0.0,
            wrist_pitch=0.0,
            wrist_roll=0.0,
        )
        ee_speed, gain = self.node.compute_ee_speed_and_gain(q_dot)
        self.assertGreater(ee_speed, self.node.max_ee_speed)
        expected_gain = self.node.max_ee_speed / ee_speed
        self.assertAlmostEqual(gain, expected_gain, places=5)
        self.assertAlmostEqual(ee_speed * gain, self.node.max_ee_speed, places=5)

    def test_joint_state_parsing(self) -> None:
        """Verify JointState message parsing updates internal joint positions correctly."""
        msg = JointState()
        msg.name = [
            'lift_joint',
            'arm_l4_joint',
            'wrist_yaw_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint',
        ]
        msg.position = [0.6, 0.1, 0.2, -0.1, 0.05]

        self.node.joint_states_callback(msg)

        self.assertTrue(self.node.has_received_joint_states)
        self.assertAlmostEqual(self.node.latest_q.lift, 0.6, places=5)
        self.assertAlmostEqual(self.node.latest_q.arm, 0.4, places=5)  # 0.1 * 4.0
        self.assertAlmostEqual(self.node.latest_q.wrist_yaw, 0.2, places=5)
        self.assertAlmostEqual(self.node.latest_q.wrist_pitch, -0.1, places=5)
        self.assertAlmostEqual(self.node.latest_q.wrist_roll, 0.05, places=5)

    def test_full_body_joint_jog_splits_twist_and_scales_uniformly(self) -> None:
        """
        Verify that a full-body JointJog containing base coordinates and manipulator joints
        is evaluated with a single uniform scalar gain, dispatches a scaled Twist to
        pub_cmd_vel, and dispatches only hardware joints to pub_joint_vel.
        """
        from unittest.mock import patch
        from control_msgs.msg import JointJog
        from geometry_msgs.msg import Twist

        # Set max_ee_speed low (0.05 m/s) to ensure scaling is triggered
        self.node.max_ee_speed = 0.05

        jog_msg = JointJog()
        jog_msg.joint_names = [
            'base_x',
            'base_y',
            'base_theta',
            'lift_joint',
            'arm_joint',
            'wrist_yaw_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint',
        ]
        # Fast command with base_theta = 0.5 and wrist_yaw = -0.5
        jog_msg.velocities = [0.2, 0.1, 0.5, 0.05, 0.05, -0.5, 0.0, 0.0]
        jog_msg.duration = 0.067

        with patch.object(self.node.pub_cmd_vel, 'publish') as cmd_vel_pub, \
                patch.object(self.node.pub_joint_vel, 'publish') as joint_vel_pub:
            self.node.joint_vel_callback(jog_msg)

        self.assertEqual(cmd_vel_pub.call_count, 1)
        self.assertEqual(joint_vel_pub.call_count, 1)

        published_twist: Twist = cmd_vel_pub.call_args[0][0]
        published_joint: JointJog = joint_vel_pub.call_args[0][0]

        # Verify base names are NOT in published JointJog
        self.assertNotIn('base_x', published_joint.joint_names)
        self.assertNotIn('base_y', published_joint.joint_names)
        self.assertNotIn('base_theta', published_joint.joint_names)
        self.assertIn('wrist_yaw_joint', published_joint.joint_names)

        # Retrieve scaled wrist_yaw and scaled base_theta
        wrist_yaw_idx = published_joint.joint_names.index('wrist_yaw_joint')
        scaled_wrist_yaw = published_joint.velocities[wrist_yaw_idx]
        scaled_base_theta = published_twist.angular.z

        # CRITICAL TEST: Direct yaw compensation proportionality must be strictly preserved!
        # Both base_theta and wrist_yaw must be scaled by the exact same gain,
        # so scaled_wrist_yaw == -scaled_base_theta!
        self.assertAlmostEqual(scaled_wrist_yaw, -scaled_base_theta, places=5)
        self.assertLess(abs(scaled_base_theta), 0.5)  # Must be scaled down


if __name__ == '__main__':
    unittest.main()
