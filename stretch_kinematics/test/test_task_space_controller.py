"""Unit and functionality tests for TaskSpaceController node."""

import unittest
from unittest.mock import patch

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy

from stretch_kinematics.nodes.task_space_controller import TaskSpaceController


class TestTaskSpaceController(unittest.TestCase):
    """Test suite for TaskSpaceController differential IK and watchdog logic."""

    @classmethod
    def setUpClass(cls) -> None:
        """Initialize ROS 2 context once for testing node instances."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        """Shutdown ROS 2 context after tests complete."""
        rclpy.shutdown()

    def setUp(self) -> None:
        """Instantiate task-space controller node before each test."""
        self.node = TaskSpaceController()

    def tearDown(self) -> None:
        """Destroy task-space controller node after each test."""
        self.node.destroy_node()

    def test_ee_cmd_vel_callback_publishes_proportional_output(self) -> None:
        """
        Verify incoming non-zero twist immediately solves IK and publishes commands.
        """
        msg = Twist()
        msg.linear.x = 0.1
        msg.linear.y = 0.05
        msg.linear.z = -0.02
        msg.angular.z = 0.2

        with patch.object(self.node.pub_base_twist, 'publish') as base_pub, \
                patch.object(self.node.pub_joint_vel, 'publish') as joint_pub:
            self.node.ee_cmd_vel_callback(msg)

        self.assertEqual(base_pub.call_count, 1)
        self.assertEqual(joint_pub.call_count, 1)

        published_base = base_pub.call_args[0][0]
        published_joint = joint_pub.call_args[0][0]

        expected_duration = 1.0 / self.node.control_rate
        self.assertAlmostEqual(published_joint.duration, expected_duration, places=5)
        self.assertGreater(
            abs(published_base.linear.x) + abs(published_base.linear.y) + abs(published_base.angular.z)
            + sum(abs(v) for v in published_joint.velocities),
            0.0
        )

    def test_ee_cmd_vel_callback_zero_twist_publishes_zeros(self) -> None:
        """
        Verify incoming zero twist publishes zeros without nullspace velocities.
        """
        msg = Twist()

        with patch.object(self.node.pub_base_twist, 'publish') as base_pub, \
                patch.object(self.node.pub_joint_vel, 'publish') as joint_pub:
            self.node.ee_cmd_vel_callback(msg)

        self.assertEqual(base_pub.call_count, 1)
        self.assertEqual(joint_pub.call_count, 1)

        published_base = base_pub.call_args[0][0]
        published_joint = joint_pub.call_args[0][0]

        expected_duration = 1.0 / self.node.control_rate
        self.assertAlmostEqual(published_joint.duration, expected_duration, places=5)
        self.assertAlmostEqual(published_base.linear.x, 0.0)
        self.assertAlmostEqual(published_base.linear.y, 0.0)
        self.assertAlmostEqual(published_base.angular.z, 0.0)
        np.testing.assert_allclose(published_joint.velocities, np.zeros(5), atol=1e-9)

    def test_odom_callback(self) -> None:
        """
        Verify odometry callback updates base positions and heading in state.
        """
        msg = Odometry()
        msg.pose.pose.position.x = 1.2
        msg.pose.pose.position.y = -0.5
        # Quaternion for theta = np.pi / 2 (z = 0.7071, w = 0.7071)
        msg.pose.pose.orientation.z = 0.70710678
        msg.pose.pose.orientation.w = 0.70710678

        self.node.odom_callback(msg)

        self.assertAlmostEqual(self.node.stretch_joint_position.base_x, 1.2, places=4)
        self.assertAlmostEqual(self.node.stretch_joint_position.base_y, -0.5, places=4)
        self.assertAlmostEqual(
            self.node.stretch_joint_position.base_theta, np.pi / 2.0, places=4
        )


if __name__ == '__main__':
    unittest.main()
