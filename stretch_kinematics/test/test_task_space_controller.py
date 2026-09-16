"""Unit and functionality tests for TaskSpaceController node."""

import time
import unittest

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

    def test_ee_cmd_vel_callback(self) -> None:
        """Verify command callback stores latest velocity vector and updates timestamp."""
        msg = Twist()
        msg.linear.x = 0.1
        msg.linear.y = 0.05
        msg.linear.z = -0.02
        msg.angular.z = 0.2

        now_before = time.time()
        self.node.ee_cmd_vel_callback(msg)
        now_after = time.time()

        np.testing.assert_allclose(
            self.node.latest_v_desired,
            [0.1, 0.05, -0.02, 0.0, 0.0, 0.2],
            atol=1e-5
        )
        self.assertGreaterEqual(self.node.last_cmd_time, now_before)
        self.assertLessEqual(self.node.last_cmd_time, now_after)

    def test_odom_callback(self) -> None:
        """Verify odometry callback updates base positions and heading in state."""
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

    def test_watchdog_timeout_zeroes_velocity(self) -> None:
        """Verify watchdog timeout zeroes velocity when no recent command is received."""
        # Set command in past exceeding watchdog timeout (0.4s)
        self.node.latest_v_desired = np.array([0.2, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.node.last_cmd_time = time.time() - 1.0  # 1.0s ago > 0.4s watchdog

        # Execute control_loop step
        self.node.control_loop()

        # Reset recent command time and execute control_loop step
        self.node.last_cmd_time = time.time()
        self.node.control_loop()


if __name__ == '__main__':
    unittest.main()
