"""Unit and functionality tests for TaskSpaceController node."""

import time
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

    def _run_loop_capturing_publishes(self) -> tuple[list[Twist], list]:
        """Tick control_loop once and return (base twists, joint jogs) published."""
        with patch.object(self.node.pub_base_twist, 'publish') as base_pub, \
                patch.object(self.node.pub_joint_vel, 'publish') as joint_pub:
            self.node.control_loop()
        base_msgs = [call.args[0] for call in base_pub.call_args_list]
        joint_msgs = [call.args[0] for call in joint_pub.call_args_list]
        return base_msgs, joint_msgs

    def test_startup_publishes_nothing(self) -> None:
        """A fresh node that has never received ee_cmd_vel must stay silent."""
        base_msgs, joint_msgs = self._run_loop_capturing_publishes()
        self.assertEqual(base_msgs, [])
        self.assertEqual(joint_msgs, [])
        self.assertFalse(self.node.is_commanding)

    def test_live_command_publishes(self) -> None:
        """A recent ee_cmd_vel keeps the node commanding on every tick."""
        self.node.latest_v_desired = np.array([0.2, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.node.last_cmd_time = time.time()

        base_msgs, joint_msgs = self._run_loop_capturing_publishes()
        self.assertEqual(len(base_msgs), 1)
        self.assertEqual(len(joint_msgs), 1)
        self.assertTrue(self.node.is_commanding)

    def test_watchdog_timeout_zeroes_velocity(self) -> None:
        """
        On timeout publish exactly one stop, then stay quiet.

        Streaming zeros onto cmd_vel would override other velocity sources
        (teleop Drive, collision monitor), so the stop must be one-shot.
        """
        # Start as if we had been flying.
        self.node.latest_v_desired = np.array([0.2, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.node.last_cmd_time = time.time()
        self._run_loop_capturing_publishes()
        self.assertTrue(self.node.is_commanding)

        # Command goes stale: expect one zero twist + zero joint jog.
        self.node.last_cmd_time = time.time() - 1.0  # 1.0s ago > 0.4s watchdog
        base_msgs, joint_msgs = self._run_loop_capturing_publishes()
        self.assertEqual(len(base_msgs), 1)
        self.assertEqual(len(joint_msgs), 1)
        stop_twist = base_msgs[0]
        self.assertAlmostEqual(stop_twist.linear.x, 0.0)
        self.assertAlmostEqual(stop_twist.linear.y, 0.0)
        self.assertAlmostEqual(stop_twist.angular.z, 0.0)
        np.testing.assert_allclose(joint_msgs[0].velocities, np.zeros(5), atol=1e-9)
        self.assertFalse(self.node.is_commanding)

        # Still stale: nothing more goes out.
        base_msgs, joint_msgs = self._run_loop_capturing_publishes()
        self.assertEqual(base_msgs, [])
        self.assertEqual(joint_msgs, [])

    def test_zero_twist_releases_immediately(self) -> None:
        """
        An all-zero ee_cmd_vel is a release, not a fresh command.

        Clients send one zero on button-up. It must expire the command at once
        so we publish a single stop on the next tick and then go quiet, instead
        of streaming zeros onto cmd_vel for the rest of the watchdog window.
        """
        live = Twist()
        live.linear.x = 0.2
        self.node.ee_cmd_vel_callback(live)
        self._run_loop_capturing_publishes()
        self.assertTrue(self.node.is_commanding)

        self.node.ee_cmd_vel_callback(Twist())
        self.assertFalse(self.node.command_is_live(time.time()))

        base_msgs, joint_msgs = self._run_loop_capturing_publishes()
        self.assertEqual(len(base_msgs), 1)
        self.assertEqual(len(joint_msgs), 1)
        self.assertAlmostEqual(base_msgs[0].linear.x, 0.0)
        np.testing.assert_allclose(joint_msgs[0].velocities, np.zeros(5), atol=1e-9)
        self.assertFalse(self.node.is_commanding)

        base_msgs, joint_msgs = self._run_loop_capturing_publishes()
        self.assertEqual(base_msgs, [])
        self.assertEqual(joint_msgs, [])

    def test_zero_twist_while_idle_publishes_nothing(self) -> None:
        """A zero with no prior live command must not wake the publishers."""
        self.node.ee_cmd_vel_callback(Twist())
        base_msgs, joint_msgs = self._run_loop_capturing_publishes()
        self.assertEqual(base_msgs, [])
        self.assertEqual(joint_msgs, [])
        self.assertFalse(self.node.is_commanding)


if __name__ == '__main__':
    unittest.main()
