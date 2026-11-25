"""Lightweight ROS 2 client node for autodocking system tests."""

from collections.abc import Callable
import math
import threading
import time
from typing import Any

from action_msgs.srv import CancelGoal
from control_msgs.action import FollowJointTrajectory
from nav_msgs.msg import Odometry
from nav2_msgs.action import DockRobot
import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState, JointState, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
from unique_identifier_msgs.msg import UUID

ARM_LINK_NAMES = ('arm_l1_joint', 'arm_l2_joint', 'arm_l3_joint', 'arm_l4_joint')


class DockClient(Node):
    """Test client node for triggering, canceling, and monitoring /dock_robot."""

    def __init__(self, name: str = 'autodocking_test_client') -> None:
        """
        Initialize action clients, service clients, topic subscriptions, and background spinner.

        Args:
            name: ROS 2 node name.

        Returns:
            None.
        """
        super().__init__(name)
        cb_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()

        self.mode: str | None = None
        self.power_supply_status: int | None = None
        self.feedback_state: int | None = None
        self.arm_pos: float | None = None
        self.base_speed: tuple[float, float] | None = None
        self._clouds_seen = [False, False]

        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        cloud_qos = rclpy.qos.qos_profile_sensor_data

        self.create_subscription(String, '/mode', self._mode_cb, latching_qos, callback_group=cb_group)
        self.create_subscription(BatteryState, '/battery', self._battery_cb, 10, callback_group=cb_group)
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10, callback_group=cb_group)
        self.create_subscription(Odometry, '/wheel_odom', self._odom_cb, 10, callback_group=cb_group)
        self._left_cloud_sub = self.create_subscription(
            PointCloud2, '/lidar_points_left', lambda _m: self._mark_cloud(0), cloud_qos, callback_group=cb_group
        )
        self._right_cloud_sub = self.create_subscription(
            PointCloud2, '/lidar_points_right', lambda _m: self._mark_cloud(1), cloud_qos, callback_group=cb_group
        )

        self._dock_client = ActionClient(self, DockRobot, '/dock_robot', callback_group=cb_group)
        self._fjt_client = ActionClient(self, FollowJointTrajectory, '/follow_joint_trajectory', callback_group=cb_group)
        self._cancel_srv = self.create_client(CancelGoal, '/dock_robot/_action/cancel_goal', callback_group=cb_group)
        self._stop_srv = self.create_client(Trigger, '/stop_the_robot', callback_group=cb_group)
        self._stow_srv = self.create_client(Trigger, '/stow_the_robot', callback_group=cb_group)

        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self)
        self._shutdown = threading.Event()
        self._thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._thread.start()

    def _spin_loop(self) -> None:
        """
        Spin the internal MultiThreadedExecutor until stop() is called.

        Args:
            None.

        Returns:
            None.
        """
        while not self._shutdown.is_set() and rclpy.ok():
            self._executor.spin_once(timeout_sec=0.05)

    def stop(self) -> None:
        """
        Stop the background spin thread and destroy the node.

        Args:
            None.

        Returns:
            None.
        """
        self._shutdown.set()
        self._thread.join(timeout=2.0)
        self._executor.shutdown()
        self.destroy_node()

    def _mode_cb(self, msg: String) -> None:
        """Update cached /mode string."""
        self.mode = msg.data

    def _battery_cb(self, msg: BatteryState) -> None:
        """Update cached BatteryState.power_supply_status."""
        self.power_supply_status = msg.power_supply_status

    def _joint_state_cb(self, msg: JointState) -> None:
        """Update cached total telescoping arm extension in meters."""
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in ARM_LINK_NAMES):
            self.arm_pos = sum(positions[name] for name in ARM_LINK_NAMES)

    def _odom_cb(self, msg: Odometry) -> None:
        """Update cached planar linear (m/s) and angular (rad/s) speeds."""
        t = msg.twist.twist
        self.base_speed = (math.hypot(t.linear.x, t.linear.y), abs(t.angular.z))

    def _mark_cloud(self, idx: int) -> None:
        """Record that a point cloud frame has been received on lidar index idx."""
        self._clouds_seen[idx] = True

    def _feedback_cb(self, feedback_msg: Any) -> None:
        """Update cached DockRobot.Feedback.state integer."""
        self.feedback_state = int(feedback_msg.feedback.state)

    def wait_until(self, predicate: Callable[[], bool], timeout_sec: float, poll_sec: float = 0.02) -> bool:
        """
        Poll predicate until it returns True or timeout_sec elapses.

        Args:
            predicate: Zero-argument callable returning True when ready.
            timeout_sec: Maximum seconds to wait.
            poll_sec: Sleep interval between checks in seconds.

        Returns:
            True if predicate succeeded before timeout_sec, False otherwise.
        """
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(poll_sec)
        return predicate()

    def _await_future(self, future: Any, timeout_sec: float, label: str) -> Any:
        """
        Block until an rclpy Future finishes and return its result.

        Args:
            future: rclpy Future to await.
            timeout_sec: Maximum wait duration in seconds.
            label: Name included in TimeoutError if the future does not complete.

        Returns:
            Resolved result of future.
        """
        if not self.wait_until(future.done, timeout_sec=timeout_sec, poll_sec=0.01):
            raise TimeoutError(f'Timed out waiting for {label}.')
        return future.result()

    def wait_for_ready(self, timeout_sec: float = 45.0) -> None:
        """
        Wait for docking_server, stretch_driver services, and dual Hesai point clouds.

        Args:
            timeout_sec: Maximum seconds to wait for stack bringup.

        Returns:
            None.
        """
        assert self._dock_client.wait_for_server(timeout_sec=timeout_sec), '/dock_robot server unavailable'
        assert self._fjt_client.wait_for_server(timeout_sec=timeout_sec), '/follow_joint_trajectory unavailable'
        assert self._cancel_srv.wait_for_service(timeout_sec=10.0), '/dock_robot/_action/cancel_goal unavailable'
        assert self._stop_srv.wait_for_service(timeout_sec=10.0), '/stop_the_robot unavailable'
        assert self._stow_srv.wait_for_service(timeout_sec=10.0), '/stow_the_robot unavailable'
        ready = self.wait_until(
            lambda: (
                self.mode is not None
                and self.power_supply_status is not None
                and self.arm_pos is not None
                and self.base_speed is not None
                and all(self._clouds_seen)
            ),
            timeout_sec=timeout_sec,
            poll_sec=0.05,
        )
        assert ready, 'Timed out waiting for initial driver topics and dual Hesai point clouds.'
        self.destroy_subscription(self._left_cloud_sub)
        self.destroy_subscription(self._right_cloud_sub)

    def send_dock_goal(self) -> ClientGoalHandle:
        """
        Send a /dock_robot goal with use_dock_id=False and navigate_to_staging_pose=False.

        Args:
            None.

        Returns:
            Accepted ClientGoalHandle for the active docking goal.
        """
        self.feedback_state = None
        goal = DockRobot.Goal()
        goal.use_dock_id = False
        goal.navigate_to_staging_pose = False
        handle = self._await_future(
            self._dock_client.send_goal_async(goal, feedback_callback=self._feedback_cb),
            timeout_sec=10.0,
            label='/dock_robot goal acceptance',
        )
        assert handle.accepted, '/dock_robot goal rejected'
        return handle

    def wait_result(self, handle: ClientGoalHandle, timeout_sec: float = 10.0) -> tuple[int, DockRobot.Result]:
        """
        Wait for a /dock_robot goal handle to reach a terminal state.

        Args:
            handle: Active ClientGoalHandle returned by send_dock_goal.
            timeout_sec: Maximum wait duration in seconds.

        Returns:
            Tuple of (GoalStatus integer, DockRobot.Result).
        """
        wrapped = self._await_future(handle.get_result_async(), timeout_sec=timeout_sec, label='/dock_robot result')
        return int(wrapped.status), wrapped.result

    def cancel_wildcard(self) -> CancelGoal.Response:
        """
        Cancel active /dock_robot goals using the wildcard zero-UUID CancelGoal service call.

        Args:
            None.

        Returns:
            CancelGoal.Response from /dock_robot/_action/cancel_goal.
        """
        req = CancelGoal.Request()
        req.goal_info.goal_id = UUID(uuid=[0] * 16)
        return self._await_future(
            self._cancel_srv.call_async(req), timeout_sec=5.0, label='/dock_robot/_action/cancel_goal'
        )

    def stop_the_robot(self) -> Trigger.Response:
        """
        Call /stop_the_robot synchronously.

        Args:
            None.

        Returns:
            Trigger.Response from /stop_the_robot.
        """
        return self._await_future(
            self._stop_srv.call_async(Trigger.Request()), timeout_sec=5.0, label='/stop_the_robot'
        )

    def stow_the_robot(self) -> Trigger.Response:
        """
        Call /stow_the_robot synchronously.

        Args:
            None.

        Returns:
            Trigger.Response from /stow_the_robot.
        """
        return self._await_future(
            self._stow_srv.call_async(Trigger.Request()), timeout_sec=35.0, label='/stow_the_robot'
        )

    def move_joints(self, positions: dict[str, float], timeout_sec: float = 15.0) -> int:
        """
        Execute a blocking single-point FollowJointTrajectory goal.

        Args:
            positions: Mapping of joint name to target position.
            timeout_sec: Maximum seconds to wait for completion.

        Returns:
            Terminal GoalStatus integer.
        """
        point = JointTrajectoryPoint()
        point.positions = list(positions.values())
        point.time_from_start = Duration(seconds=0).to_msg()
        goal = FollowJointTrajectory.Goal()
        goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()
        goal.trajectory.joint_names = list(positions.keys())
        goal.trajectory.points = [point]

        handle = self._await_future(
            self._fjt_client.send_goal_async(goal), timeout_sec=10.0, label='FJT goal acceptance'
        )
        assert handle.accepted, f'FJT goal rejected for {positions}'
        wrapped = self._await_future(handle.get_result_async(), timeout_sec=timeout_sec, label='FJT result')
        return int(wrapped.status)

    def assert_base_stopped(self, timeout_sec: float = 3.0) -> None:
        """
        Assert wheel odometry linear and angular speeds settle near zero within timeout_sec.

        Args:
            timeout_sec: Maximum seconds to wait for deceleration.

        Returns:
            None.
        """
        stopped = self.wait_until(
            lambda: self.base_speed is not None and self.base_speed[0] < 0.015 and self.base_speed[1] < 0.05,
            timeout_sec=timeout_sec,
            poll_sec=0.05,
        )
        assert stopped, f'Base did not stop within {timeout_sec:.1f}s (speed={self.base_speed})'
