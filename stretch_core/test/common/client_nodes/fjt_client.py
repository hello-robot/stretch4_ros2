import sys
import time
import numbers
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from unique_identifier_msgs.msg import UUID


class FJTClient(Node):

    def __init__(self, name='test_node'):
        super().__init__(name)
        self.dryrun = False
        reentrant_cb = ReentrantCallbackGroup()

        # Setup FollowJointTrajectory action client
        self._fjt_client = ActionClient(self,
            FollowJointTrajectory,
            'follow_joint_trajectory',
            callback_group=reentrant_cb
        )
        server_reached = self._fjt_client.wait_for_server(timeout_sec=20.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to Stretch action server. Timeout exceeded.')
            sys.exit()
        self._goal_handle = None

        # Setup subscriptions
        self.mode = None
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._mode_subscriber = self.create_subscription(String, 'mode', self._mode_callback, qos_profile=latching_qos, callback_group=reentrant_cb)
        self.q_curr = None
        self.q_full = None
        self._jointstate_subscriber = self.create_subscription(JointState, 'joint_states', self._joint_state_callback, 10, callback_group=reentrant_cb)

        # Setup background spinning
        self._spin_thread_shutdown_flag = threading.Event()
        self._spin_thread = threading.Thread(
            target=self._spinner,
            args=(self, SingleThreadedExecutor(),),
            daemon=True,
        )
        self._spin_thread.start()

        # Wait until mode is populated
        timeout = time.time() + 10.0
        while self.mode is None or self.q_curr is None or self.q_full is None:
            if time.time() > timeout:
                self.get_logger().error("Timeout waiting for mode or joint states in Client.__init__")
                # Don't exit here, let the test fail with a useful attribute error or assertion later if needed,
                # or better yet, maybe sys.exit is safer to fail fast?
                # The existing code had sys.exit() for server connection, so we can follow pattern or just return.
                # However, exiting might kill the test runner. Let's log error.
                break
            time.sleep(0.1)

    def destroy_node(self):
        self._spin_thread_shutdown_flag.set()
        self._spin_thread.join()
        super().destroy_node()

    def _spinner(self, node, executor):
        while not self._spin_thread_shutdown_flag.is_set():
            rclpy.spin_once(node, executor=executor)

    def _mode_callback(self, mode_string):
        self.mode = mode_string.data

    def _joint_state_callback(self, joint_states):
        q_curr = {}
        q_full = {}
        for joint in joint_states.name:
            i = joint_states.name.index(joint)
            pos = joint_states.position[i]
            vel = joint_states.velocity[i]
            eff = joint_states.effort[i]
            q_curr[joint] = pos
            q_full[joint] = (pos, vel, eff)
        self.q_curr = q_curr
        self.q_full = q_full

    def move_to_configuration(self, q, blocking=True, custom_contact_thresholds=False, custom_full_goal=False):
        if self.dryrun:
            return

        if self.mode not in ['position', 'navigation']:
            self.get_logger().error("move_to_configuration() only works in position/position-like modes")
            return

        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=0).to_msg()
        fjt_goal = FollowJointTrajectory.Goal()
        fjt_goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()
        fjt_goal.trajectory.joint_names = list(q.keys())
        fjt_goal.trajectory.points = [point]

        # construct goal
        if custom_full_goal:
            is_malformed_goal = not all([len(g) == 4 for g in q.values()])
            if is_malformed_goal:
                self.get_logger().error(f"move_to_configuration() received malformed goal. The 'custom_full_goal' option requires tuple with 4 values (position, velocity, acceleration, contact_threshold_effort) for each joint name, but q = {q}")
                return
            is_malformed_number = not all([isinstance(e, numbers.Real) for g in q.values() for e in g])
            if is_malformed_number:
                self.get_logger().error(f"move_to_configuration() received malformed goal. Each value must be a real number, but q = {q}")
                return
            point.positions = [g[0] for g in q.values()]
            point.velocities = [g[1] for g in q.values()]
            point.accelerations = [g[2] for g in q.values()]
            point.effort = [g[3] for g in q.values()]
        elif custom_contact_thresholds:
            is_malformed_goal = not all([len(g) == 2 for g in q.values()])
            if is_malformed_goal:
                self.get_logger().error(f"move_to_configuration() received malformed goal. The 'custom_contact_thresholds' option requires tuple with 2 values (position, contact_threshold_effort) for each joint name, but q = {q}")
                return
            is_malformed_number = not all([isinstance(e, numbers.Real) for g in q.values() for e in g])
            if is_malformed_number:
                self.get_logger().error(f"move_to_configuration() received malformed goal. Each value must be a real number, but q = {q}")
                return
            point.positions = [g[0] for g in q.values()]
            point.effort = [g[1] for g in q.values()]
        else:
            is_malformed_number = not all([isinstance(e, numbers.Real) for e in q.values()])
            if is_malformed_number:
                self.get_logger().error(f"move_to_configuration() received malformed goal. Each value must be a real number, but q = {q}")
                return
            point.positions = [e for e in q.values()]

        # send goal
        future = self._fjt_client.send_goal_async(fjt_goal)

        if not blocking:
            # ADD THIS: Use a callback to grab and save the goal handle once it's accepted
            future.add_done_callback(lambda f: setattr(self, '_goal_handle', f.result()))
            return future

        # Wait for acceptance
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 30.0:
                 raise TimeoutError("Timed out waiting for goal acceptance")
            time.sleep(0.01)

        return self._wait_for_result(future)

    def _wait_for_result(self, future, timeout=30.0):
        goal_handle = future.result()
        self._goal_handle = goal_handle

        if not goal_handle.accepted:
            raise RuntimeError("Goal was rejected by server")

        # Wait for result
        result_future = goal_handle.get_result_async()
        start_time = time.time()
        while not result_future.done():
            if time.time() - start_time > timeout:
                raise TimeoutError("Timed out waiting for action result")
            time.sleep(0.01)
            
        result = result_future.result()
        return result

    def cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info('Canceling current goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            self._goal_handle = None # Reset after calling cancel
            return cancel_future
        else:
            self.get_logger().warn('No active goal handle to cancel.')
