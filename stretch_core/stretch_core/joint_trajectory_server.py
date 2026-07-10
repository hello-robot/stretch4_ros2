#! /usr/bin/env python3
from __future__ import annotations
import copy
import threading
import importlib
from typing import TYPE_CHECKING, Any, List, Tuple, Dict, Optional

if TYPE_CHECKING:
    from stretch_core.stretch_driver import StretchDriver
    from hello_helpers.base_command_group import BaseCommandGroup

from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class JointTrajectoryAction:

    def __init__(self, node: StretchDriver):
        self.node = node
        self.rate = self.node.create_rate(100)
        self.latest_goal_uuid = None # Used for preemption
        self.latest_goal_lock = threading.Lock()

        # Initialize FollowJointTrajectory action
        large_queue_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50 # determines the maximum number of concurrent goal requests (from different clients) before goals are dropped (default is 10)
        )
        self.server = ActionServer(self.node, FollowJointTrajectory, 'follow_joint_trajectory',
                                   execute_callback=self.execute_cb,
                                   cancel_callback=self.cancel_cb,
                                   callback_group=node.main_group, # Must be part of the ReentrantCallbackGroup ...
                                   goal_service_qos_profile=large_queue_qos)
        # ... to service multiple goal requests with the MultiThreadedExecutor shared with stretch_driver

        # Discover and instantiate command groups
        self.command_groups: list[BaseCommandGroup] = []
        for joint_params in self.node.robot.robot_params['ros']['joints']:
            module_name = joint_params['py_module_name']
            class_name = joint_params['py_class_name']
            module = importlib.import_module(module_name)
            class_obj = getattr(module, class_name)
            cg = class_obj()
            self.command_groups.append(cg)
            self.node.get_logger().debug(f"Discovered {class_name}")

    def execute_cb(self, goal_handle: ServerGoalHandle) -> FollowJointTrajectory.Result:
        # Lock required because ReentrantCallbackGroup allows parallel execute_cbs
        with self.latest_goal_lock:
            self.latest_goal_uuid = bytes(goal_handle.goal_id.uuid) # Newest thread sets the latest UUID

        # Check driver is in a valid mode
        with self.node.driver_mode_lock:
            if self.node.driver_mode not in ['position', 'navigation']:
                return self.error_result(goal_handle, FollowJointTrajectory.Result.INVALID_GOAL, f"Must be in {['position', 'navigation']} modes to service FollowJointTrajectory action. Current mode = {self.node.driver_mode}", log_warn=True)

        # Check robot is homed
        if not self.node.robot.is_homed():
            return self.error_result(goal_handle, FollowJointTrajectory.Result.INVALID_GOAL, "Cannot execute goals while robot isn't homed")

        # Check joint names are valid
        goal: FollowJointTrajectory.Goal = goal_handle.request
        commanded_joint_names = goal.trajectory.joint_names
        self.node.get_logger().debug(f"New goal with joint_names = {commanded_joint_names}")
        ready = [c.activate(commanded_joint_names, self.invalid_joints_callback)
                 for c in self.command_groups]
        if not all(ready):
            # The joint names violated at least one of the command group's requirements.
            # The command group will have reported the error.
            return self.error_result(goal_handle, FollowJointTrajectory.Result.INVALID_JOINTS, "Invalid joint names")

        # Warn if missing joints
        num_valid_points = sum([c.get_num_active_joints() for c in self.command_groups])
        if num_valid_points != len(commanded_joint_names):
            err_str = f"Can only service {num_valid_points} joints out of {len(commanded_joint_names)} commanded"
            self.invalid_joints_callback(err_str)

        # Try to reach each of the commands in sequence until
        # an error is detected or success is achieved.
        for pointi, point in enumerate(goal.trajectory.points):
            self.node.get_logger().debug(f"target point #{pointi} = <{point}>")

            # Check command is valid
            valid_goals = [c.set_goal(point, self.invalid_goal_callback)
                           for c in self.command_groups]
            if not all(valid_goals):
                # At least one of the commands violated the requirements of a command group.
                # The command group will have reported the error.
                return self.error_result(goal_handle, FollowJointTrajectory.Result.INVALID_GOAL, "Invalid command")

            # Queue command (it will be sent out by the control loop in StretchDriver)
            for c in self.command_groups:
                c.queue_execution(self.node.robot)

            # Loop to track command
            goal_start_time = self.node.get_clock().now()
            while True:
                # Check if this goal has been preempted
                with self.latest_goal_lock:
                    if self.latest_goal_uuid != bytes(goal_handle.goal_id.uuid):
                        return self.error_result(goal_handle, -6, "Preempted") # -6 is our own result type to mean preempted

                # Check if this goal has been canceled
                if goal_handle.is_cancel_requested:
                    _ = [c.cancel_execution(self.node.robot) for c in self.command_groups]
                    return self.cancel_result(goal_handle, "Canceled")

                # Check if this goal has timed out
                if (self.node.get_clock().now() - goal_start_time) > self.node.action_timeout_duration:
                    return self.error_result(goal_handle, FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED, "Goal timed out")

                # Monitor progress towards the goal
                robot_status = copy.deepcopy(self.node.robot.status) # get copy of the current robot status (latest status is pulled by the control loop in StretchDriver)
                progress = [c.monitor_execution(robot_status) for c in self.command_groups]

                # TODO
        #         # One of the elements in named_errors will
        #         # be True when guarded contact is detected
        #         if any(ret == True for ret in named_errors):
        #             self.node.driver_mode_rwlock.release_read()
        #             return self.error_result(goal_handle, 100, self._contact_detected_err_str)

                # Publish progress
                self.publish_feedback(goal_handle, progress=progress)

                # Check if this goal has been completed
                joints_finished = [c.is_finished(robot_status, contact_detected_callback=self.contact_detected_callback) for c in self.command_groups]
                joints_finished = [j for j in joints_finished if j is not None]
                if all(joints_finished):
                    break

                self.rate.sleep()

        return self.success_result(goal_handle, "Completed trajectory")

    def contact_detected_callback(self, contact_str):
        self.node.get_logger().info(contact_str)

    def invalid_joints_callback(self, err_str):
        self.node.get_logger().warn(err_str)

    def invalid_goal_callback(self, err_str):
        self.node.get_logger().warn(err_str)

    def cancel_cb(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        # Check if this goal is active
        with self.latest_goal_lock:
            if self.latest_goal_uuid == bytes(goal_handle.goal_id.uuid):
                return CancelResponse.ACCEPT

        # Can't cancel an inactive goal
        return CancelResponse.REJECT

    def publish_feedback(self, goal_handle: ServerGoalHandle, progress: List[Any]) -> None:
        goal: FollowJointTrajectory.Goal = goal_handle.request
        feedback = FollowJointTrajectory.Feedback()
        commanded_joint_names = goal.trajectory.joint_names

        progress_dict = {}
        for item in progress:
            if item:
                if isinstance(item, list):
                    for joint, desired, actual, error in item:
                        progress_dict[joint] = (desired, actual, error)
                else:
                    joint, desired, actual, error = item
                    progress_dict[joint] = (desired, actual, error)
        desired_point = JointTrajectoryPoint()
        actual_point = JointTrajectoryPoint()
        error_point = JointTrajectoryPoint()
        for i, commanded_joint_name in enumerate(commanded_joint_names):
            if commanded_joint_name in progress_dict:
                desired, actual, error = progress_dict[commanded_joint_name]
                desired_point.positions.append(desired)
                actual_point.positions.append(actual)
                error_point.positions.append(error)
            else:
                # -1.0 denotes: unable to service this joint
                desired, actual, error = (-1.0, -1.0, -1.0)
                desired_point.positions.append(desired)
                actual_point.positions.append(actual)
                error_point.positions.append(error)

        feedback.desired = desired_point
        feedback.actual = actual_point
        feedback.error = error_point

        feedback.joint_names = commanded_joint_names
        feedback.header.stamp = self.node.get_clock().now().to_msg()
        goal_handle.publish_feedback(feedback)

    def error_result(self, goal_handle: ServerGoalHandle, error_code, error_str, log_warn=False):
        if log_warn:
            self.node.get_logger().warn(error_str)
        else:
            self.node.get_logger().error(error_str)
        result = FollowJointTrajectory.Result()
        result.error_code = error_code
        result.error_string = error_str
        goal_handle.abort()
        return result

    def success_result(self, goal_handle: ServerGoalHandle, success_str):
        self.node.get_logger().debug(success_str)
        result = FollowJointTrajectory.Result()
        result.error_code = result.SUCCESSFUL
        result.error_string = success_str
        goal_handle.succeed()
        return result

    def cancel_result(self, goal_handle: ServerGoalHandle, cancel_str):
        self.node.get_logger().debug(cancel_str)
        result = FollowJointTrajectory.Result()
        result.error_string = cancel_str
        goal_handle.canceled()
        return result
