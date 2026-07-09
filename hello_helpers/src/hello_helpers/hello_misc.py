#!/usr/bin/env python3

import time
import os
import sys
import glob
import math

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.clock import Clock
from rclpy.node import Node
import tf2_ros
import numpy as np
import ros2_numpy

import pyquaternion
from statistics import mean

from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Transform
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from std_msgs.msg import String
from control_msgs.msg import JointJog
import threading
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
import numbers


class HelloNode(Node):
    def __init__(self):
        self.joint_state = JointState()
        self.tool = None
        self.mode = None
        self.sensitivity = None
        self.dryrun = False

    @classmethod
    def quick_create(cls, name, namespace=''):
        i = cls()
        i.main(name, namespace)
        return i

    def spin_thread(self, executor):
        executor.spin()

    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state

    def mode_callback(self, mode):
        self.mode = mode.data

    def sensitivity_callback(self, sensitivity):
        self.sensitivity = sensitivity.data

    def tool_callback(self, tool_string):
        self.tool = tool_string.data

    def move_to_pose(self, pose, return_before_done=False, custom_contact_thresholds=False, custom_full_goal=False):
        if self.dryrun:
            return

        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=0).to_msg()
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()
        trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory_goal.trajectory.joint_names = list(pose.keys())
        trajectory_goal.trajectory.points = [point]

        # populate goal
        if custom_full_goal:
            pose_correct = all([len(joint_goal)==4 for joint_goal in pose.values()])
            if not pose_correct:
                self.get_logger().error(f"{self.node_name}'s HelloNode.move_to_pose: Not sending trajectory due to improper pose. The 'custom_full_goal' option requires tuple with 4 values (pose_target, velocity, acceleration, contact_threshold_effort) for each joint name, but pose = {pose}")
                return
            point.positions = [float(joint_goal[0]) for joint_goal in pose.values()]
            point.velocities = [float(joint_goal[1]) for joint_goal in pose.values()]
            point.accelerations = [float(joint_goal[2]) for joint_goal in pose.values()]
            point.effort = [float(joint_goal[3]) for joint_goal in pose.values()]
        elif custom_contact_thresholds:
            pose_correct = all([len(joint_goal)==2 for joint_goal in pose.values()])
            if not pose_correct:
                self.get_logger().error(f"{self.node_name}'s HelloNode.move_to_pose: Not sending trajectory due to improper pose. The 'custom_contact_thresholds' option requires tuple with 2 values (pose_target, contact_threshold_effort) for each joint name, but pose = {pose}")
                return
            point.positions = [float(joint_goal[0]) for joint_goal in pose.values()]
            point.effort = [float(joint_goal[1]) for joint_goal in pose.values()]
        else:
            pose_correct = all([isinstance(joint_position, numbers.Real) for joint_position in pose.values()])
            if not pose_correct:
                self.get_logger().error(f"{self.node_name}'s HelloNode.move_to_pose: Not sending trajectory due to improper pose. The default option requires 1 value, pose_target as integer, for each joint name, but pose = {pose}")
                return
            point.positions = [float(joint_position) for joint_position in pose.values()]

        # send goal
        send_goal_future = self.trajectory_client.send_goal_async(trajectory_goal)
        if not return_before_done:
            while not send_goal_future.done():
                time.sleep(0.1)
            goal_handle = send_goal_future.result()
            get_result_future = goal_handle.get_result_async()
            while not get_result_future.done():
                time.sleep(0.1)
            self.get_logger().debug(f"{self.node_name}'s HelloNode.move_to_pose: completed")
            return get_result_future.result()
        return send_goal_future

    def set_velocity(self, command):
        if self.dryrun:
            return

        msg = JointJog()
        for joint, vel in command.items():
            msg.joint_names.append(joint)
            msg.velocities.append(vel)

        self.vel_ctrl_pub.publish(msg)

    def get_tf(self, from_frame, to_frame):
        """Get current transform between 2 frames. Blocking for 2 secs at worst.
        """
        try:
            return self.tf2_buffer.lookup_transform(from_frame, to_frame, Time(), timeout=Duration(seconds=2.0))
        except:
            self.get_logger().warn("Could not find the transform between frames {} and {}".format(from_frame, to_frame))

    def home_the_robot(self):
        if self.dryrun:
            return

        trigger_request = Trigger.Request()
        trigger_result = self.home_the_robot_service.call_async(trigger_request)
        while not trigger_result.done():
            time.sleep(0.2)
        self.get_logger().debug(f"{self.node_name}'s HelloNode.home_the_robot: got message {trigger_result.done()}")
        return trigger_result.done()

    def stow_the_robot(self):
        if self.dryrun:
            return

        trigger_request = Trigger.Request()
        trigger_result = self.stow_the_robot_service.call_async(trigger_request)
        while not trigger_result.done():
            time.sleep(0.2)
        self.get_logger().debug(f"{self.node_name}'s HelloNode.stow_the_robot: got message {trigger_result.done()}")
        return trigger_result.done()

    def stop_the_robot(self):
        trigger_request = Trigger.Request()
        trigger_result = self.stop_the_robot_service.call_async(trigger_request)
        while not trigger_result.done():
            time.sleep(0.2)
        self.get_logger().debug(f"{self.node_name}'s HelloNode.stop_the_robot: got message {trigger_result.done()}")
        return trigger_result.done()

    def switch_sensitivity(self, sensitivity_name):
        """ (e.g. 'off', 'default','high_sensitivity_nav', 'high_sensitivity_manipulation')
        """
        request = SetParameters.Request()
        request.parameters = [Parameter(name='sensitivity', value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=sensitivity_name))]
        result_future = self.stretch_driver_set_parameters_client.call_async(request)
        while not result_future.done():
            time.sleep(0.2)
        return result_future.result()

    def switch_mode(self, mode_name):
        request = SetParameters.Request()
        request.parameters = [Parameter(name='mode', value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=mode_name))]
        result_future = self.stretch_driver_set_parameters_client.call_async(request)
        while not result_future.done():
            time.sleep(0.2)
        return result_future.result()

    def switch_to_position_mode(self):
        return self.switch_mode('position')

    def switch_to_navigation_mode(self):
        return self.switch_mode('navigation')

    # def get_logger(self):
    #     return self.get_logger()

    def get_tool(self):
        assert(self.tool is not None)
        return self.tool

    def main(self, node_name, node_topic_namespace=''):
        rclpy.init()
        super().__init__(node_name, namespace=node_topic_namespace, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.node_name = node_name
        self.get_logger().info("{0} started".format(self.node_name))

        executor = MultiThreadedExecutor()
        executor.add_node(self)

        self.new_thread = threading.Thread(target=self.spin_thread, args=(executor, ), daemon=True)
        self.new_thread.start()

        reentrant_cb = ReentrantCallbackGroup()

        self.trajectory_client = ActionClient(self, FollowJointTrajectory, 'follow_joint_trajectory', callback_group=reentrant_cb)
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')
            sys.exit()

        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer, self)

        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.tool_subscriber = self.create_subscription(String, 'tool', self.tool_callback, latching_qos, callback_group=reentrant_cb)

        self.vel_ctrl_pub = self.create_publisher(JointJog, 'joint_vel', 1)
        self.joint_states_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10, callback_group=reentrant_cb)
        self.mode_subscriber = self.create_subscription(String, 'mode', self.mode_callback, latching_qos, callback_group=reentrant_cb)
        self.sensitivity_subscriber = self.create_subscription(String, 'sensitivity', self.sensitivity_callback, latching_qos, callback_group=reentrant_cb)

        self.home_the_robot_service = self.create_client(Trigger, 'home_the_robot', callback_group=reentrant_cb)
        while not self.home_the_robot_service.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting on '/home_the_robot' service...")
        self.get_logger().debug('Node ' + self.node_name + ' connected to /home_the_robot service.')

        self.stow_the_robot_service = self.create_client(Trigger, 'stow_the_robot', callback_group=reentrant_cb)
        while not self.stow_the_robot_service.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting on '/stow_the_robot' service...")
        self.get_logger().debug('Node ' + self.node_name + ' connected to /stow_the_robot service.')

        self.stop_the_robot_service = self.create_client(Trigger, 'stop_the_robot', callback_group=reentrant_cb)
        while not self.stop_the_robot_service.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting on '/stop_the_robot' service...")
        self.get_logger().debug('Node ' + self.node_name + ' connected to /stop_the_robot service.')

        self.stretch_driver_set_parameters_client = self.create_client(SetParameters, 'stretch_driver/set_parameters', callback_group=reentrant_cb)
        while not self.stretch_driver_set_parameters_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting on '/stretch_driver/set_parameters' service...")
        self.get_logger().debug('Node ' + self.node_name + ' connected to /stretch_driver/set_parameters service.')
