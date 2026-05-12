#!/usr/bin/env python3
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, PoseStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from simple_actions.simple_client import SimpleActionClient, ResultCode
from nav2_msgs.action import ComputePathToPose
from rcl_interfaces.srv import GetParameters
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

COLORS = [
    (0xe6, 0x19, 0x4B),
    (0x3c, 0xb4, 0x4b),
    (0xff, 0xe1, 0x19),
    (0x43, 0x63, 0xd8),
    (0xf5, 0x82, 0x31),
    (0x91, 0x1e, 0xb4),
    (0x42, 0xd4, 0xf4),
    (0xf0, 0x32, 0xe6),
    (0xbf, 0xef, 0x45),
]


class GlobalPlanDemo(Node):

    def __init__(self):
        super().__init__('global_plan_demo')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.transform = TransformStamped()
        self.transform.header.frame_id = 'map'
        self.transform.child_frame_id = 'base_link'

        self.start = None
        self.goal = None

        self.planner_names = None
        self.planner_name = None
        self.queue = []
        self.future = None

        self.colors = []
        denom = float(0xff)
        for r, g, b in COLORS:
            color = ColorRGBA()
            color.r = r / denom
            color.g = g / denom
            color.b = b / denom
            color.a = 1.0
            self.colors.append(color)

        self.marker_pub = self.create_publisher(MarkerArray, '/markers', 20)
        self.marker_array = MarkerArray()

        self.param_srv = self.create_client(GetParameters, '/planner_server/get_parameters')

        self.action_client = SimpleActionClient(self, ComputePathToPose, '/compute_path_to_pose')

        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.save_pose, 1)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.save_goal, 1)

        self.timer = self.create_timer(0.1, self.timer_cb)

    def timer_cb(self):
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.transform)

        if not self.planner_names:
            if self.future is None:
                self.param_srv.wait_for_service()
                req = GetParameters.Request(names=['planner_plugins'])
                self.future = self.param_srv.call_async(req)
                return
            elif self.future.done():
                res = self.future.result()
                value = res.values[0]
                self.planner_names = value.string_array_value

    def save_pose(self, msg):
        pose = msg.pose.pose

        self.start = PoseStamped()
        self.start.header = msg.header
        self.start.pose = pose

        self.transform.transform.translation.x = pose.position.x
        self.transform.transform.translation.y = pose.position.y
        self.transform.transform.translation.z = pose.position.z

        self.transform.transform.rotation.x = pose.orientation.x
        self.transform.transform.rotation.y = pose.orientation.y
        self.transform.transform.rotation.z = pose.orientation.z
        self.transform.transform.rotation.w = pose.orientation.w

        self.initiate_new_planning_sequence()

    def save_goal(self, msg):
        self.goal = msg
        self.initiate_new_planning_sequence()

    def initiate_new_planning_sequence(self):
        if not self.start or not self.goal or not self.planner_names:
            return

        self.queue = self.planner_names[:]
        self.marker_array.markers.clear()

        self.plan_next()

    def plan_next(self):
        if not self.queue:
            return
        self.planner_name = self.queue.pop(0)

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = self.start
        goal_msg.goal = self.goal
        goal_msg.planner_id = self.planner_name
        goal_msg.use_start = True
        self.action_client.send_goal(goal_msg, self.done)

    def done(self, result_code, result):
        if result_code != ResultCode.SUCCEEDED:
            planner_status = f'{self.planner_name}: Planning failed.'
            self.get_logger().warn(planner_status)
        else:
            d = result.planning_time.sec + result.planning_time.nanosec / 1e9
            planner_status = f'{self.planner_name}: Found plan with {len(result.path.poses)} poses in {d:4f} seconds'
            self.get_logger().info(planner_status)

        index = self.planner_names.index(self.planner_name)
        color = self.colors[index % len(self.colors)]

        path_marker = Marker()
        path_marker.header = result.path.header
        path_marker.ns = self.planner_name
        path_marker.type = Marker.LINE_STRIP
        path_marker.scale.x = 0.05
        path_marker.color = color
        for pose in result.path.poses:
            path_marker.points.append(pose.pose.position)
        self.marker_array.markers.append(path_marker)

        if len(self.planner_names) > 1:
            text_marker = Marker()
            text_marker.header = self.goal.header
            text_marker.ns = self.planner_name + '_text'
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.scale.z = 0.25
            text_marker.color = color
            text_marker.pose.position.x = self.goal.pose.position.x
            text_marker.pose.position.y = self.goal.pose.position.y - index * text_marker.scale.z
            text_marker.text = planner_status
            self.marker_array.markers.append(text_marker)
        self.marker_pub.publish(self.marker_array)

        self.plan_next()


def main():
    rclpy.init()
    node = GlobalPlanDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
