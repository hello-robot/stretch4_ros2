#!/usr/bin/env python3
"""ROS 2 node that filters Twist commands with under-base hazard points."""

from __future__ import annotations

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from stretch_base_hazard.hazard_point_cache import (
    declare_hazard_filter_parameters,
    hazard_point_cache_config_from_params,
    HazardPointCache,
)


class HazardCmdVelFilterNode(Node):

    def __init__(self):
        super().__init__('hazard_cmd_vel_filter_node')
        self.declare_parameter('cmd_vel_in_topic', 'cmd_vel_unfiltered')
        self.declare_parameter('cmd_vel_out_topic', 'cmd_vel')
        declare_hazard_filter_parameters(self)

        self._cache = HazardPointCache(self, hazard_point_cache_config_from_params(self))
        self._pub = self.create_publisher(
            Twist,
            str(self.get_parameter('cmd_vel_out_topic').value),
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('cmd_vel_in_topic').value),
            self._on_twist,
            10,
        )
        self.get_logger().info(
            'hazard_cmd_vel_filter_node started '
            f'in={self.get_parameter("cmd_vel_in_topic").value} '
            f'out={self.get_parameter("cmd_vel_out_topic").value}',
        )

    def _on_twist(self, msg: Twist) -> None:
        decision = self._cache.filter_velocity(
            msg.linear.x,
            msg.linear.y,
            msg.angular.z,
        )
        result = decision.result
        out = Twist()
        out.linear.x = result.vx
        out.linear.y = result.vy
        out.linear.z = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = result.wz
        self._pub.publish(out)

        moving_linear = (
            abs(msg.linear.x) > self._cache.config.filter_config.min_linear_speed_mps
            or abs(msg.linear.y) > self._cache.config.filter_config.min_linear_speed_mps
        )
        if decision.stale and moving_linear:
            self.get_logger().warning(
                'Blocking cmd_vel linear velocity because hazard point clouds '
                'are stale or missing',
                throttle_duration_sec=1.0,
            )
        elif result.blocked_by_obstacle or result.blocked_by_cliff:
            self.get_logger().warning(
                'Blocking cmd_vel linear velocity '
                f'vx={msg.linear.x:.3f} vy={msg.linear.y:.3f} wz={msg.angular.z:.3f}; '
                f'obstacle_cells={result.blocking_obstacle_count} '
                f'cliff_cells={result.blocking_cliff_count}',
                throttle_duration_sec=0.5,
            )
        elif result.slowed_by_obstacle:
            self.get_logger().warning(
                'Slowing cmd_vel linear velocity for tight clearance '
                f'vx={msg.linear.x:.3f}->{result.vx:.3f} '
                f'vy={msg.linear.y:.3f}->{result.vy:.3f} '
                f'scale={result.obstacle_speed_scale:.2f} '
                f'soft_obstacle_cells={result.soft_obstacle_count}',
                throttle_duration_sec=0.5,
            )


def main(args=None):
    rclpy.init(args=args)
    node = HazardCmdVelFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
