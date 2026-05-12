#!/usr/bin/env python3
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class ControllerConfigSwitcher(Node):
    def __init__(self):
        super().__init__('controller_switcher')

        # QoS for publishing to /controller_selector
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.controller_pub = self.create_publisher(
            String,
            '/controller_selector',
            qos_profile
        )

        # Service that switches config based on bool
        self.srv = self.create_service(
            SetBool,
            '/switch_controller',
            self.handle_switch_request
        )

        self.get_logger().info(
            "ControllerConfigSwitcher ready. "
            "Call /switch_controller with data=true (MPPI) or false (DWB)."
        )

    def handle_switch_request(self, request, response):
        # Decide which controller & yaw tolerance
        if request.data:
            controller = 'MPPIController'
            yaw_tolerance = 3.145
        else:
            controller = 'DWBController'
            yaw_tolerance = 0.08

        # Publish the controller name
        ctrl_msg = String(data=controller)
        self.controller_pub.publish(ctrl_msg)
        self.get_logger().info(f'Published controller: "{controller}"')

        # Use ROS2 CLI to set the parameter
        cmd = [
            'ros2', 'param', 'set',
            '/controller_server',
            'general_goal_checker.yaw_goal_tolerance',
            str(yaw_tolerance)
        ]
        try:
            # run with a timeout so we don't hang forever
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=5.0
            )
        except subprocess.TimeoutExpired:
            response.success = False
            response.message = (
                f'Failed to set parameter: ros2 param set timed out.'
            )
            self.get_logger().error(response.message)
            return response

        # Inspect the CLI exit code
        if result.returncode == 0:
            response.success = True
            response.message = (
                f'Switched to {controller}; '
                f'set yaw_goal_tolerance = {yaw_tolerance}'
            )
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = (
                f'CLI error (code {result.returncode}): '
                f'{result.stderr.strip()}'
            )
            self.get_logger().error(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ControllerConfigSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
