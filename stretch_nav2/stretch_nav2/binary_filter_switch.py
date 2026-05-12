#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess
import time

### TODO check if waiting is even needed, what if robot is an ssuch an area
class BinaryStateParamSwitcher(Node):
    def __init__(self):
        super().__init__('binary_state_param_switcher')

        # Declare parameters
        self.declare_parameter('false_inflation_radius', 0.35)
        self.declare_parameter('true_inflation_radius', 0.22)
        self.declare_parameter('false_vx_max', 0.32)
        self.declare_parameter('true_vx_max', 0.22)
        self.declare_parameter('log_param_change', False)

        self.costmap_nodes = [
            '/local_costmap/local_costmap',
            '/global_costmap/global_costmap'
        ]

        self.update_param_map()
        self.last_state = None  # Changed from False to None to detect first message properly

        # Wait until /binary_state topic is being published (exists)
        self.wait_for_binary_state_topic()

        # Subscribe after we confirm topic exists
        self.subscription = self.create_subscription(
            Bool,
            '/binary_state',
            self.binary_state_callback,
            10
        )

        # Apply false params initially (or from last known state)
        self.apply_settings(self.param_map[False])

        # Log parameter change callback (optional)
        if self.get_parameter('log_param_change').value:
            self.add_on_set_parameters_callback(self.on_param_change)

        self.get_logger().info('Node ready: /binary_state_param_switcher')

    def wait_for_binary_state_topic(self):
        self.get_logger().info("Waiting for /binary_state topic to be published...")
        while rclpy.ok():
            # Check if /binary_state topic is in list of published topics
            topics = self.get_node_names_and_namespaces()
            topic_list = [name for name, _ in self.get_topic_names_and_types()]
            if '/binary_state' in topic_list:
                self.get_logger().info("/binary_state topic found, continuing...")
                break
            time.sleep(0.5)

    def update_param_map(self):
        self.param_map = {
            True: {
                'inflation_radius': self.get_parameter('true_inflation_radius').value,
                'vx_max': self.get_parameter('true_vx_max').value
            },
            False: {
                'inflation_radius': self.get_parameter('false_inflation_radius').value,
                'vx_max': self.get_parameter('false_vx_max').value
            }
        }

    def binary_state_callback(self, msg):
        if msg.data == self.last_state:
            return
        self.last_state = msg.data
        self.get_logger().info(f"Binary state changed to: {msg.data}")
        self.apply_settings(self.param_map[msg.data])

    def apply_settings(self, settings):
        inflation_radius = settings['inflation_radius']
        vx_max = settings['vx_max']

        for costmap in self.costmap_nodes:
            self.set_ros2_param(costmap, 'inflation_layer.inflation_radius', str(inflation_radius))

        self.set_ros2_param('/controller_server', 'FollowPath.vx_max', str(vx_max))

    def set_ros2_param(self, node, param, value):
        cmd = ['ros2', 'param', 'set', node, param, value]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
            if result.returncode == 0:
                self.get_logger().info(f"Set {param} on {node} = {value}")
            else:
                self.get_logger().error(f"Failed to set {param} on {node}: {result.stderr.strip()}")
        except subprocess.TimeoutExpired:
            self.get_logger().error(f"Timeout while setting {param} on {node}")

    def on_param_change(self, params):
        updated = set(p.name for p in params)
        if updated & {
            'true_inflation_radius', 'false_inflation_radius',
            'true_vx_max', 'false_vx_max'
        }:
            self.update_param_map()

        for p in params:
            self.get_logger().info(f"Parameter {p.name} changed to {p.value}")

        return rclpy.parameter.SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = BinaryStateParamSwitcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
