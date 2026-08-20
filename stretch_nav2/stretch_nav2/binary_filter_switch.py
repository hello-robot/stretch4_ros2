#!/usr/bin/env python3
"""Switch Nav2 costmap and controller parameters when /binary_state flips.
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool

COSTMAP_NODES = ('/local_costmap/local_costmap', '/global_costmap/global_costmap')
CONTROLLER_NODE = '/controller_server'
TARGET_NODES = COSTMAP_NODES + (CONTROLLER_NODE,)

INFLATION_RADIUS_PARAM = 'inflation_layer.inflation_radius'
VX_MAX_PARAM = 'FollowPath.vx_max'

# Parameters that feed self.param_map; a change to any of them rebuilds it.
PARAM_MAP_KEYS = frozenset({
    'true_inflation_radius', 'false_inflation_radius',
    'true_vx_max', 'false_vx_max',
})


class BinaryStateParamSwitcher(Node):
    def __init__(self):
        super().__init__('binary_state_param_switcher')

        self.declare_parameter('false_inflation_radius', 0.35)
        self.declare_parameter('true_inflation_radius', 0.22)
        self.declare_parameter('false_vx_max', 0.32)
        self.declare_parameter('true_vx_max', 0.22)
        self.declare_parameter('log_param_change', False)
        self.declare_parameter('retry_period_s', 1.0)

        # on_set validates only; post_set is what updates the
        # node's internal state, once those values are committed.
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.add_post_set_parameters_callback(self.on_parameters_set)

        self.update_param_map()
        self.last_state = None  # Last /binary_state value seen
        self.applied_state = None  # Which state's settings were last pushed out

        self.param_clients = {name: AsyncParameterClient(self, name) for name in TARGET_NODES}
        
        # Nav2 nodes come up on their own schedule, so a push aimed at a node that
        # is not listening yet is queued here and retried instead of being dropped.
        self.pending = {}
        retry_period = self.get_parameter('retry_period_s').value
        self.retry_timer = self.create_timer(retry_period, self.flush_pending)

        self.subscription = self.create_subscription(
            Bool,
            '/binary_state',
            self.binary_state_callback,
            10
        )

        # Apply false params initially
        self.apply_state(False)

        self.get_logger().info('Node ready: /binary_state_param_switcher')

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
        self.apply_state(msg.data)

    def apply_state(self, state):
        self.applied_state = state
        settings = self.param_map[state]
        inflation_radius = Parameter(
            INFLATION_RADIUS_PARAM, Parameter.Type.DOUBLE,
            float(settings['inflation_radius']))
        vx_max = Parameter(
            VX_MAX_PARAM, Parameter.Type.DOUBLE, float(settings['vx_max']))

        for costmap in COSTMAP_NODES:
            self.set_remote_parameters(costmap, [inflation_radius])
        self.set_remote_parameters(CONTROLLER_NODE, [vx_max])

    def set_remote_parameters(self, node_name, parameters):
        """Send a non-blocking set_parameters request; queue it if the node is down."""
        client = self.param_clients[node_name]
        if not client.services_are_ready():
            self.pending[node_name] = parameters
            self.get_logger().warn(
                f"{node_name} parameter service is not available yet; "
                f"queued {[p.name for p in parameters]} for retry")
            return

        self.pending.pop(node_name, None)
        future = client.set_parameters(parameters)
        future.add_done_callback(
            lambda done, node=node_name, params=parameters:
                self.log_set_result(done, node, params))

    def flush_pending(self):
        for node_name, parameters in list(self.pending.items()):
            if self.param_clients[node_name].services_are_ready():
                self.get_logger().info(f"{node_name} is up; applying queued parameters")
                self.set_remote_parameters(node_name, parameters)

    def log_set_result(self, future, node_name, parameters):
        try:
            response = future.result()
        except Exception as ex:
            self.get_logger().error(f"Setting parameters on {node_name} failed: {ex}")
            return

        for param, result in zip(parameters, response.results):
            if result.successful:
                self.get_logger().info(f"Set {param.name} on {node_name} = {param.value}")
            else:
                self.get_logger().error(
                    f"{node_name} rejected {param.name} = {param.value}: {result.reason}")

    def validate_parameters(self, params):
        """Validate incoming values only"""
        for p in params:
            # Parameters this node does not own are left to other callbacks;
            # rejecting them here would stop those callbacks from handling them.
            if p.name not in PARAM_MAP_KEYS:
                continue
            if p.value is None or p.value <= 0.0:
                return SetParametersResult(
                    successful=False,
                    reason=f"{p.name} must be greater than 0, got {p.value}"
                )
        return SetParametersResult(successful=True)

    def on_parameters_set(self, params):
        """Values are already validated and committed, so apply here."""
        if any(p.name in PARAM_MAP_KEYS for p in params):
            self.update_param_map()
            if self.applied_state is not None:
                self.get_logger().info(
                    f"Parameters changed; re-applying settings for state {self.applied_state}")
                self.apply_state(self.applied_state)

        if self.get_parameter('log_param_change').value:
            for p in params:
                self.get_logger().info(f"Parameter {p.name} changed to {p.value}")


def main(args=None):
    rclpy.init(args=args)
    node = BinaryStateParamSwitcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
