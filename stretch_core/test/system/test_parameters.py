import pytest
import rclpy
from common.launch_descriptions import stretch_driver_ld
from common.client_nodes.param_client import ParamClient

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_change_mode():
    rclpy.init()
    node = ParamClient('param_test_node')
    try:
        success = node.set_parameter('mode', 'position')
        assert success, "Mode change failed"
    finally:
        node.destroy_node()
        rclpy.shutdown()

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_timeout_parameter():
    rclpy.init()
    node = ParamClient('param_test_node_2')
    try:
        success = node.set_parameter('action_timeout', 0.5)
        assert success, "Timeout param change failed"
    finally:
        node.destroy_node()
        rclpy.shutdown()
