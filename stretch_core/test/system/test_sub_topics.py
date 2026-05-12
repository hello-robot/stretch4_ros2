import pytest
import rclpy
import time

from common.launch_descriptions import stretch_driver_ld
from common.client_nodes.mock_subscriber import MockSubscriberNode
from common.client_nodes.param_client import ParamClient
from geometry_msgs.msg import Twist

@pytest.fixture
def subscriber_node():
    rclpy.init()
    node = MockSubscriberNode('odom_sub_node')
    yield node
    node.stop()

@pytest.fixture
def param_client():
    node = ParamClient('cmd_vel_param_client')
    yield node
    node.destroy_node()

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_twist_in_position_and_navigation_modes(subscriber_node, param_client):
    """
    Test sending Twist messages in position mode (should not move)
    and navigation mode (should move). We monitor /wheel_odom to verify.
    """
    node = param_client

    # Create publisher
    pub = node.create_publisher(Twist, '/stretch/cmd_vel', 10)
    t = Twist()
    t.linear.x = 0.05

    # 1. POSITION MODE (Should NOT move)
    success = node.set_parameter('mode', 'position')
    assert success, "Failed to set position mode"

    # Publish and wait
    pub.publish(t)
    subscriber_node.wait_for_message('odom', timeout=2.0)

    # Check odom (should be 0 since mode rejected it)
    odom_msg = subscriber_node.latest_msgs.get('odom')
    assert odom_msg is not None, "Failed to receive odom"
    assert abs(odom_msg.twist.twist.linear.x) < 0.005, "Robot moved in position mode!"

    # 2. NAVIGATION MODE (Should move)
    success = node.set_parameter('mode', 'navigation')
    assert success, "Failed to set navigation mode"

    # Publish and wait
    pub.publish(t)
    time.sleep(1.5) # give driver a moment to accelerate
    subscriber_node.wait_for_message('odom', timeout=2.0)

    # Check odom (should be > 0 since it is moving)
    odom_msg = subscriber_node.latest_msgs.get('odom')
    assert odom_msg is not None, "Failed to receive odom"
    assert odom_msg.twist.twist.linear.x > 0.005, f"Robot did not move in navigation mode! Vel: {odom_msg.twist.twist.linear.x}"

    rclpy.shutdown()

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_battery_topic_reception(subscriber_node):
    """
    Test receiving battery messages
    """
    subscriber_node.wait_for_message('battery', timeout=2.0)

    battery_msg = subscriber_node.latest_msgs.get('battery')
    assert battery_msg is not None, "Failed to receive battery state"
    assert battery_msg.voltage > 0.0, "Battery voltage should be greater than 0"
    assert battery_msg.percentage > 0.0, "Battery percentage should be greater than 0"

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_diagnostic_topic_reception(subscriber_node):
    """
    Test receiving diagnostic messages
    """
    subscriber_node.wait_for_message('diagnostics', timeout=2.0)
    
    diag_msg = subscriber_node.latest_msgs.get('diagnostics')
    assert diag_msg is not None, "Failed to receive diagnostics state"
    assert len(diag_msg.status) > 0, "Diagnostic array is empty"
    
    # Check that we received safe_motion_manager and sentry_manager
    names = [s.name for s in diag_msg.status]
    assert 'safety_layer/safe_motion_manager' in names, "Missing safe_motion_manager diagnostics"
    assert 'safety_layer/sentry_manager' in names, "Missing sentry_manager diagnostics"
