from collections.abc import Generator
import time

from common.client_nodes.mock_subscriber import MockSubscriberNode
from common.client_nodes.param_client import ParamClient
from common.launch_descriptions import stretch_driver_ld
from geometry_msgs.msg import Twist
import pytest


@pytest.fixture
def subscriber_node() -> Generator[MockSubscriberNode, None, None]:
    """
    Create and yield a MockSubscriberNode, stopping it on teardown.

    Yields:
        An active MockSubscriberNode instance.
    """
    node = MockSubscriberNode('odom_sub_node')
    yield node
    node.stop()


@pytest.fixture
def param_client() -> Generator[ParamClient, None, None]:
    """
    Create and yield a ParamClient, destroying it on teardown.

    Yields:
        An active ParamClient instance.
    """
    node = ParamClient('cmd_vel_param_client')
    yield node
    node.destroy_node()


def drive_and_get_speed(pub, subscriber_node: MockSubscriberNode, twist: Twist) -> float:
    """Publish ``twist`` on /cmd_vel for 1.5 s, then return the base's forward speed from odom."""
    for _ in range(15):
        pub.publish(twist)
        time.sleep(0.1)
    assert subscriber_node.wait_for_message('odom', timeout=2.0), "Failed to receive odom"
    return subscriber_node.latest_msgs['odom'].twist.twist.linear.x


@pytest.mark.launch(fixture=stretch_driver_ld)
def test_twist_moves_base(
    require_homed_robot: None,
    subscriber_node: MockSubscriberNode,
    param_client: ParamClient,
) -> None:
    """
    Test that a Twist on /cmd_vel drives the base in active mode, and still does after
    setting the deprecated 'position' mode.

    'position' is no longer a driver mode: the driver stays in 'active' and only puts the
    arm joints in position control, so the base keeps following /cmd_vel.

    Args:
        require_homed_robot: Fixture ensuring the robot is homed before motion tests.
        subscriber_node: Mock subscriber node fixture.
        param_client: Parameter client node fixture.
    """
    node = param_client
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    t = Twist()
    t.linear.x = 0.05

    try:
        # 1. ACTIVE MODE (the default): the base moves
        speed = drive_and_get_speed(pub, subscriber_node, t)
        assert speed > 0.005, f"Base did not move in active mode! Vel: {speed}"

        # Stop before the next check
        pub.publish(Twist())
        time.sleep(1.0)

        # 2. DEPRECATED 'position' MODE: mapped to active, so the base still moves
        assert node.set_parameter('mode', 'position'), "Failed to set deprecated position mode"
        speed = drive_and_get_speed(pub, subscriber_node, t)
        assert speed > 0.005, f"Base did not move after setting deprecated position mode! Vel: {speed}"
    finally:
        pub.publish(Twist())


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
