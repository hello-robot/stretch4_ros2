from collections.abc import Generator

from common.client_nodes.mock_subscriber import MockSubscriberNode
from common.robot_reset import reset_robot_to_default
from launch_pytest.fixture import (
    get_event_loop_fixture,
    get_launch_context_fixture,
    get_launch_service_fixture,
)
import pytest
import rclpy

# Override launch_pytest's default function-scoped fixtures at module scope
# so @launch_pytest.fixture(scope='module') works across all test modules.
event_loop = get_event_loop_fixture(scope='module')
launch_service = get_launch_service_fixture(scope='module')
launch_context = get_launch_context_fixture(scope='module')


@pytest.fixture(scope='module', autouse=True)
def rclpy_context() -> Generator[None, None, None]:
    """
    Initialize and shut down the global rclpy context once per test module.

    Yields:
        None.
    """
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture(scope='module')
def require_homed_robot() -> Generator[None, None, None]:
    """
    Verify that the robot is homed before running motion-capable system tests,
    and reset the robot to a stowed pose in velocity mode on teardown.

    Yields:
        None.

    Raises:
        pytest.fail: If /is_homed is not received or reports False.
    """
    node = MockSubscriberNode('homing_check_node')
    try:
        received = node.wait_for_message('is_homed', timeout=15.0, fresh=False)
        msg = node.latest_msgs.get('is_homed')
        if not received or msg is None:
            pytest.fail(
                'Timed out waiting for /is_homed. Ensure stretch_driver is running and home the robot before running tests.'
            )
        if not msg.data:
            pytest.fail('Robot is not homed. Please home the robot before running tests.')
    finally:
        node.stop()

    try:
        yield
    finally:
        reset_robot_to_default()
