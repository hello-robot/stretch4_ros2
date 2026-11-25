from collections.abc import Generator
import threading
import time

from launch_pytest.fixture import (
    get_event_loop_fixture,
    get_launch_context_fixture,
    get_launch_service_fixture,
)
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

# Override launch_pytest's default function-scoped fixtures at module scope
# so @launch_pytest.fixture(scope='module') works across all test modules.
event_loop = get_event_loop_fixture(scope='module')
launch_service = get_launch_service_fixture(scope='module')
launch_context = get_launch_context_fixture(scope='module')


@pytest.fixture(scope='module', autouse=True)
def rclpy_context() -> Generator[None, None, None]:
    """
    Initialize and shut down the global rclpy context once per test module.

    Args:
        None.

    Returns:
        Generator yielding None while the rclpy context is active.
    """
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture(scope='module')
def require_homed_robot() -> Generator[None, None, None]:
    """
    Verify that the robot is homed before running hardware autodocking tests,
    and issue /stop_the_robot on module teardown.

    Args:
        None.

    Returns:
        Generator yielding None after confirming /is_homed is True.
    """
    node = Node('nav2_homing_check_node')
    homed_event = threading.Event()
    is_homed = False

    def _on_homed(msg: Bool) -> None:
        """Record latched /is_homed state."""
        nonlocal is_homed
        is_homed = bool(msg.data)
        homed_event.set()

    latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Bool, '/is_homed', _on_homed, latching_qos)
    try:
        deadline = time.monotonic() + 15.0
        while not homed_event.is_set() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not homed_event.is_set():
            pytest.fail(
                'Timed out waiting for /is_homed. Ensure stretch_driver is running and home the robot before running tests.'
            )
        if not is_homed:
            pytest.fail('Robot is not homed. Please home the robot before running tests.')
    finally:
        node.destroy_node()

    try:
        yield
    finally:
        stop_node = Node('nav2_teardown_stop_node')
        try:
            client = stop_node.create_client(Trigger, '/stop_the_robot')
            if client.wait_for_service(timeout_sec=2.0):
                future = client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(stop_node, future, timeout_sec=2.0)
        finally:
            stop_node.destroy_node()
