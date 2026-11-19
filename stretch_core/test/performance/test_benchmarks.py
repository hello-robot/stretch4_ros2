from collections.abc import Generator
import time

from common.client_nodes.fjt_client import FJTClient
from common.client_nodes.mock_subscriber import MockSubscriberNode
from common.launch_descriptions import stretch_driver_ld
import pytest


@pytest.fixture
def subscriber_node() -> Generator[MockSubscriberNode, None, None]:
    """
    Create and yield a MockSubscriberNode, stopping it on teardown.

    Yields:
        An active MockSubscriberNode instance.
    """
    node = MockSubscriberNode('perf_sub_node')
    yield node
    node.stop()


@pytest.fixture
def action_client(require_homed_robot: None) -> Generator[FJTClient, None, None]:
    """
    Create and yield an FJTClient after verifying the robot is homed, destroying it on teardown.

    Args:
        require_homed_robot: Fixture ensuring the robot is homed before motion tests.

    Yields:
        An active FJTClient instance.
    """
    node = FJTClient('perf_action_client')
    yield node
    node.destroy_node()


@pytest.mark.launch(fixture=stretch_driver_ld)
def test_topic_publish_rates(benchmark, subscriber_node: MockSubscriberNode) -> None:
    """
    Benchmark the latency to receive 10 consecutive joint state messages.

    Args:
        benchmark: The pytest-benchmark fixture.
        subscriber_node: The mock subscriber node fixture.
    """
    def measure_joint_states(node: MockSubscriberNode) -> None:
        count = 0
        while count < 10:
            assert node.wait_for_message('joint_states', timeout=1.0), 'Timed out waiting for joint_states'
            count += 1

    benchmark(measure_joint_states, subscriber_node)


@pytest.mark.launch(fixture=stretch_driver_ld)
def test_battery_publish_rates(benchmark, subscriber_node: MockSubscriberNode) -> None:
    """
    Benchmark the latency to receive 10 consecutive battery state messages.

    Args:
        benchmark: The pytest-benchmark fixture.
        subscriber_node: The mock subscriber node fixture.
    """
    def measure_battery_states(node: MockSubscriberNode) -> None:
        count = 0
        while count < 10:
            assert node.wait_for_message('battery', timeout=2.0), 'Timed out waiting for battery'
            count += 1

    benchmark(measure_battery_states, subscriber_node)


@pytest.mark.launch(fixture=stretch_driver_ld)
def test_diagnostic_publish_rates(benchmark, subscriber_node: MockSubscriberNode) -> None:
    """
    Benchmark the latency to receive 10 consecutive diagnostic messages.

    Args:
        benchmark: The pytest-benchmark fixture.
        subscriber_node: The mock subscriber node fixture.
    """
    def measure_diagnostic_states(node: MockSubscriberNode) -> None:
        count = 0
        while count < 10:
            assert node.wait_for_message('diagnostics', timeout=2.0), 'Timed out waiting for diagnostics'
            count += 1

    benchmark(measure_diagnostic_states, subscriber_node)
    
@pytest.mark.launch(fixture=stretch_driver_ld)
def test_action_preemption_latency(benchmark, action_client):
    """
    Benchmark the time it takes to preempt a goal with a new one
    """
    def preempt_goal():
        action_client.mode = 'position'
        future1 = action_client.move_to_configuration({'lift_joint': 0.8}, blocking=False)
        time.sleep(0.1)
        future2 = action_client.move_to_configuration({'lift_joint': 0.3}, blocking=False)
        gh1 = future1.result()
        gh1.get_result_async().result() # Wait for first to be canceled/preempted
        gh2 = future2.result()
        gh2.get_result_async().result() # Wait for second to finish

    benchmark(preempt_goal)


@pytest.mark.launch(fixture=stretch_driver_ld)
def test_action_cancel_latency(benchmark, action_client):
    """
    Benchmark the time it takes to cancel a goal
    """
    def cancel_goal():
        action_client.mode = 'position'
        future = action_client.move_to_configuration({'lift_joint': 0.8}, blocking=False)
        time.sleep(0.1)
        action_client.cancel_goal()
        gh = future.result()
        gh.get_result_async().result()

    benchmark(cancel_goal)

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_action_error_threshold_closeness(benchmark, action_client):
    """
    Benchmark the error (distance from target) and time it takes to settle
    """
    def settle_to_goal():
        action_client.mode = 'position'
        target = 0.5
        q = {'lift_joint': target}

        # blocking=True means it waits until is_finished() is True via the server
        result = action_client.move_to_configuration(q, blocking=True)
        assert result.status == 4 # Succeeded

        # Verify closeness
        # (Assuming client updates q_curr)
        actual = action_client.q_curr.get('lift_joint', 0.0)
        error = abs(target - actual)
        assert error < 0.05, f"Error {error} is outside of acceptable threshold"

    benchmark(settle_to_goal)

