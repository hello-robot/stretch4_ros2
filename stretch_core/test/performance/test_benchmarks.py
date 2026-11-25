import pytest
import launch_pytest
import rclpy
import time
from common.launch_descriptions import stretch_driver_ld as _stretch_driver_ld
from common.client_nodes.mock_subscriber import MockSubscriberNode
from common.client_nodes.fjt_client import FJTClient
from stretch_core.stretch_driver import StretchDriver
from action_msgs.msg import GoalStatus

# One driver for the whole module. A per-test launch races the launch singleton lock: the next
# test's launch sees the previous driver still holding it, starts nothing, and then that driver exits.
stretch_driver_ld = launch_pytest.fixture(_stretch_driver_ld.__wrapped__, scope='module')

def wait_for(future, timeout=30.0):
    """An rclpy future's result, once it has one. Future.result() itself doesn't wait:
    it returns None while the future is pending. The client nodes spin in a background
    thread, so polling done() is enough."""
    end = time.time() + timeout
    while not future.done():
        assert time.time() < end, f"Timed out after {timeout}s waiting for {future}"
        time.sleep(0.01)
    return future.result()


@pytest.fixture
def subscriber_node():
    rclpy.init()
    node = MockSubscriberNode('perf_sub_node')
    yield node
    node.stop()
    rclpy.shutdown()

@pytest.fixture
def action_client():
    rclpy.init()
    node = FJTClient('perf_action_client')
    yield node
    node.destroy_node()
    rclpy.shutdown()

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_topic_publish_rates(benchmark, subscriber_node):
    """
    Benchmark the latency to receive 10 consecutive joint state messages
    """
    def measure_joint_states(node):
        count = 0
        while count < 10:
            assert node.wait_for_message('joint_states', timeout=1.0), "No 'joint_states' message within 1s"
            count += 1

    benchmark(measure_joint_states, subscriber_node)

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_battery_publish_rates(benchmark, subscriber_node):
    """
    Benchmark the latency to receive 10 consecutive battery state messages
    """
    def measure_battery_states(node):
        count = 0
        while count < 10:
            assert node.wait_for_message('battery', timeout=1.0), "No 'battery' message within 1s"
            count += 1

    benchmark(measure_battery_states, subscriber_node)
    
@pytest.mark.launch(fixture=stretch_driver_ld)
def test_diagnostic_publish_rates(benchmark, subscriber_node):
    """
    Benchmark the latency to receive 10 consecutive diagnostic messages
    """
    def measure_diagnostic_states(node):
        count = 0
        while count < 10:
            assert node.wait_for_message('diagnostics', timeout=1.0), "No 'diagnostics' message within 1s"
            count += 1
    
    benchmark(measure_diagnostic_states, subscriber_node)
    
@pytest.mark.launch(fixture=stretch_driver_ld)
def test_action_preemption_latency(benchmark, action_client):
    """
    Benchmark the time it takes to preempt a goal with a new one
    """
    def preempt_goal():
        future1 = action_client.move_to_configuration({'lift_joint': 0.8}, blocking=False)
        time.sleep(0.1)
        future2 = action_client.move_to_configuration({'lift_joint': 0.3}, blocking=False)
        gh1 = wait_for(future1)
        gh2 = wait_for(future2)
        assert gh1.accepted and gh2.accepted, "Goal was rejected by server"
        wait_for(gh1.get_result_async()) # Wait for first to be preempted
        result2 = wait_for(gh2.get_result_async()) # Wait for second to finish
        assert result2.status == GoalStatus.STATUS_SUCCEEDED, f"Second goal ended with status {result2.status}"

    benchmark(preempt_goal)


@pytest.mark.launch(fixture=stretch_driver_ld)
def test_action_cancel_latency(benchmark, action_client):
    """
    Benchmark the time it takes to cancel a goal
    """
    def cancel_goal():
        future = action_client.move_to_configuration({'lift_joint': 0.8}, blocking=False)
        gh = wait_for(future)
        assert gh.accepted, "Goal was rejected by server"
        time.sleep(0.1)
        wait_for(gh.cancel_goal_async())
        wait_for(gh.get_result_async())

    benchmark(cancel_goal)

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_action_error_threshold_closeness(benchmark, action_client):
    """
    Benchmark the error (distance from target) and time it takes to settle
    """
    def settle_to_goal():
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


def test_driver_control_loop_benchmark(benchmark):
    """
    This is tricky without hardware, but we can benchmark the python-side iteration latency
    if we mock the hardware pull/push calls.
    We skip actual hardware init for unit test isolation.
    """
    import unittest.mock as mock

    rclpy.init()
    try:
        with mock.patch('stretch4_body.robot.robot_client.RobotClient') as MockRobot:
            # Setup mock
            mock_inst = MockRobot.return_value
            mock_inst.startup.return_value = True
            # Per-joint fields the driver's diagnostics read.
            joint_diagnostics = {'at_limit': False, 'soft_motion_limits': [None, None],
                                 'braking_distance': 0.0, 'in_collision_stop': False}
            mock_inst.status = {
                'server': {'lease_holder': 'ros2_driver', 'lease_holder_priority': 0, 'lease_expiry': 0.0},
                'omnibase': {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'x_vel': 0.0, 'y_vel': 0.0, 'theta_vel': 0.0},
                'lift': {'motor': {'pos_calibrated': 1, 'is_mg_moving': 0}, 'pos': 0.0, 'vel': 0.0, 'effort': 0.0, **joint_diagnostics},
                'arm': {'motor': {'is_mg_moving': 0, 'effort_pct': 0.0}, 'pos': 0.0, 'vel': 0.0, 'effort': 0.0, **joint_diagnostics},
                'power_periph': {
                    'runstop_event': 0,
                    'voltage': 26.0,
                    'battery_current': 1.0,
                    'temp': 30.0,
                    'battery_soc': 100.0,
                    'adapter_voltage_present': True,
                    'charger_is_charging': False
                },
                'end_of_arm': {
                    'wrist_yaw': {'pos': 0.0, 'vel': 0.0, 'effort': 0.0},
                    'wrist_pitch': {'pos': 0.0, 'vel': 0.0, 'effort': 0.0},
                    'wrist_roll': {'pos': 0.0, 'vel': 0.0, 'effort': 0.0}
                },
            }
            mock_inst.routines.status = {'active_routine': 'routine_nop'}
            mock_inst.params = {'tool': 'tool_none'}
            # The driver reads joint limits and motion profiles from robot_params; those come
            # from this robot's parameter files, which need no hardware.
            from stretch4_body.core.robot_params import RobotParams
            mock_inst.robot_params = RobotParams.get_params()[1]

            driver = StretchDriver()

            # Benchmark a single control loop iteration
            def single_loop_tick():
                driver.control_loop()

            benchmark(single_loop_tick)
    finally:
        rclpy.shutdown()

