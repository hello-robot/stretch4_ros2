import pytest
import rclpy
import time
from common.launch_descriptions import stretch_driver_ld
from common.client_nodes.mock_subscriber import MockSubscriberNode
from common.client_nodes.fjt_client import FJTClient
from stretch_core.command_groups import (
    ArmCommandGroup, LiftCommandGroup, MobileBaseCommandGroup,
    WristYawCommandGroup, WristPitchCommandGroup, WristRollCommandGroup, GripperCommandGroup
)
from stretch_core.stretch_driver import StretchDriver

@pytest.fixture
def subscriber_node():
    rclpy.init()
    node = MockSubscriberNode('perf_sub_node')
    yield node
    node.stop()

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
            node.wait_for_message('joint_states', timeout=1.0)
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
            node.wait_for_message('battery', timeout=1.0)
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
            node.wait_for_message('diagnostics', timeout=1.0)
            count += 1
    
    benchmark(measure_diagnostic_states, subscriber_node)
    
@pytest.mark.launch(fixture=stretch_driver_ld)
def test_action_preemption_latency(benchmark, action_client):
    """
    Benchmark the time it takes to preempt a goal with a new one
    """
    def preempt_goal():
        action_client.mode = 'position'
        future1 = action_client.move_to_configuration({'joint_lift': 0.8}, blocking=False)
        time.sleep(0.1)
        future2 = action_client.move_to_configuration({'joint_lift': 0.3}, blocking=False)
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
        future = action_client.move_to_configuration({'joint_lift': 0.8}, blocking=False)
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
        q = {'joint_lift': target}

        # blocking=True means it waits until is_finished() is True via the server
        result = action_client.move_to_configuration(q, blocking=True)
        assert result.status == 4 # Succeeded

        # Verify closeness
        # (Assuming client updates q_curr)
        actual = action_client.q_curr.get('joint_lift', 0.0)
        error = abs(target - actual)
        assert error < 0.05, f"Error {error} is outside of acceptable threshold"

    benchmark(settle_to_goal)


def test_command_group_monitor_execution_time(benchmark):
    """
    Unit test benchmark: how fast are the check/monitor functions inside command_groups?
    This is extremely important for the 100Hz loop.
    """
    groups = [
        ArmCommandGroup(), LiftCommandGroup(), MobileBaseCommandGroup(),
        WristYawCommandGroup(), WristPitchCommandGroup(), WristRollCommandGroup(), GripperCommandGroup()
    ]

    # Mock status dict
    robot_status = {
        'arm': {'pos': 0.1, 'motor': {'is_mg_moving': 0}},
        'lift': {'pos': 0.2, 'motor': {'is_mg_moving': 0}},
        'omnibase': {'x': 0.0, 'wheel_0': {'is_mg_moving': 0}, 'wheel_1': {'is_mg_moving': 0}, 'wheel_2': {'is_mg_moving': 0}},
        'end_of_arm': {
            'wrist_yaw': {'pos': 0.0},
            'wrist_pitch': {'pos': 0.0},
            'wrist_roll': {'pos': 0.0}
        },
        'gripper': {'pos': 0.0}
    }

    for g in groups:
        # Give them fake goals
        g.active = True
        if g.name == 'translate_mobile_base':
            g.initx = 0.0
        g.goal = {'position': 0.0}

    def run_all_monitors():
        for g in groups:
            g.monitor_execution(robot_status)
            g.is_finished(robot_status)

    benchmark(run_all_monitors)

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
            mock_inst.status = {
                'omnibase': {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'x_vel': 0.0, 'y_vel': 0.0, 'theta_vel': 0.0},
                'lift': {'motor': {'pos_calibrated': 1, 'is_mg_moving': 0}, 'pos': 0.0, 'vel': 0.0, 'effort': 0.0},
                'arm': {'motor': {'is_mg_moving': 0, 'effort_pct': 0.0}, 'pos': 0.0, 'vel': 0.0, 'effort': 0.0},
                'power_periph': {
                    'runstop_event': 0,
                    'voltage': 26.0,
                    'battery_current': 1.0,
                    'temp': 30.0,
                    'battery_soc': 100.0,
                    'charger_connected': True,
                    'charger_is_charging': False
                },
                'end_of_arm': {
                    'wrist_yaw': {'pos': 0.0, 'vel': 0.0, 'effort': 0.0},
                    'wrist_pitch': {'pos': 0.0, 'vel': 0.0, 'effort': 0.0},
                    'wrist_roll': {'pos': 0.0, 'vel': 0.0, 'effort': 0.0}
                },
                'gripper': {'pos': 0.0, 'vel': 0.0, 'effort': 0.0}
            }
            mock_inst.params = {'tool': 'tool_none'}
            mock_inst.robot_params = {'ros': {'joints': []}}

            driver = StretchDriver()

            # Benchmark a single control loop iteration
            def single_loop_tick():
                driver.control_loop()

            benchmark(single_loop_tick)
    finally:
        rclpy.shutdown()

