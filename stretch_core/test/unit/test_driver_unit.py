import unittest.mock as mock

from stretch4_body.core.robot_params import RobotParams
from stretch_core.stretch_driver import StretchDriver


def test_driver_control_loop_benchmark(benchmark) -> None:
    """
    Benchmark the python-side StretchDriver control loop iteration latency with mocked hardware.

    Args:
        benchmark: The pytest-benchmark fixture.
    """
    with mock.patch('stretch4_body.robot.robot_client.RobotClient') as MockRobot:
        mock_inst = MockRobot.return_value
        mock_inst.startup.return_value = True
        # Per-joint fields the driver's diagnostics read.
        joint_diagnostics = {
            'at_limit': False,
            'soft_motion_limits': [None, None],
            'braking_distance': 0.0,
            'in_collision_stop': False,
        }
        mock_inst.status = {
            'server': {
                'lease_holder': 'ros2_driver',
                'lease_holder_priority': 1,
                'lease_expiry': 0.0,
            },
            'omnibase': {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'x_vel': 0.0, 'y_vel': 0.0, 'theta_vel': 0.0},
            'lift': {'motor': {'pos_calibrated': 1, 'is_mg_moving': 0}, 'pos': 0.0, 'vel': 0.0, 'effort': 0.0,
                     **joint_diagnostics},
            'arm': {'motor': {'is_mg_moving': 0, 'effort_pct': 0.0}, 'pos': 0.0, 'vel': 0.0, 'effort': 0.0,
                    **joint_diagnostics},
            'power_periph': {
                'runstop_event': 0,
                'voltage': 26.0,
                'battery_current': 1.0,
                'temp': 30.0,
                'battery_soc': 100.0,
                'charger_is_charging': False,
                'adapter_voltage_present': True,
            },
            'end_of_arm': {
                'wrist_yaw': {'pos': 0.0, 'vel': 0.0, 'effort': 0.0},
                'wrist_pitch': {'pos': 0.0, 'vel': 0.0, 'effort': 0.0},
                'wrist_roll': {'pos': 0.0, 'vel': 0.0, 'effort': 0.0},
            },
        }
        mock_inst.routines.status = {'active_routine': 'routine_nop'}
        mock_inst.params = {'tool': 'tool_none'}
        # The driver reads joint limits, motion profiles and tool metadata from robot_params;
        # those come from this robot's parameter files, which need no hardware.
        mock_inst.robot_params = RobotParams.get_params()[1]

        driver = StretchDriver()
        try:
            def single_loop_tick() -> None:
                driver.control_loop()

            benchmark(single_loop_tick)
        finally:
            driver.destroy_node()
