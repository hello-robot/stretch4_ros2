import unittest.mock as mock

from stretch_core.command_groups import (
    ArmCommandGroup,
    GripperCommandGroup,
    LiftCommandGroup,
    MobileBaseCommandGroup,
    WristPitchCommandGroup,
    WristRollCommandGroup,
    WristYawCommandGroup,
)
from stretch_core.stretch_driver import StretchDriver


def test_command_group_monitor_execution_time(benchmark) -> None:
    """
    Benchmark command_groups monitor_execution and is_finished for the 100Hz loop.

    Args:
        benchmark: The pytest-benchmark fixture.
    """
    groups = [
        ArmCommandGroup(),
        LiftCommandGroup(),
        MobileBaseCommandGroup(),
        WristYawCommandGroup(),
        WristPitchCommandGroup(),
        WristRollCommandGroup(),
        GripperCommandGroup(),
    ]

    robot_status = {
        'arm': {'pos': 0.1, 'motor': {'is_mg_moving': 0, 'in_guarded_event': False}},
        'lift': {'pos': 0.2, 'motor': {'is_mg_moving': 0, 'in_guarded_event': False}},
        'omnibase': {
            'x': 0.0,
            'wheel_0': {'is_mg_moving': 0},
            'wheel_1': {'is_mg_moving': 0},
            'wheel_2': {'is_mg_moving': 0},
        },
        'end_of_arm': {
            'wrist_yaw': {'pos': 0.0},
            'wrist_pitch': {'pos': 0.0},
            'wrist_roll': {'pos': 0.0},
        },
        'gripper': {'pos': 0.0},
    }

    for g in groups:
        g.active = True
        if g.name == 'translate_mobile_base':
            g.initx = 0.0
        g.goal = {'position': 0.0}

    def run_all_monitors() -> None:
        for g in groups:
            g.monitor_execution(robot_status)
            g.is_finished(robot_status)

    benchmark(run_all_monitors)


def test_driver_control_loop_benchmark(benchmark) -> None:
    """
    Benchmark the python-side StretchDriver control loop iteration latency with mocked hardware.

    Args:
        benchmark: The pytest-benchmark fixture.
    """
    with mock.patch('stretch4_body.robot.robot_client.RobotClient') as MockRobot:
        mock_inst = MockRobot.return_value
        mock_inst.startup.return_value = True
        mock_inst.status = {
            'server': {
                'lease_holder': 'ros2_driver',
                'lease_holder_priority': 1,
                'lease_expiry': 0.0,
            },
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
                'charger_is_charging': False,
                'adapter_voltage_present': True,
            },
            'end_of_arm': {
                'wrist_yaw': {'pos': 0.0, 'vel': 0.0, 'effort': 0.0},
                'wrist_pitch': {'pos': 0.0, 'vel': 0.0, 'effort': 0.0},
                'wrist_roll': {'pos': 0.0, 'vel': 0.0, 'effort': 0.0},
            },
            'gripper': {'pos': 0.0, 'vel': 0.0, 'effort': 0.0},
        }
        mock_inst.routines.status = {'active_routine': 'routine_nop'}
        mock_inst.params = {'tool': 'tool_none'}
        mock_inst.robot_params = {
            'ros': {'joints': []},
            'lift': {'motion': {'default': {'accel_m': 0.2, 'vel_m': 0.1}}},
            'arm': {'motion': {'default': {'accel_m': 0.2, 'vel_m': 0.1}}},
            'omnibase': {
                'motion': {
                    'default': {
                        'accel_xy_m': 0.5,
                        'accel_w_r': 1.0,
                        'vel_xy_m': 0.3,
                        'vel_w_r': 0.5,
                    }
                }
            },
        }

        driver = StretchDriver()
        try:
            def single_loop_tick() -> None:
                driver.control_loop()

            benchmark(single_loop_tick)
        finally:
            driver.destroy_node()
