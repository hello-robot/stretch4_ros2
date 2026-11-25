from typing import Any
import unittest.mock as mock

from std_srvs.srv import Trigger

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


def _make_mock_robot() -> mock.MagicMock:
    """
    Create a mock RobotClient configured with minimal status and params for StretchDriver.

    Args:
        None.

    Returns:
        A configured MagicMock standing in for stretch4_body.robot.robot_client.RobotClient.
    """
    mock_inst = mock.MagicMock()
    mock_inst.startup.return_value = True
    mock_inst.is_homed.return_value = True
    mock_inst.lift.is_homed.return_value = True
    mock_inst.arm.is_homed.return_value = True
    mock_inst.end_of_arm.joints = ['wrist_yaw', 'wrist_pitch', 'wrist_roll']
    mock_inst.end_of_arm.is_homed.return_value = True
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
        'wrist_yaw': {'motion': {'default': {'accel': 1.0, 'vel': 1.0}}},
        'wrist_pitch': {'motion': {'default': {'accel': 1.0, 'vel': 1.0}}},
        'wrist_roll': {'motion': {'default': {'accel': 1.0, 'vel': 1.0}}},
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
    return mock_inst


def test_command_group_monitor_execution_time(benchmark: Any) -> None:
    """
    Benchmark command_groups monitor_execution and is_finished for the 100Hz loop.

    Args:
        benchmark: The pytest-benchmark fixture.

    Returns:
        None.
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
        """Execute monitor_execution and is_finished across all command groups."""
        for g in groups:
            g.monitor_execution(robot_status)
            g.is_finished(robot_status)

    benchmark(run_all_monitors)


def test_driver_control_loop_benchmark(benchmark: Any) -> None:
    """
    Benchmark the python-side StretchDriver control loop iteration latency with mocked hardware.

    Args:
        benchmark: The pytest-benchmark fixture.

    Returns:
        None.
    """
    with mock.patch('stretch4_body.robot.robot_client.RobotClient', return_value=_make_mock_robot()):
        driver = StretchDriver()
        try:
            def single_loop_tick() -> None:
                """Run one control_loop iteration."""
                driver.control_loop()

            benchmark(single_loop_tick)
        finally:
            driver.destroy_node()


def test_stop_the_robot_cancels_routine_and_stops_actuators() -> None:
    """
    Verify stop_the_robot_callback cancels active priority routines, stops all actuators,
    clears the active trajectory goal UUID, and publishes stop_the_robot_event.

    Args:
        None.

    Returns:
        None.
    """
    mock_robot = _make_mock_robot()
    with mock.patch('stretch4_body.robot.robot_client.RobotClient', return_value=mock_robot):
        driver = StretchDriver()
        try:
            driver.driver_mode = 'stowing'
            driver.joint_trajectory_action.latest_goal_uuid = b'1' * 16
            driver.stop_event_pub = mock.MagicMock()

            response = driver.stop_the_robot_callback(Trigger.Request(), Trigger.Response())

            assert response.success is True
            assert driver.joint_trajectory_action.latest_goal_uuid is None
            assert driver._routine_canceled is True
            mock_robot.routines.cancel_routine.assert_called_once_with(do_push=False)
            mock_robot.omnibase.hard_stop.assert_called_once()
            mock_robot.lift.move_by.assert_called_once_with(0.0)
            mock_robot.arm.move_by.assert_called_once_with(0.0)
            assert mock_robot.end_of_arm.hold_position.call_count == 3
            driver.stop_event_pub.publish.assert_called_once()
        finally:
            driver.destroy_node()


def test_stow_canceled_stops_actuators_and_restores_mode() -> None:
    """
    Verify stow_the_robot queues actuator stops, restores prior mode, and returns False on cancel.

    Args:
        None.

    Returns:
        None.
    """
    mock_robot = _make_mock_robot()
    with mock.patch('stretch4_body.robot.robot_client.RobotClient', return_value=mock_robot):
        driver = StretchDriver()
        try:
            def fake_stow(**_kwargs: Any) -> tuple[bool, int]:
                """Simulate an external stop_the_robot canceling the routine mid-flight."""
                driver._routine_canceled = True
                return False, 1

            mock_robot.routines.routine_robot_stow.side_effect = fake_stow

            succeeded, msg = driver.stow_the_robot()

            assert succeeded is False
            assert 'canceled' in msg.lower()
            assert driver.driver_mode == 'navigation'
            mock_robot.omnibase.hard_stop.assert_called_once()
            mock_robot.arm.move_by.assert_called_once_with(0.0)
            mock_robot.lift.move_by.assert_called_once_with(0.0)
        finally:
            driver.destroy_node()


def test_home_canceled_stops_actuators_and_restores_mode() -> None:
    """
    Verify home_the_robot queues actuator stops, restores prior mode, and returns False on cancel.

    Args:
        None.

    Returns:
        None.
    """
    mock_robot = _make_mock_robot()
    with mock.patch('stretch4_body.robot.robot_client.RobotClient', return_value=mock_robot):
        driver = StretchDriver()
        try:
            def fake_home(**_kwargs: Any) -> tuple[bool, int]:
                """Simulate an external stop_the_robot canceling homing mid-flight."""
                driver._routine_canceled = True
                return False, 1

            mock_robot.routines.routine_robot_home.side_effect = fake_home

            succeeded, msg = driver.home_the_robot()

            assert succeeded is False
            assert 'canceled' in msg.lower()
            assert driver.driver_mode == 'navigation'
            mock_robot.omnibase.hard_stop.assert_called_once()
            mock_robot.arm.move_by.assert_called_once_with(0.0)
            mock_robot.lift.move_by.assert_called_once_with(0.0)
        finally:
            driver.destroy_node()


def test_seat_into_dock_canceled_freewheels_and_restores_mode() -> None:
    """
    Verify seat_into_dock freewheels the base, restores prior mode, and returns False on cancel.

    Args:
        None.

    Returns:
        None.
    """
    mock_robot = _make_mock_robot()
    with mock.patch('stretch4_body.robot.robot_client.RobotClient', return_value=mock_robot):
        driver = StretchDriver()
        try:
            def fake_blind_dock(**_kwargs: Any) -> tuple[bool, int]:
                """Simulate blind docking interrupted by stop_the_robot."""
                driver._routine_canceled = True
                return True, 1

            mock_robot.routines.routine_blind_dock.side_effect = fake_blind_dock

            succeeded, msg = driver.seat_into_dock()

            assert succeeded is False
            assert msg == 'Failure'
            assert driver.driver_mode == 'navigation'
            mock_robot.omnibase.enable_freewheel_mode.assert_called_once()
        finally:
            driver.destroy_node()
