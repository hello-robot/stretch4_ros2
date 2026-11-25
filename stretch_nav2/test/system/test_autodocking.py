"""Hardware-in-the-loop system tests for /dock_robot cancellation and /stop_the_robot."""

from collections.abc import Generator
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_pytest
from launch_ros.actions import Node
from nav2_msgs.action import DockRobot
import pytest
from sensor_msgs.msg import BatteryState

from common.dock_client import DockClient


@launch_pytest.fixture(scope='module')
def autodocking_ld() -> launch.LaunchDescription:
    """
    Launch stretch_driver, dual_hesai, and docking_server.py once per test module.

    Args:
        None.

    Returns:
        LaunchDescription for the autodocking hardware stack.
    """
    stretch_core_share = Path(get_package_share_directory('stretch_core'))
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                str(stretch_core_share / 'launch' / 'stretch_driver.launch.py'),
            ]),
            launch_arguments={'action_timeout': '10.0'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                str(stretch_core_share / 'launch' / 'dual_hesai.launch.py'),
            ])
        ),
        Node(
            package='stretch_nav2',
            executable='docking_server.py',
            output='screen',
        ),
        launch_pytest.actions.ReadyToTest(),
    ])


@pytest.fixture(scope='module')
def dock_client(require_homed_robot: None) -> Generator[DockClient, None, None]:
    """
    Create and yield a module-scoped DockClient after confirming the robot is homed.

    Args:
        require_homed_robot: Fixture verifying /is_homed is True.

    Returns:
        Generator yielding a connected DockClient instance.
    """
    client = DockClient()
    try:
        client.wait_for_ready()
        if client.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
            pytest.skip(
                'Robot is already charging; place it ~1.0 m in front of the dock before running test_autodocking.'
            )
        yield client
    finally:
        client.stop()


def _extend_arm_out_of_stow(client: DockClient) -> None:
    """
    Extend the telescoping arm to 0.25 m so /dock_robot enters STOWING rather than skipping it.

    Args:
        client: Active DockClient instance.

    Returns:
        None.
    """
    assert client.wait_until(lambda: client.mode == 'navigation', timeout_sec=5.0)
    assert client.move_joints({'arm_joint': 0.25}) == GoalStatus.STATUS_SUCCEEDED


@pytest.mark.launch(fixture=autodocking_ld)
def test_dock_cancel_during_stowing(dock_client: DockClient) -> None:
    """
    Verify wildcard CancelGoal aborts stowing, restores navigation mode, and stops mid-stroke.

    Args:
        dock_client: Connected DockClient fixture.

    Returns:
        None.
    """
    _extend_arm_out_of_stow(dock_client)
    handle = dock_client.send_dock_goal()
    assert dock_client.wait_until(lambda: dock_client.mode == 'stowing', timeout_sec=5.0)
    time.sleep(0.5)

    cancel_res = dock_client.cancel_wildcard()
    assert cancel_res.return_code == CancelGoal.Response.ERROR_NONE
    assert len(cancel_res.goals_canceling) == 1

    status, result = dock_client.wait_result(handle, timeout_sec=8.0)
    assert status == GoalStatus.STATUS_CANCELED
    assert result.success is False
    assert dock_client.wait_until(lambda: dock_client.mode == 'navigation', timeout_sec=5.0)
    assert dock_client.arm_pos is not None and dock_client.arm_pos > 0.03


@pytest.mark.launch(fixture=autodocking_ld)
def test_dock_stop_during_stowing(dock_client: DockClient) -> None:
    """
    Verify /stop_the_robot aborts stowing, restores navigation mode, and stops mid-stroke.

    Args:
        dock_client: Connected DockClient fixture.

    Returns:
        None.
    """
    _extend_arm_out_of_stow(dock_client)
    handle = dock_client.send_dock_goal()
    assert dock_client.wait_until(lambda: dock_client.mode == 'stowing', timeout_sec=5.0)
    time.sleep(0.5)

    stop_res = dock_client.stop_the_robot()
    assert stop_res.success is True

    status, result = dock_client.wait_result(handle, timeout_sec=8.0)
    assert status == GoalStatus.STATUS_ABORTED
    assert result.success is False
    assert dock_client.wait_until(lambda: dock_client.mode == 'navigation', timeout_sec=5.0)
    assert dock_client.arm_pos is not None and dock_client.arm_pos > 0.03


@pytest.mark.launch(fixture=autodocking_ld)
def test_dock_cancel_during_servoing(dock_client: DockClient) -> None:
    """
    Verify wildcard CancelGoal cancels servoing and stops base motion cleanly.

    Args:
        dock_client: Connected DockClient fixture.

    Returns:
        None.
    """
    stow_res = dock_client.stow_the_robot()
    assert stow_res.success is True, f'/stow_the_robot failed: {stow_res.message}'

    handle = dock_client.send_dock_goal()
    assert dock_client.wait_until(
        lambda: dock_client.feedback_state == DockRobot.Feedback.CONTROLLING, timeout_sec=10.0
    ), 'Did not enter SERVOING (CONTROLLING); ensure charging dock is visible.'
    time.sleep(0.5)

    cancel_res = dock_client.cancel_wildcard()
    assert cancel_res.return_code == CancelGoal.Response.ERROR_NONE
    assert len(cancel_res.goals_canceling) == 1

    status, result = dock_client.wait_result(handle, timeout_sec=8.0)
    assert status == GoalStatus.STATUS_CANCELED
    assert result.success is False
    dock_client.assert_base_stopped()
    assert dock_client.mode == 'navigation'


@pytest.mark.launch(fixture=autodocking_ld)
def test_dock_stop_during_servoing(dock_client: DockClient) -> None:
    """
    Verify /stop_the_robot aborts servoing and stops base motion cleanly.

    Args:
        dock_client: Connected DockClient fixture.

    Returns:
        None.
    """
    handle = dock_client.send_dock_goal()
    assert dock_client.wait_until(
        lambda: dock_client.feedback_state == DockRobot.Feedback.CONTROLLING, timeout_sec=10.0
    ), 'Did not enter SERVOING (CONTROLLING); ensure charging dock is visible.'
    time.sleep(0.5)

    stop_res = dock_client.stop_the_robot()
    assert stop_res.success is True

    status, result = dock_client.wait_result(handle, timeout_sec=8.0)
    assert status == GoalStatus.STATUS_ABORTED
    assert result.success is False
    dock_client.assert_base_stopped()
    assert dock_client.mode == 'navigation'


@pytest.mark.launch(fixture=autodocking_ld)
def test_dock_cancel_during_blind_docking(dock_client: DockClient) -> None:
    """
    Verify wildcard CancelGoal cancels blind docking, restores navigation mode, and stops base.

    Args:
        dock_client: Connected DockClient fixture.

    Returns:
        None.
    """
    handle = dock_client.send_dock_goal()
    assert dock_client.wait_until(
        lambda: (
            dock_client.mode == 'seating'
            or dock_client.feedback_state == DockRobot.Feedback.WAIT_FOR_CHARGE
        ),
        timeout_sec=30.0,
    ), 'Did not reach BLIND_DOCKING (mode=seating / WAIT_FOR_CHARGE) within 30s.'

    cancel_res = dock_client.cancel_wildcard()
    assert cancel_res.return_code == CancelGoal.Response.ERROR_NONE
    assert len(cancel_res.goals_canceling) == 1

    status, result = dock_client.wait_result(handle, timeout_sec=8.0)
    assert status == GoalStatus.STATUS_CANCELED
    assert result.success is False
    assert dock_client.wait_until(lambda: dock_client.mode == 'navigation', timeout_sec=5.0)
    dock_client.assert_base_stopped()


@pytest.mark.launch(fixture=autodocking_ld)
def test_dock_stop_during_blind_docking(dock_client: DockClient) -> None:
    """
    Verify /stop_the_robot aborts blind docking, restores navigation mode, and stops base.

    Args:
        dock_client: Connected DockClient fixture.

    Returns:
        None.
    """
    if dock_client.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
        pytest.skip('Robot is already charging on the dock.')

    handle = dock_client.send_dock_goal()
    assert dock_client.wait_until(
        lambda: (
            dock_client.mode == 'seating'
            or dock_client.feedback_state == DockRobot.Feedback.WAIT_FOR_CHARGE
        ),
        timeout_sec=30.0,
    ), 'Did not reach BLIND_DOCKING (mode=seating / WAIT_FOR_CHARGE) within 30s.'

    stop_res = dock_client.stop_the_robot()
    assert stop_res.success is True

    status, result = dock_client.wait_result(handle, timeout_sec=8.0)
    assert status == GoalStatus.STATUS_ABORTED
    assert result.success is False
    assert dock_client.wait_until(lambda: dock_client.mode == 'navigation', timeout_sec=5.0)
    dock_client.assert_base_stopped()
