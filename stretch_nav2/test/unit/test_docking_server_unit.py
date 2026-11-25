"""Unit tests for DockingServer and DockRobotAction cancellation and stop handling."""

import threading
import time
import unittest.mock as mock

from nav2_msgs.action import DockRobot
import pytest
from std_msgs.msg import Empty

from stretch_nav2.docking_server import (
    AutodockState,
    Cancelled,
    DockFailure,
    DockingServer,
    DockRobotAction,
    Preempted,
)


def _make_dock_action() -> tuple[DockRobotAction, mock.MagicMock]:
    """
    Instantiate a real DockRobotAction backed by a mocked DockingServer node.

    Args:
        None.

    Returns:
        Tuple of (DockRobotAction instance, mocked parent DockingServer node).
    """
    node = mock.MagicMock()
    node.rate = mock.MagicMock()
    node.create_rate.return_value = node.rate
    node.stop_base = mock.MagicMock()
    node.stop_robot = mock.MagicMock()
    node.cmd_vel_pub = mock.MagicMock()
    node.full_points = 0
    node.servo_lost_count = 0
    node.servo_done = threading.Event()
    with mock.patch('stretch_nav2.docking_server.ActionServer'):
        action = DockRobotAction(node)
    node.action = action
    return action, node


def test_stop_event_cb_sets_flag_only_when_active() -> None:
    """
    Verify stop_event_cb sets stop_requested only when a goal is active and not already aborting.

    Args:
        None.

    Returns:
        None.
    """
    action, node = _make_dock_action()

    # Idle: stop_the_robot_event is ignored.
    action.goal_idle.set()
    DockingServer.stop_event_cb(node, Empty())
    assert not action.stop_requested.is_set()

    # Self-triggered abort in progress: stop_the_robot_event is ignored.
    action.goal_idle.clear()
    action._aborting.set()
    DockingServer.stop_event_cb(node, Empty())
    assert not action.stop_requested.is_set()

    # Active goal: stop_the_robot_event sets stop_requested.
    action._aborting.clear()
    DockingServer.stop_event_cb(node, Empty())
    assert action.stop_requested.is_set()


def test_check_interrupts_cancel_preempt_and_stop() -> None:
    """
    Verify check_interrupts raises Cancelled, Preempted, or DockFailure for each interrupt path.

    Args:
        None.

    Returns:
        None.
    """
    action, _ = _make_dock_action()
    my_uuid = b'\x01' * 16
    action.latest_goal_uuid = my_uuid
    goal_handle = mock.MagicMock()
    goal_handle.is_cancel_requested = False

    # Normal execution: no exception.
    action.check_interrupts(goal_handle, my_uuid)

    # Action cancel requested.
    goal_handle.is_cancel_requested = True
    with pytest.raises(Cancelled):
        action.check_interrupts(goal_handle, my_uuid)
    goal_handle.is_cancel_requested = False

    # Preempted by newer goal UUID.
    action.latest_goal_uuid = b'\x02' * 16
    with pytest.raises(Preempted):
        action.check_interrupts(goal_handle, my_uuid)
    action.latest_goal_uuid = my_uuid

    # Stopped externally via /stop_the_robot.
    action.stop_requested.set()
    with pytest.raises(DockFailure) as exc_info:
        action.check_interrupts(goal_handle, my_uuid)
    assert exc_info.value.error_code == DockRobot.Result.UNKNOWN


def test_abort_active_phase_servoing_freewheels_base() -> None:
    """
    Verify _abort_active_phase calls freewheel_the_robot when leaving SERVOING.

    Args:
        None.

    Returns:
        None.
    """
    action, node = _make_dock_action()

    action._abort_active_phase(AutodockState.SERVOING)

    node.stop_base.call_async.assert_called_once()
    node.stop_robot.call_async.assert_not_called()


def test_abort_active_phase_stowing_stops_and_waits() -> None:
    """
    Verify _abort_active_phase stops an in-flight stow once and waits for stow_future to resolve.

    Args:
        None.

    Returns:
        None.
    """
    action, node = _make_dock_action()
    stow_future = mock.MagicMock()
    stow_future.done.side_effect = [False, False, True]
    action.stow_future = stow_future

    action._abort_active_phase(AutodockState.STOWING)

    node.stop_robot.call_async.assert_called_once()
    assert not action._aborting.is_set()
    assert node.rate.sleep.call_count >= 1


def test_abort_active_phase_blind_docking_skips_duplicate_stop_when_already_stopped() -> None:
    """
    Verify _abort_active_phase waits on seating_future without re-calling stop_the_robot when
    stop_requested was already triggered externally.

    Args:
        None.

    Returns:
        None.
    """
    action, node = _make_dock_action()
    seating_future = mock.MagicMock()
    seating_future.done.side_effect = [False, True]
    action.seating_future = seating_future
    action.stop_requested.set()

    action._abort_active_phase(AutodockState.BLIND_DOCKING)

    node.stop_robot.call_async.assert_not_called()
    assert not action._aborting.is_set()


def test_step_servoing_early_exit_when_dock_not_seen() -> None:
    """
    Verify step_SERVOING raises FAILED_TO_DETECT_DOCK when dock is not seen and staging is disabled.

    Args:
        None.

    Returns:
        None.
    """
    action, node = _make_dock_action()
    goal = DockRobot.Goal()
    goal.navigate_to_staging_pose = False
    action.pending_goal = goal
    action.servo_deadline = time.time() + 25.0
    node.servo_lost_count = 10

    with pytest.raises(DockFailure) as exc_info:
        action.step_SERVOING()
    assert exc_info.value.error_code == DockRobot.Result.FAILED_TO_DETECT_DOCK


def test_cloud_cb_suppresses_cmd_vel_when_stop_requested() -> None:
    """
    Verify cloud_cb returns immediately without publishing /cmd_vel_stiff when stop_requested is set.

    Args:
        None.

    Returns:
        None.
    """
    action, node = _make_dock_action()
    action.machine.set_state(AutodockState.SERVOING)
    action.stop_requested.set()

    right_cloud = mock.MagicMock(width=100, height=1)
    left_cloud = mock.MagicMock(width=100, height=1)

    DockingServer.cloud_cb(node, right_cloud, left_cloud)

    node.cmd_vel_pub.publish.assert_not_called()
