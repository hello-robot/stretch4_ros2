from collections.abc import Generator
import time

from common.client_nodes.fjt_client import FJTClient
from common.client_nodes.param_client import ParamClient
from common.launch_descriptions import stretch_driver_ld
import pytest


@pytest.fixture
def action_client(require_homed_robot: None) -> Generator[FJTClient, None, None]:
    """
    Create and yield an FJTClient after verifying the robot is homed.

    Args:
        require_homed_robot: Fixture ensuring the robot is homed before motion tests.

    Yields:
        An active FJTClient instance.
    """
    node = FJTClient()
    yield node
    node.destroy_node()


@pytest.fixture
def gripper_limits() -> Generator[tuple[float, float], None, None]:
    """
    Yield the driver's (lower, upper) limits for gripper_joint.

    The range depends on the attached tool, so it is read from the driver's
    joint_limit.gripper_joint.{lower,upper} parameters rather than assumed.
    Skips the test when the tool has no actuated gripper.
    """
    names = ['joint_limit.gripper_joint.lower', 'joint_limit.gripper_joint.upper']
    node = ParamClient('gripper_limits_param_client')
    try:
        values = node.get_parameters(names)
    finally:
        node.destroy_node()
    if values is None:
        pytest.fail('Could not read gripper_joint limits from the driver')
    if len(values) != len(names):
        pytest.skip('Attached tool has no actuated gripper_joint')
    yield values[names[0]], values[names[1]]

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_fjt_success_single_joint_arm(action_client):
    q = {'arm_joint': 0.1}
    result = action_client.move_to_configuration(q, blocking=True)
    assert result.status == 4 # STATUS_SUCCEEDED

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_fjt_success_single_joint_lift(action_client):
    q = {'lift_joint': 0.3}
    result = action_client.move_to_configuration(q, blocking=True)
    assert result.status == 4

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_fjt_success_wrist_yaw(action_client):
    q = {'wrist_yaw_joint': 0.1}
    result = action_client.move_to_configuration(q, blocking=True)
    assert result.status == 4

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_fjt_success_wrist_pitch(action_client):
    q = {'wrist_pitch_joint': 0.1}
    result = action_client.move_to_configuration(q, blocking=True)
    assert result.status == 4

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_fjt_success_wrist_roll(action_client):
    q = {'wrist_roll_joint': 0.1}
    result = action_client.move_to_configuration(q, blocking=True)
    assert result.status == 4

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_fjt_success_gripper(action_client, gripper_limits):
    # one finger's URDF position (lower limit is closed); the range depends on the tool
    lower, upper = gripper_limits
    q = {'gripper_joint': lower + 0.75 * (upper - lower)}
    result = action_client.move_to_configuration(q, blocking=True)
    assert result.status == 4

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_fjt_success_all_joints_simultaneously(action_client, gripper_limits):
    lower, upper = gripper_limits
    q = {
        'lift_joint': 0.6,
        'arm_joint': 0.2,
        'wrist_yaw_joint': 0.0,
        'wrist_pitch_joint': 0.0,
        'wrist_roll_joint': 0.0,
        'gripper_joint': lower + 0.5 * (upper - lower),
    }
    result = action_client.move_to_configuration(q, blocking=True)
    assert result.status == 4

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_fjt_preempted(action_client):
    q1 = {'lift_joint': 0.8}
    future1 = action_client.move_to_configuration(q1, blocking=False)
    
    time.sleep(0.5) # Let first goal start
    
    q2 = {'lift_joint': 0.3}
    future2 = action_client.move_to_configuration(q2, blocking=False)
    
    # The first goal should be preempted
    res1 = action_client._wait_for_result(future1)
    assert res1.status == 6 # STATUS_ABORTED

    res2 = action_client._wait_for_result(future2)
    assert res2.status == 4 # STATUS_SUCCEEDED

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_fjt_canceled(action_client):
    q = {'lift_joint': 0.2}
    action_client.move_to_configuration(q, blocking=True)

    q = {'lift_joint': 1.1}
    future = action_client.move_to_configuration(q, blocking=False)
    time.sleep(0.5)
    action_client.cancel_goal()
    
    res = action_client._wait_for_result(future)
    assert res.status == 5 # STATUS_CANCELED
