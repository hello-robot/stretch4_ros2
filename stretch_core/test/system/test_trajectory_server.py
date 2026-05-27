import pytest
import rclpy
import time
from common.launch_descriptions import stretch_driver_ld
from common.client_nodes.fjt_client import FJTClient
from trajectory_msgs.msg import JointTrajectoryPoint

@pytest.fixture
def action_client():
    rclpy.init()
    node = FJTClient()
    yield node
    node.destroy_node()
    rclpy.shutdown()

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
def test_fjt_success_mobile_base(action_client):
    q = {'translate_mobile_base': 0.1}
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
def test_fjt_success_gripper(action_client):
    q = {'gripper_joint': -0.1}
    result = action_client.move_to_configuration(q, blocking=True)
    assert result.status == 4

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_fjt_success_all_joints_simultaneously(action_client):
    q = {
        'translate_mobile_base': 0.1,
        'lift_joint': 0.6,
        'arm_joint': 0.2,
        'wrist_yaw_joint': 0.0,
        'wrist_pitch_joint': 0.0,
        'wrist_roll_joint': 0.0,
        'gripper_joint': 0.0
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
