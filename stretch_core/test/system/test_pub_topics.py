import pytest
import rclpy
import time

from common.launch_descriptions import stretch_driver_ld
from common.client_nodes.mock_subscriber import MockSubscriberNode


@pytest.mark.launch(fixture=stretch_driver_ld)
def test_odom_published():
    rclpy.init()
    try:
        node = MockSubscriberNode()
        assert node.wait_for_message('odom', timeout=5.0), 'Did not receive odom msg!'
    finally:
        node.stop()

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_homed_published():
    rclpy.init()
    try:
        node = MockSubscriberNode()
        assert node.wait_for_message('is_homed', timeout=5.0), 'Did not receive is_homed msg!'
    finally:
        node.stop()

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_mode_published():
    rclpy.init()
    try:
        node = MockSubscriberNode()
        assert node.wait_for_message('mode', timeout=5.0), 'Did not receive mode msg!'
        assert node.latest_msgs['mode'].data == 'navigation', 'Default mode not navigation'
    finally:
        node.stop()

@pytest.mark.launch(fixture=stretch_driver_ld)
def test_tf_published_conditional():
    """
    By default broadcast_odom_tf is False, so tf should not contain odom->base_link
    This attempts to observe whether odom TF appears unexpectedly.
    """
    rclpy.init()
    try:
        node = MockSubscriberNode()
        time.sleep(1.0)
        found_odom_tf = False
        if node.latest_msgs['tf'] is not None:
            for transform in node.latest_msgs['tf'].transforms:
                if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_link':
                    found_odom_tf = True
        assert not found_odom_tf, "Odom TF broadcasted when broadcast_odom_tf is False"
    finally:
        node.stop()
