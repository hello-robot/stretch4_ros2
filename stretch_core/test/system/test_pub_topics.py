import time

from common.client_nodes.mock_subscriber import MockSubscriberNode
from common.launch_descriptions import stretch_driver_ld
import pytest


@pytest.mark.launch(fixture=stretch_driver_ld)
def test_odom_published() -> None:
    """Verify odom messages are published by stretch_driver."""
    node = MockSubscriberNode()
    try:
        assert node.wait_for_message('odom', timeout=5.0), 'Did not receive odom msg!'
    finally:
        node.stop()


@pytest.mark.launch(fixture=stretch_driver_ld)
def test_homed_published() -> None:
    """Verify is_homed messages are published by stretch_driver."""
    node = MockSubscriberNode()
    try:
        assert node.wait_for_message('is_homed', timeout=5.0), 'Did not receive is_homed msg!'
    finally:
        node.stop()


@pytest.mark.launch(fixture=stretch_driver_ld)
def test_mode_published() -> None:
    """Verify mode messages are published with default navigation mode."""
    node = MockSubscriberNode()
    try:
        assert node.wait_for_message('mode', timeout=5.0), 'Did not receive mode msg!'
        assert node.latest_msgs['mode'].data == 'navigation', 'Default mode not navigation'
    finally:
        node.stop()


@pytest.mark.launch(fixture=stretch_driver_ld)
def test_tf_published_conditional() -> None:
    """
    Verify odom->base_link TF is not broadcast when broadcast_odom_tf is False.
    """
    node = MockSubscriberNode()
    try:
        time.sleep(1.0)
        found_odom_tf = False
        if node.latest_msgs['tf'] is not None:
            for transform in node.latest_msgs['tf'].transforms:
                if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_link':
                    found_odom_tf = True
        assert not found_odom_tf, "Odom TF broadcasted when broadcast_odom_tf is False"
    finally:
        node.stop()
