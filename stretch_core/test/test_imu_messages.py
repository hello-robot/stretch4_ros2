"""The gripper IMU's ROS messages and the monotonic-to-system clock offset behind their stamps."""

import time
from types import SimpleNamespace

import numpy as np
import pytest

from stretch_core.vision.imu_messages import (
    NOT_REPORTED_COVARIANCE,
    UNKNOWN_COVARIANCE,
    imu_message,
    linear_acceleration_message,
    monotonic_to_system_offset,
    stamp_from_seconds,
)
from stretch_core.vision.vision_topics import VisionFrames, VisionTopics


def sample(angular_velocity=(0.1, 0.2, 0.3), orientation_xyzw=(0.0, 0.0, 0.0, 1.0), linear_acceleration=(0.5, 0.0, 0.0)):
    """A stand-in for stretch4_body's ImuSample."""
    return SimpleNamespace(
        timestamp=100.0,
        sequence=7,
        acceleration=np.array([0.5, -9.8, 0.0]),
        angular_velocity=None if angular_velocity is None else np.array(angular_velocity),
        orientation_xyzw=None if orientation_xyzw is None else np.array(orientation_xyzw),
        linear_acceleration=None if linear_acceleration is None else np.array(linear_acceleration),
    )


def test_offset_puts_monotonic_on_the_system_clock():
    offset = monotonic_to_system_offset()
    assert time.monotonic() + offset == pytest.approx(time.time(), abs=1e-3)


def test_stamp_from_seconds():
    stamp = stamp_from_seconds(1_700_000_000.123456789)
    assert stamp.sec == 1_700_000_000
    assert stamp.nanosec == pytest.approx(123_456_789, abs=300)  # float64 resolution at this magnitude

    # Rounding up to a whole second carries into sec rather than giving nanosec == 1e9.
    stamp = stamp_from_seconds(41.9999999999)
    assert (stamp.sec, stamp.nanosec) == (42, 0)


def test_imu_message_fields():
    stamp = stamp_from_seconds(5.0)
    msg = imu_message(sample(), stamp, 'some_frame')
    assert msg.header.frame_id == 'some_frame'
    assert msg.header.stamp == stamp
    assert (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z) == (0.5, -9.8, 0.0)
    assert (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z) == (0.1, 0.2, 0.3)
    assert (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w) == (0.0, 0.0, 0.0, 1.0)
    assert list(msg.orientation_covariance) == UNKNOWN_COVARIANCE
    assert list(msg.angular_velocity_covariance) == UNKNOWN_COVARIANCE


def test_unreported_fields_are_marked():
    msg = imu_message(sample(angular_velocity=None, orientation_xyzw=None), stamp_from_seconds(0.0), 'f')
    assert list(msg.orientation_covariance) == NOT_REPORTED_COVARIANCE
    assert list(msg.angular_velocity_covariance) == NOT_REPORTED_COVARIANCE


def test_linear_acceleration_message():
    msg = linear_acceleration_message(sample(), stamp_from_seconds(1.0), 'f')
    assert (msg.vector.x, msg.vector.y, msg.vector.z) == (0.5, 0.0, 0.0)
    assert linear_acceleration_message(sample(linear_acceleration=None), stamp_from_seconds(1.0), 'f') is None


def test_topic_and_frame_names():
    assert VisionTopics.gripper_imu() == '/cameras_gripper/imu/data'
    assert VisionTopics.gripper_imu_linear_acceleration() == '/cameras_gripper/imu/linear_acceleration'
    assert VisionFrames.gripper_imu_frame() == 'gripper_right_camera_color_optical_frame'
