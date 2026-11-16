"""ROS messages for the wrist camera's IMU samples (stretch4_body's luxonis_imu.ImuSample)."""

import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu

UNKNOWN_COVARIANCE = [0.0] * 9
"""sensor_msgs/Imu: all zeros means the covariance is unknown."""

NOT_REPORTED_COVARIANCE = [-1.0] + [0.0] * 8
"""sensor_msgs/Imu: -1 in the first element means the field is not reported."""


def monotonic_to_system_offset(attempts: int = 3) -> float:
    """Seconds to add to a time.monotonic() reading to put it on the system clock (time.time()).

    DepthAI stamps messages on the host's steady clock, which is CLOCK_MONOTONIC on Linux, after
    syncing it with the device clock. Adding this offset lands them on the system clock that ROS stamps
    with, without the arrival latency an arrival-time based estimate would add. The offset only moves
    when the system clock is slewed or stepped, so it is read fresh for every batch.

    The monotonic clock is read on both sides of the system clock, and the tightest of a few tries is
    kept, so a preemption between the reads does not leak into the offset.
    """
    best_offset, best_bracket = 0.0, float("inf")
    for _ in range(attempts):
        before = time.monotonic()
        system = time.time()
        after = time.monotonic()
        if after - before < best_bracket:
            best_bracket = after - before
            best_offset = system - 0.5 * (before + after)
    return best_offset


def stamp_from_seconds(seconds: float) -> Time:
    """A ROS Time from seconds, rounded to the nanosecond."""
    nanoseconds = round(seconds * 1e9)
    return Time(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)


def imu_message(sample, stamp: Time, frame_id: str) -> Imu:
    """sensor_msgs/Imu from an ImuSample: acceleration with gravity, angular velocity and orientation.

    Fields the IMU has not reported (no gyroscope yet, or no on-chip fusion for orientation) are marked
    with -1 in their covariance, as sensor_msgs/Imu asks. Covariances the IMU does not report are zero.
    """
    msg = Imu()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    x, y, z = sample.acceleration
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = float(x), float(y), float(z)
    msg.linear_acceleration_covariance = UNKNOWN_COVARIANCE

    if sample.angular_velocity is None:
        msg.angular_velocity_covariance = NOT_REPORTED_COVARIANCE
    else:
        x, y, z = sample.angular_velocity
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = float(x), float(y), float(z)
        msg.angular_velocity_covariance = UNKNOWN_COVARIANCE

    if sample.orientation_xyzw is None:
        msg.orientation_covariance = NOT_REPORTED_COVARIANCE
    else:
        x, y, z, w = sample.orientation_xyzw
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = float(x), float(y), float(z), float(w)
        msg.orientation_covariance = UNKNOWN_COVARIANCE

    return msg


def linear_acceleration_message(sample, stamp: Time, frame_id: str) -> Vector3Stamped | None:
    """The sample's acceleration with gravity removed, or None if it has none (no orientation yet)."""
    if sample.linear_acceleration is None:
        return None

    msg = Vector3Stamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    x, y, z = sample.linear_acceleration
    msg.vector.x, msg.vector.y, msg.vector.z = float(x), float(y), float(z)
    return msg
