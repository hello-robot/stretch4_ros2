"""Gamepad stick mapping → base Twist.

This module only converts the JOINT_SPACE base stick/shoulder logic to geometry_msgs/Twist.
"""
import math

from geometry_msgs.msg import Twist

STICK_DEADZONE = 0.1
TRIGGER_THRESHOLD = 0.9

# Match medium translation + slow angular gamepad profiles
MAX_VEL_XY = 0.3
MAX_VEL_W = 1.0
ROTATING_VEL_XY = 0.1


def gamepad_state_to_twist(gamepad_state):
    """Base-only JOINT_SPACE mapping → Twist (same stick rules as ControlMapping)."""
    rt_pulled = gamepad_state.get('right_trigger_pulled', 0.0) > TRIGGER_THRESHOLD
    left_shoulder = bool(gamepad_state.get('left_shoulder_button_pressed'))
    right_shoulder = bool(gamepad_state.get('right_shoulder_button_pressed'))
    right_stick_for_base_rotate = left_shoulder and right_shoulder

    ls_x = float(gamepad_state.get('left_stick_x', 0.0))
    ls_y = float(gamepad_state.get('left_stick_y', 0.0))
    rs_x = float(gamepad_state.get('right_stick_x', 0.0))

    if rt_pulled:
        if abs(ls_x) > STICK_DEADZONE or abs(ls_y) > STICK_DEADZONE:
            if abs(ls_y) > abs(ls_x):
                cmd_y = math.copysign(1.0, ls_y)
                cmd_x = 0.0
            else:
                cmd_y = 0.0
                cmd_x = math.copysign(1.0, -ls_x)
        else:
            cmd_y = 0.0
            cmd_x = 0.0
    else:
        cmd_y = ls_y if abs(ls_y) > STICK_DEADZONE else 0.0
        cmd_x = -ls_x if abs(ls_x) > STICK_DEADZONE else 0.0

    cmd_t = 0.0
    if not rt_pulled:
        if right_stick_for_base_rotate:
            cmd_t = -rs_x
        elif left_shoulder:
            cmd_t = 1.0
        elif right_shoulder:
            cmd_t = -1.0

    is_rotating = abs(cmd_t) >= STICK_DEADZONE
    vel_xy = ROTATING_VEL_XY if is_rotating else MAX_VEL_XY
    precision = float(gamepad_state.get('left_trigger_pulled', 0.0))
    scale = 1.0 - 0.75 * max(0.0, min(1.0, precision))

    twist = Twist()
    twist.linear.x = scale * vel_xy * cmd_y
    twist.linear.y = scale * vel_xy * cmd_x
    twist.angular.z = scale * MAX_VEL_W * cmd_t
    return twist
