from __future__ import annotations

from typing import TYPE_CHECKING, Any, Callable, Dict, List, Optional, Tuple, Union, override

from hello_helpers.base_command_group import BaseCommandGroup, check_active
from stretch4_body.utils.stretch_pose_models import RobotJoints

if TYPE_CHECKING:
    from stretch_core.stretch_driver import StretchDriver


class WristYawCommandGroup(BaseCommandGroup):

    @override
    def __init__(self) -> None:
        super().__init__('wrist_yaw_joint')

    @override
    @check_active()
    def queue_execution(self, robot: StretchDriver, **kwargs: Any) -> None:
        robot.end_of_arm.move_to(
            'wrist_yaw',
            self.goal['position'],
            self.goal['velocity'],
            self.goal['acceleration'],
        )

    @override
    @check_active()
    def monitor_execution(self, robot_status: Dict[str, Any], **kwargs: Any) -> Tuple[str, float]:
        desired = self.goal['position']
        actual = robot_status['end_of_arm']['wrist_yaw']['pos']
        self.error: float = desired - actual
        return self.name, desired, actual, self.error

    @override
    @check_active()
    def cancel_execution(self, robot: Any, **kwargs: Any) -> None:
        robot.end_of_arm.move_by('wrist_yaw', 0)

    @override
    @check_active()
    def is_finished(self, robot_status: Dict[str, Any], **kwargs: Any) -> bool:
        # TODO: switch to servo motion_generator.is_moving()
        return abs(self.error) < 0.5

    @override
    def joint_state(self, robot_status: Dict[str, Any], **kwargs: Any) -> Tuple[float, float, float]:
        yaw_status = robot_status['end_of_arm']['wrist_yaw']
        return (yaw_status['pos'], yaw_status['vel'], yaw_status['effort'])


class WristPitchCommandGroup(BaseCommandGroup):

    @override
    def __init__(self) -> None:
        super().__init__('wrist_pitch_joint')

    @override
    @check_active()
    def queue_execution(self, robot: StretchDriver, **kwargs: Any) -> None:
        robot.end_of_arm.move_to(
            'wrist_pitch',
            self.goal['position'],
            self.goal['velocity'],
            self.goal['acceleration'],
        )

    @override
    @check_active()
    def monitor_execution(self, robot_status: Dict[str, Any], **kwargs: Any) -> Tuple[str, float]:
        desired = self.goal['position']
        actual = robot_status['end_of_arm']['wrist_pitch']['pos']
        self.error: float = desired - actual
        return self.name, desired, actual, self.error

    @override
    @check_active()
    def cancel_execution(self, robot: Any, **kwargs: Any) -> None:
        robot.end_of_arm.move_by('wrist_pitch', 0)

    @override
    @check_active()
    def is_finished(self, robot_status: Dict[str, Any], **kwargs: Any) -> bool:
        # TODO: switch to servo motion_generator.is_moving()
        return abs(self.error) < 0.5

    @override
    def joint_state(self, robot_status: Dict[str, Any], **kwargs: Any) -> Tuple[float, float, float]:
        pitch_status = robot_status['end_of_arm']['wrist_pitch']
        return (pitch_status['pos'], pitch_status['vel'], pitch_status['effort'])


class WristRollCommandGroup(BaseCommandGroup):

    @override
    def __init__(self) -> None:
        super().__init__('wrist_roll_joint')

    @override
    @check_active()
    def queue_execution(self, robot: StretchDriver, **kwargs: Any) -> None:
        robot.end_of_arm.move_to(
            'wrist_roll',
            self.goal['position'],
            self.goal['velocity'],
            self.goal['acceleration'],
        )

    @override
    @check_active()
    def monitor_execution(self, robot_status: Dict[str, Any], **kwargs: Any) -> Tuple[str, float]:
        desired = self.goal['position']
        actual = robot_status['end_of_arm']['wrist_roll']['pos']
        self.error: float = desired - actual
        return self.name, desired, actual, self.error

    @override
    @check_active()
    def cancel_execution(self, robot: Any, **kwargs: Any) -> None:
        robot.end_of_arm.move_by('wrist_roll', 0)

    @override
    @check_active()
    def is_finished(self, robot_status: Dict[str, Any], **kwargs: Any) -> bool:
        # TODO: switch to servo motion_generator.is_moving()
        return abs(self.error) < 0.5

    @override
    def joint_state(self, robot_status: Dict[str, Any], **kwargs: Any) -> Tuple[float, float, float]:
        roll_status = robot_status['end_of_arm']['wrist_roll']
        return (roll_status['pos'], roll_status['vel'], roll_status['effort'])


class GripperCommandGroup(BaseCommandGroup):

    @override
    def __init__(self) -> None:
        super().__init__('gripper_joint')

    @property
    def gripper_joint_names(self) -> List[str]:
        names = list(RobotJoints.gripper.tool_joints)
        names.extend(['gripper_joint', f"{RobotJoints.gripper.value}_joint", RobotJoints.gripper.value])
        return names

    @override
    def activate(self, commanded_joint_names: List[str], invalid_joints_callback: Callable[[str], None], **kwargs: Any) -> bool:
        self.active = False
        self.index = None
        robot = kwargs.get('robot')

        for name in commanded_joint_names:
            if name in self.gripper_joint_names:
                if robot and hasattr(robot, 'end_of_arm') and RobotJoints.gripper.value not in robot.end_of_arm.joints:
                    invalid_joints_callback(f"Commanded joint '{name}', but no active gripper tool is attached.")
                    return False
                self.index = commanded_joint_names.index(name)
                self.active = True
                break
        return True

    @override
    @check_active()
    def queue_execution(self, robot: StretchDriver, **kwargs: Any) -> None:
        if hasattr(robot, 'end_of_arm') and RobotJoints.gripper.value in robot.end_of_arm.joints:
            robot.end_of_arm.move_to(
                RobotJoints.gripper.value,
                self.goal['position'],
                self.goal['velocity'],
                self.goal['acceleration'],
            )

    @override
    @check_active()
    def monitor_execution(self, robot_status: Dict[str, Any], **kwargs: Any) -> Tuple[str, float]:
        desired = self.goal['position']
        tool_status = robot_status.get('end_of_arm', {}).get(RobotJoints.gripper.value)
        if tool_status is None:
            self.error = 0.0
            return self.name, desired, 0.0, 0.0

        actual = tool_status.get('pos', tool_status.get('pos_mm', 0.0) / 1000.0)
        self.error: float = desired - actual
        return self.name, desired, actual, self.error

    @override
    @check_active()
    def cancel_execution(self, robot: Any, **kwargs: Any) -> None:
        if hasattr(robot, 'end_of_arm') and RobotJoints.gripper.value in robot.end_of_arm.joints:
            robot.end_of_arm.move_by(RobotJoints.gripper.value, 0)

    @override
    @check_active()
    def is_finished(self, robot_status: Dict[str, Any], **kwargs: Any) -> bool:
        if self.error is None:
            return True
        return abs(self.error) < 0.5

    @override
    def joint_state(self, robot_status: Dict[str, Any], **kwargs: Any) -> Tuple[float, float, float]:
        gripper_status = robot_status.get('end_of_arm', {}).get(RobotJoints.gripper.value)
        if not gripper_status:
            return (0.0, 0.0, 0.0)

        if 'gripper_conversion' in gripper_status and 'finger_rad' in gripper_status['gripper_conversion']:
            conversion = gripper_status['gripper_conversion']
            return (conversion['finger_rad'], conversion['finger_vel'], gripper_status.get('effort', 0.0))
        elif 'finger_pos' in gripper_status:
            return (gripper_status['finger_pos'], gripper_status.get('finger_vel', 0.0), gripper_status.get('effort', 0.0))
        else:
            pos_m = gripper_status.get('pos_mm', 0.0) / 1000.0
            return (pos_m, gripper_status.get('vel', 0.0), gripper_status.get('effort', 0.0))


class ArmCommandGroup(BaseCommandGroup):

    @override
    def __init__(self) -> None:
        super().__init__('arm_joint')
        self.did_start_moving = False # TODO: remove, move to Stretch Body

    @override
    @check_active()
    def queue_execution(self, robot: StretchDriver, **kwargs: Any) -> None:
        ct: Optional[float] = self.goal['contact_threshold']
        robot.arm.move_to(
            self.goal['position'],
            v_m=self.goal['velocity'],
            a_m=self.goal['acceleration'],
            contact_sensitivity_pos=ct,
            contact_sensitivity_neg=-ct if ct is not None else None,
        )
        self.did_start_moving = False

    @override
    @check_active()
    def monitor_execution(self, robot_status: Dict[str, Any], **kwargs: Any) -> Tuple[str, float]:
        desired = self.goal['position']
        actual = robot_status['arm']['pos']
        self.error: float = desired - actual
        return self.name, desired, actual, self.error

    @override
    @check_active()
    def cancel_execution(self, robot: Any, **kwargs: Any) -> None:
        robot.arm.move_by(0)

    @override
    @check_active()
    def is_finished(self, robot_status: Dict[str, Any], **kwargs: Any) -> bool:
        contact_detected_callback = kwargs.get('contact_detected_callback')
        if robot_status['arm']['motor']['in_guarded_event'] and contact_detected_callback:
            contact_detected_callback("arm guarded contact")

        # The motion generator (mg) runs in firmware and provides
        # a strong signal on whether the joint is tracking a command
        is_moving = bool(robot_status['arm']['motor']['is_mg_moving'])

        # Check did start moving first
        if is_moving:
            self.did_start_moving = True
        if not is_moving and not self.did_start_moving:
            return False

        return not is_moving

    @override
    def joint_state(self, robot_status: Dict[str, Any], **kwargs: Any) -> Tuple[float, float, float]:
        arm_status = robot_status['arm']
        return (arm_status['pos'], arm_status['vel'], arm_status['motor']['effort_pct'])


class LiftCommandGroup(BaseCommandGroup):

    @override
    def __init__(self) -> None:
        super().__init__('lift_joint')
        self.did_start_moving = False # TODO: remove, move to Stretch Body

    @override
    @check_active()
    def queue_execution(self, robot: StretchDriver, **kwargs: Any) -> None:
        ct: Optional[float] = self.goal['contact_threshold']
        robot.lift.move_to(
            self.goal['position'],
            v_m=self.goal['velocity'],
            a_m=self.goal['acceleration'],
            contact_sensitivity_pos=ct,
            contact_sensitivity_neg=-ct if ct is not None else None,
        )
        self.did_start_moving = False

    @override
    @check_active()
    def monitor_execution(self, robot_status: Dict[str, Any], **kwargs: Any) -> Tuple[str, float]:
        desired = self.goal['position']
        actual = robot_status['lift']['pos']
        self.error: float = desired - actual
        return self.name, desired, actual, self.error

    @override
    @check_active()
    def cancel_execution(self, robot: Any, **kwargs: Any) -> None:
        robot.lift.move_by(0)

    @override
    @check_active()
    def is_finished(self, robot_status: Dict[str, Any], **kwargs: Any) -> bool:
        contact_detected_callback = kwargs.get('contact_detected_callback')
        if robot_status['lift']['motor']['in_guarded_event'] and contact_detected_callback:
            contact_detected_callback("lift guarded contact")

        # The motion generator (mg) runs in firmware and provides
        # a strong signal on whether the joint is tracking a command
        is_moving = bool(robot_status['lift']['motor']['is_mg_moving'])

        # Check did start moving first
        if is_moving:
            self.did_start_moving = True
        if not is_moving and not self.did_start_moving:
            return False

        return not is_moving

    @override
    def joint_state(self, robot_status: Dict[str, Any], **kwargs: Any) -> Tuple[float, float, float]:
        lift_status = robot_status['lift']
        return (lift_status['pos'], lift_status['vel'], lift_status['motor']['effort_pct'])


# TODO: currently only supports x translation, rest is remaining
class MobileBaseCommandGroup(BaseCommandGroup):

    @override
    def __init__(self) -> None:
        super().__init__('translate_mobile_base')
        self.initx: Optional[float] = None

    @override
    @check_active()
    def queue_execution(self, robot: StretchDriver, **kwargs: Any) -> None:
        robot.omnibase.translate_by(
            self.goal['position'],
            0.0,
            v_m=self.goal['velocity'],
            a_m=self.goal['acceleration']
        )
        # Store initial x for relative motion tracking
        self.initx = robot.omnibase.status['x']

    @override
    @check_active()
    def monitor_execution(self, robot_status: Dict[str, Any], **kwargs: Any) -> Tuple[str, float]:
        desired = self.goal['position']
        actual = robot_status['omnibase']['x'] - self.initx
        self.error: float = desired - actual
        return self.name, desired, actual, self.error

    @override
    @check_active()
    def cancel_execution(self, robot: Any, **kwargs: Any) -> None:
        robot.omnibase.hard_stop()

    @override
    @check_active()
    def is_finished(self, robot_status: Dict[str, Any], **kwargs: Any) -> bool:
        # The motion generator (mg) runs in firmware and provides
        # a strong signal on whether the joint is tracking a command
        moving_wheels = [
            robot_status['omnibase']['wheel_0']['is_mg_moving'],
            robot_status['omnibase']['wheel_1']['is_mg_moving'],
            robot_status['omnibase']['wheel_2']['is_mg_moving']
        ]
        is_moving = any(moving_wheels)
        return not is_moving

    @override
    def joint_state(self, robot_status: Dict[str, Any], **kwargs: Any) -> Tuple[float, float, float]:
        return (None, None, None)
