from __future__ import annotations
from typing import TYPE_CHECKING, Tuple, Any, Dict, List, Optional, Union, override
from hello_helpers.base_command_group import BaseCommandGroup, check_active

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

    @override
    @check_active()
    def queue_execution(self, robot: StretchDriver, **kwargs: Any) -> None:
        robot.end_of_arm.move_to(
            'stretch_gripper',
            self.goal['position'],
            self.goal['velocity'],
            self.goal['acceleration'],
        )

    @override
    @check_active()
    def monitor_execution(self, robot_status: Dict[str, Any], **kwargs: Any) -> Tuple[str, float]:
        desired = self.goal['position']
        actual = robot_status['end_of_arm']['stretch_gripper']['pos']
        self.error: float = desired - actual
        return self.name, desired, actual, self.error

    @override
    @check_active()
    def cancel_execution(self, robot: Any, **kwargs: Any) -> None:
        robot.end_of_arm.move_by('stretch_gripper', 0)

    @override
    @check_active()
    def is_finished(self, robot_status: Dict[str, Any], **kwargs: Any) -> bool:
        # TODO: switch to servo motion_generator.is_moving()
        return abs(self.error) < 0.5

    @override
    def joint_state(self, robot_status: Dict[str, Any], **kwargs: Any) -> Tuple[float, float, float]:
        gripper_status = robot_status['end_of_arm']['stretch_gripper']
        conversion = gripper_status['gripper_conversion']
        return (conversion['finger_rad'], conversion['finger_vel'], gripper_status['effort'])


class ParallelGripperCommandGroup(BaseCommandGroup):

    @override
    def __init__(self) -> None:
        super().__init__('parallel_gripper_joint')

    @override
    @check_active()
    def queue_execution(self, robot: Any, **kwargs: Any) -> None:
        robot.end_of_arm.move_to('parallel_gripper', self.goal['position'], self.goal['velocity'], self.goal['acceleration'])

    @override
    @check_active()
    def monitor_execution(self, robot_status: Dict[str, Any], **kwargs: Any) -> Tuple[str, float]:
        desired_m = self.goal['position']
        actual_mm = robot_status['end_of_arm']['parallel_gripper'].get('pos_mm', 0.0)
        actual_m = actual_mm / 1000.0
        
        self.error: float = desired_m - actual_m
        return self.name, desired_m, actual_m, self.error

    @override
    @check_active()
    def cancel_execution(self, robot: Any, **kwargs: Any) -> None:
        robot.end_of_arm.move_by('parallel_gripper', 0.0)

    @override
    @check_active()
    def is_finished(self, robot_status: Dict[str, Any], **kwargs: Any) -> bool:
        return abs(self.error) < 0.002

    @override
    def joint_state(self, robot_status: Dict[str, Any], **kwargs: Any) -> Tuple[float, float, float]:
        gripper_status = robot_status['end_of_arm']['parallel_gripper']
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
        self.inity: Optional[float] = None
        self.inittheta: Optional[float] = None
        self.active_joints: List[str] = []
        self.indices: Dict[str, int] = {}
        self.goals: Dict[str, Dict[str, Any]] = {}

    @override
    def get_num_active_joints(self) -> int:
        return len(self.active_joints) if self.active else 0

    @override
    def activate(self,
                 commanded_joint_names: List[str],
                 invalid_joints_callback: Any,
                 **kwargs: Any) -> bool:
        self.active = False
        self.active_joints = []
        self.indices = {}

        # Check which of our joints are present
        for joint in ['translate_mobile_base', 'translate_mobile_base_y', 'rotate_mobile_base']:
            if joint in commanded_joint_names:
                self.active_joints.append(joint)
                self.indices[joint] = commanded_joint_names.index(joint)

        if not self.active_joints:
            return True

        # Check if there is a conflict (both translation and rotation)
        has_translation = 'translate_mobile_base' in self.active_joints or 'translate_mobile_base_y' in self.active_joints
        has_rotation = 'rotate_mobile_base' in self.active_joints

        if has_translation and has_rotation:
            invalid_joints_callback("Cannot command both translation and rotation for the mobile base in the same trajectory.")
            return False

        self.active = True
        # For compatibility with any legacy code that expects self.index to be set to something:
        if 'translate_mobile_base' in self.active_joints:
            self.index = self.indices['translate_mobile_base']
        elif 'translate_mobile_base_y' in self.active_joints:
            self.index = self.indices['translate_mobile_base_y']
        elif 'rotate_mobile_base' in self.active_joints:
            self.index = self.indices['rotate_mobile_base']

        return True

    @override
    @check_active(return_value=True)
    def set_goal(self,
                 point: JointTrajectoryPoint,
                 invalid_goal_callback: Any,
                 **kwargs: Any) -> bool:
        self.goals = {}
        for joint in self.active_joints:
            index = self.indices[joint]
            goal_pos = point.positions[index] if len(point.positions) > index else None
            if goal_pos is None:
                err_str = (f"Received goal point with positions array length={len(point.positions)}. "
                           f"This joint ({joint})'s index is {index}. Length of array must cover all joints "
                           f"listed in commanded_joint_names.")
                invalid_goal_callback(err_str)
                return False

            self.goals[joint] = {
                "position": goal_pos,
                "velocity": point.velocities[index] if len(point.velocities) > index else None,
                "acceleration": point.accelerations[index] if len(point.accelerations) > index else None,
                "contact_threshold": abs(point.effort[index]) if len(point.effort) > index else None
            }

        # Maintain self.goal for any backwards compatibility/mocking
        if self.active_joints:
            first_joint = self.active_joints[0]
            self.goal = self.goals[first_joint]

        return True

    @override
    @check_active()
    def queue_execution(self, robot: StretchDriver, **kwargs: Any) -> None:
        has_translation = 'translate_mobile_base' in self.active_joints or 'translate_mobile_base_y' in self.active_joints
        has_rotation = 'rotate_mobile_base' in self.active_joints

        if has_translation:
            x_m = self.goals['translate_mobile_base']['position'] if 'translate_mobile_base' in self.active_joints else 0.0
            y_m = self.goals['translate_mobile_base_y']['position'] if 'translate_mobile_base_y' in self.active_joints else 0.0

            # Extract velocity and acceleration (if any)
            v_m = None
            a_m = None
            if 'translate_mobile_base' in self.active_joints:
                v_m = self.goals['translate_mobile_base']['velocity']
                a_m = self.goals['translate_mobile_base']['acceleration']
            elif 'translate_mobile_base_y' in self.active_joints:
                v_m = self.goals['translate_mobile_base_y']['velocity']
                a_m = self.goals['translate_mobile_base_y']['acceleration']

            robot.omnibase.translate_by(x_m, y_m, v_m=v_m, a_m=a_m)

            # Store initial states for relative tracking
            self.initx = robot.omnibase.status['x']
            self.inity = robot.omnibase.status['y']
            self.inittheta = None

        elif has_rotation:
            w_r = self.goals['rotate_mobile_base']['position']
            v_r = self.goals['rotate_mobile_base']['velocity']
            a_r = self.goals['rotate_mobile_base']['acceleration']

            robot.omnibase.rotate_by(w_r, v_r=v_r, a_r=a_r)

            # Store initial states for relative tracking
            self.initx = None
            self.inity = None
            self.inittheta = robot.omnibase.status['theta']

    @override
    @check_active()
    def monitor_execution(self, robot_status: Dict[str, Any], **kwargs: Any) -> List[Tuple[str, float, float, float]]:
        self.errors = {}
        progress_list = []

        has_translation = 'translate_mobile_base' in self.active_joints or 'translate_mobile_base_y' in self.active_joints
        has_rotation = 'rotate_mobile_base' in self.active_joints

        if has_translation:
            if 'translate_mobile_base' in self.active_joints:
                desired_x = self.goals['translate_mobile_base']['position']
                actual_x = robot_status['omnibase']['x'] - self.initx
                error_x = desired_x - actual_x
                self.errors['translate_mobile_base'] = error_x
                progress_list.append(('translate_mobile_base', desired_x, actual_x, error_x))

            if 'translate_mobile_base_y' in self.active_joints:
                desired_y = self.goals['translate_mobile_base_y']['position']
                actual_y = robot_status['omnibase']['y'] - self.inity
                error_y = desired_y - actual_y
                self.errors['translate_mobile_base_y'] = error_y
                progress_list.append(('translate_mobile_base_y', desired_y, actual_y, error_y))

        elif has_rotation:
            if 'rotate_mobile_base' in self.active_joints:
                desired_theta = self.goals['rotate_mobile_base']['position']
                actual_theta = robot_status['omnibase']['theta'] - self.inittheta
                error_theta = desired_theta - actual_theta
                self.errors['rotate_mobile_base'] = error_theta
                progress_list.append(('rotate_mobile_base', desired_theta, actual_theta, error_theta))

        # Set self.error for compatibility with any base/monitoring code
        if self.errors:
            self.error = list(self.errors.values())[0]

        return progress_list

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
