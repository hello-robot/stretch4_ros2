from functools import wraps
from typing import List, Dict, Optional, Callable, Any, Tuple, TypeVar, ParamSpec
from trajectory_msgs.msg import JointTrajectoryPoint

# Type variables for the decorator to preserve function signatures in IDEs
P = ParamSpec("P")
R = TypeVar("R")

def check_active(return_value: Any = None):
    def decorator(func: Callable[P, R]) -> Callable[P, R]:
        @wraps(func)
        def wrapper(self: Any, *args: P.args, **kwargs: P.kwargs) -> Any:
            if not getattr(self, 'active', False):
                return return_value
            return func(self, *args, **kwargs)
        return wrapper # type: ignore
    return decorator


class BaseCommandGroup:

    def __init__(self, joint_name: str) -> None:
        """Simple command group class to extend

        Attributes
        ----------
        name: str
            joint name
        active: bool
            whether joint is executing a command
        index: int
            index of joint's goal components in JointTrajectoryPoint's arrays
        goal: dict
            joint goal
        error: float
            the error between actual and desired
        """
        self.name: str = joint_name
        self.active: bool = False
        self.index: Optional[int] = None
        self.goal: Dict[str, Optional[float]] = {
            "position": None,
            "velocity": None,
            "acceleration": None,
            "contact_threshold": None
        }
        self.error: Optional[float] = None

    def get_num_active_joints(self) -> int:
        """Returns number of active joints in the group

        Returns
        -------
        int
            the number of active joints within this group
        """
        return int(self.active)

    def activate(self,
                 commanded_joint_names: List[str],
                 invalid_joints_callback: Callable[[str], None],
                 **kwargs: Any) -> bool:
        """Activates group if its joints are in an incoming goal message

        Checks commanded joints to activate the command
        group and validates joints used correctly.

        Parameters
        ----------
        commanded_joint_names: list(str)
            list of commanded joints in the trajectory
        invalid_joints_callback: func
            error callback for misused joints in trajectory
            (e.g. if 2 incompatible joints appear together in the traj)

        Returns
        -------
        bool
            False if commanded joints invalid, else True
        """
        self.active = False
        self.index = None
        if self.name in commanded_joint_names:
            self.index = commanded_joint_names.index(self.name)
            self.active = True

        return True

    @check_active(return_value=True)
    def set_goal(self,
                 point: JointTrajectoryPoint,
                 invalid_goal_callback: Callable[[str], None],
                 **kwargs: Any) -> bool:
        """Sets goal for the joint group

        Sets and validates the goal point for the joints
        in this command group.

        Parameters
        ----------
        point: JointTrajectoryPoint
            the target point for all joints
        invalid_goal_callback: func
            error callback for invalid goal

        Returns
        -------
        bool
            False if commanded goal invalid, else True
        """
        goal_pos = point.positions[self.index] if len(point.positions) > self.index else None
        if goal_pos is None:
            err_str = (f"Received goal point with positions array length={len(point.positions)}. "
                       f"This joint ({self.name})'s index is {self.index}. Length of array must cover all joints "
                       f"listed in commanded_joint_names.")
            invalid_goal_callback(err_str)
            return False

        self.goal = {
            "position": goal_pos,
            "velocity": point.velocities[self.index] if len(point.velocities) > self.index else None,
            "acceleration": point.accelerations[self.index] if len(point.accelerations) > self.index else None,
            "contact_threshold": abs(point.effort[self.index]) if len(point.effort) > self.index else None
        }

        return True

    def queue_execution(self, robot: Any, **kwargs: Any) -> None:
        """Queues execution of the goal

        Parameters
        ----------
        robot: stretch_body.Robot
            low level robot API

        Queues the goal for execution by the main control loop.
        """
        raise NotImplementedError

    def monitor_execution(self, robot_status: Dict[str, Any], **kwargs: Any) -> Optional[Tuple[str, float]]:
        """Monitors progress of joint group

        Checks against robot's status to track progress
        towards the target point.

        This method must set self.error.

        Parameters
        ----------
        robot_status: dict
            whole robot's current status

        Returns
        -------
        tuple(str, float, float, float) or None
            (joint name, desired, actual, error) if group active, else None
        """
        raise NotImplementedError

    def cancel_exceution(self, robot: Any, **kwargs: Any) -> None:
        """Cancels execution of the goal

        Parameters
        ----------
        robot: stretch_body.Robot
            low level robot API
        """
        raise NotImplementedError

    def is_finished(self, robot_status: Dict[str, Any], **kwargs: Any) -> bool:
        """Returns whether execution finished

        Typically based on when low level API signals that
        controller deactivated.

        Parameters
        ----------
        robot_status: dict
            whole robot's current status

        Returns
        -------
        bool
            if active, whether execution finished, else True
        """
        raise NotImplementedError

    def joint_state(self, robot_status: Dict[str, Any], **kwargs: Any) -> Tuple[float, float, float]:
        """Returns state of the joint group

        Parameters
        ----------
        robot_status: dict
            whole robot's current status

        Returns
        -------
        (float, float, float)
            Current position, velocity, and effort
        """
        raise NotImplementedError
