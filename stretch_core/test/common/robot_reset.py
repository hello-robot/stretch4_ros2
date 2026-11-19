import warnings

from common.client_nodes.fjt_client import FJTClient
from common.client_nodes.param_client import ParamClient
from common.client_nodes.service_client import ServiceClientNode
from std_srvs.srv import SetBool, Trigger

STOWED_POSE: dict[str, float] = {
    'arm_joint': 0.05,
    'lift_joint': 0.5,
    'wrist_yaw_joint': 0.0,
    'wrist_pitch_joint': 0.0,
    'wrist_roll_joint': 0.0,
}


def reset_robot_to_default() -> None:
    """
    Stop motion, clear runstop, move to a stowed pose, and switch mode to velocity.

    Catches ``(Exception, SystemExit)`` so a driver/connection failure during
    teardown does not mask the underlying test result.
    """
    service_client: ServiceClientNode | None = None
    param_client: ParamClient | None = None
    fjt_client: FJTClient | None = None
    try:
        service_client = ServiceClientNode('robot_reset_service_client')
        if service_client.wait_for_service(Trigger, '/stop_the_robot', timeout_sec=2.0):
            service_client.call_service(Trigger, '/stop_the_robot', Trigger.Request())
        if service_client.wait_for_service(SetBool, '/runstop_the_robot', timeout_sec=2.0):
            req = SetBool.Request()
            req.data = False
            service_client.call_service(SetBool, '/runstop_the_robot', req)

        param_client = ParamClient('robot_reset_param_client')
        fjt_client = FJTClient('robot_reset_fjt_client')
        if param_client.set_parameter('mode', 'navigation'):
            fjt_client.move_to_configuration(STOWED_POSE, blocking=True)
        param_client.set_parameter('mode', 'velocity')
    except (Exception, SystemExit) as exc:
        warnings.warn(f'Failed to reset robot to default state: {exc}', stacklevel=2)
    finally:
        if fjt_client is not None:
            fjt_client.destroy_node()
        if param_client is not None:
            param_client.destroy_node()
        if service_client is not None:
            service_client.destroy_node()
