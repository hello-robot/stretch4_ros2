#! /usr/bin/env python3
"""Makes one or more mirror drivers (e.g. a simulated digital twin) follow a primary driver.

The primary is left alone: clients keep talking to it and get its results. The relay
watches what the primary does and repeats it on each mirror:

- trajectories: each goal the primary accepts (published by its trajectory server on
  follow_joint_trajectory/accepted_goals) is sent to the mirror's own action server, and
  canceled there when the primary's goal is canceled or aborted
- services: stop/home/stow_the_robot calls (seen through service introspection on
  <service>/_service_event) are repeated on the mirror
- runstop: the primary's is_runstopped state (which also covers the physical button)
- parameters (optional, mirror_parameters): whitelisted parameter changes, plus an
  initial sync whenever a mirror (re)appears

Command topics (cmd_vel, joint_*_cmd, joy) aren't relayed: a mirror whose
command_namespace is the primary's hears them directly.
"""
from fnmatch import fnmatch

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from control_msgs.action import FollowJointTrajectory
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterEvent
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, qos_profile_action_status_default,
                       qos_profile_parameter_events, qos_profile_system_default)
from service_msgs.msg import ServiceEventInfo
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

MIRRORED_SERVICES = ("stop_the_robot", "home_the_robot", "stow_the_robot")

# Modes the driver switches into by itself (homing, stowing, runstopped) stay local to
# each robot; only a user's switch between these is mirrored.
USER_MODES = ("active", "teleop")

# Primary goal states after which the mirror's copy should stop too.
ENDED_STATES = (GoalStatus.STATUS_CANCELING, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)


def namespace_of(fqn):
    """'/sim/stretch_mujoco_driver' -> '/sim'; '/stretch_driver' -> ''."""
    return fqn.rstrip("/").rsplit("/", 1)[0]


class Mirror:
    """Clients for one mirror driver and what the relay knows about it."""

    def __init__(self, relay, fqn):
        self.fqn = fqn
        ns = namespace_of(fqn)
        group = relay.callback_group
        self.trajectory_client = ActionClient(relay, FollowJointTrajectory, f"{ns}/follow_joint_trajectory",
                                              callback_group=group)
        self.service_clients = {name: relay.create_client(Trigger, f"{ns}/{name}", callback_group=group)
                                for name in MIRRORED_SERVICES}
        self.runstop_client = relay.create_client(SetBool, f"{ns}/runstop_the_robot", callback_group=group)
        self.set_params_client = relay.create_client(SetParameters, f"{fqn}/set_parameters", callback_group=group)
        self.list_params_client = relay.create_client(ListParameters, f"{fqn}/list_parameters", callback_group=group)

        self.online = False          # set_parameters service seen
        self.param_names = None      # parameters this mirror has (None until listed)
        self.synced = False          # initial parameter sync done
        self.runstop_sent = None     # last runstop state delivered
        self.goals = {}              # primary goal id -> mirror ClientGoalHandle, or None while pending
        self.pending_cancels = set() # primary goal ids to cancel once the mirror accepts them


class TwinRelay(Node):
    def __init__(self):
        super().__init__("stretch4_twin_relay")
        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameter("primary_node", "/stretch_driver")
        self.declare_parameter("mirror_nodes", ["/sim/stretch_mujoco_driver"])
        self.declare_parameter("mirror_trajectories", True)
        self.declare_parameter("mirror_services", True)
        self.declare_parameter("mirror_runstop", True)
        self.declare_parameter("mirror_parameters", False)
        self.declare_parameter("mirror_parameter_patterns", ["joint_mode.*", "trajectory_server.*", "mode"])
        # trajectory_server.timeout: a sim running slower than real time needs its own
        self.declare_parameter("mirror_parameter_exclude", ["trajectory_server.timeout"])
        self.declare_parameter("cancel_mirror_on_primary_success", False)

        self.primary = self.get_parameter("primary_node").value
        primary_ns = namespace_of(self.primary)
        self.mirror_parameters = self.get_parameter("mirror_parameters").value
        self.patterns = self.get_parameter("mirror_parameter_patterns").value
        self.exclude = self.get_parameter("mirror_parameter_exclude").value
        self.cancel_on_success = self.get_parameter("cancel_mirror_on_primary_success").value
        self.mirrors = [Mirror(self, fqn) for fqn in self.get_parameter("mirror_nodes").value]

        self.primary_status = {}  # goal id -> latest primary GoalStatus status
        self.primary_params = {}  # name -> latest primary value (python), for whitelisted params
        self.runstopped = None

        group = self.callback_group
        if self.get_parameter("mirror_trajectories").value:
            self.create_subscription(FollowJointTrajectory.Impl.SendGoalService.Request,
                                     f"{primary_ns}/follow_joint_trajectory/accepted_goals",
                                     self.on_primary_goal, 10, callback_group=group)
            self.create_subscription(GoalStatusArray, f"{primary_ns}/follow_joint_trajectory/_action/status",
                                     self.on_primary_status, qos_profile_action_status_default,
                                     callback_group=group)
        if self.get_parameter("mirror_services").value:
            for name in MIRRORED_SERVICES:
                self.create_subscription(Trigger.Event, f"{primary_ns}/{name}/_service_event",
                                         lambda msg, name=name: self.on_primary_service(name, msg),
                                         qos_profile_system_default, callback_group=group)
        if self.get_parameter("mirror_runstop").value:
            latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Bool, f"{primary_ns}/is_runstopped", self.on_primary_runstop, latched,
                                     callback_group=group)
        if self.mirror_parameters:
            self.create_subscription(ParameterEvent, "/parameter_events", self.on_parameter_event,
                                     qos_profile_parameter_events, callback_group=group)
            self.primary_list_client = self.create_client(ListParameters, f"{self.primary}/list_parameters",
                                                          callback_group=group)
            self.primary_get_client = self.create_client(GetParameters, f"{self.primary}/get_parameters",
                                                         callback_group=group)

        # Notices mirrors appearing (or restarting), and catches up runstop state.
        self.create_timer(1.0, self.check_mirrors, callback_group=group)
        self.get_logger().info(f"Relaying {self.primary} to {[m.fqn for m in self.mirrors]} "
                               f"(parameters {'on' if self.mirror_parameters else 'off'})")

    # --- mirror availability ------------------------------------------------------

    def check_mirrors(self):
        for mirror in self.mirrors:
            online = mirror.set_params_client.service_is_ready()
            if online and not mirror.online:
                mirror.synced = not self.mirror_parameters
                mirror.runstop_sent = None
                if self.mirror_parameters and not self.sync_parameters(mirror):
                    continue  # try again next time
                self.get_logger().info(f"Mirror {mirror.fqn} is up")
            elif not online and mirror.online:
                self.get_logger().warn(f"Mirror {mirror.fqn} went away")
                mirror.goals.clear()
                mirror.pending_cancels.clear()
            mirror.online = online
            if online and self.runstopped is not None and mirror.runstop_sent != self.runstopped:
                self.send_runstop(mirror)

    # --- trajectories ---------------------------------------------------------------

    def on_primary_goal(self, msg):
        goal_id = bytes(msg.goal_id.uuid)
        if self.primary_status.get(goal_id) in ENDED_STATES:
            return
        for mirror in self.mirrors:
            if not mirror.synced:
                self.get_logger().warn(f"Not sending goal to {mirror.fqn}: its parameters aren't synced yet")
                continue
            if not mirror.trajectory_client.server_is_ready():
                self.get_logger().warn(f"Not sending goal to {mirror.fqn}: its action server isn't available")
                continue
            mirror.goals[goal_id] = None
            future = mirror.trajectory_client.send_goal_async(msg.goal, goal_uuid=msg.goal_id)
            future.add_done_callback(lambda f, mirror=mirror: self.on_mirror_goal_response(mirror, goal_id, f))

    def on_mirror_goal_response(self, mirror, goal_id, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn(f"{mirror.fqn} rejected a mirrored trajectory")
            mirror.goals.pop(goal_id, None)
            mirror.pending_cancels.discard(goal_id)
            return
        mirror.goals[goal_id] = handle
        if goal_id in mirror.pending_cancels:
            mirror.pending_cancels.discard(goal_id)
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(
            lambda f, mirror=mirror: self.on_mirror_result(mirror, goal_id, f))

    def on_mirror_result(self, mirror, goal_id, future):
        mirror.goals.pop(goal_id, None)
        response = future.result()
        result = response.result
        outcome = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_CANCELED: "was canceled",
                   GoalStatus.STATUS_ABORTED: "aborted"}.get(response.status, f"ended with status {response.status}")
        message = f"Mirrored trajectory on {mirror.fqn} {outcome}"
        if result.error_string:
            message += f": {result.error_string}"
        (self.get_logger().info if response.status == GoalStatus.STATUS_SUCCEEDED else self.get_logger().warn)(message)

    def on_primary_status(self, msg):
        for status in msg.status_list:
            goal_id = bytes(status.goal_info.goal_id.uuid)
            if self.primary_status.get(goal_id) == status.status:
                continue
            self.primary_status[goal_id] = status.status
            if status.status in ENDED_STATES or (self.cancel_on_success and
                                                 status.status == GoalStatus.STATUS_SUCCEEDED):
                self.cancel_mirror_goals(goal_id)
        # the primary keeps finished goals' status for a while; only remember those it still lists
        listed = {bytes(s.goal_info.goal_id.uuid) for s in msg.status_list}
        self.primary_status = {g: s for g, s in self.primary_status.items() if g in listed}

    def cancel_mirror_goals(self, goal_id):
        for mirror in self.mirrors:
            if goal_id not in mirror.goals:
                continue
            handle = mirror.goals[goal_id]
            if handle is None:
                mirror.pending_cancels.add(goal_id)
            else:
                handle.cancel_goal_async()

    # --- services and runstop ---------------------------------------------------------

    def on_primary_service(self, name, msg):
        # Clients with introspection on also publish REQUEST_SENT here; repeat each call once.
        if msg.info.event_type != ServiceEventInfo.REQUEST_RECEIVED:
            return
        for mirror in self.mirrors:
            client = mirror.service_clients[name]
            if not client.service_is_ready():
                self.get_logger().warn(f"Not calling {name} on {mirror.fqn}: service unavailable")
                continue
            client.call_async(Trigger.Request()).add_done_callback(
                lambda f, mirror=mirror: self.log_service_result(mirror, name, f))

    def on_primary_runstop(self, msg):
        self.runstopped = msg.data
        for mirror in self.mirrors:
            if mirror.online:
                self.send_runstop(mirror)

    def send_runstop(self, mirror):
        if not mirror.runstop_client.service_is_ready():
            return
        state = self.runstopped
        mirror.runstop_sent = state
        mirror.runstop_client.call_async(SetBool.Request(data=state)).add_done_callback(
            lambda f, mirror=mirror: self.log_service_result(mirror, f"runstop_the_robot({state})", f))

    def log_service_result(self, mirror, name, future):
        response = future.result()
        if response is None or not response.success:
            message = response.message if response is not None else "no response"
            self.get_logger().warn(f"{name} on {mirror.fqn} failed: {message}")

    # --- parameters -------------------------------------------------------------------

    def is_mirrored_param(self, name):
        return (any(fnmatch(name, p) for p in self.patterns)
                and not any(fnmatch(name, p) for p in self.exclude))

    def should_copy(self, name, value, previous):
        if name.startswith("joint_mode.") and value == "settling":
            return False
        if name == "mode":
            return value in USER_MODES and (previous is None or previous in USER_MODES)
        return True

    def on_parameter_event(self, msg):
        if msg.node != self.primary:
            return
        for param in list(msg.new_parameters) + list(msg.changed_parameters):
            if not self.is_mirrored_param(param.name):
                continue
            value = parameter_value_to_python(param.value)
            previous = self.primary_params.get(param.name)
            self.primary_params[param.name] = value
            if not self.should_copy(param.name, value, previous):
                continue
            for mirror in self.mirrors:
                if mirror.synced:
                    self.set_mirror_param(mirror, param)

    def set_mirror_param(self, mirror, param):
        if mirror.param_names is not None and param.name not in mirror.param_names:
            return
        request = SetParameters.Request(parameters=[ParameterMsg(name=param.name, value=param.value)])
        mirror.set_params_client.call_async(request).add_done_callback(
            lambda f: self.log_param_result(mirror, param.name, f))

    def log_param_result(self, mirror, name, future):
        response = future.result()
        if response is None:
            self.get_logger().warn(f"Setting {name} on {mirror.fqn}: no response")
        elif not response.results[0].successful:
            self.get_logger().warn(f"{mirror.fqn} rejected {name}: {response.results[0].reason}")

    def sync_parameters(self, mirror):
        """Start copying the primary's current whitelisted values to a mirror that just
        appeared; False if the primary or mirror isn't ready yet."""
        if not (self.primary_list_client.service_is_ready() and mirror.list_params_client.service_is_ready()):
            self.get_logger().warn(f"Can't sync parameters to {mirror.fqn} yet; will retry",
                                   throttle_duration_sec=10.0)
            return False

        def on_mirror_list(future):
            mirror.param_names = set(future.result().result.names)
            self.primary_list_client.call_async(ListParameters.Request()).add_done_callback(on_primary_list)

        def on_primary_list(future):
            names = [n for n in future.result().result.names if self.is_mirrored_param(n)]
            self.primary_get_client.call_async(GetParameters.Request(names=names)).add_done_callback(
                lambda f: on_primary_values(names, f))

        def on_primary_values(names, future):
            for name, value in zip(names, future.result().values):
                python_value = parameter_value_to_python(value)
                self.primary_params[name] = python_value
                if self.should_copy(name, python_value, None):
                    self.set_mirror_param(mirror, ParameterMsg(name=name, value=value))
            mirror.synced = True
            self.get_logger().info(f"Synced {len(names)} parameters from {self.primary} to {mirror.fqn}")

        mirror.list_params_client.call_async(ListParameters.Request()).add_done_callback(on_mirror_list)
        return True


def main():
    rclpy.init()
    node = TwinRelay()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
