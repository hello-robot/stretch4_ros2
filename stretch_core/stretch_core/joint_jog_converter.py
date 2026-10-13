#! /usr/bin/env python3
"""Compatibility shim for the deprecated joint_vel topic.

Before the sim/real API alignment (PR #42), ranged-joint velocity control was published as
control_msgs/JointJog on joint_vel and handled by StretchDriver.velocity_callback. That
topic is gone; the driver now takes sensor_msgs/JointState on joint_velocity_cmd with the
Stretch4ROSDriver.velocity_cmd_callback.

Four differences between the old and new paradigms are handled here:

  * Joint names. joint_vel shares the driver's joint names apart from the gripper,
    which it may also call by the tool's names ('stretch_gripper_joint', or a finger
    joint such as 'gripper_finger_left_joint') rather than 'gripper_joint'.
  * Control mode. Velocity used to be a global driver mode. It is now per joint,
    so this node sets joint_mode.<joint> before forwarding a command: 'velocity' for
    the ranged joints, and 'position' for the wrists and gripper, whose legacy
    behavior is reproduced with position commands (see below). The driver also
    changes these modes on its own -- selecting the deprecated 'velocity' robot mode
    puts every velocity joint, wrists and gripper included, into velocity mode, which
    is exactly what the clients this shim exists for do -- so joint_mode.<joint> is
    tracked on /parameter_events instead of being assumed to still hold whatever this
    node last set it to.
  * JointJog.duration. Previously, joint velocities were sent with a time period,
    after which the robot stops. The new driver is designed to command velocities
    indefinitely, so this node runs its own watchdog and publishes a zero velocity
    when the duration expires to mimic the old expected behavior.
  * End-of-arm joint velocity commands. The deprecated driver handled velocity commands with
    EndOfArm.move_by commands, moving by the commanded velocity times JointJog.duration (for
    the gripper, in URDF units, i.e. one finger's). This node reproduces that by sending
    joint_position_cmds: move_to(pos + duration*v), the gripper's bounded by its joint limits.

"""

import threading

import rclpy
from control_msgs.msg import JointJog
from rcl_interfaces.msg import (
    Parameter,
    ParameterEvent,
    ParameterType,
    ParameterValue,
)
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_parameter_events
from sensor_msgs.msg import JointState

GRIPPER_JOINT = "gripper_joint"

# Other names the gripper went by: the tools' stretch4_body names and their URDF joints
# (the Stretch gripper's and the parallel gripper's).
JOINT_NAME_ALIASES = {
    name: GRIPPER_JOINT
    for name in (
        "gripper", "gripper_aperture",
        "stretch_gripper", "stretch_gripper_joint",
        "parallel_gripper", "parallel_gripper_joint",
        "gripper_finger_left_joint", "gripper_finger_right_joint",
        "finger_left_joint", "finger_right_joint",
    )
}

GRIPPER_LIMIT_PARAMS = (f"joint_limit.{GRIPPER_JOINT}.lower", f"joint_limit.{GRIPPER_JOINT}.upper")

# joint_vel carries a displacement rather than a velocity for these, so they are
# forwarded as absolute positions. See forward_wrist.
WRIST_JOINTS = ("wrist_yaw_joint", "wrist_pitch_joint", "wrist_roll_joint")


class JointJogConverter(Node):
    def __init__(self):
        super().__init__("joint_jog_converter")

        self.declare_parameter("input_topic", "joint_vel")
        self.declare_parameter("output_topic", "joint_velocity_cmd")
        # Empty means "find the driver in this node's namespace"; set it to a node name
        # (in this namespace) or a fully qualified name ('/sim/stretch_mujoco_driver').
        self.declare_parameter("driver_node", "")
        # Namespace of the command topics, as for the driver's command_namespace:
        # '' = this node's namespace, '/' = root, 'foo' = /foo.
        self.declare_parameter("command_namespace", "")
        self.declare_parameter("discovery_period", 1.0)
        self.declare_parameter("watchdog_rate", 50.0)
        self.declare_parameter("default_duration", 0.5)
        self.declare_parameter("max_duration", 2.0)
        self.declare_parameter("position_topic", "joint_position_cmd")
        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("warn_period", 30.0)

        self.input_topic = self._command_topic(self.get_parameter("input_topic").value)
        self.output_topic = self._command_topic(self.get_parameter("output_topic").value)
        self.position_topic = self._command_topic(self.get_parameter("position_topic").value)
        self.warn_period = self.get_parameter("warn_period").value

        # Deprecation warnings: loud on first use, throttled afterwards.
        self._warned = {}

        # rclpy.time.Time at which a zero velocity should be published.
        self._deadlines = {}
        self._deadline_lock = threading.Lock()


        # Handle joint mode switching
        self._joint_modes = {}
        self._mode_requests = {}
        self._mode_lock = threading.Lock()
        self._pending = {}
        self._pending_lock = threading.Lock()
        self._rejected = {}

        # Track EOA joint states.
        self._wrist_states = {}
        self._wrist_state_lock = threading.Lock()

        # The driver's joint_limit.gripper_joint.lower/upper, once known.
        self._gripper_limits = {}
        self._gripper_limits_lock = threading.Lock()
        self._get_param_client = None

        # Driver discovery: the same shim serves stretch_driver and
        # stretch_mujoco_driver, so the node name is found rather than assumed.
        self._driver_node = None
        self._driver_namespace = None
        # Fully qualified driver name, to match ParameterEvent.node.
        self._driver_fqn = None
        self._param_client = None
        self._discovery_lock = threading.Lock()

        self.vel_pub = self.create_publisher(JointState, self.output_topic, 10)
        self.pos_pub = self.create_publisher(JointState, self.position_topic, 10)
        self.create_subscription(
            JointJog, self.input_topic, self.joint_jog_callback, 10
        )
        self.create_subscription(
            JointState,
            self.get_parameter("joint_states_topic").value,
            self.joint_states_callback,
            10,
        )
        self.create_subscription(
            ParameterEvent,
            "/parameter_events",
            self.parameter_event_callback,
            qos_profile_parameter_events,
        )

        self.discover_driver_node()
        self.create_timer(
            self.get_parameter("discovery_period").value, self.discover_driver_node
        )
        self.create_timer(
            1.0 / self.get_parameter("watchdog_rate").value, self.watchdog_callback
        )

        self.get_logger().warn(
            f"Deprecation shim active: forwarding {self.input_topic} "
            f"(control_msgs/JointJog) to {self.output_topic}, and wrist and gripper "
            f"commands to {self.position_topic} (both sensor_msgs/JointState). "
            f"Publishers should migrate to {self.output_topic}; this node will be "
            "removed in a future release."
        )

    # --- end of arm joints (moved by a bounded displacement) -----------------

    def joint_states_callback(self, msg: JointState):
        for joint in (*WRIST_JOINTS, GRIPPER_JOINT):
            try:
                index = msg.name.index(joint)
            except ValueError:
                continue
            if index < len(msg.position):
                with self._wrist_state_lock:
                    self._wrist_states[joint] = msg.position[index]

    def forward_wrist(self, joint, velocity, duration, original_name):
        """Reproduce move_by(joint, v * duration) as an absolute position command.

        `velocity` is the raw JointJog value and `duration` the message's (clamped)
        duration in seconds; their product is the displacement (in radians, or the
        gripper's URDF units). Publishes a target rather than a velocity and arms no
        watchdog, because move_by moved a bounded amount and stopped. A zero command
        holds the current position, standing in for the quick_stop the deprecated
        callback issued. The wrists' joint limits are left to the driver, which
        enforces joint_limit.<joint>.upper/lower on this topic; the gripper's target is
        bounded by them instead, as move_by stopped at the end of its range.
        """
        if not self.request_joint_mode(joint, "position", original_name):
            with self._pending_lock:
                self._pending[joint] = (
                    lambda j=joint, v=velocity, d=duration, o=original_name:
                    self.forward_wrist(j, v, d, o)
                )
            return

        with self._wrist_state_lock:
            current = self._wrist_states.get(joint)

        if current is None:
            self.log_throttled(
                f"wrist_no_state_{joint}",
                f"{self.input_topic}: dropping the command for '{original_name}' "
                f"because no {joint} position has been seen on joint_states yet. "
                "The old callback moved by a delta, so an absolute target needs one.",
                level="error",
            )
            return

        target = current + velocity * duration
        if joint == GRIPPER_JOINT:
            target = self._bound_gripper_target(target)

        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = [joint]
        command.position = [float(target)]
        self.pos_pub.publish(command)

    def _bound_gripper_target(self, target):
        """Bound a gripper target by the driver's joint limits, if known yet."""
        with self._gripper_limits_lock:
            lower = self._gripper_limits.get(GRIPPER_LIMIT_PARAMS[0])
            upper = self._gripper_limits.get(GRIPPER_LIMIT_PARAMS[1])
        if lower is None or upper is None:
            self._request_gripper_limits()
            return target
        return min(max(target, lower), upper)

    def _request_gripper_limits(self):
        """Ask the driver for its gripper joint limits (answered asynchronously)."""
        with self._discovery_lock:
            client = self._get_param_client
        if client is None or not client.service_is_ready():
            return
        request = GetParameters.Request(names=list(GRIPPER_LIMIT_PARAMS))
        client.call_async(request).add_done_callback(self._gripper_limits_response)

    def _gripper_limits_response(self, future):
        try:
            values = future.result().values
        except Exception as e:
            self.get_logger().warn(f"Could not read the driver's gripper limits: {e}")
            return
        for name, value in zip(GRIPPER_LIMIT_PARAMS, values):
            self._store_gripper_limit(name, value)

    def _store_gripper_limit(self, name, value):
        if value.type != ParameterType.PARAMETER_DOUBLE:
            return
        with self._gripper_limits_lock:
            self._gripper_limits[name] = value.double_value

    # --- deprecation warnings -------------------------------------------------

    def log_throttled(self, key, message, level="warn"):
        """Log loudly the first time, then at most once per warn_period.

        rclpy's throttle_duration_sec keys off the call site and always emits on
        its first hit, which double-logs a "loud once, quiet after" pattern.
        """
        now = self.get_clock().now()
        last = self._warned.get(key)
        if last is not None and (now - last) < Duration(seconds=self.warn_period):
            return
        self._warned[key] = now
        # Separate call sites per level: rclpy caches severity by call site and
        # raises if one site is used for more than one severity.
        if level == "error":
            self.get_logger().error(message)
        else:
            self.get_logger().warn(message)

    def warn_deprecated(self, key, message):
        self.log_throttled(key, message)

    def _command_topic(self, name):
        """`name` in the command namespace (unchanged if it is unset or `name` is absolute)."""
        ns = self.get_parameter("command_namespace").value.strip()
        if not ns or name.startswith("/"):
            return name
        ns = ns.strip("/")
        return f"/{ns}/{name}" if ns else f"/{name}"

    # --- driver discovery -----------------------------------------------------

    def discover_driver_node(self):
        """Locate the driver by the home_the_robot service it advertises.

        Only a driver in this node's namespace is considered, unless driver_node is a
        fully qualified name, so that with several robots on one graph each converter
        controls its own driver.
        """
        with self._discovery_lock:
            configured = self.get_parameter("driver_node").value
            own_namespace = self.get_namespace()
            try:
                for name, namespace in self.get_node_names_and_namespaces():
                    if name == self.get_name() and namespace == own_namespace:
                        continue
                    fqn = f"{namespace.rstrip('/')}/{name}"
                    if configured.startswith("/"):
                        if fqn != configured:
                            continue
                    elif namespace != own_namespace or (configured and name != configured):
                        continue
                    try:
                        services = self.get_service_names_and_types_by_node(
                            name, namespace
                        )
                    except Exception:
                        continue
                    for service_name, service_types in services:
                        if (
                            service_name.endswith("/home_the_robot")
                            and "std_srvs/srv/Trigger" in service_types
                        ):
                            self._bind_driver(name, namespace)
                            return True
            except Exception as e:
                self.get_logger().error(f"Error during driver node discovery: {e}")
            return False

    def _bind_driver(self, name, namespace):
        if self._driver_node == name and self._driver_namespace == namespace:
            return
        prefix = namespace if namespace != "/" else ""
        service = f"{prefix}/{name}/set_parameters"
        fqn = f"{prefix}/{name}"
        self.get_logger().info(
            f"Discovered driver node '{name}' in namespace '{namespace}'; "
            f"using '{service}' to set joint control modes."
        )
        if self._param_client is not None:
            self.destroy_client(self._param_client)
            self.destroy_client(self._get_param_client)
        self._driver_node = name
        self._driver_namespace = namespace
        self._driver_fqn = fqn
        self._param_client = self.create_client(SetParameters, service)
        self._get_param_client = self.create_client(GetParameters, f"{prefix}/{name}/get_parameters")
        with self._gripper_limits_lock:
            self._gripper_limits.clear()
        # A restarted driver has forgotten whatever modes we set previously, and any
        # request that was in flight died with it.
        with self._mode_lock:
            self._joint_modes.clear()
            self._mode_requests.clear()
            self._rejected.clear()

    # --- joint mode -----------------------------------------------------------

    def request_joint_mode(self, joint, mode, original_name=None):
        """Ask the driver for joint_mode.<joint> = mode. Returns True once the driver is known to be in that mode/
        """
        with self._mode_lock:
            if self._mode_requests.get(joint) is not None:
                return False
            if self._joint_modes.get(joint) == mode:
                return True
            refused = self._rejected.get(joint)
            if refused is not None and (self.get_clock().now() - refused) < Duration(
                seconds=self.warn_period
            ):
                return False
            self._mode_requests[joint] = mode

        with self._discovery_lock:
            client = self._param_client
            driver = self._driver_node

        if client is None:
            self.get_logger().warn(
                "Cannot set joint control mode: no driver node discovered yet.",
                throttle_duration_sec=5.0,
            )
            with self._mode_lock:
                self._mode_requests.pop(joint, None)
            return False

        if not client.service_is_ready():
            self.get_logger().warn(
                f"Parameter service of driver '{driver}' is not ready yet.",
                throttle_duration_sec=5.0,
            )
            with self._mode_lock:
                self._mode_requests.pop(joint, None)
            return False

        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name=f"joint_mode.{joint}",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING, string_value=mode
                ),
            )
        ]

        self.get_logger().info(f"Requesting joint_mode.{joint} = '{mode}'...")
        future = client.call_async(request)
        future.add_done_callback(
            lambda f, j=joint, m=mode, o=original_name: self._mode_response(f, j, m, o)
        )
        return False

    def _mode_response(self, future, joint, mode, original_name=None):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to set joint_mode.{joint}: {e}")
            with self._mode_lock:
                self._mode_requests.pop(joint, None)
            return

        if result and result.results and result.results[0].successful:
            self.get_logger().info(f"joint_mode.{joint} is now '{mode}'.")
            with self._mode_lock:
                self._mode_requests.pop(joint, None)
                self._joint_modes[joint] = mode
                self._rejected.pop(joint, None)
            self.release_pending(joint)
            return

        reason = result.results[0].reason if (result and result.results) else "unknown"
        source = f"'{original_name}'" if original_name else f"'{joint}'"
        self.log_throttled(
            f"mode_rejected.{joint}",
            f"{self.input_topic}: cannot forward commands for joint {source} -- the "
            f"driver rejected joint_mode.{joint} = '{mode}' ({reason}). The joint is "
            f"either unknown to the driver or does not support that mode; the "
            f"deprecated velocity_callback ignored such joints too.",
            level="error",
        )
        with self._mode_lock:
            self._mode_requests.pop(joint, None)
            self._joint_modes.pop(joint, None)
            self._rejected[joint] = self.get_clock().now()
        with self._pending_lock:
            self._pending.pop(joint, None)

    def parameter_event_callback(self, msg: ParameterEvent):
        """Check for joint modes changed via parameters."""
        with self._discovery_lock:
            driver_fqn = self._driver_fqn
        if driver_fqn is None or msg.node != driver_fqn:
            return

        for parameter in list(msg.changed_parameters) + list(msg.new_parameters):
            if parameter.name in GRIPPER_LIMIT_PARAMS:
                self._store_gripper_limit(parameter.name, parameter.value)
                continue
            if not parameter.name.startswith("joint_mode."):
                continue
            if parameter.value.type != ParameterType.PARAMETER_STRING:
                continue

            joint = parameter.name[len("joint_mode."):]
            mode = parameter.value.string_value
            if mode == "settling":
                continue

            with self._mode_lock:
                if self._joint_modes.get(joint) == mode:
                    continue
    
                self._joint_modes[joint] = mode
    
            self.get_logger().info(
                f"Changed joint_mode.{joint} to '{mode}'."
            )

    def release_pending(self, joint):
        """Replay the command held for a joint while its mode change was in flight."""
        with self._pending_lock:
            replay = self._pending.pop(joint, None)
        if replay is not None:
            replay()

    # --- forwarding -----------------------------------------------------------

    def resolve_joint_name(self, name):
        return JOINT_NAME_ALIASES.get(name, name)

    def joint_jog_callback(self, msg: JointJog):
        self.warn_deprecated(
            "joint_vel",
            f"The {self.input_topic} message of type control_msgs/JointJog has been "
            f"deprecated. Please switch to {self.output_topic} with type "
            f"sensor_msgs/JointState.",
        )

        if any(d != 0.0 for d in msg.displacements):
            self.warn_deprecated(
                "displacements",
                f"{self.input_topic}: JointJog.displacements[] is ignored, as it was "
                f"by the deprecated driver callback. Use joint_position_cmd "
                f"(sensor_msgs/JointState) for position commands.",
            )

        duration = (
            msg.duration
            if msg.duration > 0.0
            else self.get_parameter("default_duration").value
        )
        duration = min(duration, self.get_parameter("max_duration").value)

        joints = []
        velocities = []

        for i, name in enumerate(msg.joint_names):
            if i >= len(msg.velocities):
                self.get_logger().warn(
                    f"{self.input_topic}: no velocity for joint '{name}'; ignoring."
                )
                continue

            joint = self.resolve_joint_name(name)
            velocity = float(msg.velocities[i])

            if joint in WRIST_JOINTS or joint == GRIPPER_JOINT:
                # A displacement rather than a velocity; see forward_wrist.
                self.forward_wrist(joint, velocity, duration, name)
                continue

            if not self.request_joint_mode(joint, "velocity", original_name=name):
                # Hold the command; _mode_response replays it once the driver
                # confirms the mode.
                with self._pending_lock:
                    self._pending[joint] = (
                        lambda j=joint, v=velocity, d=duration, o=name:
                        self.forward_velocity(j, v, d, o)
                    )
                continue

            joints.append(joint)
            velocities.append(velocity)

        self.publish_velocities(joints, velocities, duration)

    def forward_velocity(self, joint, velocity, duration, original_name):
        """Forward one ranged-joint velocity, re-checking joint_mode first.

        Only reached when a command was held for a mode change; the common path
        batches every joint of a JointJog into a single publish.
        """
        if not self.request_joint_mode(joint, "velocity", original_name):
            with self._pending_lock:
                self._pending[joint] = (
                    lambda j=joint, v=velocity, d=duration, o=original_name:
                    self.forward_velocity(j, v, d, o)
                )
            return

        self.publish_velocities([joint], [velocity], duration)

    def publish_velocities(self, joints, velocities, duration):
        """Publish a joint_velocity_cmd and arm the JointJog.duration watchdog."""
        if not joints:
            return

        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = list(joints)
        command.velocity = list(velocities)
        self.vel_pub.publish(command)

        deadline = self.get_clock().now() + Duration(seconds=duration)
        with self._deadline_lock:
            for joint in joints:
                self._deadlines[joint] = deadline

    # --- duration watchdog ----------------------------------------------------

    def watchdog_callback(self):
        """Publish a zero velocity for joints whose JointJog.duration has elapsed."""
        now = self.get_clock().now()
        with self._deadline_lock:
            expired = [j for j, deadline in self._deadlines.items() if now >= deadline]
            for joint in expired:
                del self._deadlines[joint]

        if not expired:
            return

        # A joint the driver has since moved out of velocity mode is already stopped:
        # change_joint_mode zeroes its velocity on the way out. Commanding it anyway
        # would only earn a "cannot send velocity command" warning per expiry.
        with self._mode_lock:
            expired = [
                j for j in expired if self._joint_modes.get(j) == "velocity"
            ]

        if not expired:
            return

        command = JointState()
        command.header.stamp = now.to_msg()
        command.name = expired
        command.velocity = [0.0] * len(expired)
        self.get_logger().debug(
            f"JointJog duration elapsed for {expired}; commanding zero velocity."
        )
        self.vel_pub.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = JointJogConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
