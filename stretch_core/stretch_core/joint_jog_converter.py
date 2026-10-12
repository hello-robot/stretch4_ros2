#! /usr/bin/env python3
"""Compatibility shim for the deprecated joint_vel topic.

Before the sim/real API alignment (PR #42), ranged-joint velocity control was published as
control_msgs/JointJog on joint_vel and handled by StretchDriver.velocity_callback. That
topic is gone; the driver now takes sensor_msgs/JointState on joint_velocity_cmd with the
Stretch4ROSDriver.velocity_cmd_callback.

Four differences between the old and new paradigms are handled here:

  * Joint names. joint_vel shares the driver's joint names apart from the gripper,
    which it calls 'stretch_gripper_joint' rather than 'gripper_joint'.
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
  * End-of-arm joint velocity commands. The deprecated drived handled velocity commands with
    EndOfArm.move_by commands. It estimated displacement by multiplying commanded velocity
    by JointJog.duration (wrists) or 300 (gripper). This node reproduces the expected velocity
    command behavior by sending joint_position_cmds: move_to(pos + duration*v) for wrists and
    move_to(pos_pct + 300*v) for the gripper.

"""

import math
import threading

import rclpy
from control_msgs.msg import JointJog
from rcl_interfaces.msg import (
    Parameter,
    ParameterEvent,
    ParameterType,
    ParameterValue,
)
from rcl_interfaces.srv import SetParameters
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_parameter_events
from sensor_msgs.msg import JointState

try:
    # Only needed to reproduce legacy gripper commands. Absent on machines that run
    # the simulator without stretch4_body; the rest of the shim works without it.
    from stretch4_body.subsystem.end_of_arm.gripper_conversion import GripperConversion
except Exception:
    GripperConversion = None

JOINT_NAME_ALIASES = {
    "stretch_gripper_joint": "gripper_joint",
    "gripper": "gripper_joint",
    "gripper_aperture": "gripper_joint",
}

# The deprecated velocity_callback multiplied gripper commands by this before handing
# them to EndOfArm.move_by as an aperture-percentage displacement. Reproducing the old
# motion means reproducing that displacement, so the factor is kept exactly.
DEFAULT_GRIPPER_LEGACY_SCALE = 300.0

# joint_states reports the gripper via GripperConversion.status_to_all, which halves the
# aperture angle ("per finger"), while joint_position_cmd is consumed by finger_to_servo,
# which does not. Both maps are affine, so the two differ by a constant. Exposed as a
# parameter because it encodes a driver-side inconsistency rather than a physical value.
DEFAULT_GRIPPER_STATE_TO_COMMAND = 2.0

# joint_states names for the gripper, in preference order (see GripperCommandGroup).
GRIPPER_STATE_NAMES = ('gripper_finger_left_joint', 'gripper_finger_right_joint')

# joint_vel carries a displacement rather than a velocity for these, so they are
# forwarded as absolute positions. See forward_wrist.
WRIST_JOINTS = ("wrist_yaw_joint", "wrist_pitch_joint", "wrist_roll_joint")


class JointJogConverter(Node):
    def __init__(self):
        super().__init__("joint_jog_converter")

        self.declare_parameter("input_topic", "joint_vel")
        self.declare_parameter("output_topic", "joint_velocity_cmd")
        # Empty means "find the driver on the graph"; set it to skip the search.
        self.declare_parameter("driver_node", "")
        self.declare_parameter("discovery_period", 1.0)
        self.declare_parameter("watchdog_rate", 50.0)
        self.declare_parameter("default_duration", 0.5)
        self.declare_parameter("max_duration", 2.0)
        self.declare_parameter("position_topic", "joint_position_cmd")
        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("gripper_legacy_scale", DEFAULT_GRIPPER_LEGACY_SCALE)
        self.declare_parameter(
            "gripper_state_to_command_scale", DEFAULT_GRIPPER_STATE_TO_COMMAND
        )
        self.declare_parameter("warn_period", 30.0)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.position_topic = self.get_parameter("position_topic").value
        self.warn_period = self.get_parameter("warn_period").value
        self.gripper_legacy_scale = self.get_parameter("gripper_legacy_scale").value
        self.gripper_state_to_command = self.get_parameter(
            "gripper_state_to_command_scale"
        ).value

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
        self._gripper_state = None
        self._gripper_state_lock = threading.Lock()
        self._wrist_states = {}
        self._wrist_state_lock = threading.Lock()

        self._setup_gripper_conversion()

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
            f"(control_msgs/JointJog) to {self.output_topic}, and legacy gripper "
            f"commands to {self.position_topic} (both sensor_msgs/JointState). "
            f"Publishers should migrate to {self.output_topic}; this node will be "
            "removed in a future release."
        )

    # --- legacy gripper reproduction -----------------------------------------

    def _setup_gripper_conversion(self):
        """Load the conversion needed to invert the driver's gripper position path.

        set_joint_position turns a joint_position_cmd value into a servo angle with
        finger_to_servo and then into a percentage. To land the gripper on the same
        percentage the deprecated move_by would have reached, this node runs that
        chain backwards, so it needs the same conversion the driver uses.
        """
        self._gripper_conversion = None
        self._gripper_servo_closed = None
        self._gripper_pct_max_open = None

        if GripperConversion is None:
            self.get_logger().warn(
                "stretch4_body is unavailable, so legacy gripper commands on "
                f"{self.input_topic} cannot be reproduced. Every other joint is "
                "forwarded normally."
            )
            return

        try:
            conversion = GripperConversion()
            servo_closed = conversion.params["servo_closed_angle"]
            servo_open = conversion.params["servo_open_angle"]
        except Exception as e:
            self.get_logger().warn(
                f"Could not load the gripper conversion ({e}); legacy gripper "
                f"commands on {self.input_topic} will not be reproduced."
            )
            return

        self._gripper_conversion = conversion
        self._gripper_servo_closed = servo_closed
        # StretchGripper.pct_max_open; a pct of -100 is fully closed.
        self._gripper_pct_max_open = 100 * abs(servo_open / servo_closed)

    def _command_rad_to_pct(self, command_rad):
        """joint_position_cmd value -> gripper percentage, as set_joint_position does."""
        servo_deg = self._gripper_conversion.finger_to_servo(command_rad)
        return -100.0 * math.radians(servo_deg) / math.radians(self._gripper_servo_closed)

    def _pct_to_command_rad(self, pct):
        """Gripper percentage -> joint_position_cmd value (the inverse of the above)."""
        servo_deg = math.degrees(
            pct * math.radians(self._gripper_servo_closed) / -100.0
        )
        return self._gripper_conversion.servo_to_finger(servo_deg)

    def joint_states_callback(self, msg: JointState):
        for joint in WRIST_JOINTS:
            try:
                index = msg.name.index(joint)
            except ValueError:
                continue
            if index < len(msg.position):
                with self._wrist_state_lock:
                    self._wrist_states[joint] = msg.position[index]

        for name in GRIPPER_STATE_NAMES:
            try:
                index = msg.name.index(name)
            except ValueError:
                continue
            if index < len(msg.position):
                with self._gripper_state_lock:
                    self._gripper_state = msg.position[index]
            return

    def forward_wrist(self, joint, velocity, duration, original_name):
        """Reproduce move_by(joint, v * duration) as an absolute position command.

        `velocity` is the raw JointJog value and `duration` the message's (clamped)
        duration in seconds; their product is the displacement in radians. Publishes a
        target rather than a velocity and arms no watchdog, because move_by moved a
        bounded amount and stopped. A zero command holds the current position, standing
        in for the quick_stop the deprecated callback issued. Joint limits are left to
        the driver, which enforces joint_limit.<joint>.upper/lower on this topic.
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

        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = [joint]
        command.position = [float(current + velocity * duration)]
        self.pos_pub.publish(command)

    def forward_gripper(self, velocity, original_name):
        """Reproduce move_by(stretch_gripper, 300 * v) as an absolute position command.

        The deprecated path moved a bounded percentage and stopped, so this publishes a
        target rather than a velocity and arms no watchdog. A zero command stands in for
        the quick_stop the old callback issued: it holds the current position.
        """
        if self._gripper_conversion is None:
            self.log_throttled(
                "gripper_unavailable",
                f"{self.input_topic}: dropping the command for '{original_name}' "
                "because the gripper conversion could not be loaded.",
                level="error",
            )
            return

        with self._gripper_state_lock:
            state = self._gripper_state

        if not self.request_joint_mode("gripper_joint", "position", original_name):
            with self._pending_lock:
                self._pending["gripper_joint"] = (
                    lambda v=velocity, o=original_name: self.forward_gripper(v, o)
                )
            return

        if state is None:
            self.log_throttled(
                "gripper_no_state",
                f"{self.input_topic}: dropping the command for '{original_name}' "
                "because no gripper position has been seen on joint_states yet. The "
                "old callback moved by a delta, so an absolute target needs one.",
                level="error",
            )
            return

        current_pct = self._command_rad_to_pct(state * self.gripper_state_to_command)
        target_pct = current_pct + self.gripper_legacy_scale * velocity
        # StretchGripper.move_to would drive past the mechanism's range; the driver's
        # own joint limits are in finger units, so bound the percentage here instead.
        target_pct = min(max(target_pct, -100.0), self._gripper_pct_max_open)

        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = ["gripper_joint"]
        command.position = [float(self._pct_to_command_rad(target_pct))]
        self.pos_pub.publish(command)

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

    # --- driver discovery -----------------------------------------------------

    def discover_driver_node(self):
        """Locate the driver by the home_the_robot service it advertises.

        Set the driver_node parameter to skip the graph search when the node name
        is known (for example under a namespace with more than one driver).
        """
        with self._discovery_lock:
            configured = self.get_parameter("driver_node").value
            try:
                for name, namespace in self.get_node_names_and_namespaces():
                    if name == self.get_name():
                        continue
                    if configured and name != configured:
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
        self._driver_node = name
        self._driver_namespace = namespace
        self._driver_fqn = fqn
        self._param_client = self.create_client(SetParameters, service)
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

            if joint == "gripper_joint":
                # Not a velocity on the wire; reproduced as a bounded position move.
                self.forward_gripper(velocity, name)
                continue

            if joint in WRIST_JOINTS:
                # Also a displacement rather than a velocity; see forward_wrist.
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
