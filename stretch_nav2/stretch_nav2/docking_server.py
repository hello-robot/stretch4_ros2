#!/usr/bin/env python3
import cv2
import os
import re
import glob
import time
import threading
import numpy as np
from enum import Enum
from collections import deque
from simple_sm import StateMachine, transition, step
from scipy.spatial.transform import Rotation

from stretch4_docking.trackers.dock_tracker import DockTracker, DockAmbiguityError
from stretch4_docking.costmap import Costmap, RingFilter, filter_clearance_velocity
from stretch4_docking.servo import XYThetaServo, Mppi
from stretch4_docking.utils import cloud_reader, ensure_stow

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_srvs.srv import Trigger
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from sensor_msgs.msg import BatteryState, JointState, PointCloud2, Image
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped
from visualization_msgs.msg import MarkerArray, Marker
from nav2_msgs.action import DockRobot
from nav2_msgs.srv import ReloadDockDatabase
PREEMPTED_ERROR_CODE = 800 # our own DockRobot.Result type to mean preempted
FAILED_TO_STOW_ERROR_CODE = 801 # couldn't stow the arm


def current_cpu() -> int:
    """Which core is this thread on right now? -1 if the kernel will not say."""
    try:
        with open('/proc/thread-self/stat', 'rb') as f:
            # comm can contain spaces and parens, so cut at the last ')' before splitting
            fields = f.read().rsplit(b')', 1)[1].split()
        return int(fields[36])  # 'processor', field 39 counting the two before comm
    except Exception:
        return -1


def fastest_cpus() -> list[int]:
    """The CPUs with the highest max clock.

    This machine is a hybrid part: 4 P-cores at 4.9GHz, 8 E-cores at 4.4, and 2
    low-power E-cores at 2.5. Every stage of the servo pipeline is single-threaded
    (numba njit without parallel=True, scipy's EDT), so the stage times are set by
    which core the callback lands on. Left alone under nav2's load the control
    callback drifts onto the LP island and the loop takes 3x as long for identical
    work. Grouping by clock rather than hardcoding 0-3 keeps this right on other
    machines.
    """
    speeds: dict[int, int] = {}
    for path in glob.glob('/sys/devices/system/cpu/cpu[0-9]*/cpufreq/cpuinfo_max_freq'):
        try:
            cpu = int(re.search(r'cpu(\d+)', path).group(1))
            with open(path) as f:
                speeds[cpu] = int(f.read().strip())
        except Exception:
            continue
    if not speeds:
        return []
    top = max(speeds.values())
    return sorted(cpu for cpu, khz in speeds.items() if khz == top)


from hello_helpers.hello_ros_viz import (
    construct_frame,
    construct_frame_se3,
    construct_twist,
    construct_colored_cloud,
    construct_triangle,
    construct_grid_marker,
    construct_image,
)

from stretch_nav2.dock_database import DockDatabase
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from message_filters import SimpleFilter, ApproximateTimeSynchronizer
import tf2_ros


class AutodockState(Enum):
    READY = 1
    STOWING = 2
    STAGING = 3
    STAGING_AND_SCANNING = 4
    SERVOING = 5
    BLIND_DOCKING = 6
    COMPLETED = 7

class ServoingResult(Enum):
    SUCCESS = 1
    MULTIPLE_DOCK_AMBIGUITY = 2

class DockFailure(Exception):
    def __init__(self, error_code, msg):
        super().__init__(msg)
        self.error_code, self.msg = error_code, msg

class Cancelled(Exception): pass
class Preempted(Exception): pass


class DockRobotAction:

    def __init__(self, node: "DockingServer"):
        self.node = node
        self.rate = self.node.create_rate(100)

        # Initialize DockRobot action
        goal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.dock_action = ActionServer(
            self.node,
            DockRobot,
            'dock_robot',
            execute_callback=self.execute_cb,
            cancel_callback=self.cancel_cb,
            callback_group=node.action_group, # Supports multiple requests, new preempting the old
            goal_service_qos_profile=goal_qos,
        )

        # Feedback
        self.feedback_states = {
            AutodockState.READY: DockRobot.Feedback.NONE,
            AutodockState.STOWING: DockRobot.Feedback.NONE,
            AutodockState.STAGING: DockRobot.Feedback.NAV_TO_STAGING_POSE,
            AutodockState.STAGING_AND_SCANNING: DockRobot.Feedback.INITIAL_PERCEPTION,
            AutodockState.SERVOING: DockRobot.Feedback.CONTROLLING,
            AutodockState.BLIND_DOCKING: DockRobot.Feedback.WAIT_FOR_CHARGE,
            AutodockState.COMPLETED: DockRobot.Feedback.NONE,
        }
        self.reacquire_count = 0
        self.goal_start = None
        self.last_feedback_time = 0.0

        self.latest_goal_uuid: bytes | None = None  # Used as a signal for preemption
        self.latest_goal_lock = threading.Lock()
        self.goal_idle = threading.Event()        # if set, no execute_dock thread owns the robot
        self.goal_idle.set()

        # FSM
        self.machine = StateMachine(
            states=AutodockState,
            initial=AutodockState.READY,
            model=self,
        )

    @transition(source=AutodockState.READY, dest=AutodockState.STOWING)
    def start_stowing(self, goal):
        self.pending_goal = goal
        self.reacquire_count = 0
        self.stow_future = None
        self.stow_settled_after = None
        self.stow_deadline = time.time() + 30.0

    @transition(source=AutodockState.STOWING, dest=AutodockState.STAGING)
    def start_navigating(self, goal):
        self.setup_prestage_nav(goal)

    @transition(source=AutodockState.STOWING, dest=AutodockState.SERVOING)
    def skip_to_servoing(self):
        self.node.reset_servo()
        self.setup_servo_deadline()

    @transition(source=AutodockState.STAGING, dest=AutodockState.STAGING_AND_SCANNING)
    def start_scanning(self):
        self.node.dock_confirm_count = 0
        self.scanning_deadline = time.time() + 5.0

    @transition(source=AutodockState.STAGING_AND_SCANNING, dest=AutodockState.SERVOING)
    def dock_found(self):
        self.node.reset_servo()
        self.setup_servo_deadline()

    @transition(source=AutodockState.SERVOING, dest=AutodockState.STAGING_AND_SCANNING)
    def dock_lost(self):
        """Hand back to Nav2 after losing the dock mid-servo, and scan on the way in again."""
        self.node.dock_confirm_count = 0
        self.setup_prestage_nav(self.pending_goal)
        self.scanning_deadline = time.time() + 5.0

    @transition(source=AutodockState.SERVOING, dest=AutodockState.BLIND_DOCKING)
    def se2_servo_done(self):
        self.init_bd_routine()

    @transition(source=AutodockState.BLIND_DOCKING, dest=AutodockState.COMPLETED)
    def bd_done(self):
        pass

    @transition(source=AutodockState.SERVOING, dest=AutodockState.COMPLETED)
    def mppi_servo_done(self):
        pass


    def cancel_cb(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        # Check if this goal is active
        with self.latest_goal_lock:
            if self.latest_goal_uuid == bytes(goal_handle.goal_id.uuid):
                return CancelResponse.ACCEPT

        # Can't cancel an inactive goal
        return CancelResponse.REJECT

    def execute_cb(self, goal_handle: ServerGoalHandle) -> DockRobot.Result:
        self.node.get_logger().debug("Docking requested")

        # Lock required because ReentrantCallbackGroup allows parallel execute_cbs
        my_uuid = bytes(goal_handle.goal_id.uuid)
        with self.latest_goal_lock:
            self.latest_goal_uuid = my_uuid # Newest thread sets the latest UUID

        # Wait for the incumbent to actually let go of the robot
        if not self.goal_idle.wait(timeout=3.0):
            return self.error_result(goal_handle, DockRobot.Result.UNKNOWN, "timed out waiting to preempt previous dock request")
        with self.latest_goal_lock:
            if self.latest_goal_uuid != my_uuid:  # someone newer arrived while we waited
                return self.error_result(goal_handle, PREEMPTED_ERROR_CODE, "this request was preempted")
            self.goal_idle.clear()

        self.goal_start = self.node.get_clock().now()
        self.reacquire_count = 0
        self.last_feedback_time = 0.0

        # FSM stepping logic
        goal: DockRobot.Goal = goal_handle.request
        try:
            # Check already charging
            if self.node.power_supply_status == None:
                raise DockFailure(DockRobot.Result.UNKNOWN, "Unknown power supply status")
            if self.node.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
                return self.success_result(goal_handle, "Already charging.")

            self.start_stowing(goal)

            while self.machine.state is not AutodockState.COMPLETED:
                self.check_interrupts(goal_handle, my_uuid)
                self.machine.step()
                self.publish_feedback(goal_handle)
                self.rate.sleep()

            return self.success_result(goal_handle)
        except Cancelled:
            return self.cancel_result(goal_handle)
        except Preempted:
            return self.error_result(goal_handle, PREEMPTED_ERROR_CODE, "this request was preempted")
        except DockFailure as e:
            return self.error_result(goal_handle, e.error_code, e.msg)
        finally:
            self.node.navigator.cancelTask()
            self.machine.set_state(AutodockState.READY)
            self.goal_idle.set()

    def publish_feedback(self, goal_handle: ServerGoalHandle) -> None:
        now = time.time()
        if now - self.last_feedback_time < 0.2: # 5Hz
            return
        self.last_feedback_time = now

        feedback = DockRobot.Feedback()
        state = self.machine.state
        if state is AutodockState.STAGING_AND_SCANNING and self.reacquire_count > 0:
            # Re-approaching after losing the dock mid-servo is nav2's RETRY
            feedback.state = DockRobot.Feedback.RETRY
        else:
            feedback.state = self.feedback_states.get(state, DockRobot.Feedback.NONE)
        feedback.docking_time = (self.node.get_clock().now() - self.goal_start).to_msg()
        feedback.num_retries = self.reacquire_count
        goal_handle.publish_feedback(feedback)

    def _preempted(self, my_uuid):
        with self.latest_goal_lock:
            return self.latest_goal_uuid != my_uuid

    def check_interrupts(self, goal_handle, my_uuid):
        if goal_handle.is_cancel_requested:
            raise Cancelled
        if self._preempted(my_uuid):
            raise Preempted

    def draw_state_machine(self):
        png = self.machine.draw(None, format='png')
        return cv2.imdecode(np.frombuffer(png, dtype=np.uint8), cv2.IMREAD_COLOR)

    def setup_prestage_nav(self, goal):
        if goal.use_dock_id:
            if goal.dock_id not in self.node.db:
                err = f"Dock ID '{goal.dock_id}' not found in database! {self.node.db}"
                self.node.get_logger().error(err)
                raise DockFailure(DockRobot.Result.DOCK_NOT_IN_DB, err)

            dock_pose = DockDatabase.dict_to_pose_stamped(self.node.db[goal.dock_id]['pose'])
            self.node.navigator.goToPose(dock_pose)
        else:
            self.node.navigator.goToPose(goal.dock_pose)

        self.staging_deadline = time.time() + goal.max_staging_time

    def finish_stowing(self):
        """Leave STOWING for whichever entry point this goal asked for."""
        if self.pending_goal.navigate_to_staging_pose:
            self.start_navigating(self.pending_goal)
        else:
            self.skip_to_servoing()

    @step(AutodockState.STOWING)
    def step_STOWING(self):
        if time.time() > self.stow_deadline:
            raise DockFailure(FAILED_TO_STOW_ERROR_CODE, "Stowing timed out")

        if self.node.joint_positions is None:
            return  # no /joint_states yet

        off = ensure_stow.joints_out_of_stow(self.node.joint_positions, self.node.stow_targets)

        # Nothing requested yet: either we are already stowed, or ask the driver to stow.
        if self.stow_future is None:
            if not off:
                self.finish_stowing()
                return
            self.node.get_logger().info(
                f"Out of stow ({ensure_stow.describe(off)}); stowing before docking")
            if not self.node.stow_robot.service_is_ready():
                raise DockFailure(FAILED_TO_STOW_ERROR_CODE, "stow_the_robot service unavailable")
            self.stow_future = self.node.stow_robot.call_async(Trigger.Request())
            return

        if not self.stow_future.done():
            return

        response = self.stow_future.result()
        if response is None or not response.success:
            reason = response.message if response is not None else "no response"
            raise DockFailure(FAILED_TO_STOW_ERROR_CODE, f"stow_the_robot refused: {reason}")

        # The driver blocks until the stow finishes, so only a reading taken after it returned
        # says anything about whether the stow took.
        if self.stow_settled_after is None:
            self.stow_settled_after = time.time()
            return
        if self.node.joint_state_time < self.stow_settled_after:
            return

        if off:
            raise DockFailure(
                FAILED_TO_STOW_ERROR_CODE, f"Stow did not take ({ensure_stow.describe(off)})")
        self.finish_stowing()

    @step(AutodockState.STAGING)
    def step_STAGING(self):
        if time.time() > self.staging_deadline:
            raise DockFailure(DockRobot.Result.FAILED_TO_STAGE, "Staging timed out")
        if self.node.navigator.isTaskComplete():
            self.start_scanning()
            return

        feedback = self.node.navigator.getFeedback()
        if feedback is not None and 0.0 < feedback.distance_remaining <= 3.0:
            self.start_scanning()

    @step(AutodockState.STAGING_AND_SCANNING)
    def step_STAGING_AND_SCANNING(self):
        if self.node.navigator.isTaskComplete():
            if self.node.navigator.getResult() != TaskResult.SUCCEEDED:
                raise DockFailure(DockRobot.Result.FAILED_TO_STAGE, "Nav2 goal rejected or navigation failed")
            else:
                if time.time() > self.scanning_deadline:
                    raise DockFailure(DockRobot.Result.FAILED_TO_DETECT_DOCK, "Reached staging pose, but no dock found")

        if self.node.dock_confirm_count >= 15:
            self.node.navigator.cancelTask()
            self.dock_found()

    def setup_servo_deadline(self):
        self.servo_deadline = time.time() + 25.0

    @step(AutodockState.SERVOING)
    def step_SERVOING(self):
        if time.time() > self.servo_deadline:
            raise DockFailure(DockRobot.Result.FAILED_TO_CONTROL, "Servoing timed out after 25s")

        dock_reacquire_limit = 2
        if self.node.servo_lost_count >= 10:
            if self.reacquire_count >= dock_reacquire_limit:
                raise DockFailure(
                    DockRobot.Result.FAILED_TO_DETECT_DOCK,
                    f"Lost the dock while servoing {self.reacquire_count + 1} times; giving up")
            self.reacquire_count += 1
            self.node.get_logger().warn(
                f"Lost the dock while servoing; navigating back in "
                f"(attempt {self.reacquire_count}/{dock_reacquire_limit})")
            self.dock_lost()
            return

        if self.node.servo_done.wait(timeout=0.01):
            code, msg = self.node.servo_outcome
            if code == ServoingResult.SUCCESS:
                self.se2_servo_done()
            elif code == ServoingResult.MULTIPLE_DOCK_AMBIGUITY:
                raise DockFailure(DockRobot.Result.FAILED_TO_DETECT_DOCK, msg)

    def init_bd_routine(self):
        self.seating_future = self.node.seat_dock.call_async(Trigger.Request())

    @step(AutodockState.BLIND_DOCKING)
    def step_BLIND_DOCKING(self):
        if not self.seating_future.done():
            return

        # A rejected call resolves with no response at all, so check before dereferencing.
        response = self.seating_future.result()
        if response is None:
            raise DockFailure(
                DockRobot.Result.FAILED_TO_CHARGE, "seat_into_dock returned no response")
        if not response.success:
            raise DockFailure(
                DockRobot.Result.FAILED_TO_CHARGE, f"No electrical contact: {response.message}")
        self.bd_done()

    def error_result(self, goal_handle: ServerGoalHandle, error_code, error_str, log_warn=False):
        if log_warn:
            self.node.get_logger().warn(error_str)
        else:
            self.node.get_logger().error(error_str)
        result = DockRobot.Result()
        result.success = False
        result.error_code = error_code
        result.error_msg = error_str
        result.num_retries = self.reacquire_count
        goal_handle.abort()
        return result

    def success_result(self, goal_handle: ServerGoalHandle, success_str='Success!'):
        self.node.get_logger().debug(success_str)
        result = DockRobot.Result()
        result.success = True
        result.error_code = DockRobot.Result.NONE
        result.error_msg = success_str
        result.num_retries = self.reacquire_count
        goal_handle.succeed()
        return result

    def cancel_result(self, goal_handle: ServerGoalHandle):
        self.node.get_logger().debug("cancellation requested")
        result = DockRobot.Result()
        result.success = False
        result.error_msg = "cancellation requested"
        result.num_retries = self.reacquire_count
        goal_handle.canceled()
        return result


class DockingServer(Node):
    def __init__(self):
        super().__init__("docking_server")

        # Dock database manager.
        #
        # `docks` follows the nav2 convention of keeping the dock list in a parameter on the
        # docking server, so `ros2 param get /docking_server docks` enumerates them and updates
        # land on /parameter_events for anything watching. It is declared before the database is
        # built, because loading the database immediately fires the sync callback.
        self.declare_parameter(
            'docks', [],
            ParameterDescriptor(
                # An empty-list default makes rclpy infer BYTE_ARRAY, which would then reject
                # every string-array update on type. Dynamic typing sidesteps that; external
                # writes are refused by on_set_parameters regardless.
                dynamic_typing=True,
                description=(
                    'Dock ids in the loaded database. Maintained by this server -- change it by '
                    'reloading the database, not by setting the parameter.'
                ),
            ),
        )
        # The RViz docking panel reads `dock_plugins` off this server to fill its dock-type
        # dropdown, and logs an error on every load if it is undeclared. We service one dock type
        # and ignore goal.dock_type entirely, so this exists to complete the nav2 convention
        # rather than to select anything.
        self.declare_parameter(
            'dock_plugins', ['stretch4_dock'],
            ParameterDescriptor(description='Dock types this server can service.'),
        )
        # Keep the servo pipeline on the fast cores. Every stage of it is
        # single-threaded, so under nav2's load the difference between a P-core and
        # the low-power island is the difference between a 30ms loop and a 130ms one.
        # Empty list disables pinning; a non-empty list pins to exactly those CPUs.
        self.declare_parameter(
            'control_cpus', [],
            ParameterDescriptor(
                dynamic_typing=True,
                description=('CPUs to pin this node to. Empty selects the highest-clocked '
                             'cores automatically; set [-1] to disable pinning entirely.'),
            ),
        )
        self.apply_cpu_affinity()

        self._syncing_docks = False
        self.add_on_set_parameters_callback(self.on_set_parameters)
        self.db = DockDatabase(self, on_load_callback=self.sync_dock_parameter)
        # That first load happens inside DockDatabase's constructor, before `self.db` is bound,
        # so its sync is a no-op -- mirror the initial contents now that it exists.
        self.sync_dock_parameter()

        # Dock tracker
        self.tracker = DockTracker()
        self.tracker.warm_start()

        # Ring filtering
        self.ring_filter = RingFilter()
        self.ring_filter.warm_start()

        # Costmap pipeline
        self.costmapper = Costmap()
        self.costmapper.warm_start()

        # Point cloud processing
        cloud_reader.warm_start()

        # Servo law
        self.scheme = 'se2_servo'
        if self.scheme == 'se2_servo':
            self.servo = XYThetaServo()
        elif self.scheme == "mppi_servo":
            ... # TODO

        # Nav2 Simple Commander
        self.navigator = BasicNavigator()

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.get_lidars2baselinktfs()

        # Callback groups
        #  - cloud_group:  mutex, to run 1 control cycle at time. cloud_cb is also
        #                  control for the servoing state.
        #  - watch_group:  watchdog timer separate, so it still fires when everything
        #                  else is blocked. That is the entire point of a watchdog.
        #  - action_group: reentrant, so a new goal / cancel can run while an
        #                  execute_dock thread is parked in its sequencing loop.
        #  - viz_group:    mutex, drawing the state machine takes a few ms of graphviz
        #                  layout, so keep it off the control and watchdog groups.
        self.cloud_group = MutuallyExclusiveCallbackGroup()
        self.watch_group = MutuallyExclusiveCallbackGroup()
        self.action_group = ReentrantCallbackGroup()
        self.viz_group = MutuallyExclusiveCallbackGroup()

        # Subscriptions
        self.create_subscription(BatteryState, 'battery', self.battery_cb, 1)
        self.power_supply_status = None
        self.create_subscription(JointState, 'joint_states', self.joint_state_cb, 1)
        self.stow_targets = ensure_stow.stow_targets()
        self.joint_positions = None
        self.joint_state_time = 0.0

        cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self.right_cloud_sub = SimpleFilter()
        self.left_cloud_sub = SimpleFilter()
        self.create_subscription(PointCloud2, '/lidar_points_right',
                                 self.right_cloud_sub.signalMessage, cloud_qos,
                                 callback_group=self.cloud_group)
        self.create_subscription(PointCloud2, '/lidar_points_left',
                                 self.left_cloud_sub.signalMessage, cloud_qos,
                                 callback_group=self.cloud_group)
        self.ts = ApproximateTimeSynchronizer(
            [self.right_cloud_sub, self.left_cloud_sub],
            queue_size=3,
            slop=0.06,
            allow_headerless=False
        )
        self.ts.registerCallback(self.cloud_cb)
        self.full_points = 0
        self.frames_dropped = 0
        self.dock_confirm_count = 0 # gates the handover to servoing
        self.servo_lost_count = 0 # gates the fall back to navigation

        # Publishers
        self.dock_cloud_pub = self.create_publisher(PointCloud2, '/autodock_cloud', 10)
        self.dock_triangle_pub = self.create_publisher(MarkerArray, '/autodock_triangle', 10)
        self.grid_marker_pub = self.create_publisher(Marker, '/autodock_costmap', 10)
        self.blockers_pub = self.create_publisher(PointCloud2, '/autodock_costmap_blockers', 10)
        self.cmd_vel_pub = self.create_publisher(TwistWithCovarianceStamped, '/cmd_vel_stiff', 10)
        self.cmd_desired_pub = self.create_publisher(TwistStamped, '/autodock_cmd_desired', 10)
        self.cmd_filtered_pub = self.create_publisher(TwistStamped, '/autodock_cmd_filtered', 10)
        sm_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, # a late-joining RViz still sees the current state
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sm_image_pub = self.create_publisher(Image, '/autodock_state_machine', sm_qos)

        # Service Clients
        self.stow_robot = self.create_client(Trigger, 'stow_the_robot')
        self.stop_base = self.create_client(Trigger, 'freewheel_the_robot')
        self.seat_dock = self.create_client(Trigger, 'seat_into_dock')

        # Actions
        self.action = DockRobotAction(self)

        # Services
        self.reload_db_srv = self.create_service(
            ReloadDockDatabase, '~/reload_database', self.reload_database_cb,
            callback_group=self.action_group)

        # Reset variables used in cloud_cb
        self.servo_done = threading.Event()
        self.reset_servo()

        # Safety Watchdog
        self.timer = self.create_timer(1.0 / 20, self.watchdog, callback_group=self.watch_group)

        # Visualizations
        self.viz_queue: deque[tuple] = deque(maxlen=1)
        self.viz_cached = None
        self.sm_last_state = None
        self.viz_timer = self.create_timer(1.0 / 20, self.publish_viz, callback_group=self.viz_group)
        self.get_logger().info("Docking Server Ready!")

    def joint_state_cb(self, msg: JointState) -> None:
        """Cache the stow-relevant joint positions, with the wall time they were received."""
        self.joint_positions = ensure_stow.positions_from_joint_state(
            dict(zip(msg.name, msg.position)))
        self.joint_state_time = time.time()

    def sync_dock_parameter(self) -> None:
        """Mirror the loaded dock ids into the `docks` parameter."""
        db = getattr(self, 'db', None)
        if db is None:
            return  # still constructing; the explicit sync after assignment covers this

        self._syncing_docks = True
        try:
            result = self.set_parameters([
                Parameter('docks', Parameter.Type.STRING_ARRAY, sorted(db.keys()))
            ])[0]
        finally:
            self._syncing_docks = False

        # A silent failure here would leave callers enumerating a stale dock list.
        if not result.successful:
            self.get_logger().error(f"Failed to publish dock list parameter: {result.reason}")

    def on_set_parameters(self, params) -> SetParametersResult:
        """Keep `docks` a read-out of the database rather than an input to it.

        It cannot be declared read_only: that would block this server from updating it too, and
        the list changes whenever the map changes or the database is reloaded.
        """
        for param in params:
            if param.name == 'docks' and not self._syncing_docks:
                return SetParametersResult(
                    successful=False,
                    reason="'docks' reflects the loaded dock database; change it with the "
                           "~/reload_database service instead.",
                )
        return SetParametersResult(successful=True)

    def reload_database_cb(self, request, response):
        """nav2_msgs/srv/ReloadDockDatabase -- load dock definitions from another file."""
        if not self.action.goal_idle.is_set():
            self.get_logger().error("Cannot reload dock database while docking!")
            response.success = False
            return response

        response.success = self.db.reload_from(request.filepath)
        if response.success:
            self.get_logger().info(
                f"Dock database reloaded from {self.db.db_filepath}: {sorted(self.db.keys())}"
            )
        return response

    def apply_cpu_affinity(self) -> None:
        requested = list(self.get_parameter('control_cpus').value or [])
        if requested == [-1]:
            self.get_logger().info('CPU pinning disabled by parameter')
            return

        cpus = requested or fastest_cpus()
        available = os.sched_getaffinity(0)
        cpus = [c for c in cpus if c in available]
        if not cpus:
            self.get_logger().warn(
                'No usable CPUs to pin to; leaving affinity alone. The servo loop may '
                'drift onto slow cores under load.')
            return

        try:
            os.sched_setaffinity(0, set(cpus))
        except OSError as e:
            self.get_logger().warn(f'Could not pin to CPUs {cpus}: {e}')
            return
        self.get_logger().info(f'Pinned to CPUs {cpus}')

    def reset_servo(self):
        if self.scheme == 'se2_servo':
            self.servo.reset()

        # Performance metrics
        self.cloud_cb_rate = 10.0
        self.last_cloud_cb_time = None
        self.print_counter = 0

        # Handoff from cloud_cb (control loop) up to execute_dock (sequencer)
        self.servo_outcome: tuple[ServoingResult | None, str | None] = (None, None)
        self.servo_lost_count = 0
        self.servo_done.clear()

    def scanning_only(self, right_cloud: PointCloud2, left_cloud: PointCloud2):
        # Read & transform both clouds to base_link
        n_r_raw = right_cloud.width * right_cloud.height
        n_l_raw = left_cloud.width * left_cloud.height
        cloud_buf = np.empty((n_r_raw + n_l_raw, 5), dtype=np.float32)
        n_right = cloud_reader.fused_read_transform_points(
            right_cloud, self._right_R, self._right_t, cloud_buf, 0)
        n_total = cloud_reader.fused_read_transform_points(
            left_cloud, self._left_R, self._left_t, cloud_buf, n_right)
        cloud_mat = cloud_buf[:n_total]

        # ID the dock in the cloud
        self.tracker.identify(cloud_mat[:, :4], allow_ambiguity=True)
        if self.tracker.is_tracking():
            self.dock_confirm_count += 1
            return
        self.dock_confirm_count = 0

    def cloud_cb(self, right_cloud: PointCloud2, left_cloud: PointCloud2):
        # Keep track of how large a point cloud should be (~115200 for single-return, ~230400 for dual-return)
        self.full_points = max(self.full_points, max(right_cloud.width * right_cloud.height, left_cloud.width * left_cloud.height))

        if self.action.machine.state is AutodockState.STAGING_AND_SCANNING:
            self.scanning_only(right_cloud, left_cloud)
            return
        if self.action.machine.state is not AutodockState.SERVOING:
            return

        cb_start = time.perf_counter()
        if self.last_cloud_cb_time is not None:
            dt = cb_start - self.last_cloud_cb_time
            if dt > 0:
                rate = 1.0 / dt
                self.cloud_cb_rate = 0.1 * rate + 0.9 * self.cloud_cb_rate
        self.last_cloud_cb_time = cb_start
        t0 = time.perf_counter()

        # Read & transform both clouds to base_link
        n_r_raw = right_cloud.width * right_cloud.height
        n_l_raw = left_cloud.width * left_cloud.height
        cloud_buf = np.empty((n_r_raw + n_l_raw, 5), dtype=np.float32)
        n_right = cloud_reader.fused_read_transform_points(
            right_cloud, self._right_R, self._right_t, cloud_buf, 0)
        n_total = cloud_reader.fused_read_transform_points(
            left_cloud, self._left_R, self._left_t, cloud_buf, n_right)
        cloud_mat = cloud_buf[:n_total]
        right_points = cloud_buf[:n_right]
        left_points = cloud_buf[n_right:n_total]

        # Warn if either lidar cloud doesn't have the expected number of points, e.g. faulty lidar or partial frame
        if right_points.shape[0] < 0.99 * self.full_points:
            self.frames_dropped += 1
            self.get_logger().error(
                f"Right cloud has {right_points.shape[0]} points, expected {self.full_points} +/- {0.01*self.full_points}. Dropped {self.frames_dropped} frames so far."
            )
            self.stop_base.call_async(Trigger.Request())
            return
        if left_points.shape[0] < 0.99 * self.full_points:
            self.frames_dropped += 1
            self.get_logger().error(
                f"Left cloud has {left_points.shape[0]} points, expected {self.full_points} +/- {0.01*self.full_points}. Dropped {self.frames_dropped} frames so far."
            )
            self.stop_base.call_async(Trigger.Request())
            return
        t1 = time.perf_counter()

        # ID the dock in the cloud
        try:
            self.tracker.identify(cloud_mat[:, :4])
        except DockAmbiguityError as e:
            self.get_logger().error(str(e))
            self.stop_base.call_async(Trigger.Request())
            self.servo_outcome = (ServoingResult.MULTIPLE_DOCK_AMBIGUITY, str(e))
            self.servo_done.set()
            return
        if not self.tracker.is_tracking():
            self.servo_lost_count += 1
            self.get_logger().warn("Don't see dock...")
            self.stop_base.call_async(Trigger.Request())
            return
        self.servo_lost_count = 0
        dock_pose = self.tracker.get_pose()
        t2 = time.perf_counter()

        # Ring filter each lidar's cloud separately
        left_filtered = self.ring_filter.process(
            left_points, sensor_origin=self._left_t, compact=True)
        right_filtered = self.ring_filter.process(
            right_points, sensor_origin=self._right_t, compact=True)
        filtered_xyz = np.vstack([left_filtered[:, :3], right_filtered[:, :3]])
        t3 = time.perf_counter()

        # Create costmap
        costmap = self.costmapper.process(filtered_xyz, dock_pose=dock_pose)
        t4 = time.perf_counter()

        # Compute & send control
        current_time = self.get_clock().now().to_msg()
        u, ctrl_viz = self.compute_control(dock_pose, costmap, current_time)
        self.cmd_vel_pub.publish(u)
        self.viz_queue.append((self.tracker.filtered_points[:, :3], self.tracker.colors, self.tracker.corners, dock_pose, *ctrl_viz, costmap))
        t5 = time.perf_counter()

        # Statistics
        self.print_counter += 1
        if self.print_counter % 5 == 0:
            self.get_logger().debug(
                f"cpu{current_cpu()} | "
                f"Rate: {self.cloud_cb_rate:.1f}Hz | "
                f"ps: {(t1 - t0) * 1000.0:.0f}ms | "
                f"ID: {(t2 - t1) * 1000.0:.0f}ms | "
                f"Ring: {(t3 - t2) * 1000.0:.0f}ms | "
                f"Map: {(t4 - t3) * 1000.0:.0f}ms | "
                f"u: {(t5 - t4) * 1000.0:.0f}ms | "
                f"Total: {(t5 - cb_start) * 1000.0:.0f}ms"
            )

    def compute_control(self, dock_pose, costmap, current_time):
        if self.scheme == 'se2_servo':
            # Predock pose 55cm in front of the dock
            x, y, z, qx, qy, qz, qw = dock_pose
            rot, t = Rotation.from_quat([qx, qy, qz, qw]), np.array([x, y, z])
            p_local = np.array([0.0, -0.55, 0.0])
            p_robot = rot.apply(p_local) + t

            # Compute u and filter based on costmap
            errx, erry, errt = p_robot[0], p_robot[1], rot.as_euler('xyz')[2]
            autodock_target = (errx, erry, errt)
            vx, vy, wz = self.servo.step(errx, erry, errt)
            cmd_desired = (vx, vy, wz)
            filtered = filter_clearance_velocity(vx, vy, wz, costmap.obstacle_xy, costmap.cliff_xy)
            cmd_filtered = (filtered.vx, filtered.vy, filtered.wz)
            blockers_xy = np.vstack([filtered.blocking_obstacles, filtered.blocking_cliffs])

            # 5mm / 1deg tolerance
            if abs(errx) < 0.005 and abs(erry) < 0.005 and abs(errt) < 0.0175:
                self.servo_outcome = (ServoingResult.SUCCESS, "")
                self.servo_done.set()

            u = TwistWithCovarianceStamped()
            u.header.stamp = current_time
            u.twist.twist.linear.x = filtered.vx
            u.twist.twist.linear.y = filtered.vy
            u.twist.twist.angular.z = filtered.wz
            return u, (autodock_target, cmd_desired, cmd_filtered, blockers_xy)

        # Any other scheme, 'mppi_servo' included, has no control law yet. Falling through would
        # return None and blow up unpacking in cloud_cb, mid-servo, so fail here instead.
        raise NotImplementedError(f"No control law implemented for scheme '{self.scheme}'")

    def get_lidars2baselinktfs(self, timeout_s=10.0):
        # Will be used to transform point clouds into base_link frame
        deadline = time.time() + timeout_s
        while True:
            try:
                self._right_lidar_to_baselink = self.tf_buffer.lookup_transform(
                    'base_link', 'lidar_right_link', rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5))
                self._left_lidar_to_baselink = self.tf_buffer.lookup_transform(
                    'base_link', 'lidar_left_link', rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5))
                break
            except tf2_ros.TransformException as e:
                if time.time() > deadline:
                    raise RuntimeError(f"lidar->base_link TF never arrived: {e}") from e
                self.get_logger().info("Waiting on lidar->base_link TF...", once=True)

        self._right_R, self._right_t = cloud_reader.transform_to_rt(self._right_lidar_to_baselink)
        self._left_R, self._left_t = cloud_reader.transform_to_rt(self._left_lidar_to_baselink)

    def publish_viz(self):
        current_time = self.get_clock().now().to_msg()

        # Rasterizing the graph costs a few ms, so only redraw when the FSM actually moves
        state = self.action.machine.state
        if state != self.sm_last_state:
            try:
                img_msg = construct_image(self.action.draw_state_machine())
                img_msg.header.stamp = current_time
                self.sm_image_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().warn(f"Couldn't draw the state machine: {e}", throttle_duration_sec=10.0)
            self.sm_last_state = state

        # Get latest servo_viz or use cached
        try:
            self.viz_cached = self.viz_queue.popleft()
        except IndexError:
            pass

        # Not initialized yet
        if self.viz_cached is None:
            return

        # Checks if cloud stream stalls, e.g. if lidar faults
        if self.last_cloud_cb_time is None:
            return
        stale = time.perf_counter() - self.last_cloud_cb_time
        if stale > 0.5: # seconds
            self.viz_cached = None
            return

        if self.action.machine.state not in [AutodockState.SERVOING]:
            return
        filtered_xyz, colors, corners, dock_pose, autodock_target, cmd_desired, cmd_filtered, blockers, costmap = self.viz_cached

        # Publish detector cloud
        detector_cloud_msg = construct_colored_cloud(filtered_xyz, colors)
        self.dock_cloud_pub.publish(detector_cloud_msg)
        self.dock_triangle_pub.publish(construct_triangle(*corners))
        if dock_pose is not None:
            t = construct_frame_se3(dock_pose, child_frame='dock')
            t.header.stamp = current_time
            self.tf_broadcaster.sendTransform(t)

        # Publish autodock_target
        if autodock_target is not None:
            t = construct_frame(autodock_target, child_frame='autodock_target')
            t.header.stamp = current_time
            self.tf_broadcaster.sendTransform(t)

        def at_viz_height(xy):
            COSTMAP_VIZ_Z = 0.02
            xy = np.asarray(xy)
            if len(xy) == 0:
                return np.zeros((0, 3))
            return np.column_stack([xy[:, :2], np.full(len(xy), COSTMAP_VIZ_Z)])

        # Publish costmap (obstacles red, cliffs orange, occlusions gray)
        costmap_msg = construct_grid_marker(
            at_viz_height(costmap.obstacle_xy),
            at_viz_height(costmap.cliff_xy),
            at_viz_height(costmap.occlusion_xy),
            costmap.resolution
        )
        costmap_msg.header.stamp = current_time
        self.grid_marker_pub.publish(costmap_msg)

        # Publish costmap active blockers (vibrant yellow, you should change point size to 10 in Rviz)
        blocker_msg = construct_colored_cloud(
            np.column_stack([blockers, np.full(len(blockers), 0.035)]),
            np.tile([255, 255, 0], (len(blockers), 1)))
        blocker_msg.header.stamp = current_time
        self.blockers_pub.publish(blocker_msg)

        # Publish servo law's desired and filtered commands
        desired_cmd = construct_twist(*cmd_desired)
        desired_cmd.header.stamp = current_time
        self.cmd_desired_pub.publish(desired_cmd)
        filtered_cmd = cmd_desired if cmd_filtered is None else cmd_filtered
        filtered_msg = construct_twist(*filtered_cmd)
        filtered_msg.header.stamp = current_time
        self.cmd_filtered_pub.publish(filtered_msg)

    def watchdog(self):
        if self.action.machine.state not in [AutodockState.SERVOING]:
            return

        # Checks if cloud stream stalls, e.g. if lidar faults
        if self.last_cloud_cb_time is None:
            return
        stale = time.perf_counter() - self.last_cloud_cb_time
        if stale > 0.25: # seconds
            self.get_logger().warn("Loop latency too high, may see abnormal behavior")
        if stale > 0.5: # seconds
            self.get_logger().error(f"No synchronized clouds for {stale:.2f}s - stopping base.")
            self.stop_base.call_async(Trigger.Request())

    def battery_cb(self, bat_msg):
        self.power_supply_status = bat_msg.power_supply_status


def main(args=None):
    try:
        rclpy.init(args=args)
        # Worst case concurrent: cloud_cb + watchdog + viz + incumbent execute_dock + preempting execute_dock + cancel_cb + 1 extra = 7
        executor = MultiThreadedExecutor(num_threads=7)
        node = DockingServer()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()