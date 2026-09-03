#!/usr/bin/env python3
import math
import time
import threading
from collections import deque
from statistics import median

import numpy as np

import rclpy
import tf2_ros
from message_filters import ApproximateTimeSynchronizer, SimpleFilter

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from nav2_msgs.action import UndockRobot

from stretch4_docking.costmap import ClearanceFilterConfig, Costmap, RingFilter
from stretch4_docking.utils import cloud_reader


# Ours, matching the 800 block the docking server uses for outcomes Nav2 has no code for.
UNDOCK_BLOCKED_ERROR_CODE = 802

UNDOCK_SPEED_MPS = 0.1        # sideways, toward the robot's right
UNDOCK_MAX_TIME_S = 5.0
# The servo's own stop radius, so "blocked" means the same thing undocking as it does docking.
UNDOCK_CLEARANCE_M = ClearanceFilterConfig().obstacle_stop_radius_m
CLOUD_STALE_S = 1.0           # a costmap older than this is not evidence about the room now
# Undocking is a sideways slide off the charger, and the dock is about 0.15m wide either
# side of centre (CostmapConfig.dock_free_half_m), so this is roughly the point past which
# the robot is off it. Short of this an undock is not worth calling a success. It is an
# inference from the costmap's dock geometry, not a measurement of the physical dock --
# worth checking against a real dock.
MIN_UNDOCK_TRAVEL_M = 0.25
# Judging on several costmaps rather than one is what keeps a single noisy frame from
# vetoing an undock, or from authorizing one.
CLEARANCE_FRAMES = 3


def max_clear_travel(hazard_xy, clearance_m, limit_m):
    """How far right the base can sweep before a hazard comes within `clearance_m`.

    Solved, not searched. Sweeping to -t traces the capsule `blocking_points` describes,
    and a hazard at (x, y) with y < 0 first falls inside it at t = |y| - sqrt(c^2 - x^2):
    a hazard with |x| > c never does, and one already inside gives t <= 0. The smallest
    such t over every hazard is how far the robot can actually go.

    This is what turns the check from a veto into a distance. The old all-or-nothing form
    refused a 0.50m undock over a cell 0.60m away -- correctly, since that cell lands
    inside the footprint after 0.50m -- while 0.33m of perfectly good travel sat there
    unused, which is far enough to get off the dock.
    """
    hazard_xy = np.asarray(hazard_xy, dtype=np.float64).reshape(-1, 2)
    if len(hazard_xy) == 0:
        return limit_m

    x, y = hazard_xy[:, 0], hazard_xy[:, 1]
    relevant = (y < 0.0) & (np.abs(x) <= clearance_m) & np.isfinite(x) & np.isfinite(y)
    if not relevant.any():
        return limit_m

    x, y = x[relevant], y[relevant]
    t_block = np.abs(y) - np.sqrt(np.maximum(clearance_m ** 2 - x ** 2, 0.0))
    return float(np.clip(t_block.min(), 0.0, limit_m))


def blocking_points(hazard_xy, travel_m, clearance_m):
    """Hazard cells the base would move toward sweeping `travel_m` to its right.

    The base is a disc, so sweeping it along -y traces a capsule: every point within
    `clearance_m` of the segment from (0, 0) to (0, -travel_m) in base_link. Testing against the
    whole segment covers the destination and everything passed on the way, which a plain
    destination-footprint check would miss.

    The `y < 0` clause matters as much as the distance. The capsule includes a cap around the
    *start* pose, so without it anything already within clearance_m of the stationary robot
    counts -- including things on its left, which moving right only takes it further from. That
    made an empty aisle read as blocked by the wall the robot was parked against. For motion
    along -y the closest point on the segment falls at t > 0 exactly when y < 0, so this keeps
    the cells the robot actually closes on and drops the ones it retreats from.
    """
    hazard_xy = np.asarray(hazard_xy, dtype=np.float64).reshape(-1, 2)
    if len(hazard_xy) == 0:
        return hazard_xy

    # Closest point on the segment: y clamped to the travelled span, x pinned to the axis.
    closest_y = np.clip(hazard_xy[:, 1], -travel_m, 0.0)
    distance = np.hypot(hazard_xy[:, 0], hazard_xy[:, 1] - closest_y)
    return hazard_xy[(distance <= clearance_m) & (hazard_xy[:, 1] < 0.0)]


class UndockingActionServer(Node):
    def __init__(self):
        super().__init__("undocking_server")

        # Perception, mirroring the docking server so both judge clearance the same way.
        self.ring_filter = RingFilter()
        self.ring_filter.warm_start()
        self.costmapper = Costmap()
        self.costmapper.warm_start()
        cloud_reader.warm_start()

        self.cloud_group = MutuallyExclusiveCallbackGroup()
        self.action_group = ReentrantCallbackGroup()

        # TF2: the clouds arrive in their own lidar frames and must be fused in base_link.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        self.get_lidars2baselinktfs()

        self.costmap_lock = threading.Lock()
        # This is the same ring filter and costmap the docking server runs, and running
        # it on every frame costs the better part of a core -- continuously, whether or
        # not anyone is undocking. Nothing reads the result except a goal, and the two
        # servers are never busy at the same time, so the cost landed entirely on the
        # docking servo: same code and same clouds, but 2-3x the stage times. Pay for it
        # only while a goal is in flight.
        self.want_clouds = threading.Event()
        self.costmap_history: deque = deque(maxlen=CLEARANCE_FRAMES)

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
            queue_size=3, slop=0.06, allow_headerless=False)
        self.ts.registerCallback(self.cloud_cb)

        # stretch_driver subscribes to cmd_vel and must be in navigation or velocity mode.
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Blind undocking: move the full distance without looking. For docks where the
        # clearance check has nothing useful to say -- a dock in a known-empty bay, or a
        # bringup where the lidars are not trusted yet. Read per goal rather than cached,
        # so `ros2 param set /undocking_server blind_undock true` takes effect on the next
        # undock without a relaunch.
        self.declare_parameter('blind_undock', False)

        self._is_busy = False
        self._status_lock = threading.Lock()
        self.rate = self.create_rate(10)  # created once; a per-goal Rate leaks a timer

        self._action_server = ActionServer(
            self,
            UndockRobot,
            "undock_robot",
            execute_callback=self.execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.action_group,
        )
        if self.get_parameter('blind_undock').value:
            self.get_logger().warn(
                "Undock server ready in BLIND mode: undocking will not check for obstacles.")
        else:
            self.get_logger().info(
                f"Undock server ready. Requires {UNDOCK_CLEARANCE_M:.2f} m clearance to the right.")

    def get_lidars2baselinktfs(self, timeout_s=10.0):
        deadline = time.time() + timeout_s
        while True:
            try:
                right = self.tf_buffer.lookup_transform(
                    'base_link', 'lidar_right_link', rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5))
                left = self.tf_buffer.lookup_transform(
                    'base_link', 'lidar_left_link', rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5))
                break
            except tf2_ros.TransformException as e:
                if time.time() > deadline:
                    raise RuntimeError(f"lidar->base_link TF never arrived: {e}") from e
                self.get_logger().info("Waiting on lidar->base_link TF...", once=True)

        self._right_R, self._right_t = cloud_reader.transform_to_rt(right)
        self._left_R, self._left_t = cloud_reader.transform_to_rt(left)

    def cloud_cb(self, right_cloud: PointCloud2, left_cloud: PointCloud2):
        """Build a costmap for the goal in flight. Idle between goals, by design."""
        if not self.want_clouds.is_set():
            return

        n_r_raw = right_cloud.width * right_cloud.height
        n_l_raw = left_cloud.width * left_cloud.height
        cloud_buf = np.empty((n_r_raw + n_l_raw, 5), dtype=np.float32)
        n_right = cloud_reader.fused_read_transform_points(
            right_cloud, self._right_R, self._right_t, cloud_buf, 0)
        n_total = cloud_reader.fused_read_transform_points(
            left_cloud, self._left_R, self._left_t, cloud_buf, n_right)

        right_filtered = self.ring_filter.process(
            cloud_buf[:n_right], sensor_origin=self._right_t, compact=True)
        left_filtered = self.ring_filter.process(
            cloud_buf[n_right:n_total], sensor_origin=self._left_t, compact=True)
        filtered_xyz = np.vstack([left_filtered[:, :3], right_filtered[:, :3]])

        costmap = self.costmapper.process(filtered_xyz)
        with self.costmap_lock:
            self.costmap_history.append((time.time(), costmap))

    def describe_blockers(self, costmap, travel_m):
        """What is stopping us, for the log line."""
        obstacles = blocking_points(costmap.obstacle_xy, travel_m, UNDOCK_CLEARANCE_M)
        cliffs = blocking_points(costmap.cliff_xy, travel_m, UNDOCK_CLEARANCE_M)

        def nearest(points):
            return min(math.hypot(x, y) for x, y in points)

        reasons = []
        if len(obstacles):
            reasons.append(f"{len(obstacles)} obstacle cells (nearest {nearest(obstacles):.2f} m)")
        if len(cliffs):
            reasons.append(f"{len(cliffs)} cliff cells (nearest {nearest(cliffs):.2f} m)")
        return ", ".join(reasons) if reasons else "nothing in the way"

    def clear_travel(self, limit_m):
        """(travel_m, detail) -- how far right the base can go, judged over recent frames.

        The median rather than the minimum, deliberately: one frame carrying a spurious
        cell should not veto an undock, and one frame that happens to miss a real
        obstacle should not authorize one. At 10Hz the three frames span 0.3s, so this
        still reacts to something genuinely appearing well inside the 5s move.

        Returns (None, reason) when there is nothing recent enough to judge on, which is
        the same refusal-to-guess the staleness check made before.
        """
        now = time.time()
        with self.costmap_lock:
            fresh = [cm for t, cm in self.costmap_history if now - t <= CLOUD_STALE_S]

        if not fresh:
            return None, "no lidar costmap within the last second"

        travels = [
            min(max_clear_travel(cm.obstacle_xy, UNDOCK_CLEARANCE_M, limit_m),
                max_clear_travel(cm.cliff_xy, UNDOCK_CLEARANCE_M, limit_m))
            for cm in fresh
        ]
        return float(median(travels)), self.describe_blockers(fresh[-1], limit_m)

    def wait_for_costmaps(self, since, count=CLEARANCE_FRAMES, timeout_s=2.0):
        """Wait for `count` costmaps built after `since`, so a goal is judged on the room now.

        Replaces the always-on pipeline: instead of a costmap that is always warm, a goal
        spins perception up and waits for the few frames the median needs. Whether what
        arrives is enough is still clear_travel's call -- on timeout it judges on however
        many frames did land, and refuses outright if that is none.
        """
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            with self.costmap_lock:
                landed = sum(1 for t, _ in self.costmap_history if t >= since)
            if landed >= count:
                return True
            self.rate.sleep()
        return False

    def _goal_callback(self, goal_request):
        """Reject new goals if we are already undocking."""
        with self._status_lock:
            if self._is_busy:
                self.get_logger().warn("Rejecting request: Robot is already undocking.")
                return GoalResponse.REJECT

            self.get_logger().info("Accepting new undock request.")
            # Set busy immediately so another request can't sneak in before execute_callback starts
            self._is_busy = True
            return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Accepts manual cancellation requests."""
        return CancelResponse.ACCEPT

    def _stop_robot(self):
        self.get_logger().info("Stopping robot movement.")
        self.cmd_vel_pub.publish(Twist())

    def _failed(self, goal_handle, error_code, message):
        self.get_logger().error(message)
        goal_handle.abort()
        return UndockRobot.Result(success=False, error_code=error_code, error_msg=message)

    def drive_right(self, goal_handle: ServerGoalHandle, travel_m: float, monitored: bool):
        """Sweep `travel_m` to the right, optionally re-checking the path as it goes.

        `monitored=False` is blind mode: the same motion, with no costmap consulted
        before or during. Everything about how far and how fast is identical, so the two
        modes cannot drift apart in the part that actually moves the robot.
        """
        twist = Twist()
        twist.linear.y = -UNDOCK_SPEED_MPS  # -y is the robot's right

        start_time = self.get_clock().now()
        # Distance drives the move, not time; the deadline is only a backstop for a base
        # that is not tracking the commanded velocity.
        deadline = Duration(seconds=(travel_m / UNDOCK_SPEED_MPS) + 2.0)
        self.get_logger().info(f"Executing undock: {travel_m:.2f} m")

        travelled = 0.0
        stopped_early = None
        while travelled < travel_m:
            if goal_handle.is_cancel_requested:
                self._stop_robot()
                goal_handle.canceled()
                return UndockRobot.Result(success=False)

            elapsed = self.get_clock().now() - start_time
            if elapsed > deadline:
                stopped_early = "timed out before covering the planned distance"
                break

            travelled = (elapsed.nanoseconds / 1e9) * UNDOCK_SPEED_MPS
            remaining = travel_m - travelled

            if monitored:
                # The costmap is in base_link, so it travels with the robot and
                # `remaining` is always measured from where the robot is now. Re-checking
                # each cycle catches something that moves in after the initial look -- the
                # old code committed to its opening judgement for the whole 5s.
                still_clear, blockers = self.clear_travel(remaining)
                if still_clear is None:
                    stopped_early = f"lost lidar mid-undock: {blockers}"
                    break
                if still_clear + 0.02 < remaining:
                    stopped_early = f"path closed up mid-undock: {blockers}"
                    break

            goal_handle.publish_feedback(UndockRobot.Feedback())
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        self._stop_robot()

        if stopped_early is not None and travelled < MIN_UNDOCK_TRAVEL_M:
            return self._failed(
                goal_handle, UNDOCK_BLOCKED_ERROR_CODE,
                f"Undock stopped after {travelled:.2f} m, short of the "
                f"{MIN_UNDOCK_TRAVEL_M:.2f} m needed to get off the dock: {stopped_early}")
        if stopped_early is not None:
            # Off the dock, just not as far out as planned. A success with a caveat, not a
            # failure -- the robot is free to navigate.
            self.get_logger().warn(
                f"Undock stopped at {travelled:.2f} m of {travel_m:.2f} m: {stopped_early}")

        goal_handle.succeed()
        return UndockRobot.Result(success=True, error_code=UndockRobot.Result.NONE)

    def execute_callback(self, goal_handle: ServerGoalHandle):
        try:
            goal: UndockRobot.Goal = goal_handle.request
            duration_sec = float(getattr(goal, "max_undocking_time", UNDOCK_MAX_TIME_S)
                                 or UNDOCK_MAX_TIME_S)
            duration_sec = max(0.5, min(duration_sec, UNDOCK_MAX_TIME_S))
            requested_m = UNDOCK_SPEED_MPS * duration_sec

            blind = bool(self.get_parameter('blind_undock').value)
            if blind:
                # Deliberately never sets want_clouds, so the ring filter and costmap stay
                # idle: blind mode costs no perception at all, not just no checking.
                travel_m = requested_m
                self.get_logger().warn(
                    f"Blind undock: sweeping {travel_m:.2f} m right without checking clearance")
                return self.drive_right(goal_handle, travel_m, monitored=False)

            # Look before moving, not while moving: the point is to not enter the space
            # until we know it is empty.
            requested_at = time.time()
            self.want_clouds.set()
            if not self.wait_for_costmaps(requested_at):
                self.get_logger().warn(
                    "Fewer than %d costmaps within 2s of the request; judging on what landed"
                    % CLEARANCE_FRAMES)

            travel_m, detail = self.clear_travel(requested_m)
            if travel_m is None:
                return self._failed(goal_handle, UNDOCK_BLOCKED_ERROR_CODE,
                                    f"Refusing to undock without perception: {detail}")
            if travel_m < MIN_UNDOCK_TRAVEL_M:
                return self._failed(
                    goal_handle, UNDOCK_BLOCKED_ERROR_CODE,
                    f"Only {travel_m:.2f} m clear to the right, need {MIN_UNDOCK_TRAVEL_M:.2f} m "
                    f"to get off the dock: {detail}")

            if travel_m < requested_m - 0.01:
                # Short but usable. Worth saying out loud, because a robot that keeps
                # undocking 0.30m when it asked for 0.50m is telling you the dock has
                # something parked next to it.
                self.get_logger().warn(
                    f"Undocking {travel_m:.2f} m instead of the requested {requested_m:.2f} m: "
                    f"{detail}")
            else:
                self.get_logger().info(f"{travel_m:.2f} m to the right is clear")

            return self.drive_right(goal_handle, travel_m, monitored=True)

        finally:
            # CRITICAL: Always reset busy status so the server can accept new goals later
            self.want_clouds.clear()
            with self._status_lock:
                self._is_busy = False

    def shutdown(self):
        self._action_server.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = UndockingActionServer()
    # cloud_cb + execute_callback + cancel_cb + TF listener thread headroom
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        # On Ctrl-C rclpy's signal handler has already shut the context down; calling it again
        # raises and turns a clean exit into a failure.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
