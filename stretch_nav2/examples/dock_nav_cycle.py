#!/usr/bin/env python3
"""Cycle the robot undock -> navigate -> dock, forever, against a clicked point.

Run it alongside autodocking_cpu.launch.py, which brings up Nav2, the docking
server and the undocking server:

    ros2 run stretch_nav2 dock_nav_cycle.py --ros-args -p dock_id:=<id in the dock database>

Then set the goal with RViz's "Publish Point" tool. Nothing moves until that
first point arrives. The point sets the goal for every later cycle, and clicking
again mid-cycle swaps in a new goal for the *next* one -- the cycle in flight
keeps the goal it started with.

Every motion goes through nav2_simple_commander's BasicNavigator: goToPose() for
the trip out, dockRobotByID() / undockRobot() for the ends of it.

A run looks like:

    (wait for the first clicked point)
    dockRobotByID()            # returns at once if already charging
    forever:
        undockRobot()
        goToPose(goal)
        sit GOAL_DWELL_S at the goal
        dockRobotByID()
        sit DOCK_DWELL_S on the charger

The startup dock is how "dock the robot first if it didn't start on the dock"
is enforced: the docking server short-circuits to success when the battery is
already charging (docking_server.py), so an unconditional dock request is a
no-op on a docked robot and a drive home on an undocked one. The first cycle
then starts immediately -- the 5 minute dwell sits *between* cycles, so the
robot charges after each run rather than before the first one.

A failed step is fatal on purpose. This is meant to run unattended, so instead
of flailing at a robot nobody is watching it logs, pushes a phone notification
through the `claude` CLI, and exits.

Ctrl-C is the only clean way out.
"""

import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import DockRobot
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# The docking server reports which phase it is in. Read the names off the action rather
# than hardcoding the numbers, so a renumbering upstream cannot quietly mislabel a phase.
DOCK_STATES = {
    DockRobot.Feedback.NONE: 'starting up',
    DockRobot.Feedback.NAV_TO_STAGING_POSE: 'navigating to the staging pose',
    DockRobot.Feedback.INITIAL_PERCEPTION: 'looking for the dock',
    DockRobot.Feedback.CONTROLLING: 'servoing onto the dock',
    DockRobot.Feedback.WAIT_FOR_CHARGE: 'seating, waiting for charge',
    DockRobot.Feedback.RETRY: 'lost the dock, navigating back in',
}


DOCK_DWELL_S = 5 * 60.0   # charge time on the dock between cycles
GOAL_DWELL_S = 10.0       # pause at the goal before heading back
NOTIFY_TIMEOUT_S = 180.0  # give up on the notification rather than hang the exit
PROGRESS_LOG_S = 15.0     # how often to log "still going" while a task runs


class CycleFailure(Exception):
    """A step of the cycle did not succeed. Fatal -- see the module docstring."""


class DockNavCycle(BasicNavigator):

    def __init__(self):
        super().__init__(node_name='dock_nav_cycle')

        self.declare_parameter('dock_id', '')
        self.dock_id = self.get_parameter('dock_id').value

        # Goal for the next cycle. None until someone clicks, which is what keeps
        # the node idle until it has somewhere to go.
        self.cycle_goal: PoseStamped | None = None
        self.create_subscription(PointStamped, 'clicked_point', self.clicked_point_cb, 1)

        # Dock retries, per cycle and across the run. On an unattended soak the trend
        # matters more than any single cycle: retries creeping up is the dock getting
        # harder to see long before it fails outright.
        self.cycle_retries = 0
        self.total_retries = 0

    def clicked_point_cb(self, msg: PointStamped) -> None:
        goal = PoseStamped()
        goal.header.frame_id = msg.header.frame_id or 'map'
        # Stamp deliberately left at zero, meaning "latest available transform".
        # A stamp taken here would be up to five minutes stale by the time the
        # next cycle actually sends the goal.
        goal.pose.position.x = msg.point.x
        goal.pose.position.y = msg.point.y
        # Publish Point carries no heading, so face map-forward and let the goal
        # checker's yaw tolerance sort it out.
        goal.pose.orientation.w = 1.0

        first = self.cycle_goal is None
        self.cycle_goal = goal
        self.info(f"{'Goal set' if first else 'New goal'}: "
                  f"({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}) "
                  f"in {goal.header.frame_id}"
                  f"{'' if first else ' -- takes effect next cycle'}")

    def spin_for(self, seconds: float, what: str) -> None:
        """Idle for `seconds`, still servicing clicked_point."""
        self.info(f'{what}: {seconds:.0f}s')
        deadline = time.monotonic() + seconds
        next_log = time.monotonic() + 30.0
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))
            if time.monotonic() >= next_log:
                self.info(f'{what}: {remaining:.0f}s left')
                next_log = time.monotonic() + 30.0

    def await_goal(self) -> None:
        """Block until a point has been clicked. Returns immediately if one has."""
        if self.cycle_goal is not None:
            return
        self.info('Waiting for a goal -- publish one with RViz\'s "Publish Point" tool.')
        while self.cycle_goal is None:
            rclpy.spin_once(self, timeout_sec=0.5)

    @staticmethod
    def describe_progress(feedback) -> str | None:
        """One progress line, whichever action's feedback this is.

        goToPose reports distance, dockRobotByID reports a phase, and undockRobot
        reports an empty message -- hence the None, which the caller turns back into a
        plain "in progress".
        """
        if feedback is None:
            return None

        distance = getattr(feedback, 'distance_remaining', None)
        if distance is not None:
            return f'{distance:.2f} m remaining'

        state = getattr(feedback, 'state', None)
        if state is None:
            return None

        parts = [DOCK_STATES.get(state, f'state {state}')]
        elapsed = getattr(feedback, 'docking_time', None)
        if elapsed is not None:
            parts.append(f'{elapsed.sec + elapsed.nanosec / 1e9:.0f}s in')
        retries = getattr(feedback, 'num_retries', 0)
        if retries:
            parts.append(f'{retries} retry' if retries == 1 else f'{retries} retries')
        return ', '.join(parts)

    def last_num_retries(self) -> int:
        """num_retries off the finished dock result; 0 for actions that have no such field.

        getResult() hands back only the status enum, so the result message itself has to
        come off the future the navigator is already holding -- one layer under the
        commander API, and the only place in this node that reaches past it.
        """
        try:
            return int(self.result_future.result().result.num_retries)
        except Exception:
            return 0

    def run_task(self, accepted: bool, what: str) -> None:
        """Wait out a BasicNavigator task and turn anything but success into a failure.

        `accepted` is what the send call returned: False means the server refused
        the goal outright, in which case there is no result future to wait on.
        """
        if not accepted:
            raise CycleFailure(f'{what}: request rejected by the server')

        next_log = time.monotonic() + PROGRESS_LOG_S
        # isTaskComplete() spins the node for up to 100ms per call, so clicked_point
        # keeps being serviced while the robot drives.
        while not self.isTaskComplete():
            if time.monotonic() >= next_log:
                progress = self.describe_progress(self.getFeedback())
                self.info(f'{what}: {progress or "in progress"}')
                next_log = time.monotonic() + PROGRESS_LOG_S

        result = self.getResult()
        retries = self.last_num_retries()
        self.cycle_retries += retries
        detail = '' if not retries else (
            f' after {retries} retry' if retries == 1 else f' after {retries} retries')

        if result != TaskResult.SUCCEEDED:
            raise CycleFailure(f'{what}: finished as {result.name}{detail}')
        self.info(f'{what}: done{detail}')

    def dock(self, what: str = 'Docking') -> None:
        self.run_task(self.dockRobotByID(self.dock_id), what)

    def undock(self) -> None:
        self.run_task(self.undockRobot(), 'Undocking')

    def navigate(self, goal: PoseStamped) -> None:
        self.run_task(
            self.goToPose(goal),
            f'Navigating to ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})')

    def run(self) -> None:
        # Not waitUntilNav2Active(): its amcl path publishes a zero initialpose in a
        # loop until /amcl_pose arrives, which would yank localization to the map
        # origin on a robot that is already localized. The non-lifecycle localizer
        # branch skips that and just waits for the navigator.
        self.info('Waiting for Nav2...')
        self.waitUntilNav2Active(localizer='robot_localization')

        # "Do nothing until a goal is set" comes first: no point driving to the dock
        # for a cycle that has nowhere to go.
        self.await_goal()

        self.info(f"Making sure the robot starts on dock '{self.dock_id}'")
        self.dock('Startup docking')
        self.cycle_retries = 0  # startup is not one of the cycles being counted

        cycle = 0
        while True:
            cycle += 1
            # Snapshot the goal so a click mid-cycle lands on the next one.
            goal = self.cycle_goal
            assert goal is not None  # only cleared by shutdown; await_goal ran already
            self.info(f'--- cycle {cycle}: '
                      f'({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}) ---')

            self.cycle_retries = 0
            self.undock()
            self.navigate(goal)
            self.spin_for(GOAL_DWELL_S, 'Pausing at the goal')
            self.dock()
            self.total_retries += self.cycle_retries
            self.info(f'--- cycle {cycle} complete, {self.cycle_retries} dock '
                      f'{"retry" if self.cycle_retries == 1 else "retries"} '
                      f'({self.total_retries} over {cycle} '
                      f'{"cycle" if cycle == 1 else "cycles"}) ---')
            self.spin_for(DOCK_DWELL_S, 'Charging on the dock')

    def notify_phone(self, message: str) -> None:
        """Best-effort phone notification via the claude CLI. Never raises."""
        prompt = ('An unattended robot test just failed and the operator is away from '
                  'the terminal, so this needs to reach their phone. Send a push '
                  'notification with exactly this message, then stop. Do not do anything '
                  f'else, do not read or edit any files. Message: {message}')
        try:
            done = subprocess.run(
                ['claude', '-p', prompt, '--allowedTools', 'PushNotification'],
                capture_output=True, text=True, timeout=NOTIFY_TIMEOUT_S, check=False)
        except FileNotFoundError:
            self.warn('No `claude` on PATH; could not send a phone notification.')
        except subprocess.TimeoutExpired:
            self.warn(f'claude CLI did not return within {NOTIFY_TIMEOUT_S:.0f}s; '
                      'phone notification may not have been sent.')
        except Exception as e:  # a failed notification must not mask the real error
            self.warn(f'Could not send a phone notification: {e}')
        else:
            if done.returncode != 0:
                self.warn(f'claude CLI exited {done.returncode}: '
                          f'{(done.stderr or done.stdout).strip()[:200]}')
            else:
                # PushNotification can decide a push is redundant and skip delivery, and
                # that still exits 0 -- log what the CLI said rather than assume it landed.
                self.info(f'Notification request returned: {(done.stdout or "").strip()[:300]}')


def main(args=None):
    rclpy.init(args=args)
    node = DockNavCycle()

    if not node.dock_id:
        node.error('dock_id is required: '
                   'ros2 run stretch_nav2 dock_nav_cycle.py --ros-args -p dock_id:=<id>')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    exit_code = 0
    try:
        node.run()
    except KeyboardInterrupt:
        node.info('Ctrl-C -- cancelling the active task and shutting down.')
        try:
            node.cancelTask()
        except Exception:
            pass
    except CycleFailure as e:
        node.error(f'Cycle failed: {e}')
        try:
            node.cancelTask()
        except Exception:
            pass
        node.notify_phone(
            f'Stretch dock/nav cycle stopped: {e} ({node.total_retries} dock retries so far)')
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
