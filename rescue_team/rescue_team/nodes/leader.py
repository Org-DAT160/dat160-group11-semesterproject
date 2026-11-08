"""
Global leader node: the ROS adapter around `Mission` (ARCHITECTURE.md sections 5, 10, 11).

The leader makes no mission decisions itself. It turns ROS callbacks into mission
events, calls `Mission.handle(event)` and executes the commands it gets back:

    /map, /<robot>/odom, action results, timer -> Event -> Mission.handle(event)
    -> [SendGoal, CancelGoal, Finish] -> send_goal_async / cancel_goal_async / log

Start-up (section 11): `Started` is sent once the simulation clock runs, `/map` has
arrived, odometry has arrived from every robot, `/set_marker_position` is available
and every `/<robot>/navigate_to` server is available, and all of that has held for
2 s. After that the same timer sends `Tick` at `tick_rate`. Until then the unmet
conditions are logged every 5 s on a steady-clock timer, which also runs when
`/clock` is missing.

Single-threaded (section 10): every call is asynchronous (`send_goal_async`,
`cancel_goal_async`) with a done callback, and nothing waits inside a callback.

Parts that come with later issues; until then the node uses simple stand-ins:
    MIS-03  reporting markers to scoring
    MIS-04  meeting points (meeting_points returns None: the meeting stays pending)
    MIS-05  GreedyAllocator, MIS-06 viewpoints (no viewpoints: robots wait after Started)
    MIS-06  path cost from a distance field (straight-line distance until then)
"""
import math
from typing import Dict, List, Optional, Sequence, Set, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point as PointMsg
from nav_msgs.msg import OccupancyGrid, Odometry
from rescue_interfaces.action import NavigateTo
from scoring_interfaces.srv import SetMarkerPosition

from rescue_team.core.geometry import distance, Point
from rescue_team.core.grid import GridMap
from rescue_team.core.mission import (
    CancelGoal, Command, Finish, GoalFinished, Mission, MissionParams,
    SendGoal, Started, Tick)

# Fixed values (ARCHITECTURE.md section 11), not parameters.
START_HOLD = 2.0        # s the start conditions must hold before Started
LOG_PERIOD = 5.0        # s between two logs of the unmet start conditions (steady clock)

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
}


class NoViewpoints:
    """
    Stand-in for GreedyAllocator until MIS-05/MIS-06: there are no viewpoints.

    Same methods as GreedyAllocator (docs/CODE_STRUCTURE.md); it never hands out a
    viewpoint, so the robots wait after Started.
    """

    def set_targets(self, targets: Sequence[Point]) -> None:
        """Accept the pass and ignore it."""

    def next_target(self, robot: str, cost) -> Optional[Point]:
        """Return None: no viewpoint."""
        return None

    def complete(self, robot: str, success: bool) -> None:
        """Do nothing: there is never a reservation."""

    def release(self, robot: str) -> None:
        """Do nothing: there is never a reservation."""

    def free_count(self) -> int:
        """Return 0: no free viewpoint."""
        return 0


class Leader(Node):
    """Turn ROS callbacks into mission events and mission commands into ROS calls."""

    def __init__(self):
        super().__init__('leader')

        # ---------- parameters (config/params.yaml) ----------
        p = {name: self.declare_parameter(name, default).value for name, default in (
            ('robots', ['tb3_0', 'tb3_1']), ('grid_resolution', 0.10),
            ('tick_rate', 2.0), ('meet_hold_time', 3.0), ('meet_max_attempts', 3),
            ('goal_timeout_base', 20.0), ('max_linear_speed', 0.26),
            ('spin_angle', 5.2), ('spin_speed', 1.5), ('spin_step', 0.87),
            ('spin_pause', 0.5),)}
        self.p = p
        self.robots: List[str] = list(p['robots'])

        # Spin timeout (section 5.1): twice the nominal turn-and-look time.
        spin_time = 2.0 * (p['spin_angle'] / p['spin_speed']
                           + math.ceil(p['spin_angle'] / p['spin_step']) * p['spin_pause'])
        params = MissionParams(
            meet_hold_time=float(p['meet_hold_time']),
            meet_max_attempts=int(p['meet_max_attempts']),
            goal_timeout_base=float(p['goal_timeout_base']),
            max_linear_speed=float(p['max_linear_speed']),
            spin_time=spin_time)
        self.get_logger().info(
            'parameters: ' + ', '.join(f'{k}={v}' for k, v in sorted(p.items()))
            + f', spin_time={spin_time:.1f} s (computed)')

        # ---------- the mission ----------
        self.allocator = NoViewpoints()             # GreedyAllocator in MIS-05/MIS-06
        self.mission = Mission(self.allocator, self.path_cost, self.meeting_points, params)

        # ---------- latest data ----------
        self.static_grid: Optional[GridMap] = None
        self.positions: Dict[str, Point] = {}       # robot -> (x, y) from odometry

        # ---------- start-up ----------
        self.started = False
        self.ready_since: Optional[float] = None    # sim time the start conditions began to hold
        self.start_time: Optional[float] = None     # sim time Started was sent

        # ---------- goals ----------
        # The newest goal sent per robot, and the handle of the newest accepted goal.
        self.sent_goal_id: Dict[str, int] = {}
        self.goal_handles: Dict[str, Tuple[int, object]] = {}
        # Goals cancelled before their handle arrived; cancelled as soon as it does.
        self.cancel_pending: Set[int] = set()

        # ---------- ROS interfaces (absolute names from `robots`, section 4) ----------
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.nav_clients: Dict[str, ActionClient] = {}
        for robot in self.robots:
            # Default argument binds the robot name now, not when the lambda runs.
            self.create_subscription(
                Odometry, f'/{robot}/odom',
                lambda msg, robot=robot: self.on_odom(robot, msg), 10)
            self.nav_clients[robot] = ActionClient(self, NavigateTo, f'/{robot}/navigate_to')
        self.report_client = self.create_client(SetMarkerPosition, '/set_marker_position')

        # Mission timer on the simulation clock: Started, then Tick at tick_rate.
        self.create_timer(1.0 / float(p['tick_rate']), self.on_timer)
        # Start-up log on the steady clock, so it also runs while /clock is missing.
        self.log_timer = self.create_timer(
            LOG_PERIOD, self.on_log_timer, clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.get_logger().info(f'leader started for robots {self.robots}, waiting for start-up')

    # ---------- helpers ----------

    def now(self) -> float:
        """Return the simulation time in seconds (0 until the first /clock message)."""
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------- sensor callbacks ----------

    def on_map(self, msg: OccupancyGrid) -> None:
        """Build the static grid once (section 6.1); planning grid and wall distance in MIS-06."""
        try:
            if self.static_grid is not None:
                return
            info = msg.info
            full = GridMap.from_occupancy_grid(
                msg.data, info.width, info.height, info.resolution,
                (info.origin.position.x, info.origin.position.y))
            factor = max(1, int(round(self.p['grid_resolution'] / info.resolution)))
            self.static_grid = full.downsample(factor)
            rows, cols = self.static_grid.occupied.shape
            self.get_logger().info(
                f'map received: {info.width}x{info.height} at {info.resolution} m, '
                f'static grid {rows}x{cols} at {self.static_grid.resolution:.2f} m')
        except Exception as e:  # a crash in a callback would stop the node
            self.get_logger().error(f'on_map: {e!r}')

    def on_odom(self, robot: str, msg: Odometry) -> None:
        """Store the robot's position (odometry equals the map pose here, SPK-01)."""
        try:
            p = msg.pose.pose.position
            self.positions[robot] = (float(p.x), float(p.y))
        except Exception as e:
            self.get_logger().error(f'on_odom({robot}): {e!r}')

    # ---------- timers ----------

    def on_timer(self) -> None:
        """Before Started: check the start conditions. After that: send Tick."""
        try:
            now = self.now()
            if self.started:
                self.dispatch(Tick(now))
                return

            if self.unmet_conditions(now):
                self.ready_since = None
                return
            if self.ready_since is None:
                self.ready_since = now
                self.get_logger().info(
                    f'all start conditions hold, Started in {START_HOLD:.0f} s')
            if now - self.ready_since >= START_HOLD:
                self.send_started(now)
        except Exception as e:
            self.get_logger().error(f'on_timer: {e!r}')

    def on_log_timer(self) -> None:
        """Until Started, every 5 s: log the unmet start conditions and missing camera topics."""
        try:
            if self.started:
                self.log_timer.cancel()
                return
            unmet = self.unmet_conditions(self.now())
            if unmet:
                self.get_logger().warn('waiting for start-up: ' + '; '.join(unmet))
            for robot in self.robots:
                for topic in (f'/{robot}/aruco_markers', f'/{robot}/camera/image_raw'):
                    if self.count_publishers(topic) == 0:
                        self.get_logger().error(
                            f'{topic} has no publisher: markers from {robot} will be missed')
        except Exception as e:
            self.get_logger().error(f'on_log_timer: {e!r}')

    # ---------- start-up (section 11) ----------

    def unmet_conditions(self, now: float) -> List[str]:
        """Return the start conditions that do not hold yet (empty when ready)."""
        unmet = []
        if now <= 0.0:
            unmet.append('simulation clock not running (no /clock)')
        if self.static_grid is None:
            unmet.append('/map not received')
        for robot in self.robots:
            if robot not in self.positions:
                unmet.append(f'no odometry from {robot}')
        if not self.report_client.service_is_ready():
            unmet.append('/set_marker_position not available (is scoring running?)')
        for robot in self.robots:
            if not self.nav_clients[robot].server_is_ready():
                unmet.append(f'/{robot}/navigate_to not available')
        return unmet

    def send_started(self, now: float) -> None:
        """Send Started with the robots' start positions and the viewpoint passes."""
        robots = {robot: self.positions[robot] for robot in self.robots}
        passes = self.compute_viewpoints()
        self.started = True
        self.start_time = now
        self.get_logger().info(
            'Started: ' + ', '.join(f'{r} at ({x:.2f}, {y:.2f})' for r, (x, y) in robots.items())
            + ', viewpoints per pass: ' + ', '.join(str(len(vp)) for vp in passes))
        self.dispatch(Started(now, robots, passes))

    def compute_viewpoints(self) -> List[List[Point]]:
        """Return the viewpoints of pass 0 and pass 1. Stand-in until MIS-06: none."""
        return [[], []]

    # ---------- functions injected into Mission ----------

    def path_cost(self, robot: str, target: Point) -> Optional[float]:
        """
        Return the travel distance from the robot to `target` (CostFn).

        Stand-in until MIS-06: the straight-line distance from the robot's latest
        position, which underestimates detours. None without a position.
        """
        position = self.positions.get(robot)
        if position is None:
            return None
        return distance(position, target)

    def meeting_points(self, exclude: Sequence[Point]) -> Optional[Dict[str, Point]]:
        """Return one meeting point per robot near the big fire. Stand-in until MIS-04: None."""
        return None

    # ---------- mission events and commands ----------

    def dispatch(self, event) -> None:
        """Hand one event to the mission, log a state change and execute the commands."""
        before = self.mission.state
        commands = self.mission.handle(event)
        after = self.mission.state
        if after != before:
            self.get_logger().info(
                f'state {before.name} -> {after.name} ({type(event).__name__})')
        self.execute(commands)

    def execute(self, commands: List[Command]) -> None:
        """Run each command as a ROS call, in order."""
        for command in commands:
            if isinstance(command, SendGoal):
                self.send_goal(command.robot, command.goal_id, command.target, command.spin)
            elif isinstance(command, CancelGoal):
                self.cancel_goal(command.robot)
            elif isinstance(command, Finish):
                elapsed = self.now() - (self.start_time or 0.0)
                self.get_logger().info(
                    f'MISSION FINISHED after {elapsed:.1f} s (simulation time): '
                    f'all markers accepted, meeting held={self.mission.meeting_done}')

    def send_goal(self, robot: str, goal_id: int, target: Point, spin: bool) -> None:
        """Send a NavigateTo goal without waiting; the answer comes in on_goal_response."""
        goal = NavigateTo.Goal()
        goal.target = PointMsg(x=float(target[0]), y=float(target[1]), z=0.0)
        goal.spin_on_arrival = bool(spin)
        client = self.nav_clients[robot]
        if not client.server_is_ready():
            self.get_logger().warn(f'/{robot}/navigate_to not ready; goal {goal_id} sent anyway')
        self.sent_goal_id[robot] = goal_id
        self.get_logger().info(
            f'{robot}: goal {goal_id} to ({target[0]:.2f}, {target[1]:.2f}), spin={spin}')
        future = client.send_goal_async(goal)
        future.add_done_callback(
            lambda f, robot=robot, goal_id=goal_id: self.on_goal_response(robot, goal_id, f))

    def on_goal_response(self, robot: str, goal_id: int, future) -> None:
        """Store the handle of an accepted goal; a rejected goal is a failed goal."""
        try:
            handle = future.result()
            if not handle.accepted:
                self.get_logger().warn(f'{robot}: goal {goal_id} rejected')
                self.dispatch(GoalFinished(self.now(), robot, goal_id, False))
                return

            current = self.goal_handles.get(robot)
            if current is None or current[0] < goal_id:
                self.goal_handles[robot] = (goal_id, handle)
            if goal_id in self.cancel_pending:
                self.cancel_pending.discard(goal_id)
                handle.cancel_goal_async()
                self.get_logger().info(f'{robot}: goal {goal_id} cancelled (after accept)')
            handle.get_result_async().add_done_callback(
                lambda f, robot=robot, goal_id=goal_id: self.on_goal_result(robot, goal_id, f))
        except Exception as e:
            self.get_logger().error(f'on_goal_response({robot}, {goal_id}): {e!r}')

    def on_goal_result(self, robot: str, goal_id: int, future) -> None:
        """Pass the result to the mission; it ignores results of replaced goals itself."""
        try:
            response = future.result()
            result = response.result
            status = STATUS_NAMES.get(response.status, str(response.status))
            self.get_logger().info(f'{robot}: goal {goal_id} ended: {result.message} ({status})')
            current = self.goal_handles.get(robot)
            if current is not None and current[0] == goal_id:
                del self.goal_handles[robot]
            self.dispatch(GoalFinished(self.now(), robot, goal_id, bool(result.success)))
        except Exception as e:
            self.get_logger().error(f'on_goal_result({robot}, {goal_id}): {e!r}')

    def cancel_goal(self, robot: str) -> None:
        """Send a cancel request for the robot's newest goal, or as soon as it is accepted."""
        goal_id = self.sent_goal_id.get(robot)
        if goal_id is None:
            return
        current = self.goal_handles.get(robot)
        if current is not None and current[0] == goal_id:
            current[1].cancel_goal_async()
            self.get_logger().info(f'{robot}: goal {goal_id} cancel requested')
        else:
            self.cancel_pending.add(goal_id)
            self.get_logger().info(f'{robot}: goal {goal_id} will be cancelled when accepted')

    def destroy_node(self) -> None:
        """Destroy the action clients first; in Foxy they fail if the node goes first."""
        for client in self.nav_clients.values():
            client.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Leader()
    try:
        rclpy.spin(node)        # single-threaded (section 10)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
