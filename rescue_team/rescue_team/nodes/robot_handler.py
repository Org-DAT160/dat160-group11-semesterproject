"""
Navigation node, one instance per robot namespace (ARCHITECTURE.md sections 6, 10).

Runs as /tb3_0/robot_handler and /tb3_1/robot_handler. It offers the `navigate_to`
action: drive to a target and, if requested, turn and look for markers there.

Planning and path following (NAV-05): DistanceTransformPlanner on the inflated grid,
then GoToPointController point by point.

Reactive layer and two robots (NAV-06, sections 6.5 and 6.6):
    safety stop   no forward motion while a lidar point that is neither a known wall
                  nor the other robot lies in the stop zone ahead
    stuck         no progress along the path for stuck_timeout -> reverse while the rear
                  is clear, plan again; `stuck` after max_replans
    peer disc     every plan avoids a disc around the other robot when possible
    conflict      stop when the other robot is on the path ahead; the robot with right
                  of way waits, the other plans a detour; after peer_wait_timeout the
                  first plans again and the second ends its goal with `peer`

Goal handling: one goal at a time. A new goal becomes the active one at once, and
an older goal that is still running notices it at its next step and ends with
`preempted`. Every velocity command goes through `publish_velocity`, which only
publishes for the active goal, so an old goal can never stop the robot after a new
goal has taken over.
"""
import math
import threading
import time
from typing import Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rescue_interfaces.action import NavigateTo
from rescue_team.core.controller import GoToPointController
from rescue_team.core.distance_transform import DistanceTransformPlanner
from rescue_team.core.geometry import distance, Pose2D, rotate, wrap_angle, yaw_from_quaternion
from rescue_team.core.grid import GridMap
from rescue_team.core.lidar import points_ahead, scan_to_points, sector_min
from rescue_team.core.path import ProgressMonitor, remaining_length
from rescue_team.core.traffic import has_right_of_way, peer_conflict, remove_near
from sensor_msgs.msg import LaserScan

CONTROL_PERIOD = 0.05      # s, wall time between two control steps
READY_PERIOD = 0.5         # s, how often the node checks whether it can start the action server
SPIN_GAIN = 6.0            # 1/s, turn rate per radian left of a spin step (slows down at the end)
SPIN_MIN_SPEED = 0.3       # rad/s, slowest turn rate, so a step always finishes
SPIN_TOLERANCE = 0.02      # rad, a spin step counts as done this close to its angle
REAR_HALF_ANGLE = 0.5      # rad, half width of the rear lidar sector checked when reversing
PEER_BEHIND_RADIUS = 0.6   # m, do not reverse while the other robot is this close behind
CONFLICT_REPLAN_PERIOD = 1.0  # s, at most one plan per second because of the other robot


class RobotHandler(Node):
    """Drive one robot to targets sent through the `navigate_to` action."""

    # AI use: first draft by Claude Opus 5.5; reviewed and adapted by Jonas.
    def __init__(self):
        super().__init__('robot_handler')

        # ---------- parameters (config/params.yaml) ----------
        p = {name: self.declare_parameter(name, default).value for name, default in (
            ('robots', ['tb3_0', 'tb3_1']),
            ('grid_resolution', 0.10),
            ('inflation_radius', 0.30),
            ('waypoint_tolerance', 0.05),
            ('peer_plan_clearance', 0.60),
            ('goal_tolerance', 0.10),
            ('turn_threshold', 0.5),
            ('max_linear_speed', 0.26),
            ('distance_gain', 1.0),
            ('max_angular_speed', 1.5),
            ('heading_gain', 1.5),
            ('spin_speed', 1.5),
            ('spin_angle', 5.2),
            ('spin_step', 0.87),
            ('spin_pause', 0.5),
            ('obstacle_stop_distance', 0.35),
            ('obstacle_half_width', 0.2),
            ('lidar_offset_x', -0.064),
            ('peer_ignore_radius', 0.4),
            ('reverse_distance', 0.3),
            ('reverse_speed', 0.1),
            ('stuck_timeout', 8.0),
            ('stuck_min_progress', 0.05),
            ('max_replans', 5),
            ('peer_stop_clearance', 0.50),
            ('path_lookahead', 1.0),
            ('peer_wait_timeout', 10.0),
        )}
        self.p = p

        # ---------- robot names ----------
        self.robot = self.get_namespace().strip('/')
        peers = [r for r in p['robots'] if r != self.robot]
        self.peer = peers[0] if peers else None
        self.get_logger().info(
            f'robot={self.robot}, peer={self.peer}, parameters: '
            + ', '.join(f'{k}={v}' for k, v in sorted(p.items())))
        if self.robot not in p['robots']:
            self.get_logger().warn(f'own namespace "{self.robot}" is not in robots={p["robots"]}')

        self.controller = GoToPointController(
            max_linear_speed=p['max_linear_speed'], max_angular_speed=p['max_angular_speed'],
            distance_gain=p['distance_gain'], heading_gain=p['heading_gain'],
            turn_threshold=p['turn_threshold'])

        self.planner = DistanceTransformPlanner(
            avoid_radius=p['peer_plan_clearance'],
            max_snap_radius=5
        )
        # ---------- latest sensor data (written by callbacks, read by the goal) ----------
        self.pose = None                 # Pose2D of this robot, map frame (odom = map, SPK-01)
        self.static_grid = None          # GridMap of /map at grid_resolution
        self.planning_grid = None        # inflated by inflation_radius, for the planner
        self.known_wall_grid = None      # inflated by one cell, for the safety stop
        self.scan = None                 # latest LaserScan (rear check when reversing)
        self.obstacle_points = []        # map points of the latest scan without walls and the peer
        self.peer_position = None        # other robot's (x, y)
        self.peer_velocity = (0.0, 0.0)  # other robot's velocity in the map frame

        # ---------- goal handling ----------
        self.active_goal = None          # the only goal that may publish velocities
        self.goal_lock = threading.Lock()
        self.action_server = None

        # ---------- ROS interfaces ----------
        # Relative names get the namespace (/tb3_0/odom ...); /map and the peer are absolute.
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        # map_server publishes the map only once, so a late subscriber needs TRANSIENT_LOCAL.
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        if self.peer:
            self.create_subscription(Odometry, f'/{self.peer}/odom', self.on_peer_odom, 10)

        self.ready_timer = self.create_timer(
            READY_PERIOD, self.on_ready_timer, clock=Clock(clock_type=ClockType.STEADY_TIME))

    # ---------- sensor callbacks ----------

    def on_map(self, msg: OccupancyGrid) -> None:
        """
        Build the static, planning and known-wall grids (section 6.1).

        AI use: logic written by Jonas; docstring by Claude Opus 5.5, which also explained the
        specification and reviewed the change.
        """
        try:
            if self.planning_grid is not None:
                return
            info = msg.info
            full = GridMap.from_occupancy_grid(
                msg.data, info.width, info.height, info.resolution,
                (info.origin.position.x, info.origin.position.y))
            factor = max(1, int(round(self.p['grid_resolution'] / info.resolution)))
            self.static_grid = full.downsample(factor)
            self.planning_grid = self.static_grid.inflate(self.p['inflation_radius'])
            self.known_wall_grid = self.static_grid.inflate(self.p['grid_resolution'])
            rows, cols = self.static_grid.occupied.shape
            self.get_logger().info(
                f'map received: {info.width}x{info.height} at {info.resolution} m, '
                f'static grid {rows}x{cols} at {self.static_grid.resolution:.2f} m')
        except Exception as e:  # noqa: B902 (a crash in a callback would stop the node)
            self.get_logger().error(f'on_map: {e!r}')

    def on_odom(self, msg: Odometry) -> None:
        """
        Store this robot's pose.

        AI use: first draft by Claude Opus 5.5; reviewed and adapted by Jonas.
        """
        try:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            self.pose = Pose2D(p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        except Exception as e:  # noqa: B902
            self.get_logger().error(f'on_odom: {e!r}')

    def on_peer_odom(self, msg: Odometry) -> None:
        """
        Store the other robot's position and its velocity in the map frame.

        AI use: first draft by Claude Opus 5.5; reviewed and adapted by Jonas.
        """
        try:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            v = msg.twist.twist.linear
            self.peer_position = (p.x, p.y)
            self.peer_velocity = rotate((v.x, v.y), yaw)  # the twist is in the robot frame
        except Exception as e:  # noqa: B902
            self.get_logger().error(f'on_peer_odom: {e!r}')

    def on_scan(self, msg: LaserScan) -> None:
        """
        Turn the scan into map points that are neither known walls nor the other robot.

        AI use: logic written by Jonas; docstring by Claude Opus 5.5, which also explained the
        specification and reviewed the change.
        """
        try:
            self.scan = msg
            if self.pose is None or self.known_wall_grid is None:
                return
            points = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                    msg.range_min, msg.range_max, self.pose,
                                    self.p['lidar_offset_x'])
            points = [pt for pt in points if not self.known_wall_grid.is_known_wall(pt)]
            if self.peer_position is not None:
                points = remove_near(points, [self.peer_position], self.p['peer_ignore_radius'])
            self.obstacle_points = points
        except Exception as e:  # noqa: B902
            self.get_logger().error(f'on_scan: {e!r}')

    def on_ready_timer(self) -> None:
        """
        Create the action server once the map and the own pose are known.

        AI use: first draft by Claude Opus 5.5; reviewed and adapted by Jonas.
        """
        try:
            if self.planning_grid is None or self.pose is None:
                return
            self.action_server = ActionServer(
                self, NavigateTo, 'navigate_to',
                execute_callback=self.execute_callback,
                goal_callback=self.goal_callback,
                cancel_callback=self.cancel_callback,
                handle_accepted_callback=self.handle_accepted_callback,
                callback_group=ReentrantCallbackGroup())
            self.ready_timer.cancel()
            self.get_logger().info(
                f'ready: navigate_to at ({self.pose.x:.2f}, {self.pose.y:.2f})')
        except Exception as e:  # noqa: B902
            self.get_logger().error(f'on_ready_timer: {e!r}')

    # ---------- action server callbacks ----------

    def goal_callback(self, goal_request) -> GoalResponse:
        """
        Accept every goal; the newest one replaces the running one.

        AI use: first draft by Claude Opus 5.5; reviewed and adapted by Jonas.
        """
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        """
        Accept every cancel request.

        AI use: first draft by Claude Opus 5.5; reviewed and adapted by Jonas.
        """
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle) -> None:
        """
        Make the new goal the active one, then start it.

        AI use: first draft by Claude Opus 5.5; reviewed and adapted by Jonas.
        """
        try:
            with self.goal_lock:
                self.active_goal = goal_handle
            goal_handle.execute()
        except Exception as e:  # noqa: B902
            self.get_logger().error(f'handle_accepted_callback: {e!r}')

    def execute_callback(self, goal_handle) -> NavigateTo.Result:
        """
        Run one goal: navigate, spin if requested, and end with the right status.

        AI use: first draft by Claude Opus 5.5; reviewed and adapted by Jonas.
        """
        target = goal_handle.request.target
        spin = goal_handle.request.spin_on_arrival
        message = 'aborted'
        try:
            self.get_logger().info(
                f'goal ({target.x:.2f}, {target.y:.2f}), spin={spin}')
            message = self.navigate(goal_handle, (target.x, target.y))
            if message == 'reached' and spin:
                message = self.spin(goal_handle)
        except Exception as e:  # noqa: B902
            self.get_logger().error(f'execute_callback: {e!r}')
            message = 'aborted'
        finally:
            self.publish_velocity(goal_handle, 0.0, 0.0)  # only stops if still the active goal

        if message == 'reached':
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
            message = 'cancelled'
        else:
            goal_handle.abort()
        self.get_logger().info(f'goal ({target.x:.2f}, {target.y:.2f}) ended: {message}')

        result = NavigateTo.Result()
        result.success = message == 'reached'
        result.message = message
        return result

    # ---------- goal steps ----------

    def plan_path(self, goal_handle, target):
        """
        Stop, then plan from the robot's position to `target` (sections 6.2-6.3).

        The plan avoids a disc of peer_plan_clearance around the other robot (section
        6.6, rule 1), except when the goal lies inside the disc or the disc leaves no
        path; then it is planned without the disc. Return the simplified path as a
        list of (x, y) points from the start cell centre to the goal cell centre, or
        None if no path exists.

        AI use: logic written by Jonas; docstring by Claude Opus 5.5, which also explained the
        specification and reviewed the change.
        """
        self.publish_velocity(goal_handle, 0.0, 0.0)

        start = (self.pose.x, self.pose.y)
        peer = self.peer_position
        if peer is not None and distance(target, peer) > self.p['peer_plan_clearance']:
            path = self.planner.plan(self.planning_grid, start, target, avoid=[peer])
            if path is not None:
                return path
        return self.planner.plan(self.planning_grid, start, target)

    def follow_path(self, goal_handle, path, conflict) -> str:
        """
        Drive along `path` point by point (section 6.4).

        path[0] is the cell the robot stands in, so it starts at path[1]. Intermediate
        points count as passed within waypoint_tolerance, the last one within
        goal_tolerance. The robot does not drive forward while an obstacle is ahead
        (section 6.5), and the goal ends with stuck when the remaining path length has
        not dropped by stuck_min_progress within stuck_timeout. The feedback is the
        remaining length along the path.

        The other robot (section 6.6): on a conflict the robot stops and the timer in
        `conflict` starts. With right of way it waits, and asks for a new plan after
        peer_wait_timeout; without right of way it asks for a new plan at once (at most
        once per second) and ends with peer after peer_wait_timeout. Waiting does not
        count as being stuck. Return reached, stuck, replan, peer, preempted or cancelled.

        AI use: logic written by Jonas; docstring by Claude Opus 5.5, which also explained the
        specification and reviewed the change.
        """
        feedback = NavigateTo.Feedback()
        index = 1
        monitor = ProgressMonitor(self.p['stuck_timeout'], self.p['stuck_min_progress'])
        monitor.reset(self.now(), remaining_length(path, index, (self.pose.x, self.pose.y)))
        while rclpy.ok():
            ended = self.check_goal(goal_handle)
            target = path[index]
            is_last = index == len(path) - 1
            tolerance = self.p['goal_tolerance'] if is_last else self.p['waypoint_tolerance']
            if ended:
                return ended

            d = distance((self.pose.x, self.pose.y), target)
            if d < tolerance:
                if is_last:
                    return 'reached'
                index += 1
                continue
            if self.peer_blocks(path, index):
                if not self.publish_velocity(goal_handle, 0.0, 0.0):
                    return 'preempted'
                now = self.now()
                if conflict['since'] is None:
                    conflict['since'] = now
                    self.get_logger().info('conflict with the other robot: stopped')
                monitor.reset(now, remaining_length(path, index, (self.pose.x, self.pose.y)))
                waited = now - conflict['since']
                right_of_way = has_right_of_way(self.robot, self.peer, self.p['robots'])
                if not right_of_way and waited >= self.p['peer_wait_timeout']:
                    return 'peer'
                replan_due = (conflict['last_replan'] is None
                              or now - conflict['last_replan'] >= CONFLICT_REPLAN_PERIOD)
                if replan_due and (not right_of_way or waited >= self.p['peer_wait_timeout']):
                    conflict['last_replan'] = now
                    return 'replan'
                time.sleep(CONTROL_PERIOD)
                continue
            conflict['since'] = None

            v, omega = self.controller.compute(self.pose, target)
            if v > 0.0 and self.obstacle_ahead():
                v, omega = 0.0, 0.0

            if not self.publish_velocity(goal_handle, v, omega):
                return 'preempted'
            remaining = remaining_length(path, index, (self.pose.x, self.pose.y))
            if monitor.update(self.now(), remaining):
                self.publish_velocity(goal_handle, 0.0, 0.0)
                return 'stuck'
            feedback.distance_left = remaining
            goal_handle.publish_feedback(feedback)
            time.sleep(CONTROL_PERIOD)
        return 'aborted'

    def recover(self, goal_handle) -> str:
        """
        Reverse up to reverse_distance while the rear is clear (section 6.5). Return done.

        AI use: logic written by Jonas; docstring by Claude Opus 5.5, which also explained the
        specification and reviewed the change.
        """
        start = (self.pose.x, self.pose.y)
        deadline = self.now() + 2.0 * self.p['reverse_distance'] / self.p['reverse_speed']
        while rclpy.ok():
            ended = self.check_goal(goal_handle)
            if ended:
                return ended
            if distance(start, (self.pose.x, self.pose.y)) >= self.p['reverse_distance']:
                break
            if not self.rear_clear() or self.now() > deadline:
                break
            if not self.publish_velocity(goal_handle, -self.p['reverse_speed'], 0.0):
                return 'preempted'
            time.sleep(CONTROL_PERIOD)
        self.publish_velocity(goal_handle, 0.0, 0.0)
        return 'done'

    def navigate(self, goal_handle, target) -> str:
        """
        Plan and follow; when stuck, reverse and plan again, up to max_replans times.

        A conflict with the other robot asks for a new plan ('replan'); those plans do not
        count as recoveries, and the conflict timer in `conflict` lives on across them.
        Return reached, no path, stuck, peer, preempted or cancelled. A path of one point
        means the robot already stands in the goal cell.

        AI use: logic written by Jonas; docstring by Claude Opus 5.5, which also explained the
        specification and reviewed the change.
        """
        replans = 0
        conflict = {'since': None, 'last_replan': None}    # survives replans of this goal
        while True:
            path = self.plan_path(goal_handle, target)
            if path is None:
                return 'no path'
            if len(path) == 1:
                return 'reached'
            result = self.follow_path(goal_handle, path, conflict)
            if result == 'replan':
                self.get_logger().info('replanning around the other robot')
                continue
            if result != 'stuck':
                return result
            replans += 1
            if replans > self.p['max_replans']:
                return 'stuck'
            self.get_logger().warn(f'stuck, recovery {replans} of {self.p["max_replans"]}')
            result = self.recover(goal_handle)
            if result != 'done':
                return result

    def spin(self, goal_handle) -> str:
        """
        Turn and look (section 6.8): equal steps of at most spin_step, a pause after each.

        Return reached, preempted or cancelled. Ends after twice the nominal time at the
        latest (then also `reached`: the target itself was reached).

        AI use: first draft by Claude Opus 5.5; reviewed and adapted by Jonas.
        """
        angle, speed = self.p['spin_angle'], self.p['spin_speed']
        steps = max(1, math.ceil(angle / self.p['spin_step']))
        step_angle = angle / steps
        nominal = angle / speed + steps * self.p['spin_pause']
        deadline = self.now() + 2.0 * nominal

        # The angle is counted from the start of the whole spin, also while the robot
        # is braking during a pause. A step that overshoots therefore makes the next
        # step shorter, and the error does not add up over the steps.
        turned, last = 0.0, self.pose.theta

        def update_turned():
            nonlocal turned, last
            turned += wrap_angle(self.pose.theta - last)
            last = self.pose.theta

        for k in range(1, steps + 1):
            # Turn until the total angle reaches k steps. Slow down over the last
            # part of the step: the robot cannot stop at once from spin_speed.
            while True:
                ended = self.check_goal(goal_handle)
                if ended:
                    return ended
                if self.now() > deadline:
                    self.get_logger().warn('spin took too long, ending it')
                    return 'reached'
                update_turned()
                remaining = k * step_angle - turned
                if remaining <= SPIN_TOLERANCE:
                    break
                omega = min(speed, max(SPIN_MIN_SPEED, SPIN_GAIN * remaining))
                if not self.publish_velocity(goal_handle, 0.0, omega):
                    return 'preempted'
                time.sleep(CONTROL_PERIOD)

            # Stand still so the camera gets sharp images (SPK-02).
            pause_end = self.now() + self.p['spin_pause']
            while self.now() < pause_end:
                ended = self.check_goal(goal_handle)
                if ended:
                    return ended
                if not self.publish_velocity(goal_handle, 0.0, 0.0):
                    return 'preempted'
                time.sleep(CONTROL_PERIOD)
                update_turned()
        return 'reached'

    # ---------- helpers ----------

    def obstacle_ahead(self) -> bool:
        """
        Return True if a point of the latest scan lies in the stop zone ahead of the robot.

        AI use: logic written by Jonas; docstring by Claude Opus 5.5, which also explained the
        specification and reviewed the change.
        """
        return points_ahead(self.obstacle_points, self.pose,
                            self.p['obstacle_stop_distance'], self.p['obstacle_half_width'])

    def rear_clear(self) -> bool:
        """
        Return True if nothing is close behind: no lidar reading and not the other robot.

        AI use: logic written by Jonas; docstring by Claude Opus 5.5, which also explained the
        specification and reviewed the change.
        """
        scan = self.scan
        if scan is None:
            return False
        rear = sector_min(scan.ranges, scan.angle_min, scan.angle_increment,
                          scan.range_min, scan.range_max,
                          math.pi - REAR_HALF_ANGLE, math.pi + REAR_HALF_ANGLE)
        if rear <= self.p['obstacle_stop_distance']:
            return False
        if self.peer_position is not None:
            dx = self.peer_position[0] - self.pose.x
            dy = self.peer_position[1] - self.pose.y
            ahead = dx * math.cos(self.pose.theta) + dy * math.sin(self.pose.theta)
            if ahead < 0.0 and math.hypot(dx, dy) < PEER_BEHIND_RADIUS:
                return False
        return True

    def peer_blocks(self, path, index) -> bool:
        """
        Return True if the remaining path runs into the other robot (section 6.6, rule 2).

        AI use: logic written by Jonas; docstring by Claude Opus 5.5, which also explained the
        specification and reviewed the change.
        """
        if self.peer_position is None:
            return False
        return peer_conflict(path[index:], (self.pose.x, self.pose.y), self.peer_position,
                             self.peer_velocity, self.p['path_lookahead'],
                             self.p['peer_stop_clearance'])

    def check_goal(self, goal_handle) -> Optional[str]:
        """
        Return preempted or cancelled if the goal must end now, otherwise None.

        AI use: first draft by Claude Opus 5.5; reviewed and adapted by Jonas.
        """
        if goal_handle is not self.active_goal:
            return 'preempted'
        if goal_handle.is_cancel_requested:
            return 'cancelled'
        return None

    def publish_velocity(self, goal_handle, v: float, omega: float) -> bool:
        """
        Clamp and publish, but only for the active goal. Return False otherwise.

        AI use: first draft by Claude Opus 5.5; reviewed and adapted by Jonas.
        """
        max_omega = max(self.p['max_angular_speed'], self.p['spin_speed'])
        twist = Twist()
        twist.linear.x = float(max(-self.p['max_linear_speed'],
                                   min(self.p['max_linear_speed'], v)))
        twist.angular.z = float(max(-max_omega, min(max_omega, omega)))
        with self.goal_lock:
            if goal_handle is not self.active_goal:
                return False
            self.cmd_pub.publish(twist)
        return True

    def now(self) -> float:
        """
        Return the simulation time in seconds (use_sim_time); used for timeouts (section 10).

        AI use: first draft by Claude Opus 5.5; reviewed and adapted by Jonas.
        """
        return self.get_clock().now().nanoseconds * 1e-9


# AI use: first draft by Claude Opus 5.5; reviewed and adapted by Jonas.
def main(args=None):
    rclpy.init(args=args)
    node = RobotHandler()
    # Explicit thread count: the Foxy default is the number of CPU cores (section 10).
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
