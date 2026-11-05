"""
Navigation node, one instance per robot namespace (ARCHITECTURE.md sections 6.7, 6.8, 10).

Runs as /tb3_0/robot_handler and /tb3_1/robot_handler. It offers the `navigate_to`
action: drive to a target and, if requested, turn and look for markers there.

NAV-04 scope: the robot drives straight to the target with GoToPointController.
Path planning (NAV-05) and the lidar safety stop, recovery and two-robot rules
(NAV-06) are added later; the `navigate` method is where they go.

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

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from rescue_interfaces.action import NavigateTo

from rescue_team.core.controller import GoToPointController
from rescue_team.core.geometry import distance, Pose2D, rotate, wrap_angle, yaw_from_quaternion
from rescue_team.core.grid import GridMap

CONTROL_PERIOD = 0.05      # s, wall time between two control steps
READY_PERIOD = 0.5         # s, how often the node checks whether it can start the action server
SPIN_GAIN = 6.0            # 1/s, turn rate per radian left of a spin step (slows down at the end)
SPIN_MIN_SPEED = 0.3       # rad/s, slowest turn rate, so a step always finishes
SPIN_TOLERANCE = 0.02      # rad, a spin step counts as done this close to its angle


class RobotHandler(Node):
    """Drive one robot to targets sent through the `navigate_to` action."""

    def __init__(self):
        super().__init__('robot_handler')

        # ---------- parameters (config/params.yaml) ----------
        p = {name: self.declare_parameter(name, default).value for name, default in (
            ('robots', ['tb3_0', 'tb3_1']),
            ('grid_resolution', 0.10),
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

        # ---------- latest sensor data (written by callbacks, read by the goal) ----------
        self.pose = None                 # Pose2D of this robot, map frame (odom = map, SPK-01)
        self.static_grid = None          # GridMap of /map at grid_resolution
        self.scan = None                 # latest LaserScan (safety stop in NAV-06)
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
        """Build the static grid at grid_resolution (planning grids are added in NAV-05)."""
        try:
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

    def on_odom(self, msg: Odometry) -> None:
        """Store this robot's pose."""
        try:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            self.pose = Pose2D(p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        except Exception as e:
            self.get_logger().error(f'on_odom: {e!r}')

    def on_peer_odom(self, msg: Odometry) -> None:
        """Store the other robot's position and its velocity in the map frame."""
        try:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            v = msg.twist.twist.linear
            self.peer_position = (p.x, p.y)
            self.peer_velocity = rotate((v.x, v.y), yaw)  # the twist is in the robot frame
        except Exception as e:
            self.get_logger().error(f'on_peer_odom: {e!r}')

    def on_scan(self, msg: LaserScan) -> None:
        """Store the latest scan (used by the safety stop in NAV-06)."""
        try:
            self.scan = msg
        except Exception as e:
            self.get_logger().error(f'on_scan: {e!r}')

    def on_ready_timer(self) -> None:
        """Create the action server once the map and the own pose are known."""
        try:
            if self.static_grid is None or self.pose is None:
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
        except Exception as e:
            self.get_logger().error(f'on_ready_timer: {e!r}')

    # ---------- action server callbacks ----------

    def goal_callback(self, goal_request) -> GoalResponse:
        """Accept every goal; the newest one replaces the running one."""
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        """Accept every cancel request."""
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle) -> None:
        """Make the new goal the active one, then start it."""
        try:
            with self.goal_lock:
                self.active_goal = goal_handle
            goal_handle.execute()
        except Exception as e:
            self.get_logger().error(f'handle_accepted_callback: {e!r}')

    def execute_callback(self, goal_handle) -> NavigateTo.Result:
        """Run one goal: navigate, spin if requested, and end with the right status."""
        target = goal_handle.request.target
        spin = goal_handle.request.spin_on_arrival
        message = 'aborted'
        try:
            self.get_logger().info(
                f'goal ({target.x:.2f}, {target.y:.2f}), spin={spin}')
            message = self.navigate(goal_handle, (target.x, target.y))
            if message == 'reached' and spin:
                message = self.spin(goal_handle)
        except Exception as e:
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

    def navigate(self, goal_handle, target) -> str:
        """
        Drive to `target`. Return reached, preempted or cancelled.

        NAV-04: straight to the target with GoToPointController. NAV-05 replaces this
        with planning and path following; NAV-06 adds the safety stop and recovery.
        """
        feedback = NavigateTo.Feedback()
        while rclpy.ok():
            ended = self.check_goal(goal_handle)
            if ended:
                return ended

            d = distance((self.pose.x, self.pose.y), target)
            if d < self.p['goal_tolerance']:
                return 'reached'

            v, omega = self.controller.compute(self.pose, target)
            if not self.publish_velocity(goal_handle, v, omega):
                return 'preempted'
            feedback.distance_left = float(d)
            goal_handle.publish_feedback(feedback)
            time.sleep(CONTROL_PERIOD)
        return 'aborted'

    def spin(self, goal_handle) -> str:
        """
        Turn and look (section 6.8): equal steps of at most spin_step, a pause after each.

        Return reached, preempted or cancelled. Ends after twice the nominal time at the
        latest (then also `reached`: the target itself was reached).
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

    def check_goal(self, goal_handle) -> Optional[str]:
        """Return preempted or cancelled if the goal must end now, otherwise None."""
        if goal_handle is not self.active_goal:
            return 'preempted'
        if goal_handle.is_cancel_requested:
            return 'cancelled'
        return None

    def publish_velocity(self, goal_handle, v: float, omega: float) -> bool:
        """Clamp and publish, but only for the active goal. Return False otherwise."""
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
        """Return the simulation time in seconds (use_sim_time); used for timeouts (section 10)."""
        return self.get_clock().now().nanoseconds * 1e-9


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
