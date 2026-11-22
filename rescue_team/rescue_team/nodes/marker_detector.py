"""
Global marker detector: ArUco detections to map-frame sightings (ARCHITECTURE.md section 7.1).

One node without a namespace for all robots (a namespaced Python TF listener receives
no transforms in Foxy, section 2). For every robot in `robots` it listens to
`/<robot>/aruco_markers`, moves every marker with ID 0-4 into the map frame with the
camera pose at the image time, and publishes one `FoundMarker` per sighting.

The camera message usually arrives before the robot pose for the same moment
(SPK-01), so a message whose transform is not available yet is kept in a queue and
tried again every 20 ms, and dropped after 1 s.

AI use: first draft by Claude Opus 5.5; reviewed and adapted by Jonas.
"""
from collections import deque
import math
import time
from typing import Optional, Tuple

from geometry_msgs.msg import Point as PointMsg
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.time import Time
from rescue_interfaces.msg import FoundMarker
from rescue_team.core.geometry import transform_point
from ros2_aruco_interfaces.msg import ArucoMarkers
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Fixed values (ARCHITECTURE.md section 13), not parameters.
VALID_IDS = range(0, 5)        # marker IDs used in the competition
RETRY_PERIOD = 0.02            # s, how often queued messages are tried again
RETRY_LIMIT = 1.0              # s, a message that still fails after this is dropped
LOG_PERIOD = 10.0              # s, how often the TF statistics are logged
MAX_QUEUE = 200                # messages; protects memory if TF stops completely


class MarkerDetector(Node):
    """Turn `/<robot>/aruco_markers` into `/found_marker` sightings in the map frame."""

    def __init__(self):
        super().__init__('marker_detector')
        self.robots = list(self.declare_parameter('robots', ['tb3_0', 'tb3_1']).value)
        self.max_range = float(self.declare_parameter('marker_max_range', 3.0).value)
        self.get_logger().info(
            f'parameters: robots={self.robots}, marker_max_range={self.max_range} m')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Messages waiting for their transform: (robot, msg, wall time when received).
        self.queue = deque()

        # Statistics for the log, reset every LOG_PERIOD.
        self.ok_at_once = 0
        self.ok_after_retry = 0
        self.dropped = 0
        self.failed_lookups = 0

        for robot in self.robots:
            # Default argument binds the robot name now, not when the lambda runs.
            self.create_subscription(
                ArucoMarkers, f'/{robot}/aruco_markers',
                lambda msg, robot=robot: self.on_markers(robot, msg), 10)
        self.publisher = self.create_publisher(FoundMarker, '/found_marker', 50)

        # Wall-clock timers: the simulation clock advances in steps of about 0.1 s,
        # which would make a 20 ms timer fire in bursts.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(RETRY_PERIOD, self.on_retry_timer, clock=steady)
        self.create_timer(LOG_PERIOD, self.on_log_timer, clock=steady)

    # ---------- callbacks ----------

    def on_markers(self, robot: str, msg: ArucoMarkers) -> None:
        """Try a new message at once; keep it in the queue if its transform is not there yet."""
        try:
            if not any(int(i) in VALID_IDS for i in msg.marker_ids):
                return
            if self.process(robot, msg):
                self.ok_at_once += 1
            else:
                if len(self.queue) >= MAX_QUEUE:
                    self.queue.popleft()
                    self.dropped += 1
                self.queue.append((robot, msg, time.monotonic()))
        except Exception as e:  # noqa: B902 (a crash in a callback would stop the node)
            self.get_logger().error(f'on_markers: {e!r}')

    def on_retry_timer(self) -> None:
        """Every 20 ms: try the queued messages again, drop the ones older than 1 s."""
        try:
            now = time.monotonic()
            waiting = deque()
            while self.queue:
                robot, msg, received = self.queue.popleft()
                if self.process(robot, msg):
                    self.ok_after_retry += 1
                elif now - received > RETRY_LIMIT:
                    self.dropped += 1
                else:
                    waiting.append((robot, msg, received))
            self.queue = waiting
        except Exception as e:  # noqa: B902
            self.get_logger().error(f'on_retry_timer: {e!r}')

    def on_log_timer(self) -> None:
        """Log the TF statistics of the last 10 s, so a TF problem shows in the log."""
        try:
            if self.ok_at_once or self.ok_after_retry or self.dropped:
                self.get_logger().info(
                    f'last {LOG_PERIOD:.0f} s: {self.ok_at_once} at once, '
                    f'{self.ok_after_retry} after retry, {self.dropped} dropped, '
                    f'{self.failed_lookups} failed lookups, {len(self.queue)} queued')
            self.ok_at_once = self.ok_after_retry = self.dropped = self.failed_lookups = 0
        except Exception as e:  # noqa: B902
            self.get_logger().error(f'on_log_timer: {e!r}')

    # ---------- processing ----------

    def process(self, robot: str, msg: ArucoMarkers) -> bool:
        """
        Publish every valid marker in `msg`; return False if the transform is not available.

        All markers in one message share the same image time. Every position is
        transformed before anything is published, so a message that has to be retried
        never publishes some of its markers twice.
        """
        stamp = Time.from_msg(msg.header.stamp)
        sightings = []
        for marker_id, pose in zip(msg.marker_ids, msg.poses):
            marker_id = int(marker_id)
            if marker_id not in VALID_IDS:
                continue
            p = pose.position
            dist = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
            if dist > self.max_range:
                continue  # far sightings are less accurate (SPK-02)
            map_point = self.to_map_frame(robot, (p.x, p.y, p.z), stamp)
            if map_point is None:
                return False
            sightings.append((marker_id, map_point, dist))

        for marker_id, map_point, dist in sightings:
            self.publish_sighting(robot, marker_id, map_point, dist, msg.header.stamp)
        return True

    def to_map_frame(self, robot: str, position: Tuple[float, float, float],
                     stamp: Time) -> Optional[Tuple[float, float, float]]:
        """Camera position at the image time into the map frame; None if TF has no answer yet."""
        try:
            # No timeout: the lookup never waits, the retry timer tries again later.
            t = self.tf_buffer.lookup_transform(
                'map', f'{robot}/camera_rgb_optical_frame', stamp)
        except TransformException:
            self.failed_lookups += 1
            return None
        tr, rot = t.transform.translation, t.transform.rotation
        return transform_point((tr.x, tr.y, tr.z), (rot.x, rot.y, rot.z, rot.w), position)

    def publish_sighting(self, robot: str, marker_id: int,
                         position: Tuple[float, float, float], distance: float, stamp) -> None:
        """Publish one FoundMarker. All fields are plain Python types (section 10)."""
        out = FoundMarker()
        out.header.stamp = stamp
        out.header.frame_id = 'map'
        out.marker_id = int(marker_id)
        out.position = PointMsg(x=float(position[0]), y=float(position[1]), z=float(position[2]))
        out.distance = float(distance)
        out.robot = str(robot)
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
