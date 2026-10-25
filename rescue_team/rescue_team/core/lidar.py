"""
Lidar helpers: turn a laser scan into map points and check for obstacles.

Pure functions on plain numbers (no ROS), so they can be tested with pytest.
robot_handler passes the fields of a sensor_msgs/LaserScan message in.

Lidar facts (ARCHITECTURE.md section 2, SPK-01): 360 readings, one per degree,
counter-clockwise from straight ahead (angle 0). Invalid readings are inf. The rays
start lidar_offset_x = -0.064 m along the robot's x axis, i.e. behind base_footprint.
"""
import math
from typing import List, Sequence

from rescue_team.core.geometry import Point, Pose2D, wrap_angle


def _valid(r: float, range_min: float, range_max: float) -> bool:
    """Check that a reading is a real number inside the lidar's range."""
    return math.isfinite(r) and range_min <= r <= range_max


def scan_to_points(ranges: Sequence[float], angle_min: float, angle_increment: float,
                   range_min: float, range_max: float, pose: Pose2D,
                   offset_x: float) -> List[Point]:
    """
    Every valid reading as a point in the map frame.

    The ray of reading i leaves the lidar at angle `angle_min + i * angle_increment`
    (robot frame). The lidar sits `offset_x` metres along the robot's x axis, so the
    ray starts there, not at the robot centre.
    """
    # Where the lidar is on the map.
    ox = pose.x + offset_x * math.cos(pose.theta)
    oy = pose.y + offset_x * math.sin(pose.theta)
    points = []
    for i, r in enumerate(ranges):
        if not _valid(r, range_min, range_max):
            continue
        a = pose.theta + angle_min + i * angle_increment  # ray direction on the map
        points.append((ox + r * math.cos(a), oy + r * math.sin(a)))
    return points


def sector_min(ranges: Sequence[float], angle_min: float, angle_increment: float,
               range_min: float, range_max: float, start: float, end: float) -> float:
    """
    Closest valid reading with an angle from `start` to `end` (robot frame, radians).

    The sector may wrap around angle 0 or pi: for example straight back is
    start = pi - 0.5, end = pi + 0.5 (= -pi + 0.5 after wrapping). Returns inf if
    no valid reading lies in the sector.
    """
    start, end = wrap_angle(start), wrap_angle(end)
    closest = math.inf
    for i, r in enumerate(ranges):
        if not _valid(r, range_min, range_max):
            continue
        a = wrap_angle(angle_min + i * angle_increment)
        if start <= end:
            inside = start <= a <= end
        else:  # the sector wraps around +-pi
            inside = a >= start or a <= end
        if inside and r < closest:
            closest = r
    return closest


def points_ahead(points: Sequence[Point], pose: Pose2D, distance: float,
                 half_width: float) -> bool:
    """
    Return True if any point lies in the rectangle in front of the robot.

    The rectangle is 0 < x < distance and -half_width < y < half_width in the robot
    frame (origin at base_footprint, x forward). Used for the safety stop.
    """
    c, s = math.cos(pose.theta), math.sin(pose.theta)
    for px, py in points:
        dx, dy = px - pose.x, py - pose.y
        # Rotate the map offset into the robot frame (the inverse of the robot's heading).
        x = c * dx + s * dy
        y = -s * dx + c * dy
        if 0.0 < x < distance and -half_width < y < half_width:
            return True
    return False
