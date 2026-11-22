"""
Shared types and geometry helpers, used by every other module.

No ROS imports here (see docs/CODE_STRUCTURE.md): these functions work on plain
numbers, so they can be tested with pytest without starting ROS.

AI use: first draft by Claude Opus 5.5; reviewed and adapted by Jonas.
"""
from dataclasses import dataclass
import math
from typing import Callable, Optional, Tuple

import numpy as np

# Position in metres in the map frame.
Point = Tuple[float, float]

# Grid cell (row, col); row follows y and col follows x.
Cell = Tuple[int, int]

# Travel distance in metres from a robot to a point; None if unreachable.
CostFn = Callable[[str, Point], Optional[float]]


@dataclass
class Pose2D:
    """Robot pose in the map frame: position in metres, heading theta in radians (0 = +x)."""

    x: float
    y: float
    theta: float


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi], so that e.g. 350 degrees becomes -10 degrees."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Heading (rotation around the z axis) from an orientation quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def distance(a: Point, b: Point) -> float:
    """Straight-line distance between two points."""
    return math.hypot(b[0] - a[0], b[1] - a[1])


def heading_to(pose: Pose2D, target: Point) -> float:
    """How far the robot must turn to face `target`: positive = turn left, in [-pi, pi]."""
    direction = math.atan2(target[1] - pose.y, target[0] - pose.x)
    return wrap_angle(direction - pose.theta)


def rotate(v: Point, angle: float) -> Point:
    """Rotate a vector by `angle` (counter-clockwise), e.g. a velocity into the map frame."""
    c, s = math.cos(angle), math.sin(angle)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1])


def transform_point(translation: Tuple[float, float, float],
                    rotation: Tuple[float, float, float, float],
                    point: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """
    Move a 3D point from one frame into another, given the transform between them.

    `translation` and `rotation` (quaternion x, y, z, w) are the pose of the source
    frame inside the target frame, as TF returns them. Used by marker_detector to turn
    a marker position seen by the camera into map coordinates:
    point_map = R * point_camera + t.
    """
    x, y, z, w = rotation
    r = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    p = r.dot(np.array(point, dtype=float)) + np.array(translation, dtype=float)
    return (float(p[0]), float(p[1]), float(p[2]))
