"""
Tests for core/lidar.py with small hand-made scans.

AI use: first draft by Claude Opus 5.5; reviewed and adapted by Jonas.
"""
import math

import pytest

from rescue_team.core.geometry import Pose2D
from rescue_team.core.lidar import points_ahead, scan_to_points, sector_min

# A scan like the TurtleBot3's: 360 readings, one per degree, from angle 0.
STEP = 2 * math.pi / 360
RANGE_MIN, RANGE_MAX = 0.12, 3.5


def scan(readings):
    """360 inf readings, with the given {degree: range} filled in."""
    ranges = [math.inf] * 360
    for deg, r in readings.items():
        ranges[deg] = r
    return ranges


def test_scan_to_points_ignores_invalid_readings():
    ranges = scan({0: 1.0, 10: math.nan, 20: 0.05, 30: 5.0})  # nan, too close, too far
    points = scan_to_points(ranges, 0.0, STEP, RANGE_MIN, RANGE_MAX, Pose2D(0, 0, 0), 0.0)
    assert len(points) == 1


def test_scan_to_points_applies_offset_and_pose():
    # Robot at (1, 2) facing +y; lidar 0.064 m behind the centre.
    # A wall 1.0 m straight ahead of the lidar lies at y = 2 - 0.064 + 1.0.
    ranges = scan({0: 1.0})
    pose = Pose2D(1.0, 2.0, math.pi / 2)
    (x, y), = scan_to_points(ranges, 0.0, STEP, RANGE_MIN, RANGE_MAX, pose, -0.064)
    assert x == pytest.approx(1.0)
    assert y == pytest.approx(2.0 - 0.064 + 1.0)


def test_scan_to_points_left_reading():
    # Robot at the origin facing +x: the reading at 90 degrees points to +y.
    ranges = scan({90: 2.0})
    (x, y), = scan_to_points(ranges, 0.0, STEP, RANGE_MIN, RANGE_MAX, Pose2D(0, 0, 0), 0.0)
    assert (x, y) == (pytest.approx(0.0, abs=1e-9), pytest.approx(2.0))


def test_sector_min_front():
    ranges = scan({0: 2.0, 5: 1.5, 90: 0.5})
    assert sector_min(ranges, 0.0, STEP, RANGE_MIN, RANGE_MAX,
                      math.radians(-10), math.radians(10)) == pytest.approx(1.5)


def test_sector_min_wraps_around_zero():
    # Readings at 355 and 3 degrees both belong to the front sector -10..10 degrees.
    ranges = scan({355: 0.8, 3: 1.2})
    assert sector_min(ranges, 0.0, STEP, RANGE_MIN, RANGE_MAX,
                      math.radians(-10), math.radians(10)) == pytest.approx(0.8)


def test_sector_min_rear_wraps_around_pi():
    # The rear sector pi +- 0.5 rad (about 151..209 degrees) contains 160 and 200.
    ranges = scan({160: 0.9, 200: 0.6, 90: 0.2})
    assert sector_min(ranges, 0.0, STEP, RANGE_MIN, RANGE_MAX,
                      math.pi - 0.5, math.pi + 0.5) == pytest.approx(0.6)


def test_sector_min_empty_is_inf():
    assert sector_min(scan({}), 0.0, STEP, RANGE_MIN, RANGE_MAX, -0.5, 0.5) == math.inf


def test_points_ahead():
    pose = Pose2D(0.0, 0.0, math.pi / 2)  # facing +y
    assert points_ahead([(0.0, 0.3)], pose, 0.35, 0.2)  # 0.3 m straight ahead
    assert not points_ahead([(0.0, 0.5)], pose, 0.35, 0.2)  # too far
    assert not points_ahead([(0.3, 0.1)], pose, 0.35, 0.2)  # beside the robot
    assert not points_ahead([(0.0, -0.3)], pose, 0.35, 0.2)  # behind the robot
