"""
Tests for core/geometry.py, against cases that can be checked by hand.

AI use: first draft by Claude Opus 5.5; reviewed and adapted by Jonas.
"""
import math

import pytest

from rescue_team.core.geometry import (distance, heading_to, Pose2D, rotate, transform_point,
                                       wrap_angle, yaw_from_quaternion)


def test_wrap_angle():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(math.radians(350)) == pytest.approx(math.radians(-10))
    assert wrap_angle(math.radians(-190)) == pytest.approx(math.radians(170))
    assert wrap_angle(4 * math.pi + 0.5) == pytest.approx(0.5)


def test_yaw_from_quaternion():
    # No rotation.
    assert yaw_from_quaternion(0, 0, 0, 1) == pytest.approx(0.0)
    # 90 degrees around z: q = (0, 0, sin(45), cos(45)).
    s = math.sin(math.pi / 4)
    assert yaw_from_quaternion(0, 0, s, s) == pytest.approx(math.pi / 2)
    # -45 degrees, the orientation of marker 4 in w1.
    assert yaw_from_quaternion(0, 0, math.sin(-math.pi / 8), math.cos(-math.pi / 8)) == \
        pytest.approx(-math.pi / 4)


def test_distance():
    assert distance((0.0, 0.0), (3.0, 4.0)) == pytest.approx(5.0)


def test_heading_to():
    pose = Pose2D(0.0, 0.0, 0.0)  # at the origin, facing +x
    assert heading_to(pose, (1.0, 0.0)) == pytest.approx(0.0)  # straight ahead
    assert heading_to(pose, (0.0, 1.0)) == pytest.approx(math.pi / 2)  # turn left
    assert heading_to(pose, (0.0, -1.0)) == pytest.approx(-math.pi / 2)  # turn right
    # Facing +y, a target straight behind needs a half turn (either sign is fine).
    assert abs(heading_to(Pose2D(0.0, 0.0, math.pi / 2), (0.0, -1.0))) == pytest.approx(math.pi)


def test_rotate():
    x, y = rotate((1.0, 0.0), math.pi / 2)
    assert (x, y) == (pytest.approx(0.0, abs=1e-12), pytest.approx(1.0))
    x, y = rotate((1.0, 1.0), math.pi)
    assert (x, y) == (pytest.approx(-1.0), pytest.approx(-1.0))


def test_transform_point_translation_only():
    assert transform_point((1.0, 2.0, 3.0), (0, 0, 0, 1), (1.0, 1.0, 1.0)) == \
        pytest.approx((2.0, 3.0, 4.0))


def test_transform_point_rotation_and_translation():
    # Source frame turned 90 degrees left and placed at (1, 0, 0):
    # a point 1 m ahead in the source frame lies at (1, 1, 0) in the target frame.
    s = math.sin(math.pi / 4)
    assert transform_point((1.0, 0.0, 0.0), (0, 0, s, s), (1.0, 0.0, 0.0)) == \
        pytest.approx((1.0, 1.0, 0.0))
