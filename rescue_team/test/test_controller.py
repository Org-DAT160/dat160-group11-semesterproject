"""Tests for core/controller.py: the two modes, slowing down and the limits."""
import math

import pytest

from rescue_team.core.controller import GoToPointController
from rescue_team.core.geometry import Pose2D

# The values from config/params.yaml.
V_MAX, W_MAX, K_D, K_H, TURN = 0.26, 1.5, 1.0, 1.5, 0.5


@pytest.fixture
def controller():
    return GoToPointController(V_MAX, W_MAX, K_D, K_H, TURN)


def test_turns_in_place_when_target_is_behind(controller):
    v, w = controller.compute(Pose2D(0.0, 0.0, 0.0), (-1.0, 0.1))
    assert v == 0.0
    assert abs(w) == pytest.approx(W_MAX)  # large error: turn at the limit


def test_turns_left_for_a_target_on_the_left(controller):
    v, w = controller.compute(Pose2D(0.0, 0.0, 0.0), (0.0, 1.0))
    assert v == 0.0
    assert w > 0  # positive omega = counter-clockwise = left


def test_drives_at_full_speed_when_facing_a_far_target(controller):
    v, w = controller.compute(Pose2D(0.0, 0.0, 0.0), (2.0, 0.0))
    assert v == pytest.approx(V_MAX)
    assert w == pytest.approx(0.0)


def test_slows_down_near_the_target(controller):
    v, _ = controller.compute(Pose2D(0.0, 0.0, 0.0), (0.1, 0.0))
    assert v == pytest.approx(K_D * 0.1)  # 0.1 m away: v = k_d * d = 0.1 m/s


def test_corrects_heading_while_driving(controller):
    # 10 degrees off: below the turn threshold, so it drives AND turns a little.
    v, w = controller.compute(Pose2D(0.0, 0.0, 0.0), (1.0, math.tan(math.radians(10))))
    assert v > 0
    assert w == pytest.approx(K_H * math.radians(10))


def test_heading_error_uses_the_short_way_round(controller):
    # Facing almost -pi, target almost +pi direction: only a small turn is needed.
    pose = Pose2D(0.0, 0.0, math.radians(-175))
    _, w = controller.compute(pose, (-1.0, math.tan(math.radians(5))))  # at +175 degrees
    assert w == pytest.approx(K_H * math.radians(-10))
