"""Tests for core/traffic.py, with hand-made positions in the map frame (metres).

Unless stated otherwise the defaults from params.yaml are used: path_lookahead
1.0 m, peer_stop_clearance 0.5 m, step 0.1 m, away_speed 0.1 m/s, away_gap 0.7 m.
I am at (0, 0) and drive along the x axis.
"""
import pytest

try:
    from rescue_team.core.traffic import has_right_of_way, peer_conflict, remove_near
except ModuleNotFoundError as error:
    # Skip only while core/traffic.py does not exist; any other import error must fail.
    if error.name != 'rescue_team.core.traffic':
        raise
    pytest.skip('core/traffic.py is not written yet (NAV-07)', allow_module_level=True)

ME = (0.0, 0.0)
AHEAD = [(3.0, 0.0)]      # straight ahead along +x, longer than the lookahead
BEHIND = [(-3.0, 0.0)]    # straight back along -x
STILL = (0.0, 0.0)


def conflict(remaining, peer, velocity=STILL, lookahead=1.0, clearance=0.5):
    """Call peer_conflict with the default parameters from params.yaml."""
    return peer_conflict(remaining, ME, peer, velocity, lookahead, clearance)


# remove_near

def test_remove_near_drops_points_close_to_the_peer():
    points = [(0.0, 0.0), (0.3, 0.0), (1.0, 0.0), (0.0, -0.2)]
    assert remove_near(points, [(0.0, 0.0)], 0.4) == [(1.0, 0.0)]


def test_remove_near_keeps_a_point_exactly_at_the_radius():
    # 'Within radius' means strictly closer than radius.
    assert remove_near([(0.5, 0.0)], [(0.0, 0.0)], 0.5) == [(0.5, 0.0)]


def test_remove_near_without_centres_keeps_everything_in_order():
    points = [(2.0, 1.0), (0.0, 0.0), (1.0, 2.0)]
    assert remove_near(points, [], 0.4) == points


def test_remove_near_checks_every_centre():
    points = [(0.0, 0.0), (5.0, 5.0), (2.5, 2.5)]
    assert remove_near(points, [(0.1, 0.0), (5.0, 5.1)], 0.4) == [(2.5, 2.5)]


def test_remove_near_does_not_change_the_input():
    points = [(0.0, 0.0), (1.0, 0.0)]
    remove_near(points, [(0.0, 0.0)], 0.4)
    assert points == [(0.0, 0.0), (1.0, 0.0)]


# has_right_of_way

def test_first_robot_in_the_list_has_right_of_way():
    robots = ['tb3_0', 'tb3_1']
    assert has_right_of_way('tb3_0', 'tb3_1', robots) is True
    assert has_right_of_way('tb3_1', 'tb3_0', robots) is False


def test_right_of_way_follows_the_list_not_the_names():
    robots = ['tb3_1', 'tb3_0']
    assert has_right_of_way('tb3_1', 'tb3_0', robots) is True
    assert has_right_of_way('tb3_0', 'tb3_1', robots) is False


# peer_conflict: the cases from the issue

def test_head_on_in_a_corridor_is_a_conflict():
    # The other robot comes towards me; at x = 0.8 my path is 0.4 m from it.
    assert conflict(AHEAD, (1.2, 0.0), velocity=(-0.2, 0.0)) is True


def test_driving_away_from_a_close_robot_is_no_conflict():
    # The other robot is only 0.3 m away, but every step takes me farther from it.
    # My own position (the first sample) is not closer to it than I am now.
    assert conflict(BEHIND, (0.3, 0.0)) is False


def test_other_robot_moving_away_is_no_conflict():
    # Without the away rule the sample at x = 0.4 would be 0.4 m from it.
    assert conflict(AHEAD, (0.8, 0.0), velocity=(0.2, 0.0)) is False


def test_other_robot_standing_on_the_path_is_a_conflict():
    assert conflict(AHEAD, (1.3, 0.0)) is True


# peer_conflict: the away rule

def test_moving_away_with_a_gap_below_away_gap_is_a_conflict():
    assert conflict(AHEAD, (0.6, 0.0), velocity=(0.2, 0.0)) is True


def test_moving_away_slower_than_away_speed_is_a_conflict():
    assert conflict(AHEAD, (0.8, 0.0), velocity=(0.05, 0.0)) is True


def test_moving_sideways_is_not_moving_away():
    # Fast, but the speed away from me is 0.
    assert conflict(AHEAD, (0.8, 0.0), velocity=(0.0, 0.3)) is True


def test_moving_away_is_measured_from_me_not_along_the_x_axis():
    # The other robot is straight up from me and moves up: away from me, along +y.
    assert conflict([(0.0, 3.0)], (0.0, 0.8), velocity=(0.0, 0.2)) is False
    # Moving along +x at the same place is sideways for me.
    assert conflict([(0.0, 3.0)], (0.0, 0.8), velocity=(0.2, 0.0)) is True


# peer_conflict: sampling the path

def test_samples_reach_the_lookahead():
    # Only the sample at exactly 1.0 m is closer than 0.5 m (0.45 m).
    assert conflict(AHEAD, (1.45, 0.0)) is True


def test_sample_exactly_at_the_clearance_is_no_conflict():
    # 'Closer than clearance' is strict: the sample at 1.0 m is exactly 0.5 m away.
    assert conflict(AHEAD, (1.5, 0.0)) is False


def test_samples_stop_at_the_lookahead():
    # Closest sample is at 1.0 m, 0.6 m from the other robot.
    assert conflict(AHEAD, (1.6, 0.0)) is False


def test_samples_follow_the_path_around_a_corner():
    # 0.5 m along x, then up. At 0.9 m along the path the sample is (0.5, 0.4),
    # 0.4 m from the other robot. No corner point is closer than 0.5 m to it, and
    # neither is a straight line from me towards the last point (0.5, 3.0).
    assert conflict([(0.5, 0.0), (0.5, 3.0)], (0.9, 0.4)) is True


def test_samples_between_waypoints_are_checked():
    # Waypoints far apart: only the samples on the segment come close.
    assert conflict([(1.0, 0.0), (1.0, 3.0)], (0.5, 0.3)) is True


def test_path_shorter_than_the_lookahead_is_not_extended():
    # The path ends at x = 0.5, 0.7 m from the other robot; x = 1.0 would be 0.2 m.
    assert conflict([(0.5, 0.0)], (1.2, 0.0)) is False


def test_end_of_a_short_path_is_sampled():
    # The path ends at x = 0.45, 0.45 m from the other robot; x = 0.4 is 0.5 m.
    assert conflict([(0.45, 0.0)], (0.9, 0.0)) is True


def test_no_path_left_is_no_conflict():
    assert conflict([], (0.3, 0.0)) is False


def test_other_robot_far_from_the_path_is_no_conflict():
    assert conflict(AHEAD, (0.5, 1.0)) is False


def test_lookahead_and_clearance_are_used():
    # Standing 1.4 m ahead: a conflict only with the default clearance.
    assert conflict(AHEAD, (1.4, 0.0), clearance=0.3) is False
    assert conflict(AHEAD, (1.4, 0.0)) is True
    # Standing 2.0 m ahead: a conflict only with a longer lookahead.
    assert conflict(AHEAD, (2.0, 0.0)) is False
    assert conflict(AHEAD, (2.0, 0.0), lookahead=1.8) is True
