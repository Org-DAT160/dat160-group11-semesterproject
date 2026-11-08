"""
Rules for two robots on one map.

Each robot_handler uses these to ignore the other robot in its lidar scan, to
detect when its own path runs into the other robot and to decide who waits
(ARCHITECTURE.md section 6.6). No ROS imports here (see docs/CODE_STRUCTURE.md):
the functions work on plain tuples, so they can be tested with pytest without
starting ROS.
"""

from typing import List, Sequence

from rescue_team.core.geometry import Point, distance
from rescue_team.core.path import remaining_length


def has_right_of_way(me: str, other: str, robots: Sequence[str]) -> bool:
    """Return True if me comes before other in the robots list."""
    return robots.index(me) < robots.index(other)


def remove_near(points: List[Point], centres: Sequence[Point], radius: float) -> List[Point]:
    """Return the points that are at least radius from every centre, in the same order."""
    return [point for point in points
            if all(distance(point, centre) >= radius for centre in centres)]


def peer_conflict(remaining: Sequence[Point], me: Point, peer: Point, peer_velocity: Point,
                  lookahead: float, clearance: float, step: float = 0.1,
                  away_speed: float = 0.1, away_gap: float = 0.7) -> bool:
    """
    Return True if my remaining path runs into the other robot.

    The path from me through remaining is sampled every step metres up to
    lookahead. A sample is a conflict if it is closer than clearance to peer and
    closer to peer than I am now, so driving away from the other robot is always
    allowed. There is no conflict while the other robot moves away from me faster
    than away_speed with a gap of at least away_gap. peer_velocity is in the map
    frame.
    """
    if not remaining:
        return False

    gap = distance(me, peer)
    if gap >= away_gap and _moving_away(me, peer, peer_velocity, away_speed):
        return False

    for sample in _samples(me, remaining, lookahead, step):
        distance_to_peer = distance(sample, peer)
        if distance_to_peer < clearance and distance_to_peer < gap:
            return True

    return False


def _moving_away(me: Point, peer: Point, peer_velocity: Point, away_speed: float) -> bool:
    """Return True if peer moves away from me faster than away_speed."""
    dx = peer[0] - me[0]
    dy = peer[1] - me[1]
    vx, vy = peer_velocity
    # The speed away from me is the dot product with the unit vector from me to
    # peer. Both sides are multiplied by the gap instead of dividing by it, so a
    # gap of 0 cannot divide by zero.
    return vx * dx + vy * dy > away_speed * distance(me, peer)


def _samples(me: Point, remaining: Sequence[Point], lookahead: float,
             step: float) -> List[Point]:
    """Points every step metres along the path from me, up to lookahead or the path end."""
    path = [me] + list(remaining)
    check_length = min(lookahead, remaining_length(path, 0, me))

    # k * step instead of adding step k times, so 10 * 0.1 is exactly 1.0. The
    # extra sample at check_length makes sure the far end is always checked.
    full_steps = int(check_length / step)
    along_path = [k * step for k in range(full_steps + 1)] + [check_length]
    return [_point_at(path, along) for along in along_path]


def _point_at(points: Sequence[Point], along: float) -> Point:
    """Point along metres along the polyline points; the last point if beyond its end."""
    travelled = 0.0
    for start, end in zip(points, points[1:]):
        segment_length = distance(start, end)
        # Segments of length 0 are skipped so the fraction never divides by zero.
        if segment_length > 0 and travelled + segment_length >= along:
            fraction = (along - travelled) / segment_length
            return (start[0] + fraction * (end[0] - start[0]),
                    start[1] + fraction * (end[1] - start[1]))
        travelled = travelled + segment_length

    return points[-1]
