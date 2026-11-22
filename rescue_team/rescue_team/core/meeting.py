"""
Meeting points for the big-fire meeting (ARCHITECTURE.md section 9).

When marker 4 (the big fire) is accepted, both robots must stand within 2 m of it at
the same time. `meeting_points` picks one point per robot near the reported marker
position:

    Point A  for the robot that is closer by travel distance: the reachable free cell
             within radius_a (meet_radius, 1.0 m) of the marker with the shortest
             travel distance for that robot.
    Point B  for the other robot: the reachable free cell within radius_b
             (meet_radius_second, 1.2 m), at least `separation` (meet_separation,
             0.8 m) from A, with the shortest travel distance for that robot. In an
             alcove where two robots do not fit, this puts the second robot at the
             opening.

Cells within exclude_radius (0.3 m) of an excluded point (a meeting point that failed
before) are skipped. If no pair exists, the result is None.

"Travel distance" comes from each robot's cost field: a distance-transform field
(`cost_field` in core/distance_transform.py) on the planning grid with the robot's
cell as source. A cell is reachable for a robot if its field value is finite; blocked
cells are inf in every field, so "reachable" also means "free".

No ROS imports here: the function works on plain arrays and can be tested with pytest
(test/test_meeting.py).

AI use: logic written by Jonas; docstrings by Claude Opus 5.5, which also explained the
specification and reviewed the changes.
"""
import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from rescue_team.core.geometry import distance, Point
from rescue_team.core.grid import GridMap


def candidates(grid: GridMap, marker: Point, field: np.ndarray, radius: float,
               exclude: Sequence[Point], exclude_radius: float) -> List[Tuple[float, Point]]:
    """
    Return (travel distance, cell centre) for every possible meeting cell of one robot.

    A cell is possible if its centre lies within `radius` of `marker`, its field value
    is finite (free and reachable for this robot), and it is more than
    `exclude_radius` from every excluded point. Sorted with the shortest travel
    distance first.
    """
    result = []
    row0, col0 = grid.world_to_cell(marker[0], marker[1])
    reach = math.ceil(radius / grid.resolution)
    for row in range(row0 - reach, row0 + reach + 1):
        for col in range(col0 - reach, col0 + reach + 1):
            if not grid.in_bounds(row, col):
                continue
            cost = field[row, col]
            centre = grid.cell_to_world(row, col)
            if not math.isfinite(cost):
                continue
            if distance(centre, marker) > radius:
                continue
            if any(distance(centre, p) <= exclude_radius for p in exclude):
                continue
            result.append((float(cost), centre))
    result.sort()
    return result


def meeting_points(grid: GridMap, marker: Point, fields: Dict[str, np.ndarray],
                   radius_a: float, radius_b: float, separation: float,
                   exclude: Sequence[Point] = (),
                   exclude_radius: float = 0.3) -> Optional[Dict[str, Point]]:
    """
    Return {robot: meeting point} for the two robots in `fields`, or None if no pair exists.

    1. Point A: for each robot, the best candidate within radius_a. The robot whose
       best candidate is cheaper gets point A.
    2. Point B: the other robot's cheapest candidate within radius_b that is at least
       `separation` from point A.
    """
    robots = list(fields)
    best_a = {}
    for robot in robots:
        options = candidates(grid, marker, fields[robot], radius_a, exclude, exclude_radius)
        if options:
            best_a[robot] = options[0]
    if not best_a:
        return None
    robot_a = min(best_a, key=lambda r: best_a[r][0])
    point_a = best_a[robot_a][1]
    robot_b = [r for r in robots if r != robot_a][0]
    options_b = candidates(grid, marker, fields[robot_b], radius_b, exclude, exclude_radius)
    for _, point in options_b:
        if distance(point, point_a) >= separation:
            return {robot_a: point_a, robot_b: point}
    return None
