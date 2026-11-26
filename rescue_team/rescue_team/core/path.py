"""
Path conversion, line-of-sight simplification, remaining length and stuck detection.

Turns the cells from the distance transform into a short list of points in metres
(ARCHITECTURE.md section 6.3) and watches the progress along it for the stuck
recovery (section 6.5). No ROS imports here (see docs/CODE_STRUCTURE.md): the
functions work on plain numbers and NumPy arrays, so they can be tested with
pytest without starting ROS.
"""
# AI-use declaration (report Appendix D): the code in this module was written by hand by
# Torgeir (NAV-03); Claude (claude.ai) was used for explanation and review, and wrote test/test_path.py.

import math
from typing import List

import numpy as np

from rescue_team.core.geometry import Cell, distance, Point
from rescue_team.core.grid import GridMap


# AI use: none - hand-written by Torgeir; Claude: explanation and review only (NAV-03).
def cells_to_points(grid: GridMap, cells: List[Cell]) -> List[Point]:
    """Convert a grid path to the centres of its cells in metres, in the same order."""
    # Reuse cell_to_world so the cell-centre formula lives in one place only.
    points = []
    for (row, col) in cells:
        points.append(grid.cell_to_world(row, col))
    return points


# AI use: none - hand-written by Torgeir; Claude: explanation and review only (NAV-03).
def line_of_sight(grid: GridMap, blocked: np.ndarray, a: Point, b: Point) -> bool:
    """
    Return True if the straight line from a to b crosses no blocked cell.

    The line is sampled every quarter cell or finer, end points included. A sample
    outside the grid counts as blocked.
    """
    length = distance(a, b)
    step = grid.resolution / 4
    # Round up so the samples are never more than a quarter cell apart;
    # at least one interval so a == b does not divide by zero.
    n = max(1, math.ceil(length / step))

    # n + 1 samples: t = 0 is a and t = 1 is b, so both end points are checked.
    for i in range(0, n + 1):
        t = i / n
        x = a[0] + (b[0] - a[0]) * t
        y = a[1] + (b[1] - a[1]) * t
        r, c = grid.world_to_cell(x, y)

        # in_bounds first: a negative index would silently read from the other side.
        if not grid.in_bounds(r, c) or blocked[r, c]:
            return False

    return True


# AI use: none - hand-written by Torgeir; Claude: explanation and review only (NAV-03).
def simplify_path(grid: GridMap, blocked: np.ndarray, points: List[Point]) -> List[Point]:
    """
    Remove the zig-zag of a grid path and keep only start, corners and goal.

    From each kept point, jump to the farthest later point in line of sight. The
    result is a new list of points taken from the input, in the same order.
    """
    # Nothing to remove; still a copy, so the caller's list is never shared.
    if len(points) <= 2:
        return points.copy()

    result = [points[0]]
    i = 0
    last = len(points) - 1

    while i < last:
        # Look from the goal back towards i: the first point in sight is the farthest.
        # The next point (i + 1) is a single grid step and always kept, so the loop
        # moves forward every round and ends.
        j = last
        while j > i + 1 and not line_of_sight(grid, blocked, points[i], points[j]):
            j = j - 1
        result.append(points[j])
        i = j
    return result


# AI use: none - hand-written by Torgeir; Claude: explanation and review only (NAV-03).
def remaining_length(points: List[Point], index: int, position: Point) -> float:
    """Distance from position to points[index] plus the rest of the path, in metres."""
    # From where the robot actually is, which may be beside the path.
    total = distance(position, points[index])

    # Only the segments ahead of the robot; k + 1 is at most the last index.
    for k in range(index, len(points) - 1):
        total = total + distance(points[k], points[k + 1])

    return total


class ProgressMonitor:
    """
    Detects a robot that makes no progress along its path.

    The robot is stuck when the remaining path length has not dropped by
    min_progress within timeout seconds. Progress is measured from the value at
    the last progress, so many small steps add up, but turning back and forth does not.
    """

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (NAV-03).
    def __init__(self, timeout: float, min_progress: float) -> None:
        """Store stuck_timeout (s) and stuck_min_progress (m)."""
        self.timeout = timeout
        self.min_progress = min_progress
        # Before the first reset, any measurement counts as progress.
        self.best_remaining = np.inf
        self.last_progress_time = 0.0

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (NAV-03).
    def reset(self, now: float, remaining: float) -> None:
        """Start again from the current remaining length, for a new plan or while waiting."""
        self.best_remaining = remaining
        self.last_progress_time = now

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (NAV-03).
    def update(self, now: float, remaining: float) -> bool:
        """Record the remaining length at time now; return True if the robot is stuck."""
        # Compare with the value at the last progress, not with the last update:
        # small steps add up, and a detour or reversing never counts as progress.
        if remaining <= self.best_remaining - self.min_progress:
            self.best_remaining = remaining
            self.last_progress_time = now
            return False
        return (now - self.last_progress_time) >= self.timeout
