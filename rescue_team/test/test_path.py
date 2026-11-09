"""
Tests for core/path.py, on hand-made grids and the w1 map.

Grids are written as text: '#' is blocked, '.' is free. The first line is row 0,
the first character is column 0. Unless stated otherwise the resolution is 1 m and
the origin is (0, 0), so the centre of cell (r, c) is (c + 0.5, r + 0.5).
"""
import math
import os

import numpy as np
import pytest

try:
    from rescue_team.core.path import (
        cells_to_points, line_of_sight, ProgressMonitor, remaining_length, simplify_path)
except ModuleNotFoundError as error:
    # Skip only while core/path.py does not exist; any other import error must fail.
    if error.name != 'rescue_team.core.path':
        raise
    pytest.skip('core/path.py is not written yet (NAV-03)', allow_module_level=True)

from rescue_team.core.distance_transform import cost_field, descend
from rescue_team.core.grid import GridMap

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
W1_YAML = os.path.join(DATA_DIR, 'map_dat160_w1.yaml')


def parse(text):
    """Turn a text grid into a bool array."""
    rows = [line.strip() for line in text.strip().splitlines()]
    return np.array([[ch == '#' for ch in row] for row in rows], dtype=bool)


def make_grid(text, resolution=1.0, origin=(0.0, 0.0)):
    return GridMap(parse(text), resolution, origin)


def centre(r, c):
    """Cell centre for resolution 1 and origin (0, 0)."""
    return (c + 0.5, r + 0.5)


def crosses_blocked(grid, blocked, a, b):
    """
    Check independently whether the line runs through blocked cells for more than 0.3 cell.

    line_of_sight samples every quarter cell or finer, so it may miss a shorter clip of
    a corner. Leaving the grid always counts.
    """
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(1, int(math.ceil(length / grid.resolution * 20)))
    step = length / n
    run = 0.0
    for i in range(n + 1):
        t = i / n
        r, c = grid.world_to_cell(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        if not grid.in_bounds(r, c):
            return True
        if blocked[r, c]:
            run += step
            if run > 0.3 * grid.resolution:
                return True
        else:
            run = 0.0
    return False


# --- cells_to_points ----------------------------------------------------------------------

def test_cells_to_points_gives_cell_centres():
    grid = make_grid('...\n...', resolution=0.5, origin=(-1.0, 2.0))
    points = cells_to_points(grid, [(0, 0), (1, 2)])
    # Column gives x, row gives y.
    assert points[0] == pytest.approx((-0.75, 2.25))
    assert points[1] == pytest.approx((0.25, 2.75))
    assert len(points) == 2


def test_cells_to_points_keeps_the_order():
    grid = make_grid('....')
    points = cells_to_points(grid, [(0, 3), (0, 1), (0, 2)])
    assert [p[0] for p in points] == pytest.approx([3.5, 1.5, 2.5])


def test_cells_to_points_of_an_empty_path_is_empty():
    assert cells_to_points(make_grid('..'), []) == []


# --- line_of_sight ------------------------------------------------------------------------

def test_line_of_sight_in_open_space():
    grid = make_grid('....\n....\n....')
    assert line_of_sight(grid, grid.occupied, centre(0, 0), centre(2, 3))


def test_line_of_sight_through_a_wall_is_false():
    grid = make_grid("""
        .....
        ..#..
        .....
    """)
    assert not line_of_sight(grid, grid.occupied, centre(1, 0), centre(1, 4))


def test_line_of_sight_passing_beside_a_wall():
    grid = make_grid("""
        .....
        ..#..
        .....
    """)
    assert line_of_sight(grid, grid.occupied, centre(0, 0), centre(0, 4))
    assert line_of_sight(grid, grid.occupied, centre(2, 0), centre(2, 4))


def test_line_of_sight_through_part_of_a_cell():
    # From (0.5, 0.5) to (2.5, 1.5): the line enters cell (1, 1) at (1.5, 1.0)
    # and leaves it at (2.0, 1.25), so it crosses the blocked cell.
    grid = make_grid("""
        ...
        .#.
        ...
    """)
    assert not line_of_sight(grid, grid.occupied, (0.5, 0.5), (2.5, 1.5))
    # Ending at y = 0.9 keeps the whole line in row 0.
    assert line_of_sight(grid, grid.occupied, (0.5, 0.5), (2.5, 0.9))


def test_line_of_sight_clipping_the_corner_of_a_cell():
    # The line y = x + 0.6 is inside cell (1, 1) for x in [1.0, 1.4], about 0.57 cells.
    # Both end points are free, and so are the cells they lie in.
    grid = make_grid("""
        ...
        .#.
        ...
    """)
    assert not line_of_sight(grid, grid.occupied, (0.2, 0.8), (1.8, 2.4))


def test_line_of_sight_samples_at_least_every_quarter_cell():
    # The line x + y = 2.3 is inside cell (1, 1) for x in [1.0, 1.3]: 0.42 cells long.
    # Samples a quarter cell apart must land in it; samples half a cell apart do not.
    grid = make_grid("""
        ...
        .#.
        ...
    """)
    assert not line_of_sight(grid, grid.occupied, (0.3, 2.0), (2.0, 0.3))


def test_line_of_sight_samples_finely_on_a_fine_grid():
    # Same corner clip, with 0.1 m cells: sampling must follow the resolution.
    grid = make_grid("""
        ...
        .#.
        ...
    """, resolution=0.1)
    assert not line_of_sight(grid, grid.occupied, (0.02, 0.08), (0.18, 0.24))


def test_line_of_sight_uses_the_blocked_array_not_the_grid():
    grid = make_grid('.....\n.....\n.....')
    blocked = parse("""
        .....
        ..#..
        .....
    """)
    assert line_of_sight(grid, grid.occupied, centre(1, 0), centre(1, 4))
    assert not line_of_sight(grid, blocked, centre(1, 0), centre(1, 4))


def test_line_of_sight_ending_in_a_blocked_cell_is_false():
    grid = make_grid('..#')
    assert not line_of_sight(grid, grid.occupied, centre(0, 0), centre(0, 2))
    assert not line_of_sight(grid, grid.occupied, centre(0, 2), centre(0, 0))


def test_line_of_sight_checks_both_end_points():
    # (2.05, 0.5) is only 0.05 cells into the blocked cell: only the end point itself
    # lies in it, so the end points must be among the samples.
    grid = make_grid('..#')
    assert not line_of_sight(grid, grid.occupied, centre(0, 0), (2.05, 0.5))
    assert not line_of_sight(grid, grid.occupied, (2.05, 0.5), centre(0, 0))


def test_line_of_sight_leaving_the_grid_is_false():
    grid = make_grid('...\n...')
    assert not line_of_sight(grid, grid.occupied, centre(0, 0), (3.5, 0.5))
    assert not line_of_sight(grid, grid.occupied, (-0.5, 0.5), centre(0, 2))


def test_line_of_sight_from_a_point_to_itself():
    grid = make_grid('.#')
    assert line_of_sight(grid, grid.occupied, centre(0, 0), centre(0, 0))
    assert not line_of_sight(grid, grid.occupied, centre(0, 1), centre(0, 1))


# --- simplify_path ------------------------------------------------------------------------

def test_simplify_a_straight_line_keeps_the_ends():
    grid = make_grid('......')
    points = [centre(0, c) for c in range(6)]
    assert simplify_path(grid, grid.occupied, points) == [centre(0, 0), centre(0, 5)]


def test_simplify_a_staircase_in_open_space():
    # The 45 degree zig-zag of an 8-connected path becomes one straight segment.
    grid = make_grid('......\n......\n......')
    cells = [(0, 0), (0, 1), (1, 2), (1, 3), (2, 4), (2, 5)]
    points = cells_to_points(grid, cells)
    assert simplify_path(grid, grid.occupied, points) == [centre(0, 0), centre(2, 5)]


def test_simplify_keeps_the_corner():
    grid = make_grid("""
        .....
        ####.
        ####.
    """)
    cells = [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (1, 4), (2, 4)]
    points = cells_to_points(grid, cells)
    assert simplify_path(grid, grid.occupied, points) == [
        centre(0, 0), centre(0, 4), centre(2, 4)]


def test_simplify_around_a_wall_on_a_descent():
    blocked = parse("""
        ......
        ..##..
        ..#...
        ......
    """)
    grid = GridMap(blocked, 1.0, (0.0, 0.0))
    field = cost_field(blocked, 1.0, [(0, 5)])
    points = cells_to_points(grid, descend(field, (3, 0)))
    simple = simplify_path(grid, blocked, points)

    assert simple[0] == points[0]
    assert simple[-1] == points[-1]
    assert len(simple) < len(points)
    # Only points from the path, in the same order.
    indices = [points.index(p) for p in simple]
    assert indices == sorted(indices)
    for a, b in zip(simple, simple[1:]):
        assert not crosses_blocked(grid, blocked, a, b)


def test_simplify_short_paths_are_unchanged():
    grid = make_grid('...')
    assert simplify_path(grid, grid.occupied, []) == []
    assert simplify_path(grid, grid.occupied, [centre(0, 1)]) == [centre(0, 1)]
    assert simplify_path(grid, grid.occupied, [centre(0, 0), centre(0, 1)]) == [
        centre(0, 0), centre(0, 1)]


def test_simplify_does_not_change_the_input():
    grid = make_grid('....')
    points = [centre(0, c) for c in range(4)]
    before = list(points)
    simplify_path(grid, grid.occupied, points)
    assert points == before


def test_simplify_uses_the_blocked_array_not_the_grid():
    grid = make_grid('.....\n.....\n.....')
    blocked = parse("""
        .....
        ..#..
        .....
    """)
    # A detour over the blocked cell (1, 2), which spans x in [2, 3] and y in [1, 2].
    p0, p1, p2, p3, p4 = (0.5, 1.5), (1.2, 0.6), (2.5, 0.4), (3.8, 0.6), (4.5, 1.5)
    points = [p0, p1, p2, p3, p4]
    # Nothing blocked in the grid itself: straight from start to goal.
    assert simplify_path(grid, grid.occupied, points) == [p0, p4]
    # With the blocked array: p0 -> p3 passes (2.0, 1.09), inside the blocked cell;
    # p0 -> p2 and p2 -> p4 stay in row 0 while they are in column 2.
    assert simplify_path(grid, blocked, points) == [p0, p2, p4]


@pytest.fixture(scope='module')
def w1_planning_grid():
    return GridMap.from_yaml(W1_YAML).downsample(10).inflate(0.30)


def test_simplified_path_on_w1_never_crosses_a_blocked_cell(w1_planning_grid):
    grid = w1_planning_grid
    blocked = grid.occupied
    # Path from the farthest reachable cell back to the spawn at (0, -1): long, with corners.
    field = cost_field(blocked, grid.resolution, [grid.world_to_cell(0.0, -1.0)])
    far = np.unravel_index(np.argmax(np.where(np.isfinite(field), field, -1.0)), field.shape)
    cells = descend(field, (int(far[0]), int(far[1])))
    points = cells_to_points(grid, cells)

    simple = simplify_path(grid, blocked, points)

    assert simple[0] == points[0]
    assert simple[-1] == points[-1]
    assert 2 < len(simple) < len(points) // 4
    for a, b in zip(simple, simple[1:]):
        assert not crosses_blocked(grid, blocked, a, b)


# --- remaining_length ---------------------------------------------------------------------

POINTS = [(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)]


def test_remaining_length_from_the_start():
    assert remaining_length(POINTS, 0, (0.0, 0.0)) == pytest.approx(7.0)


def test_remaining_length_on_the_way_to_a_point():
    # 3 m to (3, 0), then 4 m to (3, 4).
    assert remaining_length(POINTS, 1, (0.0, 0.0)) == pytest.approx(7.0)
    assert remaining_length(POINTS, 1, (1.0, 0.0)) == pytest.approx(6.0)


def test_remaining_length_off_the_path():
    # 5 m in a straight line from (0, 4) to (3, 0), plus the last 4 m.
    assert remaining_length(POINTS, 1, (0.0, 4.0)) == pytest.approx(9.0)


def test_remaining_length_on_the_last_segment():
    assert remaining_length(POINTS, 2, (3.0, 1.0)) == pytest.approx(3.0)
    assert remaining_length(POINTS, 2, (3.0, 4.0)) == pytest.approx(0.0)


# --- ProgressMonitor ----------------------------------------------------------------------

def test_progress_monitor_stuck_after_the_timeout():
    monitor = ProgressMonitor(timeout=8.0, min_progress=0.05)
    monitor.reset(0.0, 10.0)
    assert monitor.update(5.0, 10.0) is False
    assert monitor.update(7.99, 10.0) is False
    assert monitor.update(8.0, 10.0) is True


def test_progress_monitor_progress_restarts_the_clock():
    monitor = ProgressMonitor(timeout=8.0, min_progress=0.05)
    monitor.reset(0.0, 10.0)
    assert monitor.update(5.0, 9.9) is False
    assert monitor.update(12.0, 9.9) is False
    assert monitor.update(13.0, 9.9) is True


def test_progress_monitor_exactly_min_progress_counts():
    monitor = ProgressMonitor(timeout=8.0, min_progress=0.5)
    monitor.reset(0.0, 10.0)
    assert monitor.update(6.0, 9.5) is False
    assert monitor.update(13.0, 9.5) is False
    assert monitor.update(14.0, 9.5) is True


def test_progress_monitor_small_creeps_add_up():
    # Progress is measured from the value at the last progress, not from the last update.
    monitor = ProgressMonitor(timeout=8.0, min_progress=0.05)
    monitor.reset(0.0, 10.0)
    assert monitor.update(3.0, 9.98) is False
    assert monitor.update(6.0, 9.96) is False
    assert monitor.update(7.0, 9.94) is False
    # 9.94 counted as progress at t = 7, so the robot is stuck at t = 15, not t = 8.
    assert monitor.update(8.0, 9.94) is False
    assert monitor.update(14.0, 9.94) is False
    assert monitor.update(15.0, 9.94) is True


def test_progress_monitor_going_back_and_forth_is_stuck():
    # Turning back and forth: remaining goes up and down, but never 0.05 below 10.
    monitor = ProgressMonitor(timeout=8.0, min_progress=0.05)
    monitor.reset(0.0, 10.0)
    for now, remaining in [(2.0, 10.3), (4.0, 9.97), (6.0, 10.2), (7.5, 9.96)]:
        assert monitor.update(now, remaining) is False
    assert monitor.update(8.0, 10.1) is True


def test_progress_monitor_progress_is_relative_to_the_reset_value():
    # A longer remaining length after a detour is not progress.
    monitor = ProgressMonitor(timeout=8.0, min_progress=0.05)
    monitor.reset(0.0, 10.0)
    assert monitor.update(3.0, 12.0) is False
    assert monitor.update(8.0, 12.0) is True
    assert monitor.update(9.0, 9.9) is False


def test_progress_monitor_reset_starts_again():
    monitor = ProgressMonitor(timeout=8.0, min_progress=0.05)
    monitor.reset(0.0, 10.0)
    assert monitor.update(9.0, 10.0) is True
    # New plan: a new remaining length and a new clock.
    monitor.reset(9.0, 20.0)
    assert monitor.update(16.0, 20.0) is False
    assert monitor.update(17.0, 20.0) is True
    # The new reference is 20, so 19.9 is progress.
    monitor.reset(20.0, 20.0)
    assert monitor.update(27.0, 19.9) is False
