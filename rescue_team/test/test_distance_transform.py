"""
Tests for core/distance_transform.py, on hand-made grids and the w1 map.

Grids are written as text: '#' is blocked, '.' is free. The first line is row 0,
the first character is column 0 (array indexing, no row flip involved).
Expected field values are worked out by hand with h = 1 unless stated otherwise;
S stands for the diagonal step sqrt(2).
"""
import math
import os
import time

import numpy as np
import pytest

try:
    from rescue_team.core.distance_transform import (
        cost_at, cost_field, descend, nearest_free_cell, wall_distance)
except ModuleNotFoundError as error:
    # Skip only while core/distance_transform.py does not exist; any other import error must fail.
    if error.name != 'rescue_team.core.distance_transform':
        raise
    pytest.skip('core/distance_transform.py is not written yet (NAV-03)',
                allow_module_level=True)

from rescue_team.core import distance_transform
from rescue_team.core.grid import GridMap

# The planner is written after the functions above; skip only its tests until it exists.
DistanceTransformPlanner = getattr(distance_transform, 'DistanceTransformPlanner', None)
needs_planner = pytest.mark.skipif(DistanceTransformPlanner is None,
                                   reason='DistanceTransformPlanner is not written yet (NAV-03)')

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
W1_YAML = os.path.join(DATA_DIR, 'map_dat160_w1.yaml')

S = math.sqrt(2.0)
INF = math.inf


def parse(text):
    """Turn a text grid into a bool array."""
    rows = [line.strip() for line in text.strip().splitlines()]
    return np.array([[ch == '#' for ch in row] for row in rows], dtype=bool)


def assert_field(field, expected):
    """Compare a field with nested lists of expected values, inf included."""
    expected = np.array(expected, dtype=float)
    assert field.shape == expected.shape
    assert np.issubdtype(field.dtype, np.floating)
    assert np.array_equal(np.isinf(field), np.isinf(expected))
    finite = np.isfinite(expected)
    assert field[finite] == pytest.approx(expected[finite])


def assert_valid_descent(field, path, start):
    """Path starts at start, ends on a source and every step is a legal step downhill."""
    assert path[0] == start
    assert field[path[-1]] == 0.0
    for (r0, c0), (r1, c1) in zip(path, path[1:]):
        dr, dc = r1 - r0, c1 - c0
        assert max(abs(dr), abs(dc)) == 1, 'not a neighbour step'
        assert field[r1, c1] < field[r0, c0], 'not downhill'
        if dr and dc:
            # No corner cutting: both orthogonal cells beside a diagonal step are reachable.
            assert math.isfinite(field[r0 + dr, c0])
            assert math.isfinite(field[r0, c0 + dc])


# The example from the explanation: source at (0, 5), wall in columns 2-3.
EXAMPLE = """
    ......
    ..##..
    ..#...
    ......
"""
EXAMPLE_FIELD = [
    [5, 4, 3, 2, 1, 0],
    [4 + S, 5, INF, INF, S, 1],
    [5 + S, 6, INF, 2 + S, 1 + S, 2],
    [4 + 2 * S, 3 + 2 * S, 2 + 2 * S, 1 + 2 * S, 2 + S, 3],
]


# --- cost_field ---------------------------------------------------------------------------

def test_cost_field_open_row():
    field = cost_field(parse('.....'), 0.1, [(0, 0)])
    assert_field(field, [[0.0, 0.1, 0.2, 0.3, 0.4]])


def test_cost_field_small_known_grid():
    field = cost_field(parse(EXAMPLE), 1.0, [(0, 5)])
    assert_field(field, EXAMPLE_FIELD)


def test_cost_field_scales_with_resolution():
    field = cost_field(parse(EXAMPLE), 0.1, [(0, 5)])
    assert_field(field, np.array(EXAMPLE_FIELD) * 0.1)


def test_cost_field_diagonal_step_in_open_space():
    field = cost_field(parse('...\n...\n...'), 1.0, [(1, 1)])
    assert_field(field, [
        [S, 1, S],
        [1, 0, 1],
        [S, 1, S],
    ])


def test_cost_field_wall_in_the_way():
    # Straight-line distance from (0, 0) to (0, 4) is 4, but the wall forces a detour
    # through the gap in the bottom row.
    field = cost_field(parse("""
        ..#..
        ..#..
        .....
    """), 1.0, [(0, 0)])
    assert_field(field, [
        [0, 1, INF, 5 + S, 4 + 2 * S],
        [1, S, INF, 4 + S, 3 + 2 * S],
        [2, 1 + S, 2 + S, 3 + S, 4 + S],
    ])


def test_cost_field_no_corner_cutting_past_one_wall():
    # (1, 1) is diagonal to the source, but the step would cut the corner of (0, 1).
    field = cost_field(parse("""
        .#
        ..
    """), 1.0, [(0, 0)])
    assert_field(field, [
        [0, INF],
        [1, 2],
    ])


def test_cost_field_no_squeezing_between_two_corners():
    field = cost_field(parse("""
        .#
        #.
    """), 1.0, [(0, 0)])
    assert_field(field, [
        [0, INF],
        [INF, INF],
    ])


def test_cost_field_unreachable_cell_is_inf():
    field = cost_field(parse("""
        .....
        .###.
        .#.#.
        .###.
    """), 1.0, [(0, 0)])
    assert math.isinf(field[2, 2])
    assert field[3, 4] == pytest.approx(7)


def test_cost_field_several_sources_take_the_nearest():
    field = cost_field(parse('.......'), 0.5, [(0, 0), (0, 6)])
    assert_field(field, [[0.0, 0.5, 1.0, 1.5, 1.0, 0.5, 0.0]])


def test_cost_field_source_on_a_blocked_cell_is_ignored():
    field = cost_field(parse('#..'), 1.0, [(0, 0), (0, 2)])
    assert_field(field, [[INF, 1, 0]])


def test_cost_field_no_sources_gives_all_inf():
    field = cost_field(parse('...\n...'), 1.0, [])
    assert np.isinf(field).all()


def test_cost_field_accepts_a_generator_of_sources():
    field = cost_field(parse('...'), 1.0, (cell for cell in [(0, 0)]))
    assert_field(field, [[0, 1, 2]])


def test_cost_field_does_not_change_the_blocked_array():
    blocked = parse(EXAMPLE)
    before = blocked.copy()
    cost_field(blocked, 1.0, [(0, 5)])
    assert np.array_equal(blocked, before)


# --- wall_distance ------------------------------------------------------------------------

def test_wall_distance_along_a_row():
    field = wall_distance(parse('#....'), 0.1)
    assert_field(field, [[0.0, 0.1, 0.2, 0.3, 0.4]])


def test_wall_distance_around_one_occupied_cell():
    field = wall_distance(parse('...\n.#.\n...'), 0.5)
    assert_field(field, np.array([
        [S, 1, S],
        [1, 0, 1],
        [S, 1, S],
    ]) * 0.5)


def test_wall_distance_takes_the_nearest_wall():
    field = wall_distance(parse("""
        #.....
        ......
        .....#
    """), 1.0)
    # Walls at (0, 0) and (2, 5); with no blocked cells every diagonal step is allowed.
    assert_field(field, [
        [0, 1, 2, 2 * S, 1 + S, 2],
        [1, S, 1 + S, 1 + S, S, 1],
        [2, 1 + S, 2 * S, 2, 1, 0],
    ])


def test_wall_distance_without_walls_is_all_inf():
    assert np.isinf(wall_distance(parse('...\n...'), 1.0)).all()


# --- descend ------------------------------------------------------------------------------

def test_descend_ends_at_the_source():
    field = cost_field(parse(EXAMPLE), 1.0, [(0, 5)])
    path = descend(field, (3, 0))
    assert path is not None
    assert_valid_descent(field, path, (3, 0))
    assert path[-1] == (0, 5)


def test_descend_from_every_reachable_cell():
    field = cost_field(parse(EXAMPLE), 1.0, [(0, 5)])
    for r, c in np.argwhere(np.isfinite(field)):
        start = (int(r), int(c))
        path = descend(field, start)
        assert path is not None
        assert_valid_descent(field, path, start)


def test_descend_takes_the_nearest_of_several_sources():
    field = cost_field(parse('.......'), 1.0, [(0, 0), (0, 6)])
    assert descend(field, (0, 4)) == [(0, 4), (0, 5), (0, 6)]


def test_descend_from_the_source_is_just_the_source():
    field = cost_field(parse(EXAMPLE), 1.0, [(0, 5)])
    assert descend(field, (0, 5)) == [(0, 5)]


def test_descend_does_not_cut_corners():
    # The diagonal (0, 0) has the lowest value, but (0, 1) beside the step is inf.
    field = np.array([
        [0.0, INF],
        [1.0, 2.0],
    ])
    assert descend(field, (1, 1)) == [(1, 1), (1, 0), (0, 0)]


def test_descend_from_an_unreachable_cell_is_none():
    field = cost_field(parse("""
        .....
        .###.
        .#.#.
        .###.
    """), 1.0, [(0, 0)])
    assert descend(field, (2, 2)) is None


def test_descend_from_a_blocked_cell_is_none():
    field = cost_field(parse(EXAMPLE), 1.0, [(0, 5)])
    assert descend(field, (1, 2)) is None


# --- nearest_free_cell --------------------------------------------------------------------

def test_nearest_free_cell_of_a_free_cell_is_itself():
    assert nearest_free_cell(parse('...\n...'), (1, 2), 3) == (1, 2)


def test_nearest_free_cell_steps_out_of_a_wall():
    blocked = parse("""
        .....
        ###..
        #####
    """)
    assert nearest_free_cell(blocked, (2, 0), 5) == (0, 0)


def test_nearest_free_cell_is_euclidean_nearest():
    # From (3, 3): (1, 1) is first in row order but 2.83 cells away, (3, 1) is 2 away.
    blocked = parse("""
        #######
        #.#####
        #######
        #.#####
        #######
    """)
    assert nearest_free_cell(blocked, (3, 3), 5) == (3, 1)


def test_nearest_free_cell_respects_max_radius():
    # The only free cell is 4 * sqrt(2) = 5.66 cells away.
    blocked = parse("""
        .####
        #####
        #####
        #####
        #####
    """)
    assert nearest_free_cell(blocked, (4, 4), 2) is None
    assert nearest_free_cell(blocked, (4, 4), 5) is None
    assert nearest_free_cell(blocked, (4, 4), 6) == (0, 0)


def test_nearest_free_cell_from_outside_the_grid():
    assert nearest_free_cell(parse('.....\n.....'), (-1, 2), 3) == (0, 2)


def test_nearest_free_cell_returns_plain_ints():
    cell = nearest_free_cell(parse('#.'), (0, 0), 2)
    assert cell == (0, 1)
    assert all(type(v) is int for v in cell)


# --- cost_at ------------------------------------------------------------------------------

def make_grid(text, resolution, origin):
    return GridMap(parse(text), resolution, origin)


def test_cost_at_reads_the_cell_under_the_point():
    grid = make_grid('...\n.#.', 0.5, (-1.0, 2.0))
    field = cost_field(grid.occupied, grid.resolution, [(0, 0)])
    # Cell (0, 2) spans x in [0.0, 0.5), y in [2.0, 2.5).
    assert cost_at(field, grid, (0.2, 2.4)) == pytest.approx(1.0)
    # Cell (1, 2) spans y in [2.5, 3.0); reached around the wall: (0,1) -> (0,2) -> (1,2).
    assert cost_at(field, grid, (0.3, 2.6)) == pytest.approx(1.5)
    assert isinstance(cost_at(field, grid, (0.2, 2.4)), float)


def test_cost_at_blocked_or_unreachable_is_none():
    grid = make_grid('...\n.#.', 0.5, (-1.0, 2.0))
    field = cost_field(grid.occupied, grid.resolution, [(0, 0)])
    assert cost_at(field, grid, (-0.3, 2.7)) is None


def test_cost_at_outside_the_grid_is_none():
    grid = make_grid('...\n...', 0.5, (-1.0, 2.0))
    field = cost_field(grid.occupied, grid.resolution, [(0, 0)])
    assert cost_at(field, grid, (-1.2, 2.2)) is None
    assert cost_at(field, grid, (0.6, 2.2)) is None
    assert cost_at(field, grid, (0.0, 3.1)) is None


# --- w1 -----------------------------------------------------------------------------------

@pytest.fixture(scope='module')
def w1_planning_grid():
    return GridMap.from_yaml(W1_YAML).downsample(10).inflate(0.30)


def test_full_field_on_w1_is_fast(w1_planning_grid):
    grid = w1_planning_grid
    # The two spawn positions in w1 are (0, -1) and (0, 1).
    source = grid.world_to_cell(0.0, -1.0)
    other = grid.world_to_cell(0.0, 1.0)

    started = time.perf_counter()
    field = cost_field(grid.occupied, grid.resolution, [source])
    elapsed = time.perf_counter() - started

    # Generous limit: the course VM is 2-4x slower than a laptop.
    assert elapsed < 3.0
    # The other robot can be reached, and no shorter than in a straight line.
    assert field[other] >= 2.0 - 1e-9
    assert math.isfinite(field[other])
    # The free area outside the arena is closed off by the outer wall.
    assert math.isinf(field[0, 0])


# --- DistanceTransformPlanner -------------------------------------------------------------
# Resolution 1 m and origin (0, 0) unless stated otherwise: the centre of cell (r, c)
# is (c + 0.5, r + 0.5).

def centre(r, c):
    return (c + 0.5, r + 0.5)


def disc(grid, point, radius):
    """Cells whose centre is within radius of point, worked out cell by cell."""
    rows, cols = grid.occupied.shape
    mask = np.zeros((rows, cols), dtype=bool)
    for r in range(rows):
        for c in range(cols):
            x, y = grid.cell_to_world(r, c)
            mask[r, c] = math.hypot(x - point[0], y - point[1]) <= radius
    return mask


def crosses_blocked(grid, blocked, a, b):
    """
    Check independently whether the line runs through blocked cells for more than 0.3 cell.

    line_of_sight samples every quarter cell, so it may miss a shorter corner clip.
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


def assert_clear_path(grid, blocked, path):
    """Every point is in a free cell and no segment runs through a blocked cell."""
    for x, y in path:
        r, c = grid.world_to_cell(x, y)
        assert grid.in_bounds(r, c) and not blocked[r, c]
    for a, b in zip(path, path[1:]):
        assert not crosses_blocked(grid, blocked, a, b)


def path_length(path):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


OPEN = '\n'.join(['...........'] * 7)


@needs_planner
def test_plan_in_open_space_is_a_straight_line():
    grid = make_grid('........\n........\n........', 1.0, (0.0, 0.0))
    planner = DistanceTransformPlanner(avoid_radius=1.0, max_snap_radius=2)
    assert planner.plan(grid, (0.5, 1.5), (7.5, 1.5)) == [(0.5, 1.5), (7.5, 1.5)]


@needs_planner
def test_plan_goes_around_a_wall():
    grid = make_grid("""
        ........
        ...#....
        ...#....
        ...#....
        ........
    """, 1.0, (0.0, 0.0))
    planner = DistanceTransformPlanner(avoid_radius=1.0, max_snap_radius=2)
    path = planner.plan(grid, centre(2, 1), centre(2, 6))
    assert path is not None
    assert path[0] == centre(2, 1)
    assert path[-1] == centre(2, 6)
    assert len(path) > 2
    assert path_length(path) > 5.0
    assert_clear_path(grid, grid.occupied, path)


@needs_planner
def test_plan_avoid_disc_forces_a_detour():
    grid = make_grid(OPEN, 1.0, (0.0, 0.0))
    planner = DistanceTransformPlanner(avoid_radius=1.5, max_snap_radius=2)
    start, goal, other = centre(3, 0), centre(3, 10), centre(3, 5)

    assert planner.plan(grid, start, goal) == [start, goal]

    path = planner.plan(grid, start, goal, avoid=[other])
    assert path is not None
    assert path[0] == start
    assert path[-1] == goal
    assert len(path) > 2
    # Neither the points nor the simplified segments enter the disc.
    assert_clear_path(grid, disc(grid, other, 1.5), path)


@needs_planner
def test_plan_respects_several_avoid_points():
    grid = make_grid(OPEN, 1.0, (0.0, 0.0))
    planner = DistanceTransformPlanner(avoid_radius=1.2, max_snap_radius=2)
    others = [centre(2, 4), centre(5, 7)]
    path = planner.plan(grid, centre(3, 0), centre(3, 10), avoid=others)
    assert path is not None
    blocked = disc(grid, others[0], 1.2) | disc(grid, others[1], 1.2)
    assert_clear_path(grid, blocked, path)


@needs_planner
def test_plan_avoid_disc_is_for_that_call_only():
    grid = make_grid(OPEN, 1.0, (0.0, 0.0))
    before = grid.occupied.copy()
    planner = DistanceTransformPlanner(avoid_radius=1.5, max_snap_radius=2)
    start, goal = centre(3, 0), centre(3, 10)

    planner.plan(grid, start, goal, avoid=[centre(3, 5)])

    assert np.array_equal(grid.occupied, before)
    assert planner.plan(grid, start, goal) == [start, goal]


@needs_planner
def test_plan_snaps_start_and_goal_out_of_walls():
    # Start and goal lie in the blocked first and last column; each moves one cell in.
    grid = make_grid("""
        #....#
        #....#
        #....#
    """, 1.0, (0.0, 0.0))
    planner = DistanceTransformPlanner(avoid_radius=1.0, max_snap_radius=2)
    assert planner.plan(grid, centre(1, 0), centre(1, 5)) == [centre(1, 1), centre(1, 4)]


@needs_planner
def test_plan_start_or_goal_too_deep_in_a_wall_is_none():
    # The nearest free cell is 4 cells away from (1, 0).
    grid = make_grid("""
        ####....
        ####....
        ####....
    """, 1.0, (0.0, 0.0))
    assert DistanceTransformPlanner(1.0, 1).plan(grid, centre(1, 0), centre(1, 7)) is None
    assert DistanceTransformPlanner(1.0, 1).plan(grid, centre(1, 7), centre(1, 0)) is None
    path = DistanceTransformPlanner(1.0, 4).plan(grid, centre(1, 0), centre(1, 7))
    assert path == [centre(1, 4), centre(1, 7)]


@needs_planner
def test_plan_snapping_ignores_the_discs():
    # Start and goal are snapped on the grid alone, so a start or goal inside a disc
    # stays there and blocks the plan (robot_handler then plans without the disc).
    grid = make_grid(OPEN, 1.0, (0.0, 0.0))
    planner = DistanceTransformPlanner(avoid_radius=1.5, max_snap_radius=5)
    start, goal = centre(3, 0), centre(3, 10)
    assert planner.plan(grid, start, goal, avoid=[centre(3, 1)]) is None
    assert planner.plan(grid, start, goal, avoid=[centre(3, 9)]) is None


@needs_planner
def test_plan_without_a_path_is_none():
    grid = make_grid("""
        .......
        ...###.
        ...#.#.
        ...###.
    """, 1.0, (0.0, 0.0))
    planner = DistanceTransformPlanner(avoid_radius=1.0, max_snap_radius=2)
    assert planner.plan(grid, centre(0, 0), centre(2, 4)) is None
    assert planner.plan(grid, centre(2, 4), centre(0, 0)) is None


@needs_planner
def test_plan_start_and_goal_in_the_same_cell():
    grid = make_grid('...\n...', 1.0, (0.0, 0.0))
    planner = DistanceTransformPlanner(avoid_radius=1.0, max_snap_radius=2)
    assert planner.plan(grid, (1.2, 0.3), (1.8, 0.7)) == [centre(0, 1)]


@needs_planner
def test_plan_uses_metric_coordinates():
    # Resolution 0.5 m and an origin away from (0, 0): cell (1, 1) is centred at (-0.25, 2.75).
    grid = make_grid('.....\n.....\n.....', 0.5, (-1.0, 2.0))
    planner = DistanceTransformPlanner(avoid_radius=0.5, max_snap_radius=2)
    path = planner.plan(grid, (-0.3, 2.7), (0.8, 2.8))
    assert path == [pytest.approx((-0.25, 2.75)), pytest.approx((0.75, 2.75))]


@pytest.fixture(scope='module')
def w1_grids():
    static = GridMap.from_yaml(W1_YAML).downsample(10)
    return static, static.inflate(0.30)


@needs_planner
def test_plan_on_w1_with_start_and_goal_inside_inflated_walls(w1_grids):
    static, grid = w1_grids
    start, goal = (4.95, -6.65), (-3.35, 8.05)
    # Both lie next to a wall: free in the static grid, blocked in the planning grid.
    for point in (start, goal):
        r, c = grid.world_to_cell(*point)
        assert not static.occupied[r, c]
        assert grid.occupied[r, c]

    planner = DistanceTransformPlanner(avoid_radius=0.6, max_snap_radius=5)
    path = planner.plan(grid, start, goal)

    assert path is not None
    # Snapped by at most max_snap_radius cells (0.5 m) plus half a cell to the centre.
    assert math.hypot(path[0][0] - start[0], path[0][1] - start[1]) <= 0.5 + 0.08
    assert math.hypot(path[-1][0] - goal[0], path[-1][1] - goal[1]) <= 0.5 + 0.08
    assert len(path) >= 3
    assert_clear_path(grid, grid.occupied, path)


@needs_planner
def test_plan_on_w1_avoids_the_other_robot(w1_grids):
    _, grid = w1_grids
    # Robot at the spawn (0, -1) plans past the other robot standing at (0, 1).
    start, goal, other = (0.0, -1.0), (0.0, 2.5), (0.0, 1.0)
    planner = DistanceTransformPlanner(avoid_radius=0.6, max_snap_radius=5)

    path = planner.plan(grid, start, goal, avoid=[other])

    assert path is not None
    assert_clear_path(grid, grid.occupied | disc(grid, other, 0.6), path)
