"""Tests for core/viewpoints.py, on hand-made grids and the five course maps.

Grids are written as text: '#' is blocked in the planning grid, '.' is free. The
first line is row 0, the first character is column 0. Unless stated otherwise the
resolution is 1 m and the origin is (0, 0), so the lattice point (x, y) lies in
cell (floor(y), floor(x)) and the centre of cell (r, c) is (c + 0.5, r + 0.5).

The wall distance and the start field are hand-made arrays, so each test controls
exactly which cells have room to rotate, are close to a wall or are reachable.
By default every cell is 1.0 m from a wall (enough room, close enough) and
reachable.
"""
import math
import os

import numpy as np
import pytest

try:
    from rescue_team.core.viewpoints import generate_viewpoints
except ModuleNotFoundError as error:
    # Skip only while core/viewpoints.py does not exist; any other import error must fail.
    if error.name != 'rescue_team.core.viewpoints':
        raise
    pytest.skip('core/viewpoints.py is not written yet (MIS-05)', allow_module_level=True)

from rescue_team.core.distance_transform import cost_field, wall_distance
from rescue_team.core.grid import GridMap

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')

SPIN = 0.4       # spin_clearance
NEAR = 1.5       # max_wall_distance


def parse(text):
    """Turn a text grid into a bool array."""
    rows = [line.strip() for line in text.strip().splitlines()]
    return np.array([[ch == '#' for ch in row] for row in rows])


def free_grid(rows, cols, origin=(0.0, 0.0)):
    return GridMap(np.zeros((rows, cols), dtype=bool), 1.0, origin)


def run(grid, spacing, offset=(0.0, 0.0), wall=None, field=None):
    """generate_viewpoints with the defaults above, sorted for comparison."""
    shape = grid.occupied.shape
    wall = np.full(shape, 1.0) if wall is None else wall
    field = np.zeros(shape) if field is None else field
    points = generate_viewpoints(grid, wall, field, spacing, offset, SPIN, NEAR)
    return sorted(points)


def centres(cells):
    return sorted((c + 0.5, r + 0.5) for r, c in cells)


# Lattice

def test_lattice_on_an_open_grid():
    # Lattice x and y in {0, 2, 4}: cells (0, 0), (0, 2) ... (4, 4).
    expected = centres([(r, c) for r in (0, 2, 4) for c in (0, 2, 4)])
    assert run(free_grid(5, 5), 2.0) == expected


def test_offset_moves_the_lattice_for_pass_1():
    # Offset (1, 1): lattice x and y in {1, 3}; 5 is outside the map.
    expected = centres([(1, 1), (1, 3), (3, 1), (3, 3)])
    assert run(free_grid(5, 5), 2.0, offset=(1.0, 1.0)) == expected


def test_lattice_starts_at_the_map_origin():
    # Origin (-2, -1): lattice x in {-2, 0}, y in {-1, 1}: cells (0, 0), (0, 2), (2, 0), (2, 2).
    grid = free_grid(3, 3, origin=(-2.0, -1.0))
    assert run(grid, 2.0) == sorted([(-1.5, -0.5), (0.5, -0.5), (-1.5, 1.5), (0.5, 1.5)])


def test_returns_plain_floats():
    points = run(free_grid(3, 3), 2.0)
    assert points
    for x, y in points:
        assert type(x) is float and type(y) is float


# Snap

def test_blocked_lattice_cell_moves_to_the_nearest_free_cell():
    # Spacing 4: lattice cells (0, 0), (0, 4), (4, 0), (4, 4), search radius 2 cells.
    # (0, 0) and (0, 1) are blocked, so (0, 0) moves to (1, 0), one cell away.
    grid = GridMap(parse("""
        ##...
        .....
        .....
        .....
        .....
    """), 1.0, (0.0, 0.0))
    assert run(grid, 4.0) == centres([(1, 0), (0, 4), (4, 0), (4, 4)])


def test_cell_without_room_to_rotate_is_not_used():
    # Free in the planning grid, but (0, 0) and (0, 1) are too close to a wall.
    wall = np.full((5, 5), 1.0)
    wall[0, 0] = 0.3
    wall[0, 1] = 0.3
    assert run(free_grid(5, 5), 4.0, wall=wall) == centres([(1, 0), (0, 4), (4, 0), (4, 4)])


def test_spin_clearance_is_at_least_not_more_than():
    wall = np.full((5, 5), 1.0)
    wall[0, 0] = SPIN
    assert centres([(0, 0)])[0] in run(free_grid(5, 5), 4.0, wall=wall)


def test_snap_searches_within_half_the_spacing():
    # Spacing 4 in a 1 x 9 grid: lattice cells (0, 0), (0, 4), (0, 8), radius 2 cells.
    # Only (0, 2) is free: 2 cells from (0, 0) and from (0, 4), 6 cells from (0, 8).
    grid = GridMap(parse('##.######'), 1.0, (0.0, 0.0))
    assert run(grid, 4.0) == centres([(0, 2)])


def test_point_without_a_free_cell_within_half_the_spacing_is_dropped():
    # Spacing 4, radius 2 cells. The only free cell (1, 2) is sqrt(5) cells from (0, 0).
    grid = GridMap(parse("""
        ###
        ##.
    """), 1.0, (0.0, 0.0))
    assert run(grid, 4.0) == []


def test_search_radius_follows_the_resolution():
    # Resolution 0.5 m, spacing 2 m: radius 1 m is 2 cells. Lattice (0, 0) is cell (0, 0);
    # the only free cell (0, 2) is 2 cells away, its centre at x = 1.25.
    grid = GridMap(parse('######'), 0.5, (0.0, 0.0))
    grid.occupied[0, 2] = False
    assert run(grid, 2.0) == [(1.25, 0.25)]


# Filters

def test_unreachable_points_are_dropped():
    field = np.zeros((5, 5))
    field[0, 0] = np.inf
    assert run(free_grid(5, 5), 4.0, field=field) == centres([(0, 4), (4, 0), (4, 4)])


def test_reachability_is_checked_after_the_snap():
    # (0, 0) is blocked and moves to (1, 0), which is unreachable; (0, 0) itself is
    # reachable but blocked. The point is dropped, not moved again.
    grid = GridMap(parse("""
        ##...
        .....
        .....
        .....
        .....
    """), 1.0, (0.0, 0.0))
    field = np.zeros((5, 5))
    field[1, 0] = np.inf
    assert run(grid, 4.0, field=field) == centres([(0, 4), (4, 0), (4, 4)])


def test_points_far_from_a_wall_are_dropped():
    wall = np.full((5, 5), 1.0)
    wall[4, 4] = 1.6
    assert run(free_grid(5, 5), 4.0, wall=wall) == centres([(0, 0), (0, 4), (4, 0)])


def test_max_wall_distance_is_at_most_not_less_than():
    wall = np.full((5, 5), 1.0)
    wall[4, 4] = NEAR
    assert centres([(4, 4)])[0] in run(free_grid(5, 5), 4.0, wall=wall)


def test_duplicates_are_removed():
    # Spacing 2 in a 1 x 5 grid: lattice cells (0, 0), (0, 2), (0, 4), radius 1 cell.
    # Only (0, 1) is free; (0, 0) and (0, 2) both move there, (0, 4) finds nothing.
    grid = GridMap(parse('#.###'), 1.0, (0.0, 0.0))
    points = generate_viewpoints(grid, np.full((1, 5), 1.0), np.zeros((1, 5)), 2.0,
                                 (0.0, 0.0), SPIN, NEAR)
    assert points == [(1.5, 0.5)]


def test_inputs_are_not_changed():
    grid = free_grid(5, 5)
    wall = np.full((5, 5), 1.0)
    field = np.zeros((5, 5))
    run(grid, 2.0, wall=wall, field=field)
    assert not grid.occupied.any()
    assert (wall == 1.0).all()
    assert (field == 0.0).all()


# The five course maps

SPAWN = {
    'w1': [(0.0, -1.0), (0.0, 1.0)],
    'w2': [(-1.0, 1.0), (-1.0, -1.0)],
    'w3': [(-1.0, 0.5), (-1.0, -1.5)],
    'w4': [(-0.5, 0.7), (-0.5, -1.3)],
    'w5': [(0.0, -0.5), (0.0, -2.5)],
}
SPACING = 1.5


@pytest.fixture(scope='module', params=sorted(SPAWN))
def world(request):
    """Static grid, planning grid, wall distance and the field from both starts."""
    name = request.param
    static = GridMap.from_yaml(os.path.join(DATA_DIR, f'map_dat160_{name}.yaml')).downsample(10)
    planning = static.inflate(0.30)
    wall = wall_distance(static.occupied, static.resolution)
    fields = [cost_field(planning.occupied, planning.resolution, [planning.world_to_cell(*p)])
              for p in SPAWN[name]]
    # Reachable from both starts: finite in both fields.
    start_field = np.maximum(fields[0], fields[1])
    return name, planning, wall, start_field


@pytest.mark.parametrize('offset', [(0.0, 0.0), (SPACING / 2, SPACING / 2)])
def test_course_maps_give_valid_viewpoints(world, offset):
    name, planning, wall, start_field = world
    points = generate_viewpoints(planning, wall, start_field, SPACING, offset, SPIN, NEAR)
    print(f'{name} offset {offset}: {len(points)} viewpoints')

    assert len(points) >= 5
    assert len(set(points)) == len(points)
    for point in points:
        row, col = planning.world_to_cell(*point)
        assert planning.cell_to_world(row, col) == pytest.approx(point)
        assert not planning.occupied[row, col]
        assert SPIN <= wall[row, col] <= NEAR
        assert math.isfinite(start_field[row, col])
