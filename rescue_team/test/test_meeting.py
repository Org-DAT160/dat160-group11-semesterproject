"""
Tests for core/meeting.py on hand-made grids and the w1 map.

The grids use 0.1 m cells, as the planning grid does, so the real parameters can be
used: meet_radius 1.0 m, meet_radius_second 1.2 m, meet_separation 0.8 m. The cost
fields come from `cost_field` with each robot's cell as source, as in the leader.

AI use: tests written by Claude Opus 5.5.
"""
import os

import numpy as np
import pytest

try:
    from rescue_team.core.meeting import candidates, meeting_points
except ModuleNotFoundError as error:
    # Skip only while core/meeting.py does not exist; any other import error must fail.
    if error.name != 'rescue_team.core.meeting':
        raise
    pytest.skip('core/meeting.py is not written yet (MIS-04)', allow_module_level=True)

from rescue_team.core.distance_transform import cost_field, nearest_free_cell
from rescue_team.core.geometry import distance
from rescue_team.core.grid import GridMap

RES = 0.1
RADIUS_A, RADIUS_B, SEPARATION = 1.0, 1.2, 0.8

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
W1_YAML = os.path.join(DATA_DIR, 'map_dat160_w1.yaml')


# ---------- helpers ----------

def room(width=4.0, height=5.0, wall_y=2.5):
    """Open room with a wall from wall_y to the top; returns the blocked array."""
    blocked = np.zeros((int(round(height / RES)), int(round(width / RES))), dtype=bool)
    blocked[int(round(wall_y / RES)):, :] = True
    return blocked


def fields_for(grid, starts):
    """Cost field per robot, with the robot's cell as source."""
    return {robot: cost_field(grid.occupied, RES, [grid.world_to_cell(*start)])
            for robot, start in starts.items()}


def check_pair(points, marker, fields, grid):
    """Assert the rules every result must follow; return (robot A, robot B)."""
    assert set(points) == set(fields)
    near = [r for r in points if distance(points[r], marker) <= RADIUS_A + 1e-9]
    assert near, 'point A must lie within radius_a'
    for robot, point in points.items():
        assert distance(point, marker) <= RADIUS_B + 1e-9
        assert np.isfinite(fields[robot][grid.world_to_cell(*point)]), 'must be reachable'
    a, b = points.values()
    assert distance(a, b) >= SEPARATION - 1e-9


# ---------- candidates ----------

def test_candidates_sorted_cheapest_first_and_within_radius():
    grid = GridMap(room(), RES, (0.0, 0.0))
    field = cost_field(grid.occupied, RES, [grid.world_to_cell(0.5, 0.5)])
    marker = (2.0, 2.5)
    options = candidates(grid, marker, field, RADIUS_A, [], 0.3)

    assert options
    costs = [cost for cost, _ in options]
    assert costs == sorted(costs)
    assert all(distance(point, marker) <= RADIUS_A for _, point in options)
    assert all(isinstance(cost, float) for cost in costs)


def test_candidates_skip_blocked_and_unreachable_cells():
    blocked = room()
    blocked[:, 20] = True                   # a wall splits the room in two halves
    grid = GridMap(blocked, RES, (0.0, 0.0))
    field = cost_field(blocked, RES, [grid.world_to_cell(0.5, 0.5)])   # left half
    options = candidates(grid, (2.0, 2.0), field, RADIUS_B, [], 0.3)

    assert options
    for _, (x, y) in options:
        assert x < 2.0                      # nothing from the unreachable right half
        assert not blocked[grid.world_to_cell(x, y)]


def test_candidates_skip_cells_near_excluded_points():
    grid = GridMap(room(), RES, (0.0, 0.0))
    field = cost_field(grid.occupied, RES, [grid.world_to_cell(0.5, 0.5)])
    marker = (2.0, 2.5)
    first = candidates(grid, marker, field, RADIUS_A, [], 0.3)[0][1]
    options = candidates(grid, marker, field, RADIUS_A, [first], 0.3)

    assert options
    assert all(distance(point, first) > 0.3 for _, point in options)


def test_candidates_near_the_grid_edge():
    grid = GridMap(room(), RES, (0.0, 0.0))
    field = cost_field(grid.occupied, RES, [grid.world_to_cell(0.5, 0.5)])
    options = candidates(grid, (0.05, 0.05), field, RADIUS_A, [], 0.3)   # square leaves the grid
    assert options


# ---------- meeting_points ----------

def test_point_a_goes_to_the_robot_with_the_shorter_travel_distance():
    grid = GridMap(room(), RES, (0.0, 0.0))
    marker = (2.0, 2.5)
    fields = fields_for(grid, {'tb3_0': (0.3, 0.3), 'tb3_1': (2.0, 1.2)})   # tb3_1 is closer
    points = meeting_points(grid, marker, fields, RADIUS_A, RADIUS_B, SEPARATION)

    check_pair(points, marker, fields, grid)
    assert distance(points['tb3_1'], marker) <= RADIUS_A
    best_tb3_1 = candidates(grid, marker, fields['tb3_1'], RADIUS_A, [], 0.3)[0][1]
    assert points['tb3_1'] == best_tb3_1


def test_marker_in_an_alcove_where_only_one_robot_fits():
    """Alcove 0.2 m wide in the planning grid, 0.5 m deep: point B ends up at the opening."""
    blocked = room()
    blocked[25:30, 19:21] = False           # alcove x 1.9-2.1, y 2.5-3.0
    grid = GridMap(blocked, RES, (0.0, 0.0))
    marker = (2.0, 3.05)                    # on the end wall of the alcove
    fields = fields_for(grid, {'tb3_0': (0.5, 0.5), 'tb3_1': (3.6, 0.6)})
    points = meeting_points(grid, marker, fields, RADIUS_A, RADIUS_B, SEPARATION)

    check_pair(points, marker, fields, grid)
    inside = [r for r, (_, y) in points.items() if y > 2.5]
    assert len(inside) <= 1                 # never both robots in the alcove
    assert any(y < 2.5 for _, y in points.values())   # at least one at the opening


def test_excluded_point_moves_the_meeting():
    grid = GridMap(room(), RES, (0.0, 0.0))
    marker = (2.0, 2.5)
    fields = fields_for(grid, {'tb3_0': (0.5, 0.5), 'tb3_1': (3.5, 0.5)})
    first = meeting_points(grid, marker, fields, RADIUS_A, RADIUS_B, SEPARATION)
    failed = next(iter(first.values()))
    second = meeting_points(grid, marker, fields, RADIUS_A, RADIUS_B, SEPARATION,
                            exclude=[failed])

    check_pair(second, marker, fields, grid)
    assert all(distance(point, failed) > 0.3 for point in second.values())


def test_no_pair_when_the_separation_cannot_be_met():
    grid = GridMap(room(), RES, (0.0, 0.0))
    fields = fields_for(grid, {'tb3_0': (0.5, 0.5), 'tb3_1': (3.5, 0.5)})
    assert meeting_points(grid, (2.0, 2.5), fields, RADIUS_A, RADIUS_B, 3.0) is None


def test_no_pair_when_nothing_near_the_marker_is_reachable():
    blocked = room()
    blocked[15:25, 10:30] = True            # block everything below the marker
    grid = GridMap(blocked, RES, (0.0, 0.0))
    fields = fields_for(grid, {'tb3_0': (0.5, 0.5), 'tb3_1': (3.5, 0.5)})
    assert meeting_points(grid, (2.0, 2.5), fields, RADIUS_A, RADIUS_B, SEPARATION) is None


def test_no_pair_when_every_cell_is_excluded():
    grid = GridMap(room(), RES, (0.0, 0.0))
    marker = (2.0, 2.5)
    fields = fields_for(grid, {'tb3_0': (0.5, 0.5), 'tb3_1': (3.5, 0.5)})
    everything = [grid.cell_to_world(r, c) for r in range(10, 25) for c in range(5, 35)]
    assert meeting_points(grid, marker, fields, RADIUS_A, RADIUS_B, SEPARATION,
                          exclude=everything) is None


# ---------- w1 ----------

@pytest.fixture(scope='module')
def w1_planning_grid():
    return GridMap.from_yaml(W1_YAML).downsample(10).inflate(0.30)


def test_w1_marker_4_from_the_start_positions(w1_planning_grid):
    grid = w1_planning_grid
    marker = (4.799, -0.902)                # accepted position of marker 4 on w1 (MIS-03)
    fields = {}
    for robot, start in {'tb3_0': (0.0, -1.0), 'tb3_1': (0.0, 1.0)}.items():
        cell = nearest_free_cell(grid.occupied, grid.world_to_cell(*start), 5)
        fields[robot] = cost_field(grid.occupied, grid.resolution, [cell])
    points = meeting_points(grid, marker, fields, RADIUS_A, RADIUS_B, SEPARATION)

    check_pair(points, marker, fields, grid)
    true_marker = (4.813, -0.904)
    # The 2 m rule of the competition, with goal_tolerance 0.10 m.
    assert all(distance(p, true_marker) + 0.10 < 2.0 for p in points.values())
