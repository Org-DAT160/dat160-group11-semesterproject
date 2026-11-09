"""
Tests for GridMap in core/grid.py, on hand-made grids and map files.

Grids are written as text: '#' is occupied, '.' is free. The first line is row 0,
the first character is column 0 (array indexing, no row flip involved).
"""
import array
import os
import subprocess
import sys
import time

import numpy as np
import pytest

try:
    from rescue_team.core.grid import GridMap
except ModuleNotFoundError as error:
    # Skip only while core/grid.py does not exist; any other import error must fail.
    if error.name != 'rescue_team.core.grid':
        raise
    pytest.skip('core/grid.py is not written yet (NAV-02)', allow_module_level=True)

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
W1_YAML = os.path.join(DATA_DIR, 'map_dat160_w1.yaml')


def parse(text):
    """Turn a text grid into a bool array."""
    rows = [line.strip() for line in text.strip().splitlines()]
    return np.array([[ch == '#' for ch in row] for row in rows], dtype=bool)


def make_grid(text, resolution=0.1, origin=(0.0, 0.0)):
    """Build a GridMap from a text grid through from_occupancy_grid (row-major, row 0 first)."""
    occupied = parse(text)
    height, width = occupied.shape
    data = [100 if cell else 0 for cell in occupied.flatten()]
    return GridMap.from_occupancy_grid(data, width, height, resolution, origin)


def single(rows, cols, cell):
    """Text grid with one occupied cell."""
    lines = []
    for r in range(rows):
        lines.append(''.join('#' if (r, c) == cell else '.' for c in range(cols)))
    return '\n'.join(lines)


def occ_list(grid):
    """Return the occupied array as nested lists, checking it is bool ([1, 0] == [True, False])."""
    assert grid.occupied.dtype == bool
    return grid.occupied.tolist()


def assert_grid(grid, expected_text):
    expected = parse(expected_text)
    # Must stay bool: on an int array, ~occupied gives -1/-2 and every cell looks free.
    assert grid.occupied.dtype == bool
    assert grid.occupied.shape == expected.shape
    assert int(grid.occupied.sum()) == int(expected.sum())
    assert np.array_equal(grid.occupied, expected)


# --- downsample ---------------------------------------------------------------------------

def test_downsample_one_fine_cell_occupies_the_coarse_cell():
    grid = make_grid("""
        #...
        ....
        ....
        ....
    """, resolution=0.01)
    coarse = grid.downsample(2)
    assert_grid(coarse, """
        #.
        ..
    """)
    assert coarse.resolution == pytest.approx(0.02)
    assert coarse.origin == pytest.approx((0.0, 0.0))


def test_downsample_hits_the_right_coarse_cell():
    grid = make_grid("""
        ...#
        ....
        #...
        ....
    """)
    assert_grid(grid.downsample(2), """
        .#
        #.
    """)


def test_downsample_keeps_a_thin_wall():
    # A wall one fine cell thick must survive: any occupied sub-cell, not the average.
    grid = make_grid("""
        ..#...
        ..#...
        ..#...
        ..#...
        ..#...
        ..#...
    """)
    assert_grid(grid.downsample(3), """
        #.
        #.
    """)


def test_downsample_all_free_and_all_occupied():
    free = make_grid('\n'.join(['....'] * 4))
    full = make_grid('\n'.join(['####'] * 4))
    assert not free.downsample(2).occupied.any()
    assert full.downsample(2).occupied.all()
    assert full.downsample(2).occupied.shape == (2, 2)


def test_downsample_factor_one_is_a_copy():
    text = """
        #..#
        .#..
        ....
        ...#
    """
    grid = make_grid(text, resolution=0.05, origin=(-1.0, 2.0))
    copy = grid.downsample(1)
    assert_grid(copy, text)
    assert copy.resolution == pytest.approx(0.05)
    assert copy.origin == pytest.approx((-1.0, 2.0))
    copy.occupied[2, 2] = True
    assert not grid.occupied[2, 2]


def test_downsample_coarse_cell_centre_is_the_block_centre():
    grid = make_grid('\n'.join(['....'] * 4), resolution=0.01)
    coarse = grid.downsample(2)
    assert coarse.cell_to_world(0, 0) == pytest.approx((0.01, 0.01))
    assert coarse.cell_to_world(1, 1) == pytest.approx((0.03, 0.03))


def test_downsample_with_offset_origin():
    grid = make_grid('\n'.join(['....'] * 4), resolution=0.01, origin=(-10.0, -10.0))
    coarse = grid.downsample(2)
    assert coarse.origin == pytest.approx((-10.0, -10.0))
    assert coarse.cell_to_world(0, 0) == pytest.approx((-9.99, -9.99))
    assert coarse.world_to_cell(-9.965, -9.985) == (0, 1)


def test_downsample_does_not_change_the_original():
    grid = make_grid("""
        #...
        ....
        ....
        ....
    """, resolution=0.01)
    grid.downsample(2)
    assert grid.occupied.shape == (4, 4)
    assert grid.resolution == pytest.approx(0.01)


# --- inflate ------------------------------------------------------------------------------

def test_inflate_one_cell_gives_a_plus():
    grid = make_grid(single(7, 7, (3, 3)))
    assert_grid(grid.inflate(0.1), """
        .......
        .......
        ...#...
        ..###..
        ...#...
        .......
        .......
    """)


def test_inflate_one_and_a_half_cells_includes_diagonals():
    grid = make_grid(single(7, 7, (3, 3)))
    assert_grid(grid.inflate(0.15), """
        .......
        .......
        ..###..
        ..###..
        ..###..
        .......
        .......
    """)


def test_inflate_is_circular_not_square():
    # A square inflation would give 25 cells here.
    grid = make_grid(single(7, 7, (3, 3)))
    assert_grid(grid.inflate(0.2), """
        .......
        ...#...
        ..###..
        .#####.
        ..###..
        ...#...
        .......
    """)


def test_inflate_two_and_a_half_cells():
    grid = make_grid(single(7, 7, (3, 3)))
    assert_grid(grid.inflate(0.25), """
        .......
        ..###..
        .#####.
        .#####.
        .#####.
        ..###..
        .......
    """)


def test_inflate_real_parameters_float_trap():
    # 0.3 / 0.1 == 2.9999999999999996 in Python. Cells exactly 3 cells away must be included.
    grid = make_grid(single(9, 9, (4, 4)))
    inflated = grid.inflate(0.3)
    assert int(inflated.occupied.sum()) == 29
    for cell in [(1, 4), (7, 4), (4, 1), (4, 7)]:
        assert inflated.occupied[cell]
    assert inflated.occupied[2, 2]  # distance sqrt(8) ~ 2.83 <= 3
    assert not inflated.occupied[1, 3]  # distance sqrt(10) ~ 3.16 > 3


def test_inflate_zero_radius_changes_nothing():
    text = """
        #....
        ..#..
        .....
        ....#
    """
    assert_grid(make_grid(text).inflate(0.0), text)


def test_inflate_at_the_edge_does_not_crash_or_wrap():
    grid = make_grid(single(5, 5, (0, 0)))
    assert_grid(grid.inflate(0.2), """
        ###..
        ##...
        #....
        .....
        .....
    """)


def test_inflate_corridor_open_then_closed():
    corridor = '\n'.join(['#.....#'] * 5)
    grid = make_grid(corridor)
    assert_grid(grid.inflate(0.2), '\n'.join(['###.###'] * 5))
    assert grid.inflate(0.3).occupied.all()


def test_inflate_merges_nearby_cells():
    grid = make_grid('.#...#.')
    assert_grid(grid.inflate(0.2), '#######')


def test_inflate_keeps_metadata_and_original():
    grid = make_grid(single(5, 5, (2, 2)), resolution=0.1, origin=(-1.0, 3.0))
    inflated = grid.inflate(0.1)
    assert inflated.resolution == pytest.approx(0.1)
    assert inflated.origin == pytest.approx((-1.0, 3.0))
    assert inflated.occupied.shape == (5, 5)
    assert int(grid.occupied.sum()) == 1


def test_downsample_then_inflate_like_the_nodes_do():
    # 100x100 fine cells at 0.01 m -> 10x10 coarse cells at 0.1 m. Fine cell (57, 53) lies in
    # coarse cell (5, 5), which then grows into a plus with radius 0.1 m.
    fine = np.zeros((100, 100), dtype=bool)
    fine[57, 53] = True
    data = [100 if cell else 0 for cell in fine.flatten()]
    grid = GridMap.from_occupancy_grid(data, 100, 100, 0.01, (0.0, 0.0))
    planning = grid.downsample(10).inflate(0.1)
    assert planning.resolution == pytest.approx(0.1)
    assert int(planning.occupied.sum()) == 5
    for cell in [(5, 5), (4, 5), (6, 5), (5, 4), (5, 6)]:
        assert planning.occupied[cell]


# --- from_occupancy_grid ------------------------------------------------------------------

def test_from_occupancy_grid_threshold_and_unknown():
    # One row: free, below threshold, at threshold, sure wall, unknown.
    grid = GridMap.from_occupancy_grid([0, 64, 65, 100, -1], 5, 1, 0.05, (1.0, 2.0))
    assert grid.occupied.dtype == bool
    assert occ_list(grid) == [[False, False, True, True, True]]
    assert grid.resolution == pytest.approx(0.05)
    assert grid.origin == pytest.approx((1.0, 2.0))


def test_from_occupancy_grid_unknown_can_be_free():
    grid = GridMap.from_occupancy_grid([-1, 100], 2, 1, 0.1, (0.0, 0.0),
                                       unknown_is_occupied=False)
    assert occ_list(grid) == [[False, True]]


def test_from_occupancy_grid_custom_threshold():
    grid = GridMap.from_occupancy_grid([49, 50, 51], 3, 1, 0.1, (0.0, 0.0),
                                       occupied_threshold=50)
    assert occ_list(grid) == [[False, True, True]]


def test_from_occupancy_grid_accepts_ros_int8_array():
    # nav_msgs/OccupancyGrid.data arrives as array.array('b') in rclpy.
    data = array.array('b', [0, 100, -1, 0, 0, 0])
    grid = GridMap.from_occupancy_grid(data, 3, 2, 0.1, (0.0, 0.0))
    assert occ_list(grid) == [[False, True, True], [False, False, False]]


def test_from_occupancy_grid_is_row_major_with_row_0_at_origin():
    # Width 3, height 2: data[0:3] is row 0 (lowest y), data[3:6] is row 1.
    grid = GridMap.from_occupancy_grid([0, 0, 100, 0, 0, 0], 3, 2, 0.1, (0.0, 0.0))
    assert grid.occupied.shape == (2, 3)
    assert grid.occupied[0, 2]
    assert grid.occupied.sum() == 1
    # The occupied cell is near the origin's y, not at the top.
    assert grid.is_known_wall((0.25, 0.05))
    assert not grid.is_known_wall((0.25, 0.15))


# --- coordinates --------------------------------------------------------------------------

def free_grid(rows, cols, resolution=0.1, origin=(0.0, 0.0)):
    return GridMap.from_occupancy_grid([0] * (rows * cols), cols, rows, resolution, origin)


def test_cell_to_world_is_the_cell_centre_x_from_col_y_from_row():
    # Not square, so swapping row and column shows up.
    grid = free_grid(3, 5, resolution=0.1, origin=(-1.0, 2.0))
    assert grid.cell_to_world(0, 0) == pytest.approx((-0.95, 2.05))
    assert grid.cell_to_world(2, 4) == pytest.approx((-0.55, 2.25))
    assert grid.cell_to_world(1, 3) == pytest.approx((-0.65, 2.15))


def test_world_to_cell_returns_row_then_col():
    grid = free_grid(3, 5, resolution=0.1, origin=(-1.0, 2.0))
    assert grid.world_to_cell(-0.65, 2.15) == (1, 3)
    assert grid.world_to_cell(-0.99, 2.01) == (0, 0)
    assert grid.world_to_cell(-0.51, 2.29) == (2, 4)


def test_world_to_cell_returns_ints():
    grid = free_grid(3, 5)
    row, col = grid.world_to_cell(0.35, 0.15)
    assert isinstance(row, int) and isinstance(col, int)
    assert (row, col) == (1, 3)


def test_round_trip_cell_world_cell():
    grid = free_grid(20, 30, resolution=0.1, origin=(-10.0, -10.0))
    for row in range(20):
        for col in range(30):
            assert grid.world_to_cell(*grid.cell_to_world(row, col)) == (row, col)


def test_round_trip_at_map_resolution():
    # The real /map: cells of 0.01 m, origin at (-10, -10), indices up to 1999.
    grid = free_grid(1, 1, resolution=0.01, origin=(-10.0, -10.0))
    for cell in [(0, 0), (999, 1000), (1999, 1999), (1234, 567)]:
        assert grid.world_to_cell(*grid.cell_to_world(*cell)) == cell


def test_world_to_cell_rounds_down_below_the_origin():
    # int() would truncate -0.5 to 0 and put this point inside the grid.
    grid = free_grid(3, 5)
    assert grid.world_to_cell(-0.05, 0.05) == (0, -1)
    assert grid.world_to_cell(0.05, -0.05) == (-1, 0)
    assert grid.world_to_cell(-0.25, -0.15) == (-2, -3)


def test_world_to_cell_beyond_the_far_edge():
    grid = free_grid(3, 5)
    assert grid.world_to_cell(0.55, 0.05) == (0, 5)
    assert grid.world_to_cell(0.05, 0.35) == (3, 0)


def test_in_bounds():
    grid = free_grid(3, 5)
    for cell in [(0, 0), (0, 4), (2, 0), (2, 4), (1, 2)]:
        assert grid.in_bounds(*cell)
    for cell in [(-1, 0), (0, -1), (3, 0), (0, 5), (3, 5), (-1, -1), (5, 3)]:
        assert not grid.in_bounds(*cell)


# --- is_known_wall ------------------------------------------------------------------------

def test_is_known_wall_inside_the_grid():
    grid = make_grid("""
        .....
        ..#..
        .....
    """, resolution=0.1, origin=(1.0, 1.0))
    assert grid.is_known_wall((1.25, 1.15))       # centre of cell (1, 2)
    assert grid.is_known_wall((1.21, 1.19))       # inside cell (1, 2), near a corner
    assert not grid.is_known_wall((1.25, 1.05))   # cell (0, 2)
    assert not grid.is_known_wall((1.15, 1.15))   # cell (1, 1)
    assert not grid.is_known_wall((1.05, 1.05))   # cell (0, 0)


def test_is_known_wall_outside_the_grid_on_all_sides():
    grid = free_grid(3, 5, resolution=0.1, origin=(1.0, 1.0))
    assert not grid.is_known_wall((1.25, 1.15))
    assert grid.is_known_wall((0.95, 1.15))   # left
    assert grid.is_known_wall((1.55, 1.15))   # right
    assert grid.is_known_wall((1.25, 0.95))   # below
    assert grid.is_known_wall((1.25, 1.35))   # above
    assert grid.is_known_wall((-5.0, 50.0))   # far away


def test_is_known_wall_on_inflated_static_grid():
    # As in robot_handler: the static grid grown by one cell drops lidar hits next to a wall.
    static = make_grid("""
        .......
        ...#...
        .......
    """)
    known = static.inflate(0.1)
    assert known.is_known_wall((0.25, 0.15))      # next to the wall
    assert not known.is_known_wall((0.15, 0.15))  # two cells away


# --- from_yaml ----------------------------------------------------------------------------

B, W, U = 0, 254, 205   # map_server pixels: black = wall, white = free, grey = unknown


def write_map(directory, pixels, resolution=0.1, origin=(0.0, 0.0), name='test_map'):
    """Write a map_server map: PNG row 0 is the top of the map."""
    image_module = pytest.importorskip('PIL.Image')
    image = image_module.fromarray(np.array(pixels, dtype=np.uint8))
    image.save(os.path.join(directory, name + '.png'))
    yaml_path = os.path.join(directory, name + '.yaml')
    with open(yaml_path, 'w') as f:
        f.write('image: {}.png\n'.format(name))
        f.write('resolution: {}\n'.format(resolution))
        f.write('origin: [{}, {}, 0.0]\n'.format(origin[0], origin[1]))
        f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return yaml_path


def test_from_yaml_flips_rows(tmp_path):
    pytest.importorskip('yaml')
    # The wall is in the TOP image row, so it must end up in the LAST grid row (highest y).
    pixels = [[B, W, W, W],
              [W, W, W, W],
              [W, W, W, W]]
    grid = GridMap.from_yaml(write_map(str(tmp_path), pixels))
    assert grid.occupied.shape == (3, 4)
    assert occ_list(grid) == [[False] * 4, [False] * 4, [True, False, False, False]]
    assert grid.is_known_wall((0.05, 0.25))
    assert not grid.is_known_wall((0.05, 0.05))


def test_from_yaml_reads_resolution_origin_and_unknown(tmp_path):
    pytest.importorskip('yaml')
    pixels = [[W, U],
              [B, W]]
    yaml_path = write_map(str(tmp_path), pixels, resolution=0.05, origin=(-10.0, -7.5))
    grid = GridMap.from_yaml(yaml_path)
    assert grid.resolution == pytest.approx(0.05)
    assert grid.origin == pytest.approx((-10.0, -7.5))
    # Bottom image row becomes row 0: [B, W]; top row becomes row 1: [W, U], unknown occupied.
    assert occ_list(grid) == [[True, False], [False, True]]


def test_from_yaml_image_path_is_relative_to_the_yaml_file(tmp_path, monkeypatch):
    pytest.importorskip('yaml')
    yaml_path = write_map(str(tmp_path), [[B, W]])
    other = tmp_path / 'elsewhere'
    other.mkdir()
    monkeypatch.chdir(other)
    grid = GridMap.from_yaml(yaml_path)
    assert occ_list(grid) == [[True, False]]


def test_importing_grid_does_not_import_pillow():
    code = 'import sys; import rescue_team.core.grid; print("PIL" in sys.modules)'
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(p for p in sys.path if p))
    result = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True,
                            env=env, check=True)
    assert result.stdout.strip() == 'False'


# --- full-size grids ----------------------------------------------------------------------

def test_full_size_downsample_and_inflate_is_fast():
    # Same size as /map: 2000 x 2000 at 0.01 m, with an outer wall and some inner walls.
    fine = np.zeros((2000, 2000), dtype=np.int8)
    fine[0, :] = fine[-1, :] = fine[:, 0] = fine[:, -1] = 100
    fine[500:1500, 1000] = 100
    fine[1200, 200:900] = 100
    fine[300:310, 300:310] = -1
    grid = GridMap.from_occupancy_grid(fine.flatten(), 2000, 2000, 0.01, (-10.0, -10.0))
    start = time.perf_counter()
    static = grid.downsample(10)
    planning = static.inflate(0.3)
    elapsed = time.perf_counter() - start
    assert static.occupied.shape == (200, 200)
    assert static.resolution == pytest.approx(0.1)
    assert static.origin == pytest.approx((-10.0, -10.0))
    assert static.occupied[50, 100] and static.occupied[120, 50] and static.occupied[30, 30]
    assert planning.occupied.shape == (200, 200)
    # The wall in column 100 grows by 3 cells, not 4.
    assert planning.occupied[100, 103] and not planning.occupied[100, 104]
    # Generous limit: the course VM is 2-4x slower than a laptop.
    assert elapsed < 3.0


@pytest.mark.skipif(not os.path.exists(W1_YAML), reason='course map w1 not copied to test/data')
def test_w1_downsample_and_inflate():
    pytest.importorskip('yaml')
    grid = GridMap.from_yaml(W1_YAML)
    assert grid.occupied.shape == (2000, 2000)
    start = time.perf_counter()
    planning = grid.downsample(10).inflate(0.3)
    elapsed = time.perf_counter() - start
    assert planning.occupied.shape == (200, 200)
    assert planning.resolution == pytest.approx(0.1)
    assert 0 < planning.occupied.sum() < planning.occupied.size
    # Generous limit: the course VM is 2-4x slower than a laptop.
    assert elapsed < 3.0
