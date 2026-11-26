"""
Viewpoints: where the robots stop and rotate to look for markers.

The leader computes the viewpoints of both passes before the start, from the
planning grid, the wall distance of the static grid and the travel distance from
the start positions (ARCHITECTURE.md section 8.1). No ROS imports here (see
docs/CODE_STRUCTURE.md): the function works on GridMap and NumPy arrays, so it
can be tested with pytest without starting ROS.
"""
# AI-use declaration (report Appendix D): the MIS-05 functions were written by hand by
# Torgeir; the coverage functions (PR #58) were drafted by Claude (claude.ai). Notes below.

import math
from typing import List, Set, Tuple

import numpy as np

from rescue_team.core.distance_transform import nearest_free_cell
from rescue_team.core.geometry import Cell, Point
from rescue_team.core.grid import GridMap

# Wall faces (10 cm each) a coverage viewpoint must add, or it is not worth a spin.
# INT-02 play: 3 gave 30-40 % more viewpoints than 8 and no marker more.
MIN_GAIN = 8


# AI use: none - hand-written by Torgeir; Claude: explanation and review only (MIS-05).
def generate_viewpoints(grid: GridMap, wall_dist: np.ndarray, start_field: np.ndarray,
                        spacing: float, offset: Point, spin_clearance: float,
                        max_wall_distance: float) -> List[Point]:
    """
    Return the viewpoints of one pass, as cell centres in metres.

    Each lattice point moves to the nearest cell within spacing / 2 that is free in
    the planning grid and at least spin_clearance from the nearest wall. It is kept
    if it is reachable (finite in start_field) and at most max_wall_distance from
    a wall. Two points that move to the same cell give one viewpoint. wall_dist and
    start_field have the same shape as the grid.
    """
    # A cell is unusable if it is a wall in the planning grid or too narrow to rotate.
    unusable = grid.occupied | (wall_dist < spin_clearance)
    # nearest_free_cell counts in cells; 1e-9 keeps e.g. 3.9999999 from rounding to 3.
    search_radius = math.floor(spacing / 2 / grid.resolution + 1e-9)

    used_cells: Set[Cell] = set()
    viewpoints: List[Point] = []

    for x, y in _lattice(grid, spacing, offset):
        cell = nearest_free_cell(unusable, grid.world_to_cell(x, y), search_radius)
        if cell is None or cell in used_cells:
            continue

        row, col = cell
        if not math.isfinite(start_field[row, col]):
            continue
        if wall_dist[row, col] > max_wall_distance:
            continue

        used_cells.add(cell)
        viewpoints.append(grid.cell_to_world(row, col))

    return viewpoints


# AI use: drafted by Claude (claude.ai); reviewed and tested by Jonas (coverage, PR #58).
def generate_coverage_viewpoints(grid: GridMap, planning: GridMap, wall_dist: np.ndarray,
                                 start_field: np.ndarray, clearance: float, step: float,
                                 min_range: float, max_range: float,
                                 max_angle: float) -> List[Point]:
    """
    Return viewpoints that see every reachable wall face from the front (INT-02 play).

    A marker hangs flat on a wall and is only detected when the camera sees it from
    the front and from 1-3 m (SPK-02). A lattice does not know where the walls are,
    so a marker in a narrow corridor may have no viewpoint in front of it. Here every
    free cell next to a wall is a wall face with a normal pointing away from the wall.
    A candidate (a usable, reachable cell on a lattice of `step` metres) sees a face if
    it is between min_range and max_range away, at most max_angle degrees off the
    face normal, and the straight line between them crosses no wall. Greedy set cover
    then picks the candidate that sees the most faces not seen yet, until no
    candidate adds at least MIN_GAIN faces. The step should be one cell (0.1 m): in
    a corridor the cells far enough from the marker and from the other wall can be a
    single row, which a coarser lattice skips (w3 marker 1). `grid` is the static
    grid (walls, line of sight), `planning` the inflated grid (where the robot can
    stand).

    AI use: logic written by Jonas; docstring by Claude Opus 5.5, which also explained the
    specification and reviewed the change.
    """
    faces = _wall_faces(grid.occupied)
    if not faces:
        return []
    usable = (~planning.occupied) & (wall_dist >= clearance) & np.isfinite(start_field)
    cells_per_step = max(1, int(round(step / grid.resolution)))
    rows, cols = usable.shape
    candidates = [(r, c) for r in range(0, rows, cells_per_step)
                  for c in range(0, cols, cells_per_step) if usable[r, c]]

    seen_by = _seen_faces(grid, faces, candidates, min_range, max_range, max_angle)
    not_seen: Set[int] = set().union(*seen_by) if seen_by else set()
    chosen: List[int] = []
    while not_seen:
        best = max(range(len(candidates)), key=lambda i: len(seen_by[i] & not_seen))
        gain = seen_by[best] & not_seen
        if len(gain) < MIN_GAIN:
            break
        not_seen -= gain
        chosen.append(best)
    return [grid.cell_to_world(*candidates[i]) for i in chosen]


# AI use: drafted by Claude (claude.ai); reviewed and tested by Jonas (coverage, PR #58).
def _seen_faces(grid: GridMap, faces: List[Tuple[int, int, int, int]],
                candidates: List[Cell], min_range: float, max_range: float,
                max_angle: float) -> List[Set[int]]:
    """
    Return, for every candidate cell, the indices of the faces it sees (see above).

    AI use: logic written by Jonas; docstring by Claude Opus 5.5, which also explained the
    specification and reviewed the change.
    """
    face = np.array(faces, dtype=float)
    face_row, face_col, normal_row, normal_col = face.T
    cos_max = math.cos(math.radians(max_angle))
    samples = np.linspace(0.0, 1.0, int(max_range / (grid.resolution / 2)) + 1)
    seen_by: List[Set[int]] = []
    for row, col in candidates:
        d_row, d_col = row - face_row, col - face_col
        length = np.hypot(d_row, d_col)
        dist = length * grid.resolution
        with np.errstate(invalid='ignore', divide='ignore'):
            cos_angle = (d_row * normal_row + d_col * normal_col) / length
        idx = np.nonzero((dist >= min_range) & (dist <= max_range) & (cos_angle >= cos_max))[0]
        # Line of sight: sample the segment from each face cell to the candidate.
        ray_rows = np.rint(face_row[idx, None] + samples * d_row[idx, None]).astype(int)
        ray_cols = np.rint(face_col[idx, None] + samples * d_col[idx, None]).astype(int)
        blocked = grid.occupied[ray_rows, ray_cols].any(axis=1)
        seen_by.append(set(idx[~blocked].tolist()))
    return seen_by


# AI use: drafted by Claude (claude.ai); reviewed and tested by Jonas (coverage, PR #58).
def _wall_faces(occupied: np.ndarray) -> List[Tuple[int, int, int, int]]:
    """
    Return (row, col, normal_row, normal_col) for every free cell next to a wall.

    AI use: logic written by Jonas; docstring by Claude Opus 5.5, which also explained the
    specification and reviewed the change.
    """
    rows, cols = occupied.shape
    faces = []
    for d_row, d_col in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        # wall_next[r, c] is occupied[r + d_row, c + d_col] (False outside the grid).
        wall_next = np.zeros_like(occupied)
        wall_next[max(0, -d_row):rows - max(0, d_row), max(0, -d_col):cols - max(0, d_col)] = \
            occupied[max(0, d_row):rows - max(0, -d_row), max(0, d_col):cols - max(0, -d_col)]
        for row, col in zip(*np.nonzero(wall_next & ~occupied)):
            faces.append((int(row), int(col), -d_row, -d_col))
    return faces


# AI use: none - hand-written by Torgeir; Claude: explanation and review only (MIS-05).
def _lattice(grid: GridMap, spacing: float, offset: Point) -> List[Point]:
    """Return lattice points every spacing metres from the map origin plus offset, row by row."""
    rows, cols = grid.occupied.shape
    right = grid.origin[0] + cols * grid.resolution
    top = grid.origin[1] + rows * grid.resolution

    start_x = grid.origin[0] + offset[0]
    start_y = grid.origin[1] + offset[1]

    # Count steps with integers and multiply (as in core/traffic.py): adding spacing
    # again and again collects rounding errors, so a point at the edge could come
    # and go by chance.
    points: List[Point] = []
    row = 0
    while start_y + row * spacing < top:
        y = start_y + row * spacing
        col = 0
        while start_x + col * spacing < right:
            points.append((start_x + col * spacing, y))
            col = col + 1
        row = row + 1
    return points
