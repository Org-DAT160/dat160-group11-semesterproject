"""
Viewpoints: where the robots stop and rotate to look for markers.

The leader computes the viewpoints of both passes before the start, from the
planning grid, the wall distance of the static grid and the travel distance from
the start positions (ARCHITECTURE.md section 8.1). No ROS imports here (see
docs/CODE_STRUCTURE.md): the function works on GridMap and NumPy arrays, so it
can be tested with pytest without starting ROS.
"""

import math
from typing import List, Set

import numpy as np

from rescue_team.core.distance_transform import nearest_free_cell
from rescue_team.core.geometry import Cell, Point
from rescue_team.core.grid import GridMap


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
