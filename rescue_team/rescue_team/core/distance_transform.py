"""
Distance transform: travel distance on the planning grid.

Follows Corke, Robotics, Vision and Control, chapter 5 (ARCHITECTURE.md
section 6.2). One relaxation serves several purposes: the path to a goal, the
travel cost to every viewpoint and the distance to the nearest wall.

No ROS imports here (see docs/CODE_STRUCTURE.md): the functions work on NumPy
arrays, so they can be tested with pytest without starting ROS.
"""
import math
from typing import Iterable, List, Optional

import numpy as np

from rescue_team.core.geometry import Cell, Point
from rescue_team.core.grid import GridMap

# The 8-neighbourhood as (row, col) offsets. A step is diagonal when both are non-zero.
NEIGHBOURS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def _shift(a: np.ndarray, dr: int, dc: int, fill) -> np.ndarray:
    """Copy of a where cell (r, c) holds a[r + dr, c + dc]; fill outside the grid."""
    rows, cols = a.shape

    # Start with fill everywhere: border cells without a neighbour in this direction keep it.
    out = np.full(a.shape, fill, dtype=a.dtype)

    # Cells in out that have a neighbour inside the grid, and where those neighbours are in a.
    # The two ranges have the same length, so the block from a fits exactly.
    to_r0, to_r1 = max(0, -dr), rows - max(0, dr)
    from_r0, from_r1 = max(0, dr), rows - max(0, -dr)

    to_c0, to_c1 = max(0, -dc), cols - max(0, dc)
    from_c0, from_c1 = max(0, dc), cols - max(0, -dc)

    # One block copy instead of a Python loop over every cell.
    out[to_r0:to_r1, to_c0:to_c1] = a[from_r0:from_r1, from_c0:from_c1]

    return out


def _relax(blocked: np.ndarray, resolution: float, field: np.ndarray) -> np.ndarray:
    """
    Spread the field from its zero cells until no value changes.

    Every round, each cell takes the smallest of its own value and
    neighbour + step over the 8 neighbours, for the whole grid at once.
    """
    h = resolution
    diagonal = math.sqrt(2) * h
    free = ~blocked

    while True:
        # Read from field and write to new, so a round only uses last round's values.
        new = field.copy()

        for dr, dc in NEIGHBOURS:
            # inf outside the grid: there is no shorter way in from beyond the border.
            neighbour = _shift(field, dr, dc, np.inf)

            if dr != 0 and dc != 0:
                # A diagonal step only if both orthogonal cells beside it are free,
                # so a path never cuts the corner of an obstacle.
                side_a = _shift(free, dr, 0, False)
                side_b = _shift(free, 0, dc, False)
                candidate = np.where(side_a & side_b, neighbour + diagonal, np.inf)
            else:
                candidate = neighbour + h

            new = np.minimum(new, candidate)

        # Blocked cells never get a distance, however close a neighbour is.
        new[blocked] = np.inf

        # Done when a whole round changed nothing. Values only decrease, so this ends.
        if np.array_equal(new, field):
            return new
        field = new


def cost_field(blocked: np.ndarray, resolution: float,
               sources: Iterable[Cell]) -> np.ndarray:
    """
    Travel distance in metres from the nearest source to each cell.

    Blocked cells and cells that no source can reach are inf. A source on a
    blocked cell is ignored.
    """
    # inf means "no path found yet"; the relaxation can only lower it.
    field = np.full(blocked.shape, np.inf)

    for (r, c) in sources:
        if not blocked[r, c]:
            field[r, c] = 0

    return _relax(blocked, resolution, field)


def wall_distance(occupied: np.ndarray, resolution: float) -> np.ndarray:
    """Distance in metres from each cell to the nearest occupied cell."""
    # Every occupied cell is a source. Nothing is blocked: if the walls were blocked,
    # _relax would set them back to inf, but a wall is 0 m from the nearest wall.
    field = np.where(occupied, 0.0, np.inf)

    not_blocked = np.zeros(occupied.shape, dtype=bool)

    return _relax(not_blocked, resolution, field)


def descend(field: np.ndarray, start: Cell) -> Optional[List[Cell]]:
    """
    Steepest descent from start to a source.

    Steps to the lowest neighbour until it reaches a cell with cost 0, with the
    same corner rule as the field. None if start is blocked or unreachable.
    """
    rows, cols = field.shape

    r, c = start

    if not math.isfinite(field[r, c]):
        return None

    path = [(r, c)]

    # Plain Python loops: we follow one path, not the whole grid.
    while field[r, c] > 0:
        best = None

        for dr, dc in NEIGHBOURS:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue

            # Corner rule: skip a diagonal step if either orthogonal cell beside it is
            # blocked or unreachable. Both lie inside the grid when (nr, nc) does.
            if (dr != 0
                    and dc != 0
                    and (not math.isfinite(field[r + dr, c])
                         or not math.isfinite(field[r, c + dc]))):
                continue

            if best is None or field[nr, nc] < field[best]:
                best = (nr, nc)

        # Safety net for a field not made by cost_field: without a lower neighbour
        # the loop would never end.
        if best is None or field[best] >= field[r, c]:
            return None

        r, c = best

        path.append((r, c))

    return path


def nearest_free_cell(blocked: np.ndarray, cell: Cell,
                      max_radius: int) -> Optional[Cell]:
    """
    Nearest free cell within max_radius cells (Euclidean), or None.

    Moves a start or goal out of an inflated obstacle. A free cell returns
    itself, and cell may lie outside the grid.
    """
    rows, cols = blocked.shape
    r0, c0 = cell

    best = None
    best_d2 = None

    # Search the square around cell and keep only offsets inside the circle.
    for dr in range(-max_radius, max_radius + 1):
        for dc in range(-max_radius, max_radius + 1):
            # Squared distances. Same order as real distances, no square root needed.
            d2 = dr ** 2 + dc ** 2

            if d2 > max_radius ** 2:
                continue

            r, c = r0 + dr, c0 + dc

            if not (0 <= r < rows and 0 <= c < cols):
                continue

            if blocked[r, c]:
                continue

            # Check every candidate. The first free cell found is not always the nearest.
            if best is None or d2 < best_d2:
                best, best_d2 = (r, c), d2

    return best


def cost_at(field: np.ndarray, grid: GridMap, point: Point) -> Optional[float]:
    """Field value at a point in metres; None if outside the grid or unreachable."""
    x, y = point
    r, c = grid.world_to_cell(x, y)

    # Check before indexing: a negative index would silently read from the other side.
    if not grid.in_bounds(r, c):
        return None

    value = field[r, c]

    if not math.isfinite(value):
        return None

    # A plain float instead of np.float64.
    return float(value)
