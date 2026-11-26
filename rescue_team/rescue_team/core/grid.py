"""
Planning grid: an occupancy grid with metric coordinates.

GridMap is built once from /map (or from a map file in tests), downsampled to
the static grid and inflated so the robot can be planned as a point
(ARCHITECTURE.md section 6.1). No ROS imports: it works on plain numbers and
NumPy arrays, so it can be tested with pytest without starting ROS.
"""
# AI-use declaration (report Appendix D): the code in this module was written by hand by
# Torgeir (NAV-02); Claude (claude.ai) was used for explanation and review, and wrote test/test_grid.py.
import math
import os

import numpy as np


class GridMap:
    """Occupancy grid with metric placement."""

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (NAV-02).
    def __init__(self, occupied, resolution, origin):
        self.occupied = occupied
        self.resolution = resolution
        self.origin = origin

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (NAV-02).
    @classmethod
    def from_occupancy_grid(cls, data, width, height,
                            resolution, origin,
                            occupied_threshold=65,
                            unknown_is_occupied=True):
        """Build from the fields of a nav_msgs/OccupancyGrid."""
        # One flat list, row by row with row 0 first, becomes height rows of width cells.
        values = np.array(data, dtype=np.int16).reshape(height, width)
        occupied = values >= occupied_threshold

        if unknown_is_occupied:
            occupied |= values == -1

        return cls(occupied, float(resolution), (float(origin[0]), float(origin[1])))

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (NAV-02).
    @classmethod
    def from_yaml(cls, yaml_path):
        """
        Load a map file (.yaml + image) used in tests.

        Uses the same thresholds as map_server, so the result matches /map.
        """
        # Imported here as the robot nodes do not need Pillow or PyYAML-
        from PIL import Image
        import yaml

        with open(yaml_path) as f:
            meta = yaml.safe_load(f)

        # The image path in the yaml file is relative to the yaml file, not the cwd.
        map_folder = os.path.dirname(yaml_path)
        image_path = os.path.join(map_folder, meta['image'])
        pixels = np.asarray(Image.open(image_path).convert('L'), dtype=float)

        # Darker means more likely a wall: black = 1.0, white = 0.0.
        p_wall = (255.0 - pixels) / 255.0

        # Unknown unless a threshold says otherwise.
        values = np.full(pixels.shape, -1, dtype=np.int16)
        values[p_wall > meta['occupied_thresh']] = 100
        values[p_wall < meta['free_thresh']] = 0

        # Image row 0 is the top of the map, grid row 0 is the bottom.
        values = values[::-1]
        height, width = values.shape

        # Origin in the yaml file is [x, y, yaw], the grid only needs x and y.
        return cls.from_occupancy_grid(values.flatten(), width, height,
                                       meta['resolution'], meta['origin'][:2])

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (NAV-02).
    def world_to_cell(self, x, y):
        """Convert a point in metres to the (row, col) of the cell that contains it."""
        origin_x = self.origin[0]
        origin_y = self.origin[1]
        resolution = self.resolution

        row = math.floor((y - origin_y) / resolution)
        col = math.floor((x - origin_x) / resolution)

        return row, col

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (NAV-02).
    def cell_to_world(self, row, col):
        """Convert a (row, col) cell to the point in metres at the centre of the cell."""
        origin_x = self.origin[0]
        origin_y = self.origin[1]
        resolution = self.resolution

        x = origin_x + (col + 0.5) * resolution
        y = origin_y + (row + 0.5) * resolution

        return x, y

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (NAV-02).
    def in_bounds(self, row, col):
        """Return True if the cell (row, col) lies inside the grid."""
        rows, cols = self.occupied.shape
        row_ok = 0 <= row < rows
        col_ok = 0 <= col < cols

        return row_ok and col_ok

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (NAV-02).
    def is_known_wall(self, point):
        """Return True if the point is in an occupied cell or outside the grid."""
        x, y = point
        row, col = self.world_to_cell(x, y)

        if not self.in_bounds(row, col):
            return True

        return bool(self.occupied[row, col])

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (NAV-02).
    def downsample(self, factor):
        """Return a coarser grid. A coarse cell is occupied if any of its fine cells is."""
        rows, cols = self.occupied.shape
        coarse_rows = rows // factor
        coarse_cols = cols // factor
        coarse = np.zeros((coarse_rows, coarse_cols), dtype=bool)

        for r in range(coarse_rows):
            for c in range(coarse_cols):
                block = self.occupied[r * factor:(r + 1) * factor,
                                      c * factor:(c + 1) * factor]
                coarse[r, c] = block.any()

        return GridMap(coarse, self.resolution * factor, self.origin)

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (NAV-02).
    def inflate(self, radius):
        """Return a grid where every occupied cell is grown by radius metres (circular)."""
        r = radius / self.resolution + 1e-9
        reach = math.floor(r)

        rows, cols = self.occupied.shape
        result = self.occupied.copy()

        for dr in range(-reach, reach + 1):
            for dc in range(-reach, reach + 1):
                if dr * dr + dc * dc > r * r:
                    continue
                if abs(dr) >= rows or abs(dc) >= cols:
                    continue

                from_r0 = max(0, -dr)
                from_r1 = rows - max(0, dr)
                to_r0 = max(0, dr)
                to_r1 = rows - max(0, -dr)

                from_c0 = max(0, -dc)
                from_c1 = cols - max(0, dc)
                to_c0 = max(0, dc)
                to_c1 = cols - max(0, -dc)

                src = self.occupied[from_r0:from_r1, from_c0:from_c1]
                result[to_r0:to_r1, to_c0:to_c1] |= src

        return GridMap(result, self.resolution, self.origin)
