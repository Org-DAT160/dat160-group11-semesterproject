"""Shared types and geometry helpers. No ROS imports (see docs/CODE_STRUCTURE.md)."""
from typing import Callable, Optional, Tuple

# Position in metres in the map frame.
Point = Tuple[float, float]

# Grid cell (row, col); row follows y and col follows x.
Cell = Tuple[int, int]

# Travel distance in metres from a robot to a point; None if unreachable.
CostFn = Callable[[str, Point], Optional[float]]
