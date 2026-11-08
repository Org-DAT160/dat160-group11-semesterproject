"""
Greedy allocation of viewpoints to robots.

When a robot has no goal, the leader asks the allocator for its next viewpoint
(ARCHITECTURE.md section 8.2). No ROS imports here (see docs/CODE_STRUCTURE.md):
the travel distance comes in as a cost function, so the allocator can be tested
with pytest and a fake cost function.
"""

from typing import Dict, List, Optional, Sequence, Set

from rescue_team.core.geometry import CostFn, Point


class GreedyAllocator:
    """Gives each idle robot the cheapest free viewpoint and reserves it."""

    def __init__(self) -> None:
        self.set_targets([])

    def set_targets(self, targets: Sequence[Point]) -> None:
        """Load the viewpoints of a pass; nothing is visited, failed or reserved."""
        self._targets: List[Point] = list(targets)
        self._visited: Set[Point] = set()
        self._dropped: Set[Point] = set()
        # Viewpoint -> the robot that failed it once; only the other robot may try it.
        self._failed_by: Dict[Point, str] = {}
        # Robot -> its reserved viewpoint. Keyed by robot, since each robot holds
        # at most one reservation and complete and release only know the robot.
        self._reserved: Dict[str, Point] = {}

    def free_count(self) -> int:
        """Return the number of viewpoints that are neither visited, dropped nor reserved."""
        count = 0
        for target in self._targets:
            if self._is_free(target):
                count = count + 1
        return count

    def next_target(self, robot: str, cost: CostFn) -> Optional[Point]:
        """
        Reserve and return the cheapest free viewpoint for robot, or None.

        Viewpoints with cost None and viewpoints this robot has failed are skipped.
        """
        # A robot holds at most one reservation; its old one may be the cheapest again.
        self.release(robot)

        cheapest_target: Optional[Point] = None
        cheapest_cost: Optional[float] = None
        for target in self._targets:
            if not self._is_free(target):
                continue
            if self._failed_by.get(target) == robot:
                continue
            # 'is None', not 'not': a cost of 0.0 is reachable.
            target_cost = cost(robot, target)
            if target_cost is None:
                continue
            # Strictly cheaper, so the first viewpoint in the list wins a tie.
            if cheapest_cost is None or target_cost < cheapest_cost:
                cheapest_target = target
                cheapest_cost = target_cost

        if cheapest_target is None:
            return None

        self._reserved[robot] = cheapest_target
        return cheapest_target

    def complete(self, robot: str, success: bool) -> None:
        """
        End the robot's reservation.

        Success marks the viewpoint visited. The first failure frees it for the
        other robot only, the second failure drops it. Does nothing without a
        reservation.
        """
        target = self._reserved.pop(robot, None)
        if target is None:
            return

        if success:
            self._visited.add(target)
        elif target in self._failed_by:
            self._dropped.add(target)
        else:
            self._failed_by[target] = robot

    def release(self, robot: str) -> None:
        """Free the robot's reservation without marking the viewpoint (meeting start)."""
        # The default None makes this do nothing when the robot has no reservation.
        self._reserved.pop(robot, None)

    def _is_free(self, target: Point) -> bool:
        if target in self._visited:
            return False
        if target in self._dropped:
            return False
        if target in self._reserved.values():
            return False
        return True
