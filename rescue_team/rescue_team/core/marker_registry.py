"""
Marker register: sightings per marker ID and when and what to report to scoring.

The leader keeps every sighting from the marker detector here and asks which
reports are due (ARCHITECTURE.md section 7.2). No ROS imports here (see
docs/CODE_STRUCTURE.md): time is passed in as seconds, so the reporting
lifecycle can be tested with pytest without starting ROS.
"""
# AI-use declaration (report Appendix D): the code in this module was written by hand by
# Torgeir (PER-01); Claude (claude.ai) was used for explanation and review, and wrote test/test_marker_registry.py.

from dataclasses import dataclass, field
from enum import Enum
from statistics import median
from typing import Dict, List, Optional, Sequence, Tuple

from rescue_team.core.geometry import Point


@dataclass
class Sighting:
    """One detection of a marker, already in the map frame."""

    marker_id: int
    x: float
    y: float
    distance: float  # from the robot to the marker, in metres
    robot: str
    stamp_ns: int


@dataclass
class Report:
    """A position sent to scoring; a resend keeps the attempt number."""

    marker_id: int
    attempt: int
    position: Point


class _Step(Enum):
    """What the next new report of a marker is based on."""

    MEDIAN_OF_ALL = 1     # first report, and the first after waiting
    CLOSER_HALF = 2       # after one rejection
    WAIT_FOR_NEW = 3      # after a second rejection


@dataclass
class _MarkerState:
    sightings: List[Sighting] = field(default_factory=list)
    reported: Dict[int, Point] = field(default_factory=dict)  # attempt -> position
    attempt: int = 0                  # newest attempt sent, 0 before the first
    awaiting_answer: bool = False     # the newest attempt has no answer yet
    sent_at: float = 0.0              # time of the newest send or resend
    next_step: _Step = _Step.MEDIAN_OF_ALL
    last_step: _Step = _Step.MEDIAN_OF_ALL  # what the newest attempt was based on
    sightings_before_wait: int = 0
    accepted_position: Optional[Point] = None


class MarkerRegistry:
    """Collects sightings and decides when and what to report to scoring."""

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (PER-01).
    def __init__(self, min_sightings: int, resend_timeout: float = 5.0,
                 marker_ids: Sequence[int] = (0, 1, 2, 3, 4)) -> None:
        self.min_sightings = min_sightings
        self.resend_timeout = resend_timeout
        self._markers = {marker_id: _MarkerState() for marker_id in marker_ids}

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (PER-01).
    def add(self, sighting: Sighting) -> bool:
        """Store the sighting if its ID is known and not accepted; return whether it was kept."""
        state = self._markers.get(sighting.marker_id)
        if state is None or state.accepted_position is not None:
            return False
        state.sightings.append(sighting)
        return True

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (PER-01).
    def estimate(self, marker_id: int) -> Optional[Point]:
        """Return the median x and y of all sightings of the marker, or None without any."""
        state = self._markers.get(marker_id)
        if state is None or not state.sightings:
            return None
        return _median_position(state.sightings)

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (PER-01).
    def reports_due(self, now: float) -> List[Report]:
        """
        Return the new reports and the resends that are due at time now (seconds).

        Every returned report is marked as sent at now.
        """
        reports = []
        for marker_id, state in self._markers.items():
            if state.accepted_position is not None:
                continue
            if state.awaiting_answer:
                report = self._resend_if_due(marker_id, state, now)
            else:
                report = self._new_report_if_ready(marker_id, state, now)
            if report is not None:
                reports.append(report)
        return reports

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (PER-01).
    def on_result(self, marker_id: int, attempt: int, accepted: bool) -> bool:
        """
        Store the answer from scoring; return True if this answer accepted the marker.

        Any true accepts the marker with the position of that attempt, also for an
        older attempt: scoring answers a repeated report of an accepted marker with
        false, so a true can arrive after a false. A false only counts for the newest
        attempt and only once.
        """
        state = self._markers.get(marker_id)
        if state is None or state.accepted_position is not None:
            return False
        if attempt not in state.reported:
            return False

        if accepted:
            state.accepted_position = state.reported[attempt]
            return True

        if attempt == state.attempt and state.awaiting_answer:
            state.awaiting_answer = False
            if state.last_step == _Step.MEDIAN_OF_ALL:
                state.next_step = _Step.CLOSER_HALF
            else:
                state.next_step = _Step.WAIT_FOR_NEW
                state.sightings_before_wait = len(state.sightings)
        return False

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (PER-01).
    def is_accepted(self, marker_id: int) -> bool:
        """Return True if scoring has accepted the marker."""
        return self.accepted_position(marker_id) is not None

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (PER-01).
    def accepted_position(self, marker_id: int) -> Optional[Point]:
        """Return the reported position of an accepted marker, or None."""
        state = self._markers.get(marker_id)
        return state.accepted_position if state is not None else None

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (PER-01).
    def all_accepted(self) -> bool:
        """Return True if every marker ID is accepted."""
        return all(state.accepted_position is not None for state in self._markers.values())

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (PER-01).
    def _resend_if_due(self, marker_id: int, state: _MarkerState,
                       now: float) -> Optional[Report]:
        if now - state.sent_at < self.resend_timeout:
            return None
        state.sent_at = now
        return Report(marker_id, state.attempt, state.reported[state.attempt])

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (PER-01).
    def _new_report_if_ready(self, marker_id: int, state: _MarkerState,
                             now: float) -> Optional[Report]:
        planned = self._next_position(state)
        if planned is None:
            return None
        position, step = planned

        state.attempt = state.attempt + 1
        state.reported[state.attempt] = position
        state.last_step = step
        state.awaiting_answer = True
        state.sent_at = now
        return Report(marker_id, state.attempt, position)

    # AI use: none - hand-written by Torgeir; Claude: explanation and review only (PER-01).
    def _next_position(self, state: _MarkerState) -> Optional[Tuple[Point, _Step]]:
        """Return the next new position and the step it is based on, or None if not ready."""
        count = len(state.sightings)

        if state.next_step == _Step.MEDIAN_OF_ALL:
            if count < self.min_sightings:
                return None
            return _median_position(state.sightings), _Step.MEDIAN_OF_ALL

        if state.next_step == _Step.CLOSER_HALF:
            return _median_position(_closer_half(state.sightings)), _Step.CLOSER_HALF

        # WAIT_FOR_NEW: once there are enough new sightings, start over on all of them.
        if count - state.sightings_before_wait < self.min_sightings:
            return None
        return _median_position(state.sightings), _Step.MEDIAN_OF_ALL


# AI use: none - hand-written by Torgeir; Claude: explanation and review only (PER-01).
def _median_position(sightings: Sequence[Sighting]) -> Point:
    # The median of x and y separately ignores a single outlier, unlike the mean.
    return (median(s.x for s in sightings), median(s.y for s in sightings))


# AI use: none - hand-written by Torgeir; Claude: explanation and review only (PER-01).
def _closer_half(sightings: Sequence[Sighting]) -> List[Sighting]:
    # Close sightings are more accurate. At least two, so one bad sighting cannot
    # decide the position alone.
    count = min(len(sightings), max(2, len(sightings) // 2))
    return sorted(sightings, key=lambda s: s.distance)[:count]
