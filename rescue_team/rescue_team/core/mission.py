"""
Mission state machine of the leader (ARCHITECTURE.md section 5).

`Mission` decides where each robot drives next; it never drives anything itself.
The leader node turns ROS callbacks into events, calls `Mission.handle(event)` and
executes the commands it gets back as ROS calls:

    ROS callback -> Event -> Mission.handle(event) -> [Command, ...] -> ROS call

No ROS imports and no clock here (docs/CODE_STRUCTURE.md): every event carries the
time `now`, and the allocator, the cost function and the meeting function are
injected, so the whole mission can be tested with pytest and fake dependencies
(test/test_mission.py).

States (section 5.2):

    WAIT_FOR_SYSTEM  before the start; markers accepted now are only recorded
    EXPLORE          the robots visit viewpoints, pass after pass (0, 1, 0, ...)
    MEET             both robots drive to the big fire (marker 4) and wait there
    DONE             all five markers accepted and the meeting held; ignores everything

A typical run:

    Started                    -> SendGoal to a viewpoint for each robot     EXPLORE
    GoalFinished (robot)       -> SendGoal to that robot's next viewpoint    EXPLORE
    MarkerAccepted(4)          -> SendGoal to a meeting point for each robot MEET
    GoalFinished x 2, Tick ... -> hold for meet_hold_time, then viewpoints   EXPLORE
    MarkerAccepted(all five)   -> CancelGoal for each robot, Finish()        DONE

Every goal has an id (`goal_id`, always increasing) and a deadline. A result with an
old id belongs to a goal that was replaced and is ignored; a goal past its deadline
is cancelled on the next Tick and counts as a failure.

AI use: logic written by Jonas; docstrings by Claude Opus 5.5, which also explained the
specification and reviewed the changes.
"""
from dataclasses import dataclass
import enum
from typing import Callable, Dict, List, Optional, Sequence, Set, Union

from rescue_team.core.geometry import CostFn, Point


# ---------- states ----------

class MissionState(enum.Enum):
    """The four states of the mission (section 5)."""

    WAIT_FOR_SYSTEM = 'wait_for_system'
    EXPLORE = 'explore'
    MEET = 'meet'
    DONE = 'done'


# ---------- events (all carry now: float, the time in seconds) ----------

@dataclass
class Started:
    """The system is ready: robot start positions (right-of-way order) and the viewpoint passes."""

    now: float
    robots: Dict[str, Point]
    passes: List[List[Point]]


@dataclass
class GoalFinished:
    """A NavigateTo result arrived, or the goal was rejected."""

    now: float
    robot: str
    goal_id: int
    success: bool


@dataclass
class MarkerAccepted:
    """Scoring accepted a marker report; all_accepted is True once all five are accepted."""

    now: float
    marker_id: int
    all_accepted: bool


@dataclass
class Tick:
    """The mission timer fired (tick_rate)."""

    now: float


# ---------- commands ----------

@dataclass
class SendGoal:
    """Send a NavigateTo goal; it replaces the robot's current goal."""

    robot: str
    goal_id: int
    target: Point
    spin: bool


@dataclass
class CancelGoal:
    """Cancel the robot's active goal."""

    robot: str


@dataclass
class Finish:
    """The mission is over: log the result, send nothing more."""


# Any of the three commands; handle() returns a list of them.
Command = Union[SendGoal, CancelGoal, Finish]


# ---------- parameters ----------

@dataclass
class MissionParams:
    """Parameters of the mission, given by the leader (section 5.1)."""

    meet_hold_time: float      # s both robots stay at the meeting points
    meet_max_attempts: int     # failed meeting attempts before exploring again
    goal_timeout_base: float   # s added to every goal deadline
    max_linear_speed: float    # m/s, for the travel time in the deadline
    spin_time: float           # s, spin timeout, added to the deadline of a goal with spin


# The big fire: the marker both robots have to meet at.
BIG_FIRE_ID = 4


# ---------- the state machine ----------

class Mission:
    """
    The mission state machine: `handle(event)` is the only entry point.

    Dependencies (injected):
        allocator  GreedyAllocator (section 8.2): hands out viewpoints of the loaded pass
        cost       CostFn: travel distance robot -> point in metres, None if unreachable
        meeting    meeting(exclude) -> {robot: point} near the big fire, or None
        params     MissionParams

    State kept between events (section 5.1):
        goal_id / goal_target / goal_deadline   the active goal per robot; a robot
                                                without an entry has no goal (is free)
        pass_index                              the pass loaded in the allocator
        meeting_pending / meeting_done          ID 4 accepted / meeting held
        all_accepted                            scoring has accepted all five markers
        meet_*, arrived, hold_start             the current meeting
    """

    def __init__(self, allocator, cost: CostFn,
                 meeting: Callable[[Sequence[Point]], Optional[Dict[str, Point]]],
                 params: MissionParams):
        # Injected dependencies (section 5.1).
        self.allocator = allocator      # GreedyAllocator (section 8.2), or a fake in tests
        self.cost = cost                # travel distance robot -> point, None if unreachable
        self.meeting = meeting          # meeting points for the excluded points, or None
        self.params = params

        self._state = MissionState.WAIT_FOR_SYSTEM

        # Filled by Started.
        self.robots: List[str] = []             # robot names in right-of-way order
        self.passes: List[List[Point]] = []     # viewpoints of pass 0 and pass 1
        self.pass_index = 0                     # the pass loaded in the allocator

        # Active goal per robot; a robot without an entry has no active goal.
        self.next_goal_id = 1
        self.goal_id: Dict[str, int] = {}
        self.goal_target: Dict[str, Point] = {}
        self.goal_deadline: Dict[str, float] = {}

        # Mission progress.
        self.meeting_pending = False            # ID 4 accepted, meeting not held yet
        self._meeting_done = False
        self.all_accepted = False               # scoring has accepted all five markers

        # The meeting (sections 5.2 and 5.3).
        self.meet_attempts = 0
        self.meet_exclude: List[Point] = []     # meeting points that failed
        self.meet_targets: Dict[str, Point] = {}
        self.arrived: Set[str] = set()          # robots that reached their meeting point
        self.hold_start: Optional[float] = None  # time both robots had arrived

    @property
    def state(self) -> MissionState:
        """Return the current state, for logging and tests."""
        return self._state

    @property
    def meeting_done(self) -> bool:
        """Return True once the big-fire meeting has been held."""
        return self._meeting_done

    # ---------- entry point ----------

    def handle(self, event) -> List[Command]:
        """
        Handle one event and return the commands for the leader (section 5.2).

        DONE ignores every event. A GoalFinished with an old goal_id is ignored in
        every state: it answers a goal that was replaced by a newer one (or the
        robot has no goal any more), so it says nothing about the current goal.
        Otherwise dispatch on the state to _handle_wait, _handle_explore or
        _handle_meet.
        """
        if self._state == MissionState.DONE:
            return []
        if isinstance(event, GoalFinished) and event.goal_id != self.goal_id.get(event.robot):
            return []
        if self._state == MissionState.WAIT_FOR_SYSTEM:
            return self._handle_wait(event)
        if self._state == MissionState.EXPLORE:
            return self._handle_explore(event)
        if self._state == MissionState.MEET:
            return self._handle_meet(event)
        return []

    # ---------- one method per state ----------

    def _handle_wait(self, event) -> List[Command]:
        """
        WAIT_FOR_SYSTEM: before the start (section 5.2).

        MarkerAccepted: only record it; ID 4 -> meeting_pending = True. The robots
        do not move before Started.
        Started: store robots and passes, load pass 0 into the allocator, go to
        EXPLORE. If ID 4 was already accepted, try the meeting at once; if still in
        EXPLORE, give both robots a viewpoint.
        """
        if isinstance(event, MarkerAccepted):
            self.all_accepted = event.all_accepted
            if event.marker_id == BIG_FIRE_ID:
                self.meeting_pending = True
            return []
        if isinstance(event, Started):
            self.robots = list(event.robots)        # the names, in right-of-way order
            self.passes = event.passes
            self.allocator.set_targets(self.passes[0])
            self._state = MissionState.EXPLORE
            commands = []
            if self.meeting_pending:
                commands += self._try_start_meeting(event.now)
            if self._state == MissionState.EXPLORE:     # the meeting did not start
                commands += self._assign(event.now)
            return commands
        return []

    def _handle_explore(self, event) -> List[Command]:
        """
        EXPLORE: the robots visit viewpoints (section 5.2).

        GoalFinished: report the result to the allocator (visited, or failed for this
        robot), free the robot, give it its next viewpoint.
        MarkerAccepted(4): meeting_pending = True; try to start the meeting.
        MarkerAccepted (other ID): record it; all five accepted and meeting done -> finish.
        Tick: goals past their deadline are cancelled and count as failures; all five
        accepted while the meeting is pending -> try the meeting again; if still in
        EXPLORE, give free robots a viewpoint.
        """
        if isinstance(event, GoalFinished):
            self.allocator.complete(event.robot, event.success)
            self._clear_goal(event.robot)
            return self._assign(event.now)

        if isinstance(event, MarkerAccepted):
            self.all_accepted = event.all_accepted
            if event.marker_id == BIG_FIRE_ID:
                self.meeting_pending = True
                return self._try_start_meeting(event.now)
            if self.all_accepted and self._meeting_done:
                return self._finish()
            return []

        if isinstance(event, Tick):
            commands = []
            for robot in self._expired(event.now):
                commands.append(CancelGoal(robot))
                self.allocator.complete(robot, False)
                self._clear_goal(robot)
            if self.all_accepted and self.meeting_pending:
                commands += self._try_start_meeting(event.now)
            if self._state == MissionState.EXPLORE:     # the meeting did not start
                commands += self._assign(event.now)
            return commands
        return []

    def _handle_meet(self, event) -> List[Command]:
        """
        MEET: both robots drive to the big fire and wait there (section 5.2).

        GoalFinished, success: the robot has arrived; both arrived -> the hold starts.
        The arrived robot has no goal any more, so its deadline cannot run out while
        it waits.
        GoalFinished, failure: _meeting_failed (new points, or give up for now).
        MarkerAccepted (other ID): record it.
        Tick: a passed deadline counts as a meeting failure. Hold time over -> the
        meeting is done; all five accepted -> finish, otherwise back to EXPLORE.
        """
        if isinstance(event, GoalFinished):
            self._clear_goal(event.robot)
            if event.success:
                self.arrived.add(event.robot)
                if len(self.arrived) == len(self.robots):
                    self.hold_start = event.now
                return []
            return self._meeting_failed(event.robot, event.now)

        if isinstance(event, MarkerAccepted):
            self.all_accepted = event.all_accepted
            return []

        if isinstance(event, Tick):
            commands = []
            for robot in self._expired(event.now):
                commands.append(CancelGoal(robot))
                self._clear_goal(robot)
                commands += self._meeting_failed(robot, event.now)
                if self._state != MissionState.MEET:    # the meeting was given up
                    return commands
            if (self.hold_start is not None
                    and event.now - self.hold_start >= self.params.meet_hold_time):
                self._meeting_done = True
                self.meeting_pending = False
                if self.all_accepted:
                    return commands + self._finish()
                self._state = MissionState.EXPLORE
                commands += self._assign(event.now)
            return commands
        return []

    # ---------- helpers (section 5.3) ----------

    def _send(self, robot: str, target: Point, spin: bool, now: float) -> SendGoal:
        """
        Make a SendGoal with a new goal_id and store it as the robot's goal (section 5.1).

        deadline = now + goal_timeout_base + 3 * cost / max_linear_speed
                   (+ spin_time if the goal spins)

        Three times the straight travel time leaves room for turning, detours and
        planning. Without a cost (unreachable) only the base is used.
        """
        goal_id = self.next_goal_id
        self.next_goal_id += 1
        cost = self.cost(robot, target)
        if cost is None:
            cost = 0.0
        deadline = now + self.params.goal_timeout_base + 3 * cost / self.params.max_linear_speed
        if spin:
            deadline += self.params.spin_time
        self.goal_id[robot] = goal_id
        self.goal_target[robot] = target
        self.goal_deadline[robot] = deadline
        return SendGoal(robot, goal_id, target, spin)

    def _clear_goal(self, robot: str) -> None:
        """Forget the robot's active goal; the robot is free afterwards."""
        self.goal_id.pop(robot, None)
        self.goal_target.pop(robot, None)
        self.goal_deadline.pop(robot, None)

    def _assign(self, now: float) -> List[Command]:
        """
        Give every robot without an active goal its next viewpoint (section 5.3).

        EXPLORE only. When the allocator has nothing for a robot and no free
        viewpoint is left, the pass is used up: load the next one (0, 1, 0, ...).
        A pending meeting is tried first at a pass switch, and nothing is assigned
        if it starts; otherwise the allocator is asked once more. A robot that still
        gets nothing waits for the next event. Viewpoint goals always spin.
        """
        commands = []
        if self._state != MissionState.EXPLORE:
            return commands
        for robot in self.robots:
            if robot in self.goal_id:
                continue                    # still busy with its goal
            target = self.allocator.next_target(robot, self.cost)
            if target is None and self.allocator.free_count() == 0:
                # Pass used up: load the next one.
                self.pass_index = (self.pass_index + 1) % len(self.passes)
                self.allocator.set_targets(self.passes[self.pass_index])
                if self.meeting_pending:
                    commands += self._try_start_meeting(now)
                    if self._state == MissionState.MEET:
                        return commands
                target = self.allocator.next_target(robot, self.cost)
            if target is not None:
                commands.append(self._send(robot, target, True, now))
        return commands

    def _expired(self, now: float) -> List[str]:
        """Return the robots whose active goal has passed its deadline."""
        expired = []
        for robot, deadline in self.goal_deadline.items():
            if now > deadline:
                expired.append(robot)
        return expired

    def _try_start_meeting(self, now: float) -> List[Command]:
        """
        Try to start the meeting (section 5.3).

        Ask for meeting points without the excluded ones; if there are none, forget
        the excluded points and ask once more. Still none: stay in EXPLORE with the
        meeting pending and return no commands. The points are computed before
        anything is changed, so a failed attempt leaves exploration untouched.
        Otherwise release both viewpoint reservations, send both robots to their
        points without spin and go to MEET.
        """
        self.meet_attempts = 0
        points = self.meeting(self.meet_exclude)
        if points is None:
            self.meet_exclude = []
            points = self.meeting(self.meet_exclude)
        if points is None:
            return []
        commands = []
        for robot in self.robots:
            self.allocator.release(robot)
            self.meet_targets[robot] = points[robot]
            commands.append(self._send(robot, points[robot], False, now))
        self.arrived = set()
        self.hold_start = None
        self._state = MissionState.MEET
        return commands

    def _meeting_failed(self, robot: str, now: float) -> List[Command]:
        """
        Handle a robot that did not reach its meeting point (section 5.2, MEET failure).

        Its point is excluded and the attempt counted. Below meet_max_attempts: ask for
        new points and send only the robots whose point changed (they are no longer
        counted as arrived; the hold restarts). At the limit or without points: give
        up for now and explore again; the meeting stays pending and is retried at the
        next pass switch, or on every Tick once all five markers are accepted.
        """
        self.meet_exclude.append(self.meet_targets[robot])
        self.meet_attempts += 1
        if self.meet_attempts < self.params.meet_max_attempts:
            points = self.meeting(self.meet_exclude)
            if points is not None:
                commands = []
                for r in self.robots:
                    if points[r] != self.meet_targets.get(r):
                        self.meet_targets[r] = points[r]
                        self.arrived.discard(r)
                        commands.append(self._send(r, points[r], False, now))
                self.hold_start = None
                return commands
        self.hold_start = None
        self._state = MissionState.EXPLORE
        return self._assign(now)

    def _finish(self) -> List[Command]:
        """End the mission: CancelGoal for every active goal, then Finish(); go to DONE."""
        commands = []
        for robot in self.goal_id:
            commands.append(CancelGoal(robot))
        commands.append(Finish())
        self._state = MissionState.DONE
        return commands
