"""
Tests for core/mission.py with a fake allocator, cost function and meeting function.

The acceptance criteria of MIS-01 (#17) ask for: every row of the transition table
(ARCHITECTURE.md section 5.2), ID 4 accepted before `Started`, meeting failures up to
`meet_max_attempts`, retry of the meeting at a pass switch and once all five markers
are accepted, results with an old `goal_id`, and `Finish` cancelling all goals.

AI use: tests written by Claude Opus 5.5.
"""
import math

import pytest

try:
    from rescue_team.core.mission import (
        CancelGoal, Finish, GoalFinished, MarkerAccepted, Mission, MissionParams,
        MissionState, SendGoal, Started, Tick)
except ModuleNotFoundError as error:
    # Skip only while core/mission.py does not exist; any other import error must fail.
    if error.name != 'rescue_team.core.mission':
        raise
    pytest.skip('core/mission.py is not written yet (MIS-01)', allow_module_level=True)


# ---------- fakes ----------

START = {'tb3_0': (0.0, -1.0), 'tb3_1': (0.0, 1.0)}
PASS_0 = [(1.0, -1.0), (1.0, 1.0), (3.0, -1.0), (3.0, 1.0)]
PASS_1 = [(2.0, 0.0)]
UNREACHABLE = (99.0, 99.0)

# Meeting points per robot, in the order they are offered; excluded points are skipped.
MEET_CHOICES = {
    'tb3_0': [(5.0, 5.0), (6.0, 5.0), (7.0, 5.0), (8.0, 5.0)],
    'tb3_1': [(5.5, 5.0), (6.5, 5.0), (7.5, 5.0), (8.5, 5.0)],
}

PARAMS = MissionParams(meet_hold_time=3.0, meet_max_attempts=3, goal_timeout_base=20.0,
                       max_linear_speed=0.25, spin_time=13.0)


def fake_cost(robot, point):
    """Straight-line distance from the robot's start position; None for UNREACHABLE."""
    if point == UNREACHABLE:
        return None
    return math.dist(START[robot], point)


class FakeAllocator:
    """Same behaviour as GreedyAllocator in docs/CODE_STRUCTURE.md, on a plain list."""

    def __init__(self):
        self.set_targets([])

    def set_targets(self, targets):
        self.targets = list(targets)
        self.reserved = {}          # robot -> point
        self.visited = set()
        self.failed = {}            # point -> robots that failed it
        self.dropped = set()
        self.loaded = getattr(self, 'loaded', []) + [list(targets)]

    def _free(self, point):
        return (point not in self.visited and point not in self.dropped
                and point not in self.reserved.values())

    def next_target(self, robot, cost):
        options = [p for p in self.targets
                   if self._free(p) and robot not in self.failed.get(p, set())
                   and cost(robot, p) is not None]
        if not options:
            return None
        best = min(options, key=lambda p: cost(robot, p))
        self.reserved[robot] = best
        return best

    def complete(self, robot, success):
        point = self.reserved.pop(robot, None)
        if point is None:
            return
        if success:
            self.visited.add(point)
            return
        self.failed.setdefault(point, set()).add(robot)
        if len(self.failed[point]) >= 2:
            self.dropped.add(point)

    def release(self, robot):
        self.reserved.pop(robot, None)

    def free_count(self):
        return sum(1 for p in self.targets if self._free(p))


class FakeMeeting:
    """Meeting function: first non-excluded choice per robot; None for the first `fail` calls."""

    def __init__(self, fail=0):
        self.fail = fail
        self.calls = []

    def __call__(self, exclude):
        self.calls.append(list(exclude))
        if self.fail > 0:
            self.fail -= 1
            return None
        points = {}
        for robot, choices in MEET_CHOICES.items():
            free = [p for p in choices if p not in exclude]
            if not free:
                return None
            points[robot] = free[0]
        return points


# ---------- helpers ----------

def make(meeting=None):
    """Return a new mission, its fake allocator and its fake meeting function."""
    allocator = FakeAllocator()
    meeting = meeting if meeting is not None else FakeMeeting()
    return Mission(allocator, fake_cost, meeting, PARAMS), allocator, meeting


def start(mission, now=0.0, passes=None):
    """Send Started with both robots and two passes."""
    return mission.handle(Started(now, dict(START), passes or [PASS_0, PASS_1]))


def finish(mission, robot, now, success=True):
    """Send GoalFinished for the robot's active goal."""
    return mission.handle(GoalFinished(now, robot, mission.goal_id[robot], success))


def sends(commands):
    """Return the SendGoal commands by robot."""
    return {c.robot: c for c in commands if isinstance(c, SendGoal)}


def cancels(commands):
    """Return the robots with a CancelGoal command."""
    return {c.robot for c in commands if isinstance(c, CancelGoal)}


def start_meeting(now=1.0, meeting=None):
    """Return a mission that is in MEET, entered at `now` after ID 4 was accepted."""
    mission, allocator, meeting = make(meeting)
    start(mission)
    mission.handle(MarkerAccepted(now, 4, False))
    assert mission.state == MissionState.MEET
    return mission, allocator, meeting


# ---------- WAIT_FOR_SYSTEM ----------

def test_starts_in_wait_for_system():
    mission, _, _ = make()
    assert mission.state == MissionState.WAIT_FOR_SYSTEM
    assert not mission.meeting_done


def test_wait_marker_is_only_recorded():
    mission, _, _ = make()
    assert mission.handle(MarkerAccepted(0.0, 2, False)) == []
    assert mission.state == MissionState.WAIT_FOR_SYSTEM
    assert not mission.meeting_pending


def test_wait_ignores_tick_and_goal_finished():
    mission, _, _ = make()
    assert mission.handle(Tick(0.0)) == []
    assert mission.handle(GoalFinished(0.0, 'tb3_0', 1, True)) == []
    assert mission.state == MissionState.WAIT_FOR_SYSTEM


def test_started_loads_pass_0_and_sends_viewpoints_with_spin():
    mission, allocator, _ = make()
    commands = start(mission)
    assert mission.state == MissionState.EXPLORE
    assert allocator.loaded[-1] == PASS_0
    goals = sends(commands)
    assert goals['tb3_0'].target == (1.0, -1.0)
    assert goals['tb3_1'].target == (1.0, 1.0)
    assert goals['tb3_0'].spin and goals['tb3_1'].spin
    assert mission.robots == ['tb3_0', 'tb3_1']


def test_id4_accepted_before_started_goes_to_meet():
    mission, _, _ = make()
    assert mission.handle(MarkerAccepted(0.0, 4, False)) == []
    assert mission.meeting_pending
    assert mission.state == MissionState.WAIT_FOR_SYSTEM

    goals = sends(start(mission))
    assert mission.state == MissionState.MEET
    assert goals['tb3_0'].target == (5.0, 5.0)
    assert goals['tb3_1'].target == (5.5, 5.0)
    assert not goals['tb3_0'].spin and not goals['tb3_1'].spin


def test_id4_before_started_without_meeting_points_explores():
    mission, _, _ = make(FakeMeeting(fail=99))
    mission.handle(MarkerAccepted(0.0, 4, False))
    goals = sends(start(mission))
    assert mission.state == MissionState.EXPLORE
    assert mission.meeting_pending
    assert goals['tb3_0'].spin


# ---------- goal ids and deadlines ----------

def test_every_goal_gets_a_new_increasing_id():
    mission, _, _ = make()
    first = sends(start(mission))
    second = sends(finish(mission, 'tb3_0', 10.0))
    ids = [first['tb3_0'].goal_id, first['tb3_1'].goal_id, second['tb3_0'].goal_id]
    assert ids == sorted(ids)
    assert len(set(ids)) == 3


def test_deadline_with_spin():
    mission, _, _ = make()
    start(mission, now=0.0)
    # tb3_0 drives 1.0 m: 20 + 3 * 1.0 / 0.25 + 13 (spin)
    assert mission.goal_deadline['tb3_0'] == pytest.approx(45.0)


def test_deadline_without_spin_for_meeting_goals():
    mission, _, _ = start_meeting(now=1.0)
    distance = math.dist(START['tb3_0'], (5.0, 5.0))
    assert mission.goal_deadline['tb3_0'] == pytest.approx(1.0 + 20.0 + 3 * distance / 0.25)


def test_deadline_without_cost_uses_the_base_only():
    mission, _, _ = make()
    mission._send('tb3_0', UNREACHABLE, False, 10.0)
    assert mission.goal_deadline['tb3_0'] == pytest.approx(30.0)


def test_result_with_old_goal_id_is_ignored():
    mission, allocator, _ = make()
    start(mission)
    old_id = mission.goal_id['tb3_0']
    finish(mission, 'tb3_0', 10.0)          # tb3_0 gets a new goal
    new_id = mission.goal_id['tb3_0']

    assert mission.handle(GoalFinished(11.0, 'tb3_0', old_id, False)) == []
    assert mission.goal_id['tb3_0'] == new_id
    assert not allocator.failed


# ---------- EXPLORE ----------

def test_explore_success_marks_visited_and_assigns_next():
    mission, allocator, _ = make()
    start(mission)
    goals = sends(finish(mission, 'tb3_0', 10.0, success=True))
    assert (1.0, -1.0) in allocator.visited
    assert goals['tb3_0'].target == (3.0, -1.0)
    assert 'tb3_1' not in goals                     # tb3_1 still has its goal


def test_explore_failure_is_reported_to_the_allocator():
    mission, allocator, _ = make()
    start(mission)
    goals = sends(finish(mission, 'tb3_0', 10.0, success=False))
    assert allocator.failed[(1.0, -1.0)] == {'tb3_0'}
    assert goals['tb3_0'].target != (1.0, -1.0)


def test_pass_switch_order_0_1_0():
    mission, allocator, _ = make()
    start(mission, passes=[[(1.0, -1.0), (1.0, 1.0)], [(2.0, 0.0)]])

    goals = sends(finish(mission, 'tb3_0', 10.0))   # pass 0 used up
    assert mission.pass_index == 1
    assert goals['tb3_0'].target == (2.0, 0.0)

    goals = sends(finish(mission, 'tb3_1', 11.0))   # pass 1 used up
    assert mission.pass_index == 0
    assert goals['tb3_1'].target == (1.0, 1.0)


def test_pass_switch_retries_a_pending_meeting():
    mission, _, meeting = make(FakeMeeting(fail=2))  # both tries of the first attempt fail
    start(mission, passes=[[(1.0, -1.0), (1.0, 1.0)], [(2.0, 0.0)]])
    assert mission.handle(MarkerAccepted(5.0, 4, False)) == []
    assert mission.state == MissionState.EXPLORE
    assert mission.meeting_pending

    goals = sends(finish(mission, 'tb3_0', 10.0))   # pass switch
    assert mission.state == MissionState.MEET
    assert goals['tb3_0'].target == (5.0, 5.0)
    assert not goals['tb3_0'].spin


def test_explore_id4_starts_the_meeting_and_releases_reservations():
    mission, allocator, _ = make()
    start(mission)
    old_id = mission.goal_id['tb3_0']
    goals = sends(mission.handle(MarkerAccepted(5.0, 4, False)))

    assert mission.state == MissionState.MEET
    assert allocator.reserved == {}
    assert goals['tb3_0'].target == (5.0, 5.0)
    assert goals['tb3_1'].target == (5.5, 5.0)
    # The preempted exploration goal answers late: ignored.
    assert mission.handle(GoalFinished(6.0, 'tb3_0', old_id, False)) == []
    assert mission.state == MissionState.MEET


def test_explore_other_marker_is_only_recorded():
    mission, _, _ = make()
    start(mission)
    assert mission.handle(MarkerAccepted(5.0, 2, False)) == []
    assert mission.state == MissionState.EXPLORE


def test_explore_all_accepted_before_meeting_does_not_finish():
    mission, _, _ = make(FakeMeeting(fail=2))
    start(mission)
    mission.handle(MarkerAccepted(5.0, 4, False))   # meeting cannot start yet
    assert mission.handle(MarkerAccepted(6.0, 0, True)) == []
    assert mission.state == MissionState.EXPLORE


def test_tick_retries_the_meeting_once_all_are_accepted():
    mission, _, _ = make(FakeMeeting(fail=2))
    start(mission)
    mission.handle(MarkerAccepted(5.0, 4, False))
    mission.handle(MarkerAccepted(6.0, 0, True))

    goals = sends(mission.handle(Tick(7.0)))
    assert mission.state == MissionState.MEET
    assert goals['tb3_0'].target == (5.0, 5.0)


def test_tick_without_expired_goals_sends_nothing():
    mission, _, _ = make()
    start(mission, now=0.0)
    assert mission.handle(Tick(10.0)) == []


def test_tick_cancels_expired_goals_as_failures():
    mission, allocator, _ = make()
    start(mission, now=0.0)                         # both deadlines are 45.0
    commands = mission.handle(Tick(46.0))

    assert cancels(commands) == {'tb3_0', 'tb3_1'}
    assert allocator.failed[(1.0, -1.0)] == {'tb3_0'}
    assert allocator.failed[(1.0, 1.0)] == {'tb3_1'}
    goals = sends(commands)
    assert goals['tb3_0'].target != (1.0, -1.0)     # not the viewpoint it failed
    assert goals['tb3_1'].target != (1.0, 1.0)


# ---------- MEET ----------

def test_meet_hold_starts_when_both_arrived():
    mission, _, _ = start_meeting(now=1.0)
    assert finish(mission, 'tb3_0', 10.0) == []
    assert mission.hold_start is None
    assert finish(mission, 'tb3_1', 12.0) == []
    assert mission.hold_start == 12.0


def test_meet_hold_done_goes_back_to_explore():
    mission, _, _ = start_meeting(now=1.0)
    finish(mission, 'tb3_0', 10.0)
    finish(mission, 'tb3_1', 12.0)

    assert mission.handle(Tick(14.0)) == []         # 2 s < meet_hold_time
    assert mission.state == MissionState.MEET

    goals = sends(mission.handle(Tick(15.0)))
    assert mission.state == MissionState.EXPLORE
    assert mission.meeting_done
    assert not mission.meeting_pending
    assert goals['tb3_0'].spin and goals['tb3_1'].spin


def test_meet_hold_done_and_all_accepted_finishes():
    mission, _, _ = start_meeting(now=1.0)
    assert mission.handle(MarkerAccepted(5.0, 0, True)) == []
    assert mission.state == MissionState.MEET
    finish(mission, 'tb3_0', 10.0)
    finish(mission, 'tb3_1', 12.0)

    assert mission.handle(Tick(15.0)) == [Finish()]
    assert mission.state == MissionState.DONE


def test_meet_failure_sends_only_changed_points():
    mission, _, meeting = start_meeting(now=1.0)
    finish(mission, 'tb3_1', 10.0)                  # tb3_1 arrived
    goals = sends(finish(mission, 'tb3_0', 11.0, success=False))

    assert meeting.calls[-1] == [(5.0, 5.0)]
    assert set(goals) == {'tb3_0'}
    assert goals['tb3_0'].target == (6.0, 5.0)
    assert mission.arrived == {'tb3_1'}
    assert mission.meet_attempts == 1
    assert mission.state == MissionState.MEET


def test_meet_failures_up_to_max_attempts_then_explore():
    mission, _, meeting = start_meeting(now=1.0)
    for attempt in range(1, PARAMS.meet_max_attempts):
        finish(mission, 'tb3_0', 10.0 + attempt, success=False)
        assert mission.state == MissionState.MEET
        assert mission.meet_attempts == attempt

    goals = sends(finish(mission, 'tb3_0', 20.0, success=False))
    assert mission.state == MissionState.EXPLORE
    assert mission.meeting_pending                  # retried later
    assert not mission.meeting_done
    assert goals['tb3_0'].spin                      # a viewpoint again
    assert meeting.calls[-1] == [(5.0, 5.0), (6.0, 5.0)]


def test_meet_passed_deadline_counts_as_failure():
    mission, _, _ = start_meeting(now=1.0)
    finish(mission, 'tb3_1', 10.0)
    commands = mission.handle(Tick(mission.goal_deadline['tb3_0'] + 1.0))

    assert cancels(commands) == {'tb3_0'}
    assert sends(commands)['tb3_0'].target == (6.0, 5.0)
    assert mission.meet_attempts == 1


def test_meet_marker_is_only_recorded():
    mission, _, _ = start_meeting(now=1.0)
    assert mission.handle(MarkerAccepted(5.0, 2, True)) == []
    assert mission.all_accepted
    assert mission.state == MissionState.MEET


# ---------- DONE ----------

def test_finish_cancels_all_goals():
    mission, _, _ = start_meeting(now=1.0)
    finish(mission, 'tb3_0', 10.0)
    finish(mission, 'tb3_1', 12.0)
    mission.handle(Tick(15.0))                      # meeting done, both explore again

    commands = mission.handle(MarkerAccepted(20.0, 0, True))
    assert cancels(commands) == {'tb3_0', 'tb3_1'}
    assert commands[-1] == Finish()
    assert mission.state == MissionState.DONE


def test_done_ignores_every_event():
    mission, _, _ = start_meeting(now=1.0)
    mission.handle(MarkerAccepted(5.0, 0, True))
    finish(mission, 'tb3_0', 10.0)
    finish(mission, 'tb3_1', 12.0)
    mission.handle(Tick(15.0))
    assert mission.state == MissionState.DONE

    assert mission.handle(Tick(20.0)) == []
    assert mission.handle(MarkerAccepted(20.0, 3, True)) == []
    assert mission.handle(GoalFinished(20.0, 'tb3_0', 1, True)) == []
    assert mission.handle(Started(20.0, dict(START), [PASS_0])) == []
