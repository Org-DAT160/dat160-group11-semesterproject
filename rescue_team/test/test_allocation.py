"""Tests for core/allocation.py, with a fake cost function.

The cost function looks the travel distance up in a table per robot, so the tests
choose the costs directly. A viewpoint missing from a robot's table is
unreachable for it (cost None).
"""
import pytest

try:
    from rescue_team.core.allocation import GreedyAllocator
except ModuleNotFoundError as error:
    # Skip only while core/allocation.py does not exist; any other import error must fail.
    if error.name != 'rescue_team.core.allocation':
        raise
    pytest.skip('core/allocation.py is not written yet (MIS-05)', allow_module_level=True)

P = (0.0, 0.0)
Q = (1.0, 0.0)
R = (2.0, 0.0)
S = (3.0, 0.0)
ALL = [P, Q, R, S]


def table_cost(table):
    """Cost function from {robot: {viewpoint: cost}}; missing entries are unreachable."""
    def cost(robot, viewpoint):
        return table[robot].get(viewpoint)
    return cost


# tb3_0 likes P best, tb3_1 likes S best; both can reach everything.
COST = table_cost({
    'tb3_0': {P: 1.0, Q: 2.0, R: 3.0, S: 4.0},
    'tb3_1': {P: 4.0, Q: 3.0, R: 2.0, S: 1.0},
})
# Both robots like the same order: P, Q, R, S.
SAME = table_cost({
    'tb3_0': {P: 1.0, Q: 2.0, R: 3.0, S: 4.0},
    'tb3_1': {P: 1.0, Q: 2.0, R: 3.0, S: 4.0},
})


def allocator(targets=ALL):
    a = GreedyAllocator()
    a.set_targets(targets)
    return a


# next_target

def test_cheapest_free_viewpoint():
    assert allocator().next_target('tb3_0', COST) == P
    assert allocator().next_target('tb3_1', COST) == S


def test_cost_is_asked_for_the_robot_and_every_viewpoint():
    calls = []

    def cost(robot, viewpoint):
        calls.append((robot, viewpoint))
        return 1.0

    allocator().next_target('tb3_1', cost)
    assert sorted(calls) == sorted(('tb3_1', v) for v in ALL)


def test_unreachable_viewpoints_are_skipped():
    cost = table_cost({'tb3_0': {R: 5.0, S: 9.0}})
    assert allocator().next_target('tb3_0', cost) == R


def test_none_when_nothing_is_reachable():
    cost = table_cost({'tb3_0': {}})
    assert allocator().next_target('tb3_0', cost) is None


def test_none_without_targets():
    assert allocator([]).next_target('tb3_0', COST) is None


def test_a_cost_of_zero_is_reachable():
    cost = table_cost({'tb3_0': {Q: 0.0, R: 1.0}})
    assert allocator().next_target('tb3_0', cost) == Q


def test_equal_costs_pick_the_first_in_the_list():
    cost = table_cost({'tb3_0': {P: 2.0, Q: 1.0, R: 1.0, S: 1.0}})
    assert allocator().next_target('tb3_0', cost) == Q


def test_reservations_split_two_robots():
    a = allocator()
    assert a.next_target('tb3_0', SAME) == P
    assert a.next_target('tb3_1', SAME) == Q


def test_none_when_every_viewpoint_is_reserved():
    a = allocator([P, Q])
    a.next_target('tb3_0', SAME)
    a.next_target('tb3_1', SAME)
    assert a.next_target('tb3_2', table_cost({'tb3_2': {P: 1.0, Q: 1.0}})) is None


def test_a_robot_holds_at_most_one_reservation():
    a = allocator()
    a.next_target('tb3_0', SAME)
    a.next_target('tb3_0', SAME)
    assert a.free_count() == 3


def test_asking_again_can_give_the_robot_its_own_reservation_back():
    # The old reservation is released first, so it is free again for this robot.
    a = allocator()
    assert a.next_target('tb3_0', SAME) == P
    assert a.next_target('tb3_0', SAME) == P


# complete

def test_success_marks_the_viewpoint_visited():
    a = allocator()
    a.next_target('tb3_0', SAME)                      # P
    a.complete('tb3_0', True)
    assert a.free_count() == 3
    assert a.next_target('tb3_0', SAME) == Q
    assert a.next_target('tb3_1', SAME) == R


def test_first_failure_frees_it_for_the_other_robot_only():
    a = allocator()
    a.next_target('tb3_0', SAME)                      # P
    a.complete('tb3_0', False)
    assert a.free_count() == 4
    assert a.next_target('tb3_0', SAME) == Q
    assert a.next_target('tb3_1', SAME) == P


def test_second_failure_drops_the_viewpoint():
    a = allocator()
    a.next_target('tb3_0', SAME)                      # P
    a.complete('tb3_0', False)
    assert a.next_target('tb3_1', SAME) == P
    a.complete('tb3_1', False)
    assert a.free_count() == 3
    assert a.next_target('tb3_0', SAME) == Q
    assert a.next_target('tb3_1', SAME) == R


def test_complete_ends_the_reservation():
    a = allocator([P])
    a.next_target('tb3_0', SAME)
    a.complete('tb3_0', True)
    a.complete('tb3_0', False)                        # no reservation any more
    assert a.free_count() == 0
    assert a.next_target('tb3_1', SAME) is None


def test_complete_without_a_reservation_does_nothing():
    a = allocator()
    a.complete('tb3_0', True)
    a.complete('tb3_0', False)
    assert a.free_count() == 4
    assert a.next_target('tb3_0', SAME) == P


def test_complete_only_touches_the_robots_own_reservation():
    a = allocator()
    a.next_target('tb3_0', SAME)                      # P
    a.next_target('tb3_1', SAME)                      # Q
    a.complete('tb3_1', True)
    assert a.free_count() == 2                        # P reserved, Q visited
    a.complete('tb3_0', True)
    assert a.free_count() == 2
    assert a.next_target('tb3_0', SAME) == R


# release

def test_release_frees_the_viewpoint_without_marking_it():
    a = allocator()
    a.next_target('tb3_0', SAME)                      # P
    a.release('tb3_0')
    assert a.free_count() == 4
    assert a.next_target('tb3_0', SAME) == P


def test_release_without_a_reservation_does_nothing():
    a = allocator()
    a.release('tb3_0')
    assert a.free_count() == 4


def test_release_keeps_an_earlier_failure():
    a = allocator()
    a.next_target('tb3_0', SAME)                      # P
    a.complete('tb3_0', False)
    a.next_target('tb3_1', SAME)                      # P, free for tb3_1 only
    a.release('tb3_1')
    assert a.next_target('tb3_0', SAME) == Q


def test_release_then_complete_does_nothing():
    a = allocator()
    a.next_target('tb3_0', SAME)                      # P
    a.release('tb3_0')
    a.complete('tb3_0', True)
    assert a.free_count() == 4


# free_count and set_targets

def test_free_count_counts_neither_visited_dropped_nor_reserved():
    a = allocator()
    assert a.free_count() == 4
    a.next_target('tb3_0', SAME)                      # P reserved
    assert a.free_count() == 3
    a.next_target('tb3_1', SAME)                      # Q reserved
    assert a.free_count() == 2
    a.complete('tb3_1', True)                         # Q visited, R and S free
    assert a.free_count() == 2
    a.complete('tb3_0', False)                        # P free for tb3_1 only, still free
    assert a.free_count() == 3


def test_set_targets_starts_a_new_pass():
    a = allocator()
    a.next_target('tb3_0', SAME)                      # P
    a.complete('tb3_0', True)
    a.next_target('tb3_0', SAME)                      # Q
    a.complete('tb3_0', False)
    a.next_target('tb3_1', SAME)                      # Q, reserved

    a.set_targets([P, Q, R])
    assert a.free_count() == 3                        # nothing visited, failed or reserved
    a.complete('tb3_1', True)                         # the old reservation is gone
    assert a.free_count() == 3
    assert a.next_target('tb3_0', SAME) == P
