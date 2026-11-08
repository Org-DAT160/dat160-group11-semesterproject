"""Tests for core/marker_registry.py, with hand-made sightings.

Unless stated otherwise the registry uses min_sightings = 3, the default
resend_timeout of 5 s and the marker IDs 0-4. Times are in seconds. Attempts are
numbered from 1. Positions are chosen so that every median is exact.
"""
import pytest

try:
    from rescue_team.core.marker_registry import MarkerRegistry, Report, Sighting
except ModuleNotFoundError as error:
    # Skip only while core/marker_registry.py does not exist; any other import error must fail.
    if error.name != 'rescue_team.core.marker_registry':
        raise
    pytest.skip('core/marker_registry.py is not written yet (PER-01)', allow_module_level=True)


def sighting(marker_id, x, y, distance=1.0, robot='tb3_0', stamp_ns=0):
    return Sighting(marker_id=marker_id, x=x, y=y, distance=distance, robot=robot,
                    stamp_ns=stamp_ns)


def add_all(registry, marker_id, positions):
    """Add one sighting per (x, y), seen from distance x, and check that each was kept."""
    for x, y in positions:
        assert registry.add(sighting(marker_id, x, y, distance=x)) is True


def single_report(registry, now):
    """reports_due(now) must return exactly one report; return it."""
    reports = registry.reports_due(now)
    assert len(reports) == 1, reports
    return reports[0]


def assert_report(report, marker_id, attempt, position):
    assert report.marker_id == marker_id
    assert report.attempt == attempt
    assert report.position == pytest.approx(position)


def registry_with_first_report(positions=((1.0, 1.0), (2.0, 2.0), (3.0, 3.0))):
    """Marker 0 with three sightings and its first report sent at t = 0."""
    registry = MarkerRegistry(min_sightings=3)
    add_all(registry, 0, positions)
    single_report(registry, 0.0)
    return registry


# Data classes

def test_sighting_and_report_fields():
    s = Sighting(marker_id=2, x=1.5, y=-0.5, distance=0.8, robot='tb3_1', stamp_ns=42)
    assert (s.marker_id, s.x, s.y, s.distance, s.robot, s.stamp_ns) == (
        2, 1.5, -0.5, 0.8, 'tb3_1', 42)
    r = Report(marker_id=2, attempt=1, position=(1.5, -0.5))
    assert (r.marker_id, r.attempt, r.position) == (2, 1, (1.5, -0.5))


# add and estimate

def test_unknown_id_is_ignored():
    registry = MarkerRegistry(min_sightings=1)
    assert registry.add(sighting(7, 1.0, 1.0)) is False
    assert registry.estimate(7) is None
    assert registry.reports_due(0.0) == []


def test_marker_ids_parameter_is_used():
    registry = MarkerRegistry(min_sightings=1, marker_ids=(5, 6))
    assert registry.add(sighting(0, 1.0, 1.0)) is False
    assert registry.add(sighting(5, 1.0, 1.0)) is True


def test_estimate_without_sightings_is_none():
    assert MarkerRegistry(min_sightings=3).estimate(0) is None


def test_median_ignores_one_outlier():
    registry = MarkerRegistry(min_sightings=3)
    add_all(registry, 0, [(1.0, 2.0), (1.2, 1.8), (5.0, 9.0)])
    assert registry.estimate(0) == pytest.approx((1.2, 2.0))
    assert_report(single_report(registry, 0.0), 0, 1, (1.2, 2.0))


def test_median_of_an_even_number_is_the_mean_of_the_middle_two():
    registry = MarkerRegistry(min_sightings=3)
    add_all(registry, 0, [(1.0, 4.0), (2.0, 3.0), (3.0, 2.0), (9.0, 1.0)])
    assert registry.estimate(0) == pytest.approx((2.5, 2.5))


# First report

def test_no_report_before_min_sightings():
    registry = MarkerRegistry(min_sightings=3)
    add_all(registry, 0, [(1.0, 1.0), (2.0, 2.0)])
    assert registry.reports_due(0.0) == []


def test_first_report_once_there_are_min_sightings():
    registry = MarkerRegistry(min_sightings=3)
    add_all(registry, 0, [(1.0, 1.0), (2.0, 2.0)])
    registry.reports_due(0.0)
    add_all(registry, 0, [(3.0, 3.0)])
    assert_report(single_report(registry, 1.0), 0, 1, (2.0, 2.0))


def test_a_sent_report_is_not_sent_again_before_the_timeout():
    registry = registry_with_first_report()
    add_all(registry, 0, [(4.0, 4.0), (5.0, 5.0), (6.0, 6.0)])
    assert registry.reports_due(4.9) == []


def test_markers_are_reported_independently():
    registry = MarkerRegistry(min_sightings=2)
    add_all(registry, 1, [(1.0, 1.0), (3.0, 3.0)])
    add_all(registry, 3, [(5.0, 0.0), (7.0, 0.0)])
    add_all(registry, 4, [(9.0, 9.0)])
    reports = sorted(registry.reports_due(0.0), key=lambda r: r.marker_id)
    assert [r.marker_id for r in reports] == [1, 3]
    assert_report(reports[0], 1, 1, (2.0, 2.0))
    assert_report(reports[1], 3, 1, (6.0, 0.0))


# Resend

def test_resend_after_the_timeout_keeps_attempt_and_position():
    registry = registry_with_first_report()
    # New sightings do not change a report that is waiting for an answer.
    add_all(registry, 0, [(9.0, 9.0), (9.0, 9.0)])
    assert_report(single_report(registry, 5.0), 0, 1, (2.0, 2.0))


def test_resend_timer_restarts_at_each_resend():
    registry = registry_with_first_report()
    single_report(registry, 5.0)
    assert registry.reports_due(9.9) == []
    assert_report(single_report(registry, 10.0), 0, 1, (2.0, 2.0))


def test_resend_timeout_parameter_is_used():
    registry = MarkerRegistry(min_sightings=1, resend_timeout=2.0)
    add_all(registry, 0, [(1.0, 1.0)])
    single_report(registry, 0.0)
    assert registry.reports_due(1.9) == []
    assert_report(single_report(registry, 2.0), 0, 1, (1.0, 1.0))


# Rejections

def test_closer_half_at_once_after_a_rejection():
    registry = MarkerRegistry(min_sightings=3)
    registry.add(sighting(0, 1.0, 1.0, distance=0.5))
    registry.add(sighting(0, 1.2, 1.4, distance=1.0))
    registry.add(sighting(0, 2.0, 2.0, distance=2.0))
    registry.add(sighting(0, 3.0, 3.0, distance=3.0))
    assert_report(single_report(registry, 0.0), 0, 1, (1.6, 1.7))
    assert registry.on_result(0, 1, False) is False
    # At once, without waiting for the timeout: the two closest sightings.
    assert_report(single_report(registry, 0.1), 0, 2, (1.1, 1.2))


def test_closer_half_has_at_least_two_sightings():
    registry = MarkerRegistry(min_sightings=3)
    registry.add(sighting(0, 4.0, 4.0, distance=3.0))
    registry.add(sighting(0, 0.0, 0.0, distance=1.0))
    registry.add(sighting(0, 1.0, 1.0, distance=2.0))
    single_report(registry, 0.0)
    registry.on_result(0, 1, False)
    assert_report(single_report(registry, 0.0), 0, 2, (0.5, 0.5))


def test_retry_after_two_rejections_waits_for_new_sightings():
    registry = registry_with_first_report()
    registry.on_result(0, 1, False)
    single_report(registry, 1.0)                                # attempt 2
    # Sightings while attempt 2 waits for an answer are not new for the retry.
    add_all(registry, 0, [(4.0, 4.0)])
    registry.on_result(0, 2, False)

    assert registry.reports_due(2.0) == []
    add_all(registry, 0, [(5.0, 5.0), (6.0, 6.0)])
    assert registry.reports_due(3.0) == []
    assert registry.reports_due(60.0) == []                     # no resend while waiting
    add_all(registry, 0, [(7.0, 7.0)])

    # Both steps again on all seven sightings: median, then the closer half
    # (7 // 2 = 3 sightings, seen from 1, 2 and 3 m).
    assert_report(single_report(registry, 61.0), 0, 3, (4.0, 4.0))
    registry.on_result(0, 3, False)
    assert_report(single_report(registry, 61.5), 0, 4, (2.0, 2.0))

    # Another rejection waits for min_sightings new sightings again.
    registry.on_result(0, 4, False)
    add_all(registry, 0, [(8.0, 8.0), (8.0, 8.0)])
    assert registry.reports_due(62.0) == []
    add_all(registry, 0, [(8.0, 8.0)])
    # Ten sightings with x = 1..7, 8, 8, 8: the median is (5 + 6) / 2.
    assert_report(single_report(registry, 63.0), 0, 5, (5.5, 5.5))


def test_false_for_an_older_attempt_is_ignored():
    registry = registry_with_first_report()
    single_report(registry, 5.0)                                # resend of attempt 1
    registry.on_result(0, 1, False)                             # answer to one of them
    single_report(registry, 5.5)                                # attempt 2
    assert registry.on_result(0, 1, False) is False             # answer to the other
    # Still one rejection: attempt 2 waits for its answer and is resent.
    assert registry.reports_due(6.0) == []
    assert_report(single_report(registry, 10.5), 0, 2, (1.5, 1.5))


def test_two_falses_for_one_attempt_count_as_one_rejection():
    registry = registry_with_first_report()
    single_report(registry, 5.0)                                # resend of attempt 1
    # Both answers arrive before the next reports_due call.
    registry.on_result(0, 1, False)
    assert registry.on_result(0, 1, False) is False
    # One rejection: the closer half at once, not waiting for new sightings.
    assert_report(single_report(registry, 5.5), 0, 2, (1.5, 1.5))


def test_answer_for_a_report_never_sent_is_ignored():
    registry = registry_with_first_report()
    assert registry.on_result(0, 2, True) is False
    assert registry.on_result(0, 2, False) is False
    assert registry.is_accepted(0) is False
    assert registry.reports_due(1.0) == []


def test_answer_for_an_unknown_marker_is_ignored():
    registry = registry_with_first_report()
    assert registry.on_result(7, 1, True) is False
    assert registry.on_result(3, 1, True) is False
    assert registry.is_accepted(3) is False


# Acceptance

def test_true_accepts_the_marker_with_the_reported_position():
    registry = registry_with_first_report()
    assert registry.accepted_position(0) is None
    assert registry.on_result(0, 1, True) is True
    assert registry.is_accepted(0) is True
    assert registry.accepted_position(0) == pytest.approx((2.0, 2.0))


def test_late_true_after_a_resend_accepts_with_that_reports_position():
    registry = registry_with_first_report()
    single_report(registry, 5.0)                                # resend of attempt 1
    # Scoring answers the repeated report of an accepted marker with false.
    registry.on_result(0, 1, False)
    single_report(registry, 5.5)                                # attempt 2, closer half
    assert registry.on_result(0, 1, True) is True               # the late answer
    assert registry.accepted_position(0) == pytest.approx((2.0, 2.0))
    # The answer to attempt 2 changes nothing.
    assert registry.on_result(0, 2, False) is False
    assert registry.is_accepted(0) is True
    assert registry.accepted_position(0) == pytest.approx((2.0, 2.0))


def test_true_for_the_newest_attempt_uses_its_position():
    registry = registry_with_first_report()
    registry.on_result(0, 1, False)
    single_report(registry, 1.0)                                # attempt 2
    assert registry.on_result(0, 2, True) is True
    assert registry.accepted_position(0) == pytest.approx((1.5, 1.5))


def test_accepted_marker_is_never_reported_again():
    registry = registry_with_first_report()
    registry.on_result(0, 1, True)
    assert registry.add(sighting(0, 5.0, 5.0)) is False
    assert registry.reports_due(5.0) == []
    assert registry.reports_due(100.0) == []
    assert registry.on_result(0, 1, True) is False              # already accepted
    assert registry.accepted_position(0) == pytest.approx((2.0, 2.0))


def test_all_accepted_needs_every_marker():
    registry = MarkerRegistry(min_sightings=1, marker_ids=(0, 1))
    add_all(registry, 0, [(1.0, 1.0)])
    add_all(registry, 1, [(2.0, 2.0)])
    registry.reports_due(0.0)
    assert registry.all_accepted() is False
    registry.on_result(0, 1, True)
    assert registry.all_accepted() is False
    registry.on_result(1, 1, True)
    assert registry.all_accepted() is True


def test_all_accepted_with_the_default_five_markers():
    registry = MarkerRegistry(min_sightings=1)
    for marker_id in range(4):
        add_all(registry, marker_id, [(1.0, 1.0)])
    registry.reports_due(0.0)
    for marker_id in range(4):
        registry.on_result(marker_id, 1, True)
    assert registry.all_accepted() is False
