"""Pure gap tracking and manoeuvre clearance rules."""

import pytest

from perception.tracking import GapTracker, lane_free_for_manoeuvre


def test_tracker_warmup_and_own_speed() -> None:
    tracker = GapTracker()
    tracker.update(30.0, 8.0, 0.1)
    assert tracker.speed is None
    tracker.update(29.6, 8.0, 0.1)
    assert tracker.speed is None
    tracker.update(29.2, 8.0, 0.1)
    assert tracker.speed == pytest.approx(4.0)
    tracker.update(28.6, 8.0, 0.1)
    assert tracker.speed == pytest.approx(3.4)


@pytest.mark.parametrize("gap", [None, 40.0])
def test_tracker_resets_on_missing_or_changed_object(gap) -> None:
    tracker = GapTracker()
    for value in (30.0, 29.5, 29.0):
        tracker.update(value, 5.0, 0.1)
    tracker.update(gap, 5.0, 0.1)
    assert tracker.speed is None
    tracker.update(39.5 if gap else 29.0, 5.0, 0.1)
    assert tracker.speed is None


def test_tracker_supports_negative_object_speed_and_rejects_bad_dt() -> None:
    tracker = GapTracker()
    for gap in (30.0, 28.5, 27.0):
        tracker.update(gap, 8.0, 0.1)
    assert tracker.speed == pytest.approx(-7.0)
    with pytest.raises(ValueError):
        tracker.update(26.0, 8.0, 0.0)


@pytest.mark.parametrize("gap,closing,free", [
    (None, None, True), (60.0, None, False), (60.0, 15.0, False),
    (60.0, 0.0, True), (10.0, -2.0, False), (55.0, 5.0, False),
    (55.1, 5.0, True),
])
def test_lane_horizon(gap, closing, free) -> None:
    assert lane_free_for_manoeuvre(gap, closing, 9.0, 10.0) == free


def test_debounce_requires_continuous_clear_and_blocks_immediately():
    from perception.tracking import Debounce

    debounce = Debounce(0.5)
    for _ in range(9):
        assert not debounce.update(True, 0.05)
    assert debounce.update(True, 0.05)
    assert not debounce.update(False, 0.05)
    for _ in range(9):
        assert not debounce.update(True, 0.05)
    assert debounce.update(True, 0.05)


def test_debounce_uses_elapsed_time_and_configurable_hold():
    from perception.tracking import Debounce

    debounce = Debounce(0.3)
    assert not debounce.update(True, 0.1)
    assert debounce.update(True, 0.2)
    assert not debounce.update(False, 0.1)
    assert Debounce(0).update(True, 0.05)
