"""The right arm stows inside the base, by way of the posture it used to stow in."""

import pytest

from avaa_solution.approach_node import (RIGHT_TUCK, RIGHT_TUCK_VIA, duration_msg,
                                         right_tuck_points)


def test_the_stow_passes_the_old_tuck_before_ending_at_the_new_one():
    points = right_tuck_points(6.0)
    assert [pose for pose, _ in points] == [RIGHT_TUCK_VIA, RIGHT_TUCK]
    assert points[0][1] == pytest.approx(3.6)
    assert points[1][1] == pytest.approx(6.0)


def test_the_stow_times_increase():
    times = [at for _, at in right_tuck_points(5.0)]
    assert times == sorted(times) and times[0] > 0.0


def test_fractional_seconds_survive_the_message():
    msg = duration_msg(3.6)
    assert msg.sec == 3
    assert msg.nanosec == pytest.approx(600_000_000, abs=1)
