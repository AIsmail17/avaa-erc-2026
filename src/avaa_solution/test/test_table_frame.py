"""Tests for finding the bin from the legs of its table."""

import math

import pytest

from avaa_solution.table_frame import (LEG_FACE_BIAS, bin_from_legs, leg_candidates,
                                       standoff_goal)

# Measured after the twentieth full run: the robot at world (0.679, -1.919) heading -171.2
# degrees, and the four leg clusters the front laser returned, as (centroid, returns).
MEASURED_LEGS = [((1.806, -1.540), 8), ((1.088, -1.420), 12), ((1.607, -2.847), 5),
                 ((0.889, -2.732), 6)]
# Where Gazebo put the bin's centre, (-0.977, +0.001), seen from that pose.
TRUE_BIN = (1.343, -2.151)


def face(centre, count, width=0.05):
    """Spread ``count`` returns across a leg face centred on ``centre``, square to the ray."""
    reach = math.hypot(*centre)
    px, py = -centre[1] / reach, centre[0] / reach
    return [(centre[0] + px * width * (k / (count - 1) - 0.5),
             centre[1] + py * width * (k / (count - 1) - 0.5)) for k in range(count)]


def test_the_measured_leg_clusters_are_found_as_legs():
    points = [p for centre, count in MEASURED_LEGS for p in face(centre, count)]
    legs = leg_candidates(points)
    assert len(legs) == 4
    for (centre, _), leg in zip(MEASURED_LEGS, legs):
        shift = math.hypot(leg[0] - centre[0], leg[1] - centre[1])
        assert shift == pytest.approx(LEG_FACE_BIAS, abs=1e-6)


def test_the_bin_centre_from_the_measured_legs_is_within_a_few_centimetres():
    points = [p for centre, count in MEASURED_LEGS for p in face(centre, count)]
    found = bin_from_legs(leg_candidates(points))
    assert found is not None
    centre, _ = found
    assert math.hypot(centre[0] - TRUE_BIN[0], centre[1] - TRUE_BIN[1]) < 0.03


def test_square_on_the_bin_is_straight_ahead_behind_the_legs():
    centre, normal = bin_from_legs([(1.2, 0.665), (1.2, -0.665)])
    assert centre == pytest.approx((1.542, 0.0))
    assert normal == pytest.approx((1.0, 0.0))


def test_a_pair_that_disagrees_with_the_sighting_is_not_believed():
    # Two things 1.33 m apart but nowhere near where perception sees the bin.
    assert bin_from_legs([(1.2, 0.665), (1.2, -0.665)], near=(3.0, 2.0)) is None


def test_a_leg_and_a_stray_the_wrong_distance_apart_make_no_table():
    assert bin_from_legs([(1.2, 0.665), (1.2, -0.2)]) is None


def test_single_returns_and_long_walls_are_not_legs():
    wall = [(2.0, -1.0 + 0.02 * k) for k in range(40)]
    assert leg_candidates([(1.0, 0.0)] + wall) == []


def test_the_standoff_puts_the_bin_straight_ahead():
    goal, heading = standoff_goal((1.542, 0.0), (1.0, 0.0), 0.82)
    assert goal == pytest.approx((0.722, 0.0))
    assert heading == pytest.approx(0.0)
