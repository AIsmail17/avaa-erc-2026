"""Tests for reading the front laser in the robot's frame, and the room it leaves."""

import math

import pytest

from avaa_solution.deliver_node import (BASE_HALF_LENGTH, BASE_HALF_WIDTH, FRONT_LASER_XY,
                                        base_gap, blocked_by, front_clearance,
                                        scan_points_in_base)

START = math.radians(-134.0)
STEP = math.radians(1.0)


def scan_with(returns, count=270):
    """Build a front scan holding the given {scan angle in degrees: range} and nothing else."""
    ranges = [float("inf")] * count
    for degrees, distance in returns.items():
        ranges[int(round((math.radians(degrees) - START) / STEP))] = distance
    return ranges


def test_scan_angle_minus_45_is_straight_ahead():
    [(x, y)] = scan_points_in_base(scan_with({-45: 1.0}), START, STEP)
    assert x == pytest.approx(FRONT_LASER_XY[0] + 1.0, abs=1e-3)
    assert y == pytest.approx(FRONT_LASER_XY[1], abs=1e-3)


def test_scan_angle_plus_45_is_to_the_right():
    [(x, y)] = scan_points_in_base(scan_with({45: 1.0}), START, STEP)
    assert x == pytest.approx(FRONT_LASER_XY[0], abs=1e-3)
    assert y == pytest.approx(FRONT_LASER_XY[1] - 1.0, abs=1e-3)


def test_a_return_inside_the_base_box_is_the_robot():
    assert scan_points_in_base(scan_with({-45: 0.05}), START, STEP) == []


def test_the_eighteenth_runs_table_leg_is_beside_the_front_corner():
    # Seen at (0.243, -0.256) in base_link: 8 mm outside the box, 47 degrees to the right,
    # and not in the base's path straight ahead.
    ahead, nearest = front_clearance([(0.243, -0.256)])
    assert ahead is None
    assert nearest == pytest.approx(0.008, abs=0.002)


def test_room_ahead_is_measured_from_the_bumper():
    ahead, nearest = front_clearance([(1.0, 0.1), (2.0, 0.0)])
    assert ahead == pytest.approx(1.0 - BASE_HALF_LENGTH)
    assert nearest == pytest.approx(1.0 - BASE_HALF_LENGTH)


def test_what_is_behind_the_base_is_not_in_its_way():
    assert front_clearance([(-0.8, 0.0)]) == (None, None)


def test_base_gap_is_zero_inside_and_straight_line_off_a_corner():
    assert base_gap(0.0, 0.0) == 0.0
    assert base_gap(BASE_HALF_LENGTH + 0.03, BASE_HALF_WIDTH + 0.04) == pytest.approx(0.05)


def test_something_beside_the_base_far_from_the_bin_does_not_stop_it():
    # The nineteenth full run: 1.40 m clear ahead, 0.08 m beside, the bin 1.00 m away.
    assert not blocked_by(1.40, 0.075, 1.00, 0.10, 0.08)


def test_a_table_leg_beside_the_corner_near_the_bin_stops_it():
    # The eighteenth full run: the leg 8 mm beside the front corner, the bin 0.76 m away.
    assert blocked_by(None, 0.008, 0.76, 0.10, 0.08)


def test_something_in_the_path_stops_it_anywhere():
    assert blocked_by(0.05, 0.05, 2.50, 0.10, 0.08)


def test_nothing_in_view_does_not_stop_it():
    assert not blocked_by(None, None, 0.80, 0.10, 0.08)
