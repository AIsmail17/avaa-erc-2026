"""Tests for reading the front laser in the robot's frame, and the room it leaves."""

import math

import pytest

from avaa_solution.deliver_node import (BASE_HALF_LENGTH, BASE_HALF_WIDTH, FRONT_LASER_XY,
                                        base_gap, front_clearance, scan_points_in_base)

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
