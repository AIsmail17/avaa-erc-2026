"""Tests for the row-to-height mapping and the gripper command values.

The row direction is the risky part: the rules number the stocked rows 1-4 without saying
which end row 1 is. Getting it backwards costs the identification points and sends the arm
to the wrong shelf, so both directions are pinned down here.
"""

import pytest

from avaa_solution.grasp_node import (
    DEFAULT_ROW_HEIGHTS,
    GRIPPER_CLAMP,
    GRIPPER_OPEN,
    TORSO_BIAS,
    row_to_height,
)

HEIGHTS = [1.391, 1.061, 0.731, 0.401]   # top shelf first


def test_defaults_are_ordered_top_shelf_first():
    assert DEFAULT_ROW_HEIGHTS == sorted(DEFAULT_ROW_HEIGHTS, reverse=True)


def test_rows_are_evenly_spaced():
    gaps = [a - b for a, b in zip(DEFAULT_ROW_HEIGHTS, DEFAULT_ROW_HEIGHTS[1:])]
    for gap in gaps:
        assert gap == pytest.approx(0.33, abs=0.005)


@pytest.mark.parametrize("row,expected", [(1, 1.391), (2, 1.061), (3, 0.731), (4, 0.401)])
def test_top_down_numbering(row, expected):
    assert row_to_height(row, HEIGHTS, top_down=True) == pytest.approx(expected)


@pytest.mark.parametrize("row,expected", [(1, 0.401), (2, 0.731), (3, 1.061), (4, 1.391)])
def test_bottom_up_numbering(row, expected):
    assert row_to_height(row, HEIGHTS, top_down=False) == pytest.approx(expected)


def test_the_two_directions_are_mirror_images():
    top = [row_to_height(r, HEIGHTS, True) for r in range(1, 5)]
    bottom = [row_to_height(r, HEIGHTS, False) for r in range(1, 5)]
    assert top == list(reversed(bottom))


@pytest.mark.parametrize("row", [0, 5, -1, 99])
def test_out_of_range_rows_return_none(row):
    # Must surface rather than quietly aiming at the nearest shelf.
    assert row_to_height(row, HEIGHTS) is None


def test_empty_height_table_returns_none():
    assert row_to_height(1, []) is None


def test_gripper_open_clears_a_book_and_clamp_closes_past_it():
    # Measured span curve: span ~= 0.028 + 0.82 * joint. A book is 0.030 m thick.
    def span(joint):
        return 0.028 + 0.82 * joint

    assert span(GRIPPER_OPEN) > 0.030 * 1.8, "open span should give clear approach room"
    assert span(GRIPPER_CLAMP) < 0.030, "clamp must close past the book thickness"


def test_torso_is_not_compensated_without_evidence():
    """The torso tracks its command exactly, so nothing should be added to it.

    This test previously asserted a 2.5-3 cm compensation, from a measurement taken while
    orphaned processes were holding the simulation at a third of real time and no
    trajectory had time to finish. Re-measured on a clean sim, commands of 0.15, 0.20,
    0.25 and 0.30 all settle at exactly the commanded height, error 0.0000 m.

    The stale compensation was putting the gripper 28 mm above the book, which on its own
    is enough to close the fingers over the top corner of a 30 mm spine rather than round
    it. It cost several runs that reported success.
    """
    assert TORSO_BIAS == pytest.approx(0.0, abs=0.005)
