"""Tests for the box-size gate, which refuses a fix of the gripper's own image.

The gate exists because of one grasp, run 23 on 2026-09-11. The book's box held
22-23 px at 0.75-0.77 m for four seconds while the arm reached in; in the second
the open gripper came between the lens and the book, the blob read 39, 47, 56 and
67 px wide at the same range, its centre sliding 80 mm to the side with the hand.
Every one of those was published as the target book, the servo re-aimed onto its
own gripper during the close, and the jaws closed 84 mm beside the book -- an
empty hand carried to a delivery that then worked perfectly.

The numbers in the cases below are read off that run's log, so the gate is tested
against the thing it has to catch rather than against a made-up jump.
"""

import pytest

from avaa_solution.perception_node import BOX_SIZE_BAND, box_size_matches

# The book's fixes through the reach, at a range the base held to within 20 mm:
# (range in metres, box width in pixels). Their width x range product sits at
# 16.6-17.3 throughout, which is the constant this gate defends.
BOOK_THROUGH_THE_REACH = [
    (0.760, 22), (0.764, 22), (0.767, 23), (0.756, 22),
    (0.753, 23), (0.755, 22), (0.756, 22), (0.753, 23),
]

# The fixes that arrived once the gripper was in front of the lens, same run:
# the box growing while the range held, which a book that cannot move cannot do.
GRIPPER_IN_FRONT = [
    (0.780, 39), (0.822, 47), (0.816, 47), (0.828, 56), (0.824, 61), (0.832, 67),
]


def test_no_history_accepts_anything():
    ok, expected = box_size_matches(22, 0.75, [])
    assert ok
    assert expected is None


def test_a_book_like_the_book_is_accepted():
    ok, _ = box_size_matches(22, 0.75, BOOK_THROUGH_THE_REACH)
    assert ok


@pytest.mark.parametrize("range_m,width", GRIPPER_IN_FRONT)
def test_the_grippers_image_is_refused(range_m, width):
    """Every phantom fix of the run that provoked the gate."""
    ok, expected = box_size_matches(width, range_m, BOOK_THROUGH_THE_REACH)
    assert not ok
    # And the expected width it quotes is the book's, not the blob's.
    assert 15 <= expected <= 25


def test_the_approach_tracks_the_one_over_range_law():
    """The width grows as the range falls, and the gate must not refuse that.

    A run of accepted fixes from 2.5 m in to 0.5 m: each one's width is within a
    couple of pixels of a width x range product of 17, which is the book.
    """
    approach = [(2.50, 7), (2.10, 8), (1.60, 11), (1.20, 14),
                (0.90, 19), (0.65, 26), (0.50, 34)]
    for i, (range_m, width) in enumerate(approach):
        ok, _ = box_size_matches(width, range_m, approach[:i])
        assert ok, "fix %d (%r) was refused by its own history" % (i, (range_m, width))


def test_a_pixel_of_quantisation_is_not_a_refusal():
    """At five pixels a book, one pixel is a fifth -- well inside the band."""
    ok, _ = box_size_matches(6, 3.0, [(3.0, 5), (3.1, 5), (2.9, 5)])
    assert ok


def test_the_band_admits_the_margins_it_promises():
    samples = [(0.75, 22)]
    at = BOX_SIZE_BAND * 22
    ok_at, _ = box_size_matches(22 + at, 0.75, samples)
    ok_over, _ = box_size_matches(22 + at + 1, 0.75, samples)
    assert ok_at
    assert not ok_over
