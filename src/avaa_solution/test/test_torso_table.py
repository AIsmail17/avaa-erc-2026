"""Tests for the torso height each row is reached into with.

TORSO_FOR_HEIGHT is a measurement (tools/torsomap.py, tools/shelftorso.py), so these do not
re-derive its numbers. They hold down what the measurement found -- the torsos each row was
accepted at -- so that moving a number outside that fails here rather than as a run that
stops at the posture search, and they check the table still belongs to the node's rows.
"""

import pytest

from avaa_solution.grasp_node import (DEFAULT_ROW_HEIGHTS, TORSO_FOR_HEIGHT, TORSO_SLACK,
                                      torso_for_height)

# grasp_node's grasp_below_centre_m default.
BELOW_CENTRE = 0.045
TORSO_TRAVEL = (0.0, 0.35)

# Torsos accepted, and walked all the way into the shelf, in every cell the maps measured:
# 30 mm either side of each row, and for rows 3 and 4 with the shelf in the scene at two
# book faces as well as without it.
ACCEPTED = {
    1: (0.25, 0.35),
    2: (0.00, 0.35),
    3: (0.00, 0.05),
    4: (0.25, 0.35),
}


def pre_grasp_height(row):
    return DEFAULT_ROW_HEIGHTS[row - 1] - BELOW_CENTRE


def test_the_table_belongs_to_the_rows_the_node_reaches_into():
    """If the row heights move, these torsos were measured for somewhere else."""
    keys = [height for height, _ in TORSO_FOR_HEIGHT]
    wanted = [pre_grasp_height(row) for row in range(1, len(DEFAULT_ROW_HEIGHTS) + 1)]
    assert keys == pytest.approx(wanted, abs=0.001)


@pytest.mark.parametrize("row", sorted(ACCEPTED))
@pytest.mark.parametrize("offset", (-0.03, 0.0, 0.03))
def test_the_pinned_range_stays_inside_what_was_accepted(row, offset):
    torso = torso_for_height(pre_grasp_height(row) + offset)
    low = max(torso - TORSO_SLACK, TORSO_TRAVEL[0])
    high = min(torso + TORSO_SLACK, TORSO_TRAVEL[1])
    accepted_low, accepted_high = ACCEPTED[row]
    assert accepted_low - 1e-9 <= low
    assert high <= accepted_high + 1e-9


@pytest.mark.parametrize("row", sorted(ACCEPTED))
def test_an_estimate_several_centimetres_off_keeps_its_row(row):
    heights = [pre_grasp_height(row) + offset for offset in (-0.08, 0.0, 0.08)]
    assert len({torso_for_height(height) for height in heights}) == 1
