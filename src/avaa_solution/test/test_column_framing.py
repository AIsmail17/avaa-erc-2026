"""Tests for the head tilt that frames a whole shelf column while its row is read."""

import math

import pytest

from avaa_solution import arena
from avaa_solution.approach_node import (BOOK_EDGE_MARGIN, BOOK_HALF_HEIGHT, CAMERA_HALF_V,
                                         HEAD_TILT_MAX, HEAD_TILT_MIN, MARKER_EDGE_MARGIN,
                                         MARKER_Z, column_framing_tilt)

# The depth optical frame above the floor with the head level (approach_node, MARKER_Z).
CAMERA_Z = 1.160
BOTTOM_Z = arena.ROW_HEIGHTS_WORLD[-1] - BOOK_HALF_HEIGHT


def column_in_view(tilt, distance):
    marker = math.atan2(MARKER_Z - CAMERA_Z, distance)
    bottom = math.atan2(BOTTOM_Z - CAMERA_Z, distance)
    return (marker <= tilt + CAMERA_HALF_V - MARKER_EDGE_MARGIN + 1e-9
            and bottom >= tilt - CAMERA_HALF_V + BOOK_EDGE_MARGIN - 1e-9)


def test_the_sixteenth_run_would_have_seen_the_whole_column():
    # Its last search left the head 18 degrees up at 3.09 m, which cut the bottom book off.
    assert not column_in_view(math.radians(18.0), 3.09)
    tilt, fits = column_framing_tilt(CAMERA_Z, 3.09, MARKER_Z, BOTTOM_Z)
    assert fits
    assert column_in_view(tilt, 3.09)


@pytest.mark.parametrize("distance", [2.3, 2.8, 3.5, 4.5])
def test_the_column_fits_from_where_the_approach_centres(distance):
    tilt, fits = column_framing_tilt(CAMERA_Z, distance, MARKER_Z, BOTTOM_Z)
    assert fits
    assert column_in_view(tilt, distance)


def test_too_close_keeps_the_markers_readable():
    tilt, fits = column_framing_tilt(CAMERA_Z, 1.2, MARKER_Z, BOTTOM_Z)
    assert not fits
    marker = math.atan2(MARKER_Z - CAMERA_Z, 1.2)
    assert tilt == pytest.approx(min(HEAD_TILT_MAX, marker - (CAMERA_HALF_V - MARKER_EDGE_MARGIN)))


@pytest.mark.parametrize("distance", [0.4, 1.0, 10.0])
def test_the_tilt_stays_inside_the_joint_limits(distance):
    tilt, _ = column_framing_tilt(CAMERA_Z, distance, MARKER_Z, BOTTOM_Z)
    assert HEAD_TILT_MIN <= tilt <= HEAD_TILT_MAX
