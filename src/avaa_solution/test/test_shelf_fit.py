"""The shelf fit must ignore whatever lies beyond the shelf.

Close to the shelf the forward cone reaches past the end of it and picks up the far
wall. Fitting a single line through both surfaces put the answer 17 degrees out with a
residual large enough that the guard refused, and the approach then handed an unsquared
base to a grasp that reaches along the base x axis. The robot finished 35.8 degrees off
square and came away empty.

These test the fitting arithmetic directly, on synthetic scans, so they run without a
simulator.
"""

import math

import numpy as np

import pytest

FACE_BAND = 0.60


def fit(points):
    """Reproduce _shelf_angle arithmetic: near-surface slab, one trim, then the guard."""
    if len(points) < 12:
        return None
    xs = np.array([p[0] for p in points])
    ys = np.array([p[1] for p in points])

    near = float(np.percentile(xs, 10))
    keep = xs <= near + FACE_BAND
    xs, ys = xs[keep], ys[keep]
    if len(xs) < 12:
        return None

    slope, intercept = np.polyfit(ys, xs, 1)
    spread = xs - (slope * ys + intercept)
    keep = np.abs(spread) <= 3.0 * max(float(np.std(spread)), 0.01)
    if 12 <= int(keep.sum()) < len(xs):
        xs, ys = xs[keep], ys[keep]
        slope, intercept = np.polyfit(ys, xs, 1)

    residual = float(np.std(xs - (slope * ys + intercept)))
    if residual > 0.08:
        return None
    return float(math.atan(slope))


def face(yaw, distance=0.9, n=120, half_angle=0.45):
    """Build a flat wall of constant world x, seen by a robot yawed by yaw.

    In the robot frame such a wall is the line x = distance / cos(yaw) + tan(yaw) * y,
    which is exactly where the fit reads the yaw from. A ray at bearing b meets it at
    range distance / cos(b + yaw).
    """
    points = []
    for i in range(n):
        bearing = -half_angle + 2 * half_angle * i / (n - 1)
        denominator = math.cos(bearing + yaw)
        if denominator <= 1e-3:
            continue   # the wall is edge-on or behind at this bearing
        r = distance / denominator
        points.append((r * math.cos(bearing), r * math.sin(bearing)))
    return points


@pytest.mark.parametrize("yaw_deg", [0.0, -10.0, -35.8, 12.0, 25.0])
def test_clean_face_recovers_the_yaw(yaw_deg):
    angle = fit(face(math.radians(yaw_deg)))
    assert angle is not None, "refused a clean face at %.1f deg" % yaw_deg
    assert abs(math.degrees(angle) - yaw_deg) < 3.0


def test_far_wall_beyond_the_shelf_is_ignored():
    """The exact failure: a near face plus returns from a wall 2.5 m further out."""
    points = face(math.radians(-35.8))
    # A third of the cone falls past the end of the shelf and lands on the far wall.
    points += [(3.6 + 0.02 * i, -0.9 + 0.03 * i) for i in range(40)]
    angle = fit(points)
    assert angle is not None, "the far wall still defeats the fit"
    assert abs(math.degrees(angle) + 35.8) < 5.0


def test_a_protruding_book_does_not_drag_the_answer():
    points = face(math.radians(-8.0))
    points += [(0.55, 0.02 * i) for i in range(6)]   # one book pulled forward
    angle = fit(points)
    assert angle is not None
    assert abs(math.degrees(angle) + 8.0) < 4.0


def test_two_surfaces_at_similar_range_are_still_refused():
    """The guard must survive: a genuine corner is not something to square against."""
    points = [(0.9, -0.6 + 0.01 * i) for i in range(60)]     # wall facing us
    points += [(0.9 + 0.012 * i, 0.0 + 0.01 * i) for i in range(60)]   # wall at 50 deg
    assert fit(points) is None


def test_too_few_returns_is_refused():
    assert fit([(0.9, 0.01 * i) for i in range(8)]) is None
