"""Ways to carry the book over the bin's rim when straight up is refused."""

import numpy as np
import pytest

from avaa_solution.deliver_node import (BIN_HALF_LENGTH, BIN_TO_TABLE_EDGE, BOOK_BELOW_GRIP,
                                        LIFT_FORWARD_MAX, lift_routes)

RIM = 0.8738
DEPTH = 0.21


def labels(routes):
    return [label for label, _ in routes]


def test_straight_up_is_tried_first_and_ends_over_the_bin():
    above = np.array([0.843, -0.066, 1.079])
    routes = lift_routes([0.38, 0.121, 0.95], above, 0.843, RIM, DEPTH)
    assert labels(routes)[0] == "up, then across"
    for _, points in routes:
        assert np.allclose(points[-1], above)


def test_forward_stops_short_of_the_bin_wall_when_the_foot_clears_the_table():
    # The laptop run: gripper at [0.38, 0.121], lift to 1.079 refused against the head.
    here = [0.38, 0.121, 0.95]
    assert here[2] - BOOK_BELOW_GRIP > RIM - DEPTH + 0.02
    routes = dict(lift_routes(here, np.array([0.843, -0.066, 1.079]), 0.843, RIM, DEPTH))
    forward = routes["forward, up, then across"][0]
    assert forward[0] == pytest.approx(min(LIFT_FORWARD_MAX, 0.843 - BIN_HALF_LENGTH - 0.05))
    assert forward[2] == pytest.approx(0.95)


def test_forward_stops_short_of_the_table_edge_when_the_foot_is_below_the_table_top():
    here = [0.30, 0.1, 0.70]
    routes = dict(lift_routes(here, np.array([0.843, 0.0, 1.079]), 0.843, RIM, DEPTH))
    forward = routes["forward, up, then across"][0]
    assert forward[0] == pytest.approx(0.843 - BIN_TO_TABLE_EDGE - 0.05)


def test_no_forward_route_when_already_as_far_forward_as_allowed():
    routes = lift_routes([0.60, 0.0, 0.95], np.array([0.9, 0.0, 1.079]), 0.9, RIM, DEPTH)
    assert "forward, up, then across" not in labels(routes)
