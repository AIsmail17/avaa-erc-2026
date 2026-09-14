"""The acquire checkpoint leaves small sideways offsets to the final drive's strafe."""

from avaa_solution.approach_node import acquire_arrived


def test_within_the_pose_tolerance():
    assert acquire_arrived(0.03, -0.03, 0.06)


def test_a_small_sideways_offset_is_left_to_the_strafe():
    # The laptop run: 10 mm behind, 80 mm to the side, chased into a 90 degree turn.
    assert acquire_arrived(-0.01, 0.08, 0.06)


def test_a_large_sideways_offset_is_still_driven():
    assert not acquire_arrived(0.0, 0.40, 0.06)


def test_far_from_the_pose_is_still_driven():
    assert not acquire_arrived(0.37, -0.23, 0.06)
