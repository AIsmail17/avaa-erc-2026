"""Tests for the mission's refusal of a verdict from a controller it never saw alive.

Two laptop trials on 2026-09-11 started, and their missions reported

    phase approach -> grasp at 0.2 s

while the approach node of the trial itself was still searching, 5 m from the
shelf, with no marker in view. What had happened: the controllers of a previous,
crashed run were still alive inside the container, frozen at their last state --
'done' -- and publishing it on their state topic every tick. The mission of the
new trial heard 'done' before its own controllers had said anything at all, and
obeyed.

No controller reaches 'done' in under a minute, and all of them publish their
state from the instant they start, so a terminal state as the FIRST state ever
heard from a controller cannot belong to this trial. The tests below hold the
mission to exactly that, and nothing more: a verdict from a controller seen alive
is obeyed, a verdict from one never heard from is not, and a working state from
either is always kept.
"""

from avaa_solution.mission_node import terminal_from_a_ghost


def test_a_done_from_a_controller_never_seen_alive_is_a_ghost():
    assert terminal_from_a_ghost("done", False)


def test_a_failed_from_a_controller_never_seen_alive_is_a_ghost():
    assert terminal_from_a_ghost("failed", False)


def test_a_done_from_a_controller_seen_alive_is_a_verdict():
    assert not terminal_from_a_ghost("done", True)


def test_a_working_state_from_a_controller_never_seen_alive_is_kept():
    # 'searching' is what a live approach says while it works. Refusing it would
    # stall a trial whose approach was merely quiet for a moment, which is why
    # only the terminal states are gated.
    assert not terminal_from_a_ghost("searching", False)
    assert not terminal_from_a_ghost("tucking", False)
    assert not terminal_from_a_ghost("", False)
