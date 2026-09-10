"""The shelf's plate order is pieced together from partial views, not waited for whole.

On 2026-09-10 one oblique frame read the plates as [1, 5, 3, 4, 2] when they were
[5, 3, 1, 4, 2] and was latched; the vote that replaced it then never concluded on the
fifth run, which never once had all five plates in one frame. These pin down the rule that
replaced both: runs of plates seen together, and the one order most of them agree with.
"""

from collections import Counter
from types import SimpleNamespace

from avaa_solution.perception_node import (
    contiguous_run,
    order_from_windows,
    shelf_order_winner,
)

TRUE = (5, 3, 1, 4, 2)
MISREAD = (1, 5, 3, 4, 2)


def plate(digit, cx, width=40, confident=True):
    return SimpleNamespace(digit=digit, x=int(cx - width / 2), w=width, cx=float(cx),
                           confident=confident)


def test_nothing_seen_decides_nothing():
    assert order_from_windows(Counter()) is None


def test_one_whole_shelf_frame_answers():
    assert order_from_windows(Counter({TRUE: 1})) == TRUE


def test_overlapping_partial_views_pin_the_order():
    assert order_from_windows(Counter({(5, 3, 1, 4): 1, (3, 1, 4, 2): 1})) == TRUE


def test_a_run_that_leaves_an_end_open_decides_nothing():
    # [3, 1, 4] could have 5 and 2 either way round it.
    assert order_from_windows(Counter({(3, 1, 4): 4})) is None


def test_the_misread_is_outvoted_by_the_frames_that_read_it_right():
    tally = Counter({MISREAD: 1, (5, 3, 1, 4): 2, (3, 1, 4, 2): 2})
    assert order_from_windows(tally) == TRUE


def test_the_fifth_run_order_from_its_frames():
    # 1 sliced by the border was dropped, leaving [2, 3, 5]; the shelf was [4, 1, 2, 3, 5].
    tally = Counter({(2, 3, 5): 3, (4, 1, 2, 3): 2})
    assert order_from_windows(tally) == (4, 1, 2, 3, 5)


def test_a_plate_on_the_border_is_dropped_not_the_frame():
    plates = [plate(1, 15), plate(2, 170), plate(3, 360), plate(5, 550)]
    plates[0].x = 0
    assert contiguous_run(plates, 640) == (2, 3, 5)


def test_a_missed_plate_in_the_middle_breaks_the_run():
    plates = [plate(2, 100), plate(3, 250), plate(4, 550)]
    assert contiguous_run(plates, 640) is None


def test_foreshortened_plates_seen_from_the_side_are_still_a_run():
    # Gaps of 90, 120 and 160 px: each neighbour within a third of the last, but the
    # extremes 1.78 apart -- which the old rule refused.
    plates = [plate(4, 100), plate(2, 190), plate(5, 310), plate(3, 470)]
    assert contiguous_run(plates, 640) == (4, 2, 5, 3)


def test_too_few_whole_plates_is_not_a_run():
    assert contiguous_run([plate(2, 200), plate(3, 350)], 640) is None


def test_the_old_whole_order_rule_still_agrees():
    assert shelf_order_winner(Counter({TRUE: 3})) == TRUE
