"""The whole-shelf marker order is decided by a vote, not by the first frame that qualifies.

On 2026-09-10 one oblique frame read the plates as [1, 5, 3, 4, 2] when they were
[5, 3, 1, 4, 2], passed every check -- five confident markers, a permutation -- and was
latched for the rest of the run, reporting marker 3 on the wrong column. These pin down
the rule that replaced it.
"""

from collections import Counter

from avaa_solution.perception_node import shelf_order_winner

TRUE = (5, 3, 1, 4, 2)
MISREAD = (1, 5, 3, 4, 2)


def test_nothing_read_decides_nothing():
    assert shelf_order_winner(Counter()) is None


def test_a_single_clean_reading_answers_provisionally():
    # The fourth full run saw the whole shelf so rarely that three agreeing readings never
    # came, and no column was reported at all. One reading now gives an answer.
    assert shelf_order_winner(Counter({TRUE: 1})) == TRUE


def test_a_single_misread_is_overturned_once_the_truth_is_read():
    tally = Counter({MISREAD: 1})
    assert shelf_order_winner(tally) == MISREAD
    tally[TRUE] += 3
    assert shelf_order_winner(tally) == TRUE


def test_three_agreeing_readings_decide_it():
    assert shelf_order_winner(Counter({TRUE: 3})) == TRUE


def test_one_bad_frame_does_not_stop_a_clear_answer():
    assert shelf_order_winner(Counter({MISREAD: 1, TRUE: 3})) == TRUE


def test_a_split_vote_decides_nothing():
    assert shelf_order_winner(Counter({MISREAD: 3, TRUE: 3})) is None


def test_a_later_majority_takes_over():
    tally = Counter({MISREAD: 3})
    assert shelf_order_winner(tally) == MISREAD
    tally[TRUE] += 8
    assert shelf_order_winner(tally) == TRUE
