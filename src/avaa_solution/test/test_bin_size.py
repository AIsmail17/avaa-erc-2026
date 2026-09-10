"""A red blob is only the bin if it is bin-sized in metres, not just bin-shaped in pixels.

On 2026-09-10 delivery drove at a red book lying on its side on the shelf, reported as
"bin in view at 1.08 m". A book has one dimension over 160 mm and the bin has none under
210 mm, so size in metres tells them apart from any side.
"""

from avaa_solution.perception_node import bin_sized

FX = FY = 337.2096   # the head camera, both streams


def pixels(metres, depth):
    return metres * FX / depth


def test_the_bin_from_the_front_is_the_bin():
    depth = 2.0
    assert bin_sized(pixels(0.50, depth), pixels(0.21, depth), depth, FX, FY)


def test_the_bin_from_its_short_end_is_still_the_bin():
    depth = 1.5
    assert bin_sized(pixels(0.31, depth), pixels(0.21, depth), depth, FX, FY)


def test_a_book_lying_flat_is_not_the_bin():
    depth = 0.75
    assert not bin_sized(pixels(0.25, depth), pixels(0.03, depth), depth, FX, FY)


def test_a_book_standing_up_is_not_the_bin():
    depth = 0.75
    assert not bin_sized(pixels(0.03, depth), pixels(0.25, depth), depth, FX, FY)


def test_a_book_seen_end_on_and_close_is_not_the_bin():
    depth = 0.40
    assert not bin_sized(pixels(0.16, depth), pixels(0.25, depth), depth, FX, FY)


def test_no_depth_is_not_a_bin():
    assert not bin_sized(200, 100, 0.0, FX, FY)
