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


def area_px(width_m, height_m, fill, depth):
    return pixels(width_m, depth) * pixels(height_m, depth) * fill


def test_a_flat_book_as_wide_as_its_diagonal_is_still_not_the_bin():
    # The third run: the box of a flat book seen from above reached 279 mm, and its
    # diagonal is 297. Its red area is at most its top face, 0.25 x 0.16.
    depth = 1.36
    w, h = pixels(0.29, depth), pixels(0.155, depth)
    area = area_px(0.25, 0.16, 1.0, depth)
    assert not bin_sized(w, h, depth, FX, FY, area)


def test_the_bin_from_its_short_end_passes_the_area_gate_too():
    depth = 1.5
    w, h = pixels(0.31, depth), pixels(0.21, depth)
    assert bin_sized(w, h, depth, FX, FY, area_px(0.31, 0.21, 0.9, depth))


def test_the_bin_from_the_front_passes_the_area_gate():
    depth = 2.0
    w, h = pixels(0.50, depth), pixels(0.21, depth)
    assert bin_sized(w, h, depth, FX, FY, area_px(0.50, 0.21, 0.85, depth))
