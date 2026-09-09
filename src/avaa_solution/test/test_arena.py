"""The one constant that joins base_link to the world, checked against the robot.

This is the test that was missing. base_link was taken to be 0.186 m above the floor for
the whole of this project's life and the URDF puts it at 0.0762, so every shelf row was
aimed 110 mm low -- and nothing caught it, because the tools that measured the miss
converted ground truth into base_link with the same wrong number as the code that aimed
the arm. Both sides moved together and the difference came out as zero.

A constant that appears on both sides of a comparison is never tested by that comparison.
So this one is checked against a source that cannot move with it: the robot description
itself.
"""

import os
import tokenize
import xml.etree.ElementTree as ET

import pytest

from avaa_solution import arena
from avaa_solution.grasp_node import DEFAULT_ROW_HEIGHTS
from avaa_solution.perception_node import BIN_RIM_BASE_Z, ROW_HEIGHTS_BASE

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.normpath(
    os.path.join(HERE, "..", "..", "erc_description", "urdf", "tiago_pro.urdf"))


def _base_footprint_joint_z():
    """Read how far base_link stands above the floor, from the URDF itself."""
    root = ET.parse(URDF).getroot()
    for joint in root.iter("joint"):
        child = joint.find("child")
        parent = joint.find("parent")
        origin = joint.find("origin")
        if child is None or parent is None or origin is None:
            continue
        if (child.get("link") == "base_link"
                and parent.get("link") == "base_footprint"):
            return float(origin.get("xyz").split()[2])
    return None


@pytest.mark.skipif(not os.path.exists(URDF), reason="robot description not present")
def test_base_link_height_matches_the_robot_description():
    urdf_z = _base_footprint_joint_z()
    assert urdf_z is not None, "no base_footprint -> base_link joint in the URDF"
    assert arena.BASE_LINK_Z == pytest.approx(urdf_z, abs=1e-4)


def test_rows_are_derived_rather_than_written_out():
    """Nobody may hardcode a row height that has drifted from the base height."""
    expected = [z - arena.BASE_LINK_Z for z in arena.ROW_HEIGHTS_WORLD]
    assert arena.ROW_HEIGHTS_BASE == pytest.approx(expected, abs=1e-4)


def test_every_node_agrees_about_the_rows():
    assert list(DEFAULT_ROW_HEIGHTS) == pytest.approx(arena.ROW_HEIGHTS_BASE)
    assert list(ROW_HEIGHTS_BASE) == pytest.approx(arena.ROW_HEIGHTS_BASE)


def test_rows_are_a_row_spacing_apart():
    gaps = [a - b for a, b in
            zip(arena.ROW_HEIGHTS_BASE, arena.ROW_HEIGHTS_BASE[1:])]
    for gap in gaps:
        assert gap == pytest.approx(arena.ROW_SPACING, abs=0.005)


def test_the_bin_rim_is_derived_from_the_same_base_height():
    assert BIN_RIM_BASE_Z == pytest.approx(
        arena.BIN_RIM_WORLD_Z - arena.BASE_LINK_Z, abs=1e-4)


def test_the_old_wrong_constant_is_gone_from_the_solution():
    """0.186 was the wrong floor-to-base_link height; it must not come back.

    Numbers only. The comments are free to keep saying what the old value was and what
    it cost, which is most of the reason the comments are there.
    """
    package = os.path.normpath(os.path.join(HERE, "..", "avaa_solution"))
    offenders = []
    for folder, _, files in os.walk(package):
        for name in sorted(files):
            if not name.endswith(".py"):
                continue
            with open(os.path.join(folder, name), "rb") as handle:
                for token in tokenize.tokenize(handle.readline):
                    if token.type != tokenize.NUMBER:
                        continue
                    try:
                        value = float(token.string)
                    except ValueError:
                        continue
                    if abs(value - 0.186) < 1e-9:
                        offenders.append("%s:%d" % (name, token.start[0]))
    assert not offenders, "0.186 is back in %s" % ", ".join(offenders)
