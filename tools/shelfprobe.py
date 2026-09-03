#!/usr/bin/env python3
"""Which part of the shelf model blocks the pre-grasp?

    tools/in-sim shelfprobe.py

With the shelf in the planning scene, every candidate pre-grasp posture came back in
collision at every close standing distance, while against the robot alone nearly all of
them were clear. So the shelf model is doing it, and the question is which part and by
how much.

grasp_node describes the shelf as one slab per row:

    x from face - 0.05 to face + 0.25       (0.30 deep, pushed 50 mm IN FRONT of the
                                             book face)
    y from -2.4 to +2.4                     (4.8 m wide)
    z the board, 40 mm thick

This varies the front edge and the width in turn, and counts how many of twelve postures
survive each, so the cost of each modelling choice is a number rather than an opinion.
"""
import sys

import numpy as np
import rclpy

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.kinematics.arm_chain import ArmChain      # noqa: E402
from avaa_solution.moveit_client import MoveItClient         # noqa: E402

CHAIN = ["torso_lift_joint"] + ["arm_left_%d_joint" % i for i in range(1, 8)]
ROW_HEIGHTS = [1.391, 1.061, 0.731, 0.401]
BOARD_DROP = 0.145
SHELF_DEPTH = 0.30
BELOW_CENTRE = 0.045
STANDOFF = 0.15
APPROACH = [1.0, 0.0, 0.0]
CLOSING = [0.0, 1.0, 0.0]

FACE = 0.70


def build(client, front_offset, width, placed):
    for name in list(placed):
        client.remove_object(name)
    placed.clear()
    centre_x = FACE + SHELF_DEPTH / 2.0 + front_offset
    for j, board in enumerate(ROW_HEIGHTS):
        name = "shelf_board_%d" % j
        if client.add_box(name, "base_link",
                          (centre_x, 0.0, board - BOARD_DROP),
                          (SHELF_DEPTH, width, 0.04)):
            placed.append(name)
    if client.add_box("shelf_back", "base_link",
                      (FACE + SHELF_DEPTH, 0.0, 0.9), (0.04, width, 1.8)):
        placed.append("shelf_back")


def count_clear(client, chain, z):
    point = np.array([max(FACE - STANDOFF, 0.34), 0.16, z])
    clear = 0
    for attempt in range(12):
        seed = None if attempt == 0 else [
            float(np.random.uniform(lo, hi)) for lo, hi in chain.limits]
        solution = chain.ik(point, seed=seed, approach=APPROACH, closing=CLOSING)
        if solution is None:
            continue
        if client.state_valid(CHAIN, list(solution)) is not False:
            clear += 1
    return clear


def main():
    rclpy.init()
    chain = ArmChain.from_urdf()
    client = MoveItClient("shelfprobe")
    if not client.wait_until_ready(40.0):
        print("move_group is not running")
        return
    placed = []

    print("book face at %.2f m, pre-grasp at %.2f m\n" % (FACE, FACE - STANDOFF))
    print("clear postures out of 12, by row\n")
    print("%-34s %5s %5s %5s %5s" % ("shelf model", "row1", "row2", "row3", "row4"))

    cases = [
        ("as shipped: front -0.05, width 4.8", -0.05, 4.8),
        ("front at the face, width 4.8", 0.00, 4.8),
        ("front 50 mm behind, width 4.8", 0.05, 4.8),
        ("as shipped, but only 1.0 m wide", -0.05, 1.0),
        ("front at face, only 1.0 m wide", 0.00, 1.0),
        ("no shelf at all", None, None),
    ]
    for label, offset, width in cases:
        if offset is None:
            for name in list(placed):
                client.remove_object(name)
            placed.clear()
        else:
            build(client, offset, width, placed)
        cells = [count_clear(client, chain, h - BELOW_CENTRE) for h in ROW_HEIGHTS]
        print("%-34s %5d %5d %5d %5d" % (label, *cells))

    for name in placed:
        client.remove_object(name)
    client.shutdown()
    rclpy.shutdown()


main()
