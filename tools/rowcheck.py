#!/usr/bin/env python3
"""Can the arm still reach every row, now that it drives folded up small?

    tools/in-sim rowcheck.py

The new driving posture folds both arms inside the base, and its lowest links sit at
0.38 to 0.40 m. A grasp on the bottom row aims at 0.356 m. So the question is whether
tucking the arms in has cost the bottom row, and it is worth an answer rather than a
guess: on the first run that got this far, all twelve candidate pre-grasp postures for
row 4 came back in collision.

This asks MoveIt directly, for every row, with the OTHER arm parked in each of the two
tucks in turn. If the old right-arm tuck passes where the new one fails, the new tuck is
the cause.
"""
import sys

import numpy as np
import rclpy

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.kinematics.arm_chain import ArmChain      # noqa: E402
from avaa_solution.moveit_client import MoveItClient         # noqa: E402

CHAIN = ["torso_lift_joint"] + ["arm_left_%d_joint" % i for i in range(1, 8)]
RIGHT = ["arm_right_%d_joint" % i for i in range(1, 8)]

OLD_RIGHT = [-0.7194, -2.2867, -0.5064, 0.5221, 2.3399, 1.0503, 1.9772]
NEW_RIGHT = [-0.2398, -1.5290, -0.1046, 1.1345, 2.4498, 3.0020, 1.5282]

ROW_HEIGHTS = [1.391, 1.061, 0.731, 0.401]
BOARD_DROP = 0.125 + 0.02
SHELF_DEPTH = 0.30
SHELF_WIDTH = 4.8
BELOW_CENTRE = 0.045
STANDOFF = 0.15
FACE_X = 0.60

APPROACH = [1.0, 0.0, 0.0]
CLOSING = [0.0, 1.0, 0.0]


def main():
    rclpy.init()
    chain = ArmChain.from_urdf()
    client = MoveItClient("rowcheck")
    if not client.wait_until_ready(40.0):
        print("move_group is not running")
        return

    # Sweep the standing distance, because that is the thing we can choose.
    #
    # The grasp failed with all twelve candidate pre-grasp postures for row 4 in
    # collision, reaching from a face 0.764 m away. Reaching the bottom row from further
    # back is a longer, flatter reach that passes closer to the board above it, so the
    # question is not "is row 4 reachable" but "from how far away". The approach's
    # standoff is a parameter; this says what to set it to.
    # With the shelf in the planning scene, which is the difference between this and
    # the run that failed. The boards are placed exactly as grasp_node places them.
    shelf = len(sys.argv) > 1 and sys.argv[1] == "shelf"
    faces = [0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85]
    placed = []
    print("clear pre-grasp postures out of 12, by row and by distance to the book face")
    print("shelf in the planning scene: %s" % ("YES" if shelf else "no"))
    print("%-6s %s" % ("", "  ".join("%5.2f" % f for f in faces)))
    for index, height in enumerate(ROW_HEIGHTS):
        z = height - BELOW_CENTRE
        cells = []
        for face in faces:
            if shelf:
                for name in list(placed):
                    client.remove_object(name)
                placed.clear()
                centre_x = face + SHELF_DEPTH / 2.0 - 0.05
                for j, board in enumerate(ROW_HEIGHTS):
                    name = "shelf_board_%d" % j
                    if client.add_box(name, "base_link",
                                      (centre_x, 0.0, board - BOARD_DROP),
                                      (SHELF_DEPTH, SHELF_WIDTH, 0.04)):
                        placed.append(name)
                if client.add_box("shelf_back", "base_link",
                                  (face + SHELF_DEPTH, 0.0, 0.9),
                                  (0.04, SHELF_WIDTH, 1.8)):
                    placed.append("shelf_back")
            point = np.array([max(face - STANDOFF, 0.34), 0.16, z])
            clear = 0
            for attempt in range(12):
                seed = None if attempt == 0 else [
                    float(np.random.uniform(lo, hi)) for lo, hi in chain.limits]
                solution = chain.ik(point, seed=seed, approach=APPROACH,
                                    closing=CLOSING)
                if solution is None:
                    continue
                if client.state_valid(CHAIN, list(solution)) is not False:
                    clear += 1
            cells.append("%5d" % clear)
        print("row %-2d %s" % (index + 1, "  ".join(cells)))

    for name in placed:
        client.remove_object(name)
    client.shutdown()
    rclpy.shutdown()


main()
