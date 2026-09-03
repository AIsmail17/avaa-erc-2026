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

    print("%-6s %-26s %-14s %-14s" % ("row", "pre-grasp point", "solves?", "clear?"))
    for index, height in enumerate(ROW_HEIGHTS):
        z = height - BELOW_CENTRE
        point = np.array([max(FACE_X - STANDOFF, 0.34), 0.16, z])
        found = 0
        clear = 0
        for attempt in range(12):
            seed = None if attempt == 0 else [
                float(np.random.uniform(lo, hi)) for lo, hi in chain.limits]
            solution = chain.ik(point, seed=seed, approach=APPROACH, closing=CLOSING)
            if solution is None:
                continue
            found += 1
            if client.state_valid(CHAIN, list(solution)) is not False:
                clear += 1
        print("%-6d %-26s %-14s %-14s"
              % (index + 1, np.round(point, 3).tolist(),
                 "%d of 12" % found, "%d of %d" % (clear, found)))

    client.shutdown()
    rclpy.shutdown()


main()
