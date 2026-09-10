#!/usr/bin/env python3
"""Find a joint posture that carries a book clear of the base, for joint-space planning.

    python3 carrypose.py

Delivery walks a straight Cartesian line from wherever the grasp left the gripper to
carry_point, [0.34, 0.10, 1.00] in base_link, and every run the line was refused: from the
stow the gripper starts low and behind-left of the base, and the line climbs through the
robot's side. A posture reached by a planned joint motion does not need that line.

Pure kinematics, no simulator: solve for the gripper at carry_point with the jaws facing
forward, from a few seeds, over a range of torso heights, and report where the gripper and the
lowest arm link end up -- a carried book hangs 125 mm below the gripper and must clear the
0.30 m-high laser plane and the base footprint.
"""
import sys

import numpy as np

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.kinematics.arm_chain import ArmChain   # noqa: E402

CARRY_POINT = [0.34, 0.10, 1.00]
TUCK_POSE = [0.3877, -1.6152, 0.0717, 0.0408, 0.5074, -1.4708, 0.6063]
BOOK_BELOW_GRIP = 0.125 - 0.045
FOOTPRINT_HALF = (0.27, 0.27)


def main():
    chain = ArmChain.from_urdf()
    print("joints: %s" % chain.joint_names)
    seeds = {
        "from the tuck, torso 0.10": [0.10] + TUCK_POSE,
        "from the tuck, torso 0.25": [0.25] + TUCK_POSE,
        "unseeded": None,
        "zeros": [0.10] + [0.0] * 7,
    }
    for label, seed in seeds.items():
        sol = chain.ik(CARRY_POINT, seed=seed, approach=[1.0, 0.0, 0.0], closing=[0.0, 1.0, 0.0])
        if sol is None:
            print("%-28s no solution" % label)
            continue
        pose = chain.fk(sol)
        grip = pose[:3, 3]
        origins = chain.joint_origins(sol)
        lowest = min(float(o[2]) for o in origins)
        widest = max(abs(float(o[1])) for o in origins)
        book_bottom = float(grip[2]) - BOOK_BELOW_GRIP - 0.125
        margin = min(min(v - lo, hi - v) for v, (lo, hi) in zip(sol, chain.limits))
        print("%-28s gripper (%.3f, %+.3f, %.3f)  lowest link %.3f  widest |y| %.3f  "
              "book bottom %.3f  tightest joint margin %.0f deg"
              % (label, grip[0], grip[1], grip[2], lowest, widest, book_bottom, np.degrees(margin)))
        print("%-28s posture %s" % ("", np.round(sol, 4).tolist()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
