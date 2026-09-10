#!/usr/bin/env python3
"""A carry posture with every joint kept inside its limits, not pressed against them.

    python3 carrypose2.py [margin_rad]

carrypose.py found the gripper at carry_point from the tuck, with the lowest arm link at
0.82 m and the book's bottom at 0.80 m -- clear of the base and the laser -- but with two joints
exactly on their limits, which a planner is entitled to refuse as a goal. This narrows every
joint's range by a margin and solves again, from the tuck-seeded answer and a few others, and
also tries carry points a little either side in case the exact one only exists on a limit.
"""
import sys

import numpy as np

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.kinematics.arm_chain import ArmChain   # noqa: E402

MARGIN = float(sys.argv[1]) if len(sys.argv) > 1 else 0.05
TUCK_POSE = [0.3877, -1.6152, 0.0717, 0.0408, 0.5074, -1.4708, 0.6063]
FOUND = [0.3328, 0.357, -2.4435, 2.618, -2.4435, -0.1261, -0.7759, 2.3912]
BOOK_BELOW_GRIP = 0.125 - 0.045


def narrow(chain, margin):
    touched = 0
    for joint in chain.moving:
        for lo_name, hi_name in (("lower", "upper"), ("low", "high"), ("min", "max")):
            if hasattr(joint, lo_name) and hasattr(joint, hi_name):
                lo, hi = getattr(joint, lo_name), getattr(joint, hi_name)
                if hi - lo > 2 * margin:
                    setattr(joint, lo_name, lo + margin)
                    setattr(joint, hi_name, hi - margin)
                    touched += 1
                break
    return touched


def describe(chain, full_chain, sol, target):
    pose = chain.fk(sol)
    grip = pose[:3, 3]
    origins = chain.joint_origins(sol)
    lowest = min(float(o[2]) for o in origins)
    widest = max(abs(float(o[1])) for o in origins)
    margin = min(min(v - lo, hi - v) for v, (lo, hi) in zip(sol, full_chain.limits))
    err = float(np.linalg.norm(grip - np.asarray(target)))
    return ("gripper (%.3f, %+.3f, %.3f) err %.0f mm  lowest link %.3f  widest |y| %.3f  "
            "book bottom %.3f  margin to the real limits %.1f deg"
            % (grip[0], grip[1], grip[2], err * 1000, lowest, widest,
               float(grip[2]) - BOOK_BELOW_GRIP - 0.125, np.degrees(margin)))


def main():
    full = ArmChain.from_urdf()
    chain = ArmChain.from_urdf()
    n = narrow(chain, MARGIN)
    print("narrowed %d joint ranges by %.2f rad" % (n, MARGIN))
    if n == 0:
        print("could not find limit attributes on the joints: %s" % dir(chain.moving[0]))
        return 1
    targets = [[0.34, 0.10, 1.00], [0.32, 0.12, 1.00], [0.30, 0.10, 1.05],
               [0.34, 0.14, 0.95], [0.28, 0.12, 1.10]]
    seeds = {"the limit-bound answer": FOUND, "the tuck, torso 0.30": [0.30] + TUCK_POSE,
             "unseeded": None}
    best = None
    for target in targets:
        for label, seed in seeds.items():
            sol = chain.ik(target, seed=seed, approach=[1.0, 0.0, 0.0], closing=[0.0, 1.0, 0.0])
            if sol is None:
                continue
            margin = min(min(v - lo, hi - v) for v, (lo, hi) in zip(sol, full.limits))
            origins = chain.joint_origins(sol)
            widest = max(abs(float(o[1])) for o in origins)
            lowest = min(float(o[2]) for o in origins)
            score = (margin >= MARGIN * 0.9, lowest > 0.6, -widest, margin)
            if best is None or score > best[0]:
                best = (score, target, label, sol)
            print("%-18s from %-24s %s" % (target, label, describe(chain, full, sol, target)))
    if best is None:
        print("no carry posture inside the narrowed limits")
        return 1
    _, target, label, sol = best
    print("")
    print("BEST: target %s from %s" % (target, label))
    print("      %s" % describe(chain, full, sol, target))
    print("      posture %s" % np.round(sol, 4).tolist())
    return 0


if __name__ == "__main__":
    sys.exit(main())
