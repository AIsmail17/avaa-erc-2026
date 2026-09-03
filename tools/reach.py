#!/usr/bin/env python3
"""How far out is the arm holding itself, and how close is that to all it has?

    tools/in-sim reach.py [standoff ...]

The last run reached the pre-grasp to within two millimetres, opened the gripper, planned
nine waypoints into the shelf, and then sagged 445 mm and could not climb back. The line
that explained it was printed three minutes earlier: "worst joint at 101% of rated".

The torque estimate is not the thing to gate on -- the code says why, it reads a quarter
of the load on one joint and six times it on another. But there is a geometric quantity
underneath that is not in doubt: how far the target is from the SHOULDER, against how far
the shoulder can reach at all. A joint holds the weight of everything beyond it times how
far out that weight sits, and at full extension both the lever and the Jacobian are at
their worst at once -- which is also why 200 IK solves got rejected afterwards.

So this measures the maximum reach numerically, then prints, for every row and every
standoff, the fraction of it the grasp would use.
"""
import math
import sys

import numpy as np

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.kinematics.arm_chain import ArmChain  # noqa: E402

ROWS = [1.391, 1.061, 0.731, 0.401]
SHOULDER_BASE_Z = 0.677
BOOK_HALF_DEPTH = 0.08
SHOULDER_Y = 0.159
GRASP_DEPTH = 0.11


def shoulder_of(chain, values):
    """Where the arm hangs from: the second moving joint, after the torso lift."""
    return chain.joint_origins(values)[1]


def max_reach(chain, torso, tries=4000):
    """Greatest shoulder-to-tip distance, found by sampling then hill climbing.

    Sampled rather than derived because the link offsets are not all collinear -- the
    sum of the link lengths is an upper bound the arm cannot actually attain.
    """
    rng = np.random.default_rng(0)
    limits = chain.limits
    best = (0.0, None)
    for _ in range(tries):
        values = [torso] + [rng.uniform(lo, hi) for lo, hi in limits[1:]]
        origins = chain.joint_origins(values)
        d = float(np.linalg.norm(origins[-1] - origins[1]))
        if d > best[0]:
            best = (d, values)
    values = list(best[1])
    step = 0.4
    while step > 1e-3:
        improved = False
        for i in range(1, len(values)):
            for delta in (step, -step):
                trial = list(values)
                lo, hi = limits[i]
                trial[i] = float(np.clip(trial[i] + delta, lo, hi))
                origins = chain.joint_origins(trial)
                d = float(np.linalg.norm(origins[-1] - origins[1]))
                if d > best[0]:
                    best = (d, trial)
                    values = trial
                    improved = True
        if not improved:
            step *= 0.5
    return best[0], best[1]


def main():
    standoffs = [float(v) for v in sys.argv[1:]] or [
        0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85]
    chain = ArmChain.from_urdf()

    print("torso range %.2f to %.2f m\n" % tuple(chain.limits[0]))
    for torso in (0.0, 0.175, 0.35):
        reach, _ = max_reach(chain, torso)
        origins = chain.joint_origins([torso] + [0.0] * 7)
        print("torso %.3f: shoulder at %s, max reach %.3f m"
              % (torso, np.round(origins[1], 3).tolist(), reach))
    reach, _ = max_reach(chain, 0.35)
    print()

    print("fraction of maximum reach used, by row and standoff")
    print("(standoff is base_link to the book's centre; the grasp point is %.2f m"
          " past the face)\n" % GRASP_DEPTH)
    print("%-9s %-8s %-8s %-8s %-8s %-8s" %
          ("standoff", "torso", "row 1", "row 2", "row 3", "row 4"))
    for standoff in standoffs:
        face_x = standoff - BOOK_HALF_DEPTH
        grasp_x = face_x + GRASP_DEPTH
        marks = []
        torsos = []
        for height in ROWS:
            torso = float(np.clip(height - SHOULDER_BASE_Z + 0.25, 0.0, 0.35))
            torsos.append(torso)
            shoulder = shoulder_of(chain, [torso] + [0.0] * 7)
            reach_t, _ = max_reach(chain, torso)
            need = math.dist(
                [grasp_x, SHOULDER_Y, height],
                [float(shoulder[0]), float(shoulder[1]), float(shoulder[2])])
            marks.append("%.0f%%" % (100.0 * need / reach_t))
        print("%-9.2f %-8s %-8s %-8s %-8s %-8s"
              % (standoff, "/".join("%.2f" % t for t in torsos[:1] * 1), *marks))

    print("\nthe run that sagged stood at a face distance of 0.825 m, row 1")
    torso = float(np.clip(ROWS[0] - SHOULDER_BASE_Z + 0.25, 0.0, 0.35))
    shoulder = shoulder_of(chain, [torso] + [0.0] * 7)
    reach_t, _ = max_reach(chain, torso)
    need = math.dist([0.935, 0.2, 1.346],
                     [float(shoulder[0]), float(shoulder[1]), float(shoulder[2])])
    print("  target [0.935, 0.2, 1.346], torso pinned to %.3f, shoulder z %.3f"
          % (torso, float(shoulder[2])))
    print("  the target is %.3f m ABOVE the shoulder, not below it"
          % (1.346 - float(shoulder[2])))
    print("  shoulder to target %.3f m of a possible %.3f: %.0f%%"
          % (need, reach_t, 100.0 * need / reach_t))


main()
