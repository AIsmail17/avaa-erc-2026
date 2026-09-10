#!/usr/bin/env python3
"""Where sideways should the book be, for the arm rather than for the base?

    tools/in-sim aimsweep.py [grasp_x] [offsets...]

The approach stops with the book straight in front of the LEFT SHOULDER, 0.159 m to the
left of base_link, rather than in front of the base. That was measured once and it was a
large improvement -- the same grasp went from missing by 136 mm to missing by 8 -- but
0.159 is the shoulder's own offset, chosen by geometry rather than by asking the arm.
The shoulder is where the chain starts, not where it works best. A 7-DoF arm reaching
into a 0.30 m shelf opening has to keep its elbow out of the boards and its wrist square
to the book, and neither of those is necessarily happiest directly ahead of joint 1.

So this asks the arm, over the whole plausible range, at every row. Pure kinematics: the
same ArmChain and the same approach and closing directions the grasp itself solves with,
no simulator and no planner, so it answers quickly and answers about the arm alone.

Reported for each offset and row:

    ok       the IK converged with the wrist square -- position within 5 mm and both
             axes within 15 degrees
    margin   how close the tightest joint sits to its limit, in degrees. A solution
             pressed against a stop cannot be servoed afterwards, and the servo is what
             takes up the last 12 mm
    torque   the worst gravity load as a fraction of that joint's effort limit
    elbow    how far the widest arm link sits from the target's own line, sideways. The
             shelf opening is 0.30 m deep and the boards are 0.33 apart, so a link
             swinging wide is a link that meets a board
"""
import sys

import numpy as np

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution import arena                                 # noqa: E402
from avaa_solution.kinematics.arm_chain import ArmChain         # noqa: E402

GRASP_X = float(sys.argv[1]) if len(sys.argv) > 1 else 0.75
OFFSETS = [float(a) for a in sys.argv[2:]] or [
    -0.05, 0.00, 0.05, 0.10, 0.159, 0.20, 0.25, 0.30, 0.35]

APPROACH = [1.0, 0.0, 0.0]
CLOSING = [0.0, 1.0, 0.0]
BELOW_CENTRE = 0.045          # grasp_node's grasp_below_centre_m


def angle_between(a, b):
    a = np.asarray(a, float) / np.linalg.norm(a)
    b = np.asarray(b, float) / np.linalg.norm(b)
    return float(np.degrees(np.arccos(float(np.clip(a @ b, -1.0, 1.0)))))


def score(chain, target, seed=None):
    """Solve one grasp pose and describe how comfortable the solution is."""
    solution = chain.ik(target, seed=seed, approach=APPROACH, closing=CLOSING)
    if solution is None:
        return None
    pose = chain.fk(solution)
    position_error = float(np.linalg.norm(pose[:3, 3] - np.asarray(target, float)))
    approach_error = angle_between(pose[:3, 0], APPROACH)
    closing_axis = pose[:3, 1]
    sign = 1.0 if float(closing_axis @ np.asarray(CLOSING, float)) >= 0 else -1.0
    closing_error = angle_between(closing_axis, sign * np.asarray(CLOSING, float))

    margin = min(min(value - lo, hi - value)
                 for value, (lo, hi) in zip(solution, chain.limits))
    torques = chain.gravity_torque(solution)
    limits = chain.effort_limits()
    load = max(abs(t) / e for t, e in zip(torques, limits) if e > 0)

    origins = chain.joint_origins(solution)
    elbow = max(abs(float(o[1]) - target[1]) for o in origins)

    ok = (position_error <= 0.005 and approach_error <= 15.0
          and closing_error <= 15.0)
    return {"ok": ok, "solution": solution, "pos": position_error,
            "approach": approach_error, "closing": closing_error,
            "margin": margin, "load": load, "elbow": elbow}


def main():
    chain = ArmChain.from_urdf()
    print("grasp reach x = %.2f m, approach %s, closing %s" % (GRASP_X, APPROACH, CLOSING))
    print("rows in base_link: %s" % arena.ROW_HEIGHTS_BASE)
    print("")

    totals = {}
    for row_index, height in enumerate(arena.ROW_HEIGHTS_BASE, start=1):
        z = height - BELOW_CENTRE
        print("row %d, gripper z = %.3f" % (row_index, z))
        print("   offset    ok   pos mm  approach  closing   margin deg  torque  elbow")
        print("   " + "-" * 72)
        for offset in OFFSETS:
            target = [GRASP_X, offset, z]
            best = None
            # Several restarts, because the solver is seeded and a 7-DoF arm has more
            # than one way to stand. Take the most comfortable solution, not the first.
            for seed in (None, [0.0] * len(chain.joint_names)):
                got = score(chain, target, seed)
                if got is None:
                    continue
                if best is None or (got["ok"], got["margin"]) > (best["ok"], best["margin"]):
                    best = got
            if best is None:
                print("   %+6.3f    no solution" % offset)
                continue
            totals.setdefault(offset, []).append(best)
            print("   %+6.3f   %3s   %6.1f    %5.1f     %5.1f      %6.1f    %5.2f  %+6.3f"
                  % (offset, "yes" if best["ok"] else "NO",
                     best["pos"] * 1000, best["approach"], best["closing"],
                     np.degrees(best["margin"]), best["load"], best["elbow"]))
        print("")

    print("summary over all four rows")
    print("   offset   rows solved   worst margin deg   worst torque   worst elbow")
    print("   " + "-" * 70)
    for offset in OFFSETS:
        got = totals.get(offset, [])
        solved = sum(1 for g in got if g["ok"])
        if not got:
            print("   %+6.3f        0" % offset)
            continue
        print("   %+6.3f        %d/%d            %6.1f          %5.2f        %+6.3f"
              % (offset, solved, len(arena.ROW_HEIGHTS_BASE),
                 np.degrees(min(g["margin"] for g in got)),
                 max(g["load"] for g in got),
                 max(g["elbow"] for g in got)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
