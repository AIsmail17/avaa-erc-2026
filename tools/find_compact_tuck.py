#!/usr/bin/env python3
"""Find driving postures that keep the arms INSIDE the base.

    tools/in-sim find_compact_tuck.py [left|right|both] [seconds]

The existing driving posture is collision free, which is what it was chosen for, and it
is not compact. Measured from TF with the robot standing tucked:

    arm_left_4_link          x=+0.449   179 mm in FRONT of the base
    gripper_right_base_link  x=-0.516   246 mm BEHIND it

The base is 0.54 m across, so an elbow 180 mm proud of it leads the robot into every
shelf edge it drives past, and a hand trailing 250 mm behind catches the table on the way
out. Being collision free in an empty world does not help: the check is against the robot
itself, and the shelf is not part of the robot.

So this searches for a posture that is inside the footprint instead. The cost is the
worst overhang of any point along any link -- sampled along the segments, not just at the
joint origins, because a link's middle sticks out further than its ends -- with a floor
under the height so the arm does not fold down through the base, and a preference for
keeping the hand close in.

Analytic FK only, no simulator needed for the search. The winner still has to be tried in
the simulator, which tools/armcheck.py and tools/footprint.py will judge.
"""
import math
import sys
import time

import numpy as np

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.kinematics.arm_chain import ArmChain  # noqa: E402

# The TIAGo Pro base, and a little margin: a link exactly on the edge still catches.
HALF = 0.27
MARGIN = 0.02

# Below this the arm is folding into the base itself; above it, into the head.
MIN_Z = 0.35
MAX_Z = 1.30

TORSO = 0.15

CURRENT = {
    "left": [2.1521, 0.3824, 1.2785, -2.1517, 0.8325, 0.1926, 1.3944],
    "right": [-0.7194, -2.2867, -0.5064, 0.5221, 2.3399, 1.0503, 1.9772],
}

TIPS = {
    "left": "gripper_left_grasping_link",
    "right": "gripper_right_grasping_link",
}


def chain_for(side):
    for tip in (TIPS[side], "gripper_%s_base_link" % side, "arm_%s_7_link" % side):
        try:
            return ArmChain.from_urdf(tip=tip), tip
        except Exception:  # noqa: BLE001 - try the next candidate tip
            continue
    raise SystemExit("no usable chain for the %s arm" % side)


def sampled_points(chain, values):
    """Points along every link, not merely the joint origins."""
    origins = list(chain.joint_origins(values))
    points = []
    for a, b in zip(origins, origins[1:]):
        for t in (0.0, 0.25, 0.5, 0.75, 1.0):
            points.append(a + t * (b - a))
    if origins:
        points.append(origins[-1])
    return points


def cost(chain, values):
    """Worst overhang in metres, plus penalties for folding into the robot."""
    points = sampled_points(chain, values)
    worst = 0.0
    penalty = 0.0
    for p in points:
        over = max(abs(p[0]) - (HALF - MARGIN), abs(p[1]) - (HALF - MARGIN))
        worst = max(worst, over)
        if p[2] < MIN_Z:
            penalty += (MIN_Z - p[2]) * 3.0
        if p[2] > MAX_Z:
            penalty += (p[2] - MAX_Z) * 3.0
    # A mild pull towards keeping the hand near the body, to break ties sensibly.
    tip = points[-1]
    tidy = 0.05 * float(np.linalg.norm(tip[:2]))
    return worst + penalty + tidy, worst


def search(side, seconds):
    chain, tip = chain_for(side)
    names = chain.joint_names
    limits = list(chain.limits)
    print("%s arm: %d joints to %s" % (side, len(names), tip))

    # The torso is pinned: it is where it is for driving, and moving it changes the
    # reach of everything above it.
    torso_index = [i for i, n in enumerate(names) if "torso" in n]

    def fix(values):
        out = list(values)
        for i in torso_index:
            out[i] = TORSO
        return [float(np.clip(v, lo, hi)) for v, (lo, hi) in zip(out, limits)]

    start = list(CURRENT[side])
    if torso_index:
        start = [TORSO] + start
    start = fix(start)
    best = fix(start)
    best_cost, best_over = cost(chain, best)
    print("  starting overhang %.0f mm" % (best_over * 1000))

    rng = np.random.default_rng(7)
    end = time.time() + seconds
    tries = 0
    shortlist = []
    while time.time() < end:
        tries += 1
        # Half the effort refining the best so far, half from a fresh random posture,
        # so the search neither sticks in the first dip nor forgets what it found.
        if tries % 2 == 0:
            scale = rng.choice([0.05, 0.2, 0.6])
            candidate = fix([v + rng.normal(0.0, scale) for v in best])
        else:
            candidate = fix([rng.uniform(lo, hi) for lo, hi in limits])
        value, over = cost(chain, candidate)
        if over <= 0.16:
            shortlist.append((over, tuple(candidate)))
        if value < best_cost:
            best_cost, best_over, best = value, over, candidate

    print("  %d candidates, best overhang %.0f mm (geometry only)"
          % (tries, best_over * 1000))

    # Geometry alone will fold the arm straight through the torso.
    #
    # The base is 0.54 m across and the torso stands in the middle of it, so "inside the
    # footprint" and "inside the robot" are nearly the same region. The first search run
    # returned a posture whose every point sat within 0.25 m of the mast between z=0.77
    # and z=1.07, which is exactly where the torso is. So the shortlist is re-ranked by
    # asking MoveIt, which checks the whole robot rather than the one arm, and the best
    # posture that is actually valid wins.
    valid = None
    if shortlist:
        try:
            from avaa_solution.moveit_client import MoveItClient
            client = MoveItClient("compact_tuck_check")
            if client.wait_until_ready(30.0):
                names_no_torso = [n for i, n in enumerate(names) if i not in torso_index]
                checked = 0
                for over, candidate in sorted(shortlist)[:400]:
                    if side != "left":
                        break  # no planning group for the right arm; verified in sim
                    checked += 1
                    if client.state_valid(names, list(candidate)) is not False:
                        valid = (over, candidate)
                        break
                print("  checked %d shortlisted postures against MoveIt" % checked)
                _ = names_no_torso
            else:
                print("  move_group is not up; skipping the collision check")
            client.shutdown()
        except Exception as exc:  # noqa: BLE001 - the search is still useful without it
            print("  collision check unavailable (%r)" % exc)

    if valid is not None:
        best_over, best = valid
        print("  best COLLISION-FREE overhang %.0f mm" % (best_over * 1000))

    joints = [v for i, v in enumerate(best) if i not in torso_index]
    print("  %s_TUCK = [%s]"
          % (side.upper(), ", ".join("%.4f" % v for v in joints)))
    points = sampled_points(chain, best)
    hi = max(points, key=lambda p: max(abs(p[0]), abs(p[1])))
    print("  furthest point x=%+.3f y=%+.3f z=%+.3f" % tuple(hi))
    print("  height range %.2f .. %.2f m"
          % (min(p[2] for p in points), max(p[2] for p in points)))
    return joints, best_over


def main():
    import rclpy
    rclpy.init()
    which = sys.argv[1] if len(sys.argv) > 1 else "both"
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0
    sides = ("left", "right") if which == "both" else (which,)
    for side in sides:
        search(side, seconds)
        print()
    rclpy.shutdown()


main()
