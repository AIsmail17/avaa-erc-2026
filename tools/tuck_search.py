#!/usr/bin/env python3
"""Find the most compact DRIVING posture that is still collision free.

    tools/in-sim tuck_search.py [left|right] [seconds]

The posture the robot drives in was chosen for being collision free, and it is not
compact. Measured from TF with the robot standing tucked:

    arm_left_4_link          x=+0.449   179 mm in FRONT of the base
    gripper_right_base_link  x=-0.516   246 mm BEHIND it

The base is 0.54 m across. An elbow 180 mm proud of it leads the robot into every shelf
edge it drives past, and a hand trailing 250 mm behind catches on the table on the way
out. Collision-free does not help with either: that check is against the robot itself,
and the shelf is not part of the robot.

Searching for compactness alone does not work. Every one of the 400 zero-overhang
postures a random search produced was rejected by MoveIt, and rightly: the torso stands
in the middle of the base, so "inside the footprint" and "inside the robot" are nearly
the same volume. The arm has to live in the shell between them.

So this hill-climbs from the posture in use, which is known good, and accepts a step only
if it is both more compact AND still collision free. It cannot return anything worse than
what the robot does today, and every answer it gives has been checked.
"""
import sys
import time

import numpy as np
import rclpy

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.kinematics.arm_chain import ArmChain      # noqa: E402
from avaa_solution.moveit_client import MoveItClient         # noqa: E402

HALF = 0.27
MARGIN = 0.02
MIN_Z = 0.35
TORSO = 0.15

CURRENT = {
    "left": [2.1521, 0.3824, 1.2785, -2.1517, 0.8325, 0.1926, 1.3944],
    "right": [-0.7194, -2.2867, -0.5064, 0.5221, 2.3399, 1.0503, 1.9772],
}


def chain_for(side):
    for tip in ("gripper_%s_grasping_link" % side, "gripper_%s_base_link" % side):
        try:
            return ArmChain.from_urdf(tip=tip)
        except Exception:  # noqa: BLE001
            continue
    raise SystemExit("no chain for the %s arm" % side)


def overhang(chain, values):
    """The worst distance any point on the arm reaches outside the base footprint."""
    origins = list(chain.joint_origins(values))
    worst = 0.0
    for a, b in zip(origins, origins[1:]):
        for t in (0.0, 0.25, 0.5, 0.75, 1.0):
            p = a + t * (b - a)
            worst = max(worst,
                        abs(p[0]) - (HALF - MARGIN),
                        abs(p[1]) - (HALF - MARGIN))
            if p[2] < MIN_Z:
                worst += (MIN_Z - p[2])
    return max(worst, 0.0)


def main():
    side = sys.argv[1] if len(sys.argv) > 1 else "left"
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0

    rclpy.init()
    chain = chain_for(side)
    names = list(chain.joint_names)
    limits = list(chain.limits)
    torso_at = [i for i, n in enumerate(names) if "torso" in n]

    client = MoveItClient("tuck_search")
    if not client.wait_until_ready(40.0):
        print("move_group is not running; start it with moveit.launch.py")
        return
    group = "arm_left_torso" if side == "left" else "arm_right_torso"

    def fix(values):
        out = [float(np.clip(v, lo, hi)) for v, (lo, hi) in zip(values, limits)]
        for i in torso_at:
            out[i] = TORSO
        return out

    def valid(values):
        return client.state_valid(names, list(values), group=group) is not False

    best = fix(([TORSO] if torso_at else []) + list(CURRENT[side]))
    if not valid(best):
        print("the posture in use is reported invalid; not trusting the check")
        return
    best_over = overhang(chain, best)
    print("%s arm: starting at %.0f mm outside the base" % (side, best_over * 1000))

    rng = np.random.default_rng(11)
    end = time.time() + seconds
    steps = 0
    accepted = 0
    while time.time() < end and best_over > 0.0:
        steps += 1
        scale = rng.choice([0.03, 0.10, 0.30, 0.80])
        # Move a few joints at a time, not all of them: a posture pressed against the
        # collision boundary is easier to improve one axis at a time.
        candidate = list(best)
        for index in rng.choice(len(candidate), size=rng.integers(1, 4), replace=False):
            if index in torso_at:
                continue
            candidate[index] += rng.normal(0.0, scale)
        candidate = fix(candidate)
        over = overhang(chain, candidate)
        if over >= best_over:
            continue
        if not valid(candidate):
            continue
        best, best_over = candidate, over
        accepted += 1
        print("  %5.0f mm  (step %d)" % (best_over * 1000, steps), flush=True)

    joints = [v for i, v in enumerate(best) if i not in torso_at]
    print()
    print("%s arm: %.0f mm outside the base after %d steps (%d accepted)"
          % (side, best_over * 1000, steps, accepted))
    print("%s_TUCK = [%s]" % (side.upper(), ", ".join("%.4f" % v for v in joints)))

    origins = list(chain.joint_origins(best))
    worst = max(origins, key=lambda p: max(abs(p[0]), abs(p[1])))
    print("furthest joint x=%+.3f y=%+.3f z=%+.3f" % tuple(worst))
    print("height range %.2f .. %.2f m"
          % (min(p[2] for p in origins), max(p[2] for p in origins)))

    client.shutdown()
    rclpy.shutdown()


main()
