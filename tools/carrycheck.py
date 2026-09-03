#!/usr/bin/env python3
"""Where can the arm hold the book while the base drives, without hitting the robot?

    tools/in-sim carrycheck.py

deliver_node carries the book to [0.34, 0.10, 1.00] and there is nothing recorded about
where that came from. It matters more than an arbitrary point should: the collection bin
is across the arena, the drive is the longest motion in the run, and a book that fouls
the torso on the way is a dropped book and a lost point.

The planning scene has never known the robot is holding anything -- there is no attach in
the MoveIt client -- so every posture the delivery has ever validated was validated for an
empty gripper. This puts the book in the scene as a box where the gripper would be
holding it, and asks the same question again.

A contact between the book and the gripper's own pads is not a collision, it is a grasp,
so those pairs are allowed. Anything else is not.
"""
import sys
import time

import numpy as np
import rclpy

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.kinematics.arm_chain import ArmChain  # noqa: E402
from avaa_solution.moveit_client import MoveItClient  # noqa: E402

CHAIN = ["torso_lift_joint"] + ["arm_left_%d_joint" % i for i in range(1, 8)]
CARRY_APPROACH = [1.0, 0.0, 0.0]
CARRY_CLOSING = [0.0, 1.0, 0.0]

# The book as it hangs in the jaws, in metres, relative to the grasping frame. The
# gripper goes 110 mm past the book's face into a book 160 mm deep, so it stands 50 mm
# proud of the fingers and 110 mm behind them; it is 30 mm across the spine, and 250 mm
# tall gripped 45 mm below its centre.
BOOK_SIZE = (0.16, 0.03, 0.25)
BOOK_OFFSET = (-0.03, 0.0, 0.045)

# Contacts with these are the grasp itself, not a collision.
HOLDING = ("gripper_left", "arm_left_7")


def main():
    rclpy.init()
    client = MoveItClient("carrycheck")
    if not client.wait_until_ready(40.0):
        print("move_group not up")
        return 1
    chain = ArmChain.from_urdf()

    xs = [0.28, 0.34, 0.40, 0.46, 0.52]
    ys = [0.05, 0.10, 0.16, 0.22]
    zs = [0.85, 1.00, 1.15]

    print("book %s m, hanging %+.3f %+.3f %+.3f from the grasping frame\n"
          % (BOOK_SIZE, *BOOK_OFFSET))
    print("%-22s %-8s %-9s %s" % ("carry point", "verdict", "reach", "what it hits"))

    best = None
    for z in zs:
        for x in xs:
            for y in ys:
                point = np.array([x, y, z])
                solution = None
                for _ in range(14):
                    candidate = chain.ik(point, approach=CARRY_APPROACH,
                                         closing=CARRY_CLOSING)
                    if candidate is not None:
                        solution = candidate
                        break
                if solution is None:
                    print("%-22s %-8s" % (np.round(point, 2).tolist(), "no IK"))
                    continue

                where = chain.fk(solution)[:3, 3]
                centre = (float(where[0]) + BOOK_OFFSET[0],
                          float(where[1]) + BOOK_OFFSET[1],
                          float(where[2]) + BOOK_OFFSET[2])
                client.add_box("carried_book", "base_link", centre, BOOK_SIZE)
                time.sleep(0.25)

                valid = client.state_valid(CHAIN, solution)
                hits = client.why_invalid() if valid is False else ""
                # Contacts that are only the jaws holding the book are the grasp.
                real = valid is not False or not any(
                    tag in hits for tag in ("torso", "base", "head", "arm_right",
                                            "arm_left_1", "arm_left_2", "arm_left_3",
                                            "arm_left_4", "arm_left_5"))
                shoulder = chain.joint_origins(solution)[1]
                span = float(np.linalg.norm(where - shoulder))
                verdict = "clear" if real else "FOULS"
                print("%-22s %-8s %-9.3f %s"
                      % (np.round(point, 2).tolist(), verdict, span,
                         hits[:52] if hits else ""))
                if real and (best is None or span < best[0]):
                    best = (span, point, solution)
                client.remove_object("carried_book")
                time.sleep(0.15)

    print()
    if best is None:
        print("no carry point holds the book clear of the robot")
    else:
        span, point, solution = best
        print("tightest clear carry point: %s, arm spanning %.3f m"
              % (np.round(point, 3).tolist(), span))
        print("posture %s" % np.round(solution, 3).tolist())

    client.shutdown()
    rclpy.shutdown()
    return 0


sys.exit(main())
