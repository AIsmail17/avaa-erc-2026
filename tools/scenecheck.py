#!/usr/bin/env python3
"""Does taking a box out of the planning scene actually take it out?

    tools/in-sim scenecheck.py

The grasp puts the shelf into the scene for the big unfolding move, takes it out again at
the pre-grasp, and then walks the reach in waypoint by waypoint. On one run that reach
came back "obstructed 12% of the way in" 41 milliseconds after the removal was logged --
with the shelf gone, the only thing left to collide with is the robot itself, which makes
a self-collision at the very first waypoint of a 200 mm straight line surprising.

So this tests the mechanism rather than the geometry. Put a box exactly where the arm is,
confirm the scene now calls that posture invalid, remove the box, and confirm it calls it
valid again. If the second check still fails, removal is not doing what the grasp assumes
and every reach is being planned against a shelf that is supposed to be gone.

ApplyPlanningScene is a service and the client waits for its reply, so a positive result
here also rules out a race between the removal and the check.
"""
import sys
import time

import numpy as np
import rclpy

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.kinematics.arm_chain import ArmChain  # noqa: E402
from avaa_solution.moveit_client import MoveItClient  # noqa: E402

CHAIN = ["torso_lift_joint"] + ["arm_left_%d_joint" % i for i in range(1, 8)]


def main():
    rclpy.init()
    client = MoveItClient("scenecheck")
    if not client.wait_until_ready(40.0):
        print("move_group not up")
        return 1
    chain = ArmChain.from_urdf()

    posture = [0.20] + [0.0, -0.6, 0.0, -1.2, 0.0, 0.0, 0.0]
    where = chain.fk(posture)[:3, 3]
    print("test posture puts the gripper at %s\n" % np.round(where, 3).tolist())

    def verdict(label):
        valid = client.state_valid(CHAIN, posture)
        note = "" if valid is not False else "   hits: %s" % client.why_invalid()[:60]
        print("  %-28s %s%s" % (label, "VALID" if valid is not False else "INVALID", note))
        return valid

    print("with an empty scene")
    clean = verdict("before adding anything")

    # A box straddling the gripper, so the posture must become invalid.
    ok = client.add_box("scenecheck_block", "base_link",
                        (float(where[0]), float(where[1]), float(where[2])),
                        (0.30, 0.30, 0.30))
    print("\nadd_box returned %s" % ok)
    time.sleep(0.5)
    blocked = verdict("with a box on the gripper")

    ok = client.remove_object("scenecheck_block")
    print("\nremove_object returned %s" % ok)
    # No sleep. The grasp does not wait either, and if a wait is what makes the
    # difference then that is the finding.
    immediate = verdict("immediately after removing")
    time.sleep(1.0)
    later = verdict("one second after removing")

    print()
    if blocked is not False:
        print("The box did not block the posture, so this test proves nothing about")
        print("removal. Check add_box first.")
    elif immediate is not False:
        print("Removal is immediate and complete. The reach being obstructed is a real")
        print("collision with the robot itself, not a shelf that failed to leave.")
    elif later is not False:
        print("Removal WORKS but is not visible straight away. The grasp checks its")
        print("reach 41 ms after removing and is therefore planning against the shelf.")
    else:
        print("Removal DOES NOT WORK. The box is still in the scene a second later, so")
        print("every reach the grasp plans is checked against a shelf it thinks it")
        print("removed. remove_object sends no header.frame_id, which MoveIt can")
        print("require for a REMOVE to match.")
    client.remove_object("scenecheck_block")
    client.shutdown()
    rclpy.shutdown()
    return 0


sys.exit(main())
