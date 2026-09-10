#!/usr/bin/env python3
"""Can the arm get from the carry posture to over the bin, and down into it?

    tools/in-sim aboveprobe.py

The fifteenth full run drove to the bin and stopped with it 0.63 m ahead, and the straight
line from the carry posture to a point over its middle was refused at once: "no clear line
to a point above the bin [0.627, -0.027, 1.014]". The laptop run reached nearly the same
point, [0.632, -0.015, 1.014], lifted, and then found "no clear line down into the bin".

The delivery controller says nothing about why a line fails. This walks the same lines the
way deliver_node._straight does -- seeded from the last step, CARRY_APPROACH and
CARRY_CLOSING, no pin -- and asks /check_state_validity about every step for the arm group,
with the head at the tilt the controller would have it at for that range. For each line it
reports how far it got and, where it stopped, whether the IK had no answer or what the
posture hit.

It also tries lifting straight up first and then across, since the carried book's foot rides
below the rim and a line that climbs while it moves forward drags the book through the wall.
"""
import math
import sys
import time

import numpy as np
import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution import arena                                         # noqa: E402
from avaa_solution.deliver_node import (BOOK_BELOW_GRIP, CARRY_APPROACH,  # noqa: E402
                                        CARRY_CLOSING)
from avaa_solution.grasp_node import CARRY_POSTURE                       # noqa: E402
from avaa_solution.kinematics.arm_chain import ArmChain                  # noqa: E402
from avaa_solution.moveit_client import ARM_GROUP                        # noqa: E402

JOINTS = ["torso_lift_joint"] + ["arm_left_%d_joint" % i for i in range(1, 8)]
RIM = arena.BIN_RIM_BASE_Z
RIM_CLEARANCE = 0.06
BIN_DEPTH = 0.21
RELEASE_GAP = 0.02
ABOVE_Z = RIM + BOOK_BELOW_GRIP + RIM_CLEARANCE
LOW_Z = RIM - BIN_DEPTH + RELEASE_GAP + BOOK_BELOW_GRIP
CAMERA_HEIGHT = 1.20


def tilt_for(x):
    return float(np.clip(-math.atan2(CAMERA_HEIGHT - RIM, max(x, 0.20)), -1.047, 0.349))


def main():
    chain = ArmChain.from_urdf()
    rclpy.init()
    node = rclpy.create_node("aboveprobe")
    latest = {}

    def on_joints(msg):
        for name, pos in zip(msg.name, msg.position):
            latest[name] = pos

    node.create_subscription(JointState, "/joint_states", on_joints, 10)
    client = node.create_client(GetStateValidity, "/check_state_validity")
    deadline = time.time() + 30.0
    while time.time() < deadline and not client.service_is_ready():
        rclpy.spin_once(node, timeout_sec=0.2)
    if not client.service_is_ready():
        print("no /check_state_validity: move_group is not running")
        return 1
    end = time.time() + 3.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    def check(positions, tilt):
        names = list(latest.keys())
        values = [latest[n] for n in names]
        overrides = dict(zip(JOINTS, positions))
        overrides["head_1_joint"] = 0.0
        overrides["head_2_joint"] = tilt
        for joint, value in overrides.items():
            if joint in names:
                values[names.index(joint)] = float(value)
            else:
                names.append(joint)
                values.append(float(value))
        request = GetStateValidity.Request()
        request.robot_state = RobotState(joint_state=JointState(name=names, position=values))
        request.group_name = ARM_GROUP
        future = client.call_async(request)
        stop = time.time() + 10.0
        while time.time() < stop and not future.done():
            rclpy.spin_once(node, timeout_sec=0.05)
        if not future.done():
            return False, "no answer"
        result = future.result()
        pairs = sorted({"%s/%s" % (c.contact_body_1, c.contact_body_2) for c in result.contacts})
        return bool(result.valid), "; ".join(pairs[:3])

    def walk(start_solution, legs, tilt):
        """Walk each (from, to, steps) leg in turn. Return (steps done, steps total, why)."""
        seed = list(start_solution)
        done = 0
        total = sum(steps for _, _, steps in legs)
        for a, b, steps in legs:
            a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
            for step in range(1, steps + 1):
                point = a + (step / float(steps)) * (b - a)
                sol = chain.ik(point, seed=seed, approach=CARRY_APPROACH, closing=CARRY_CLOSING)
                if sol is None:
                    return done, total, "no IK at %s" % np.round(point, 3).tolist()
                ok, why = check(sol, tilt)
                if not ok:
                    return done, total, "invalid at %s: %s" % (np.round(point, 3).tolist(), why)
                seed = list(sol)
                done += 1
        return done, total, "clear"

    start = list(CARRY_POSTURE)
    here = chain.fk(start)[:3, 3]
    print("carry point %s; above z %.3f, release z %.3f, rim %.3f"
          % (np.round(here, 3).tolist(), ABOVE_Z, LOW_Z, RIM))
    for tilt in (tilt_for(0.63), tilt_for(0.80)):
        ok, why = check(start, tilt)
        print("the carry posture at head tilt %+.2f: %s %s" % (tilt, "valid" if ok else "INVALID", why))
    print("")
    print("%-44s %-6s %s" % ("line", "steps", "result"))
    print("-" * 110)
    cases = []
    for label, x, y in (("run 15's point", 0.627, -0.027), ("the laptop's point", 0.632, -0.015)):
        cases.append(("%s, one straight line" % label, x, [(here, [x, y, ABOVE_Z], 6)]))
    for x in (0.63, 0.70, 0.80):
        for y in (-0.05, 0.0, 0.05):
            up = [here[0], here[1], ABOVE_Z]
            cases.append(("up, then across to (%.2f, %+.2f)" % (x, y), x,
                          [(here, up, 3), (up, [x, y, ABOVE_Z], 6)]))
    results = {}
    for label, x, legs in cases:
        done, total, why = walk(start, legs, tilt_for(x))
        results[label] = (done == total)
        print("%-44s %2d/%-3d %s" % (label, done, total, why))

    print("")
    print("down into the bin from over it (after an up-then-across that got there)")
    for x in (0.63, 0.70, 0.80):
        for y in (-0.05, 0.0, 0.05):
            up = [here[0], here[1], ABOVE_Z]
            legs = [(here, up, 3), (up, [x, y, ABOVE_Z], 6), ([x, y, ABOVE_Z], [x, y, LOW_Z], 5)]
            done, total, why = walk(start, legs, tilt_for(x))
            print("%-44s %2d/%-3d %s" % ("down at (%.2f, %+.2f)" % (x, y), done, total, why))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
