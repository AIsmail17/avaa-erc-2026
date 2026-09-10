#!/usr/bin/env python3
"""Which release points can the arm reach from the carry posture, down into the bin and out?

    tools/in-sim placeprobe.py

tools/aboveprobe.py showed the placement is decided by self-collision, not by the bin: the
left arm folds its elbow into the torso and the right arm when the point over the bin is close
in and to the right. Run 15's line to [0.627, -0.027] was refused at step 4 for exactly that,
while the laptop's to [0.632, -0.015] was clear. Lifting straight up first and then going
across was clear at 0.70 and 0.80 m for every y tried; at 0.63 it was not.

This walks the whole placement for a grid of release points, the way deliver_node._straight
walks each leg -- seeded from the last step, CARRY_APPROACH and CARRY_CLOSING, no pin, every
step put to /check_state_validity for the arm group with the head at the tilt the controller
would hold for that range:

    up      from the carry point straight up to the height that clears the rim
    across  to over the release point
    down    to the release height
    out     back up to clear the rim, as _do_release does

A cell reads "ok", or the leg and step it stopped at. The reasons follow the table.
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
ABOVE_Z = RIM + BOOK_BELOW_GRIP + 0.06
LOW_Z = RIM - 0.21 + 0.02 + BOOK_BELOW_GRIP
CAMERA_HEIGHT = 1.20
XS = (0.72, 0.76, 0.80, 0.84, 0.88)
YS = (-0.06, -0.03, 0.0, 0.03, 0.06)


def tilt_for(x):
    return float(np.clip(-math.atan2(CAMERA_HEIGHT - RIM, max(x, 0.20)), -1.047, 0.349))


def main():
    chain = ArmChain.from_urdf()
    rclpy.init()
    node = rclpy.create_node("placeprobe")
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

    start = list(CARRY_POSTURE)
    here = chain.fk(start)[:3, 3]
    print("carry point %s; over the rim at z %.3f, release at z %.3f"
          % (np.round(here, 3).tolist(), ABOVE_Z, LOW_Z))
    reasons = {}
    table = {}
    for x in XS:
        tilt = tilt_for(x)
        for y in YS:
            up = [here[0], here[1], ABOVE_Z]
            over = [x, y, ABOVE_Z]
            low = [x, y, LOW_Z]
            legs = [("up", here, up, 3), ("across", up, over, 6), ("down", over, low, 5),
                    ("out", low, over, 4)]
            seed = list(start)
            cell = "ok"
            for leg, a, b, steps in legs:
                a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
                failed = None
                for step in range(1, steps + 1):
                    point = a + (step / float(steps)) * (b - a)
                    sol = chain.ik(point, seed=seed, approach=CARRY_APPROACH, closing=CARRY_CLOSING)
                    if sol is None:
                        failed = "no IK"
                    else:
                        ok, why = check(sol, tilt)
                        if not ok:
                            failed = why
                    if failed is not None:
                        cell = "%s %d" % (leg, step)
                        reasons.setdefault(failed, []).append("(%.2f, %+.2f) %s" % (x, y, cell))
                        break
                    seed = list(sol)
                if failed is not None:
                    break
            table[(x, y)] = cell
    print("")
    print("%-8s" % "x \\ y" + "".join("%12s" % ("%+.2f" % y) for y in YS))
    for x in XS:
        print("%-8s" % ("%.2f" % x) + "".join("%12s" % table[(x, y)] for y in YS))
    if reasons:
        print("")
        for why, where in reasons.items():
            print("%s -- %s" % (why, ", ".join(where)))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
