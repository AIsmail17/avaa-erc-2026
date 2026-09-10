#!/usr/bin/env python3
"""Search for a carry posture MoveIt accepts: book held clear of the base, arm clear of the head.

    tools/in-sim carrysearch.py

CARRY_POSTURE from tools/carrypose2.py reached the carry point with every joint inside its limits
and was refused by /check_state_validity: head_2_link against arm_left_6_link. A gripper held at
1.00 m right in front of the head puts the wrist into it. This walks a grid of carry points --
further forward, lower, further left -- solves each from several seeds with the joint ranges
narrowed, asks MoveIt about every solution, and ranks the valid ones: book bottom high above the
base, arm within the footprint, joints away from their limits, and the book off the camera centre.
"""
import itertools
import sys
import time

import numpy as np
import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.kinematics.arm_chain import ArmChain   # noqa: E402

JOINTS = ["torso_lift_joint"] + ["arm_left_%d_joint" % i for i in range(1, 8)]
TUCK_POSE = [0.3877, -1.6152, 0.0717, 0.0408, 0.5074, -1.4708, 0.6063]
MARGIN = 0.05
BOOK_BELOW_GRIP = 0.125 - 0.045
XS = [0.30, 0.38, 0.46]
YS = [0.12, 0.20, 0.26]
ZS = [0.80, 0.90, 1.00]


def narrow(chain, margin):
    for joint in chain.moving:
        if hasattr(joint, "lower") and hasattr(joint, "upper") and joint.upper - joint.lower > 2 * margin:
            joint.lower += margin
            joint.upper -= margin


def main():
    full = ArmChain.from_urdf()
    chain = ArmChain.from_urdf()
    narrow(chain, MARGIN)

    rclpy.init()
    node = rclpy.create_node("carrysearch")
    latest = {}

    def on_joints(msg):
        for name, pos in zip(msg.name, msg.position):
            latest[name] = pos

    node.create_subscription(JointState, "/joint_states", on_joints, 10)
    client = node.create_client(GetStateValidity, "/check_state_validity")
    deadline = time.time() + 20.0
    while time.time() < deadline and not client.service_is_ready():
        rclpy.spin_once(node, timeout_sec=0.2)
    if not client.service_is_ready():
        print("no /check_state_validity: move_group is not running")
        return 1
    end = time.time() + 3.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    def valid(positions):
        names = list(latest.keys())
        values = [latest[n] for n in names]
        for joint, value in zip(JOINTS, positions):
            if joint in names:
                values[names.index(joint)] = float(value)
            else:
                names.append(joint)
                values.append(float(value))
        request = GetStateValidity.Request()
        request.robot_state = RobotState(joint_state=JointState(name=names, position=values))
        request.group_name = ""
        future = client.call_async(request)
        stop = time.time() + 10.0
        while time.time() < stop and not future.done():
            rclpy.spin_once(node, timeout_sec=0.05)
        if not future.done():
            return None, "no answer"
        result = future.result()
        pairs = sorted({"%s/%s" % (c.contact_body_1, c.contact_body_2) for c in result.contacts})
        return result.valid, ";".join(pairs[:2])

    seeds = [("tuck t0.05", [0.05] + TUCK_POSE), ("tuck t0.20", [0.20] + TUCK_POSE),
             ("tuck t0.35", [0.35] + TUCK_POSE), ("unseeded", None)]
    found = []
    tried = 0
    refused = {}
    for x, y, z in itertools.product(XS, YS, ZS):
        target = [x, y, z]
        for label, seed in seeds:
            sol = chain.ik(target, seed=seed, approach=[1.0, 0.0, 0.0], closing=[0.0, 1.0, 0.0])
            if sol is None:
                continue
            tried += 1
            ok, why = valid(sol)
            if not ok:
                refused[why] = refused.get(why, 0) + 1
                continue
            origins = chain.joint_origins(sol)
            lowest = min(float(o[2]) for o in origins)
            widest = max(abs(float(o[1])) for o in origins)
            margin = min(min(v - lo, hi - v) for v, (lo, hi) in zip(sol, full.limits))
            book_bottom = z - BOOK_BELOW_GRIP - 0.125
            score = (book_bottom >= 0.55, widest <= 0.27, round(float(np.degrees(margin)), 0) >= 3,
                     y, book_bottom, -widest)
            found.append((score, target, label, sol, lowest, widest, margin, book_bottom))
    print("solutions tried %d, valid %d" % (tried, len(found)))
    print("refusals by contact: %s" % dict(sorted(refused.items(), key=lambda kv: -kv[1])[:5]))
    found.sort(key=lambda f: f[0], reverse=True)
    for score, target, label, sol, lowest, widest, margin, book_bottom in found[:6]:
        print("VALID %s from %-10s book bottom %.2f  lowest link %.2f  widest |y| %.2f  margin %.1f deg"
              % (target, label, book_bottom, lowest, widest, np.degrees(margin)))
        print("      posture %s" % np.round(sol, 4).tolist())
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
