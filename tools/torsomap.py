#!/usr/bin/env python3
"""Which torso heights give an accepted pre-grasp and a clear reach, for every row?

    tools/in-sim torsomap.py [grasp_depth] [tries]

tools/torsotest.py found row 3's pre-grasp accepted at a torso of 0.00-0.10 and refused or
unsolvable from 0.15 to 0.35, where _torso_for pins it. _torso_for is also what the reach walk and
the re-aim pin the torso with, point by point, so a new rule has to hold from the pre-grasp all
the way to the grasp, and at the heights either side of each row that a book estimate a few
centimetres off would ask for.

For each row's pre-grasp height and 30 mm either side, and each torso from 0.00 to 0.35, this
solves the pre-grasp as the posture search does, asks /check_state_validity about it, and from
each accepted one walks the straight line to the grasp point as _reach_clearance does -- seeded
from the last step, torso pinned, every step checked. A cell reads accepted/tries and the best
percentage of the line walked; * marks the torso the node pins today.
"""
import sys
import time

import numpy as np
import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution import arena                                      # noqa: E402
from avaa_solution.grasp_node import (GRASP_APPROACH, GRASP_CLOSING,  # noqa: E402
                                      REACH_STEPS, SHOULDER_BASE_Z)
from avaa_solution.kinematics.arm_chain import ArmChain               # noqa: E402

GRASP_DEPTH = float(sys.argv[1]) if len(sys.argv) > 1 else 0.06
TRIES = int(sys.argv[2]) if len(sys.argv) > 2 else 2
JOINTS = ["torso_lift_joint"] + ["arm_left_%d_joint" % i for i in range(1, 8)]
FACE_X = 0.741
STANDOFF = 0.15
Y = 0.158
BELOW_CENTRE = 0.045
SLACK = 0.03
OFFSETS = (-0.03, 0.0, 0.03)
TORSOS = [round(float(t), 2) for t in np.arange(0.0, 0.351, 0.05)]


def cost_towards(chain, ideal):
    def cost(values):
        torso = abs(float(values[0]) - ideal)
        crowding = sum(max(0.0, 0.20 - min(v - lo, hi - v))
                       for v, (lo, hi) in zip(values, chain.limits))
        return 10.0 * torso + crowding
    return cost


def main():
    chain = ArmChain.from_urdf()
    rclpy.init()
    node = rclpy.create_node("torsomap")
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
        return bool(future.done() and future.result().valid)

    def walk(start, pre, grasp, pin):
        seed = list(start)
        for step in range(1, REACH_STEPS + 1):
            point = pre + (step / float(REACH_STEPS)) * (grasp - pre)
            sol = chain.ik(point, seed=seed, approach=GRASP_APPROACH, closing=GRASP_CLOSING,
                           pin={"torso_lift_joint": pin})
            if sol is None or not valid(sol):
                return (step - 1) / float(REACH_STEPS)
            seed = sol
        return 1.0

    pre_x = max(FACE_X - STANDOFF, 0.34)
    grasp_x = FACE_X + GRASP_DEPTH
    print("pre-grasp x %.3f, grasp x %.3f, %d tries a cell, %d steps a walk"
          % (pre_x, grasp_x, TRIES, REACH_STEPS), flush=True)
    print("%-26s" % "height" + "".join("%10s" % ("torso %.2f" % t) for t in TORSOS), flush=True)
    for row, row_height in enumerate(arena.ROW_HEIGHTS_BASE, start=1):
        for offset in OFFSETS:
            z = row_height - BELOW_CENTRE + offset
            pinned = float(np.clip(z - SHOULDER_BASE_Z + 0.25, 0.0, 0.35))
            nearest = min(TORSOS, key=lambda t: abs(t - pinned))
            pre = np.array([pre_x, Y, z])
            grasp = np.array([grasp_x, Y, z])
            began = time.time()
            cells = []
            for t in TORSOS:
                accepted = 0
                best = None
                for _ in range(TRIES):
                    sol = chain.ik(pre, seed=None, approach=GRASP_APPROACH,
                                   closing=GRASP_CLOSING, prefer=cost_towards(chain, t),
                                   pin={"torso_lift_joint": (t, SLACK)})
                    if sol is None or not valid(sol):
                        continue
                    accepted += 1
                    fraction = walk(sol, pre, grasp, (t, SLACK))
                    best = fraction if best is None else max(best, fraction)
                mark = "*" if t == nearest else " "
                reach = "  -" if best is None else "%3d" % round(best * 100)
                cells.append("%10s" % ("%s%d/%d %s" % (mark, accepted, TRIES, reach)))
            label = "row %d %+.2f z=%.3f pin %.2f" % (row, offset, z, pinned)
            print("%-26s%s  (%.0f s)" % (label, "".join(cells), time.time() - began), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
