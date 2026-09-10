#!/usr/bin/env python3
"""Which torso height lets the arm reach a row's pre-grasp without folding into the torso?

    tools/in-sim torsotest.py [row] [tries]

tools/seedtest.py showed row 3's pre-grasp postures refused five or six times in six whatever the
IK starts from, while rows 1, 2 and 4 are accepted. Row 3 is also the one row where the search's
two torso preferences disagree: _torso_for pins the torso to put the shoulder 0.25 m above the
target, which clips to 0.35 +/- 0.10 for a target at z=0.796, while _posture_cost pulls towards
the shoulder level with it, a torso of 0.119. On the other rows both clip to the same end.

This makes the search's IK call at the row's pre-grasp point as the node makes it, with the cost
alone, and with the torso pinned at each of several heights and the cost pulling the same way,
and asks /check_state_validity about every answer. It reports how many were accepted, what the
refused ones hit, and how far forward and how low the joints of the accepted ones go -- a posture
clear of the robot can still put an elbow into the shelf, whose face is at x=0.741.
"""
import collections
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
                                      SHOULDER_BASE_Z)
from avaa_solution.kinematics.arm_chain import ArmChain               # noqa: E402

ROW = int(sys.argv[1]) if len(sys.argv) > 1 else 3
TRIES = int(sys.argv[2]) if len(sys.argv) > 2 else 6
JOINTS = ["torso_lift_joint"] + ["arm_left_%d_joint" % i for i in range(1, 8)]
FACE_X = 0.741
STANDOFF = 0.15
Y = 0.158
BELOW_CENTRE = 0.045
SLACK = 0.03


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
    node = rclpy.create_node("torsotest")
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

    def check(positions):
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
            return False, ["no answer"]
        result = future.result()
        pairs = sorted({"%s/%s" % (c.contact_body_1, c.contact_body_2) for c in result.contacts})
        return bool(result.valid), pairs

    z = arena.ROW_HEIGHTS_BASE[ROW - 1] - BELOW_CENTRE
    target = [max(FACE_X - STANDOFF, 0.34), Y, z]
    node_pin = float(np.clip(z - SHOULDER_BASE_Z + 0.25, 0.0, 0.35))
    node_cost = float(np.clip(z - SHOULDER_BASE_Z, 0.0, 0.35))
    cases = [("as the node asks", (node_pin, 0.10), node_cost),
             ("cost only, no pin", None, node_cost)]
    cases += [("torso %.2f" % t, (t, SLACK), t) for t in np.arange(0.0, 0.351, 0.05)]

    print("row %d pre-grasp at (%.3f, %.3f, %.3f); the node pins the torso at %.3f +/- 0.10 "
          "and its cost pulls towards %.3f" % (ROW, target[0], target[1], target[2],
                                               node_pin, node_cost))
    print("%-18s %8s %6s  %-17s %6s %6s  %s" % ("case", "accepted", "solved", "torso accepted",
                                              "max x", "min z", "refused for"))
    print("-" * 110)
    for label, pin, ideal in cases:
        accepted = []
        solved = 0
        refusals = collections.Counter()
        for _ in range(TRIES):
            kwargs = dict(seed=None, approach=GRASP_APPROACH, closing=GRASP_CLOSING,
                          prefer=cost_towards(chain, ideal))
            if pin is not None:
                kwargs["pin"] = {"torso_lift_joint": pin}
            sol = chain.ik(target, **kwargs)
            if sol is None:
                continue
            solved += 1
            ok, pairs = check(sol)
            if ok:
                accepted.append(sol)
            else:
                refusals.update(pairs)
        if accepted:
            torsos = [float(s[0]) for s in accepted]
            origins = [chain.joint_origins(s) for s in accepted]
            max_x = max(float(o[0]) for os_ in origins for o in os_)
            min_z = min(float(o[2]) for os_ in origins for o in os_)
            torso_text = "%.2f..%.2f" % (min(torsos), max(torsos))
            x_text, z_text = "%.3f" % max_x, "%.3f" % min_z
        else:
            torso_text, x_text, z_text = "-", "-", "-"
        refused = ", ".join("%s x%d" % (p, n) for p, n in refusals.most_common(3))
        print("%-18s %4d/%-3d %6d  %-17s %6s %6s  %s"
              % (label, len(accepted), TRIES, solved, torso_text, x_text, z_text, refused))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
