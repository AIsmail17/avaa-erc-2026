#!/usr/bin/env python3
"""The torso map again for the two lower rows, with the shelf in the planning scene.

    tools/in-sim shelftorso.py [grasp_depth] [tries]

tools/torsomap.py measured which torso heights give row 3 and row 4 an accepted pre-grasp and a
clear walk to the grasp -- 0.00 to 0.10 for row 3, 0.15 to 0.35 for row 4 -- against the robot
alone. The posture search asks with the shelf in the scene, and a torso that suits the robot can
still slope the forearm down through the board above the bottom row. This puts the node's own
shelf boxes into the planning scene -- _add_shelf's boards and back, placed from the book face --
at two book-face distances seen on real runs, measures the same cells, and takes the boxes out
again. A cell reads accepted/tries and the best percentage of the line walked; each line ends
with what the refusals hit most.
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
from avaa_solution.grasp_node import (BOARD_DROP, GRASP_APPROACH,     # noqa: E402
                                      GRASP_CLOSING, REACH_STEPS, SHELF_DEPTH,
                                      SHELF_WIDTH)
from avaa_solution.kinematics.arm_chain import ArmChain               # noqa: E402
from avaa_solution.moveit_client import MoveItClient                  # noqa: E402

GRASP_DEPTH = float(sys.argv[1]) if len(sys.argv) > 1 else 0.11
TRIES = int(sys.argv[2]) if len(sys.argv) > 2 else 2
JOINTS = ["torso_lift_joint"] + ["arm_left_%d_joint" % i for i in range(1, 8)]
FACES = (0.705, 0.741)     # the laptop run's measured book face, and the thirteenth run's
ROWS = (3, 4)
STANDOFF = 0.15
Y = 0.158
BELOW_CENTRE = 0.045
SLACK = 0.03
OFFSETS = (-0.03, 0.0, 0.03)
TORSOS = [round(float(t), 2) for t in np.arange(0.0, 0.351, 0.05)]
BOXES = ["shelf_board_%d" % i for i in range(len(arena.ROW_HEIGHTS_BASE))] + ["shelf_back"]


def cost_towards(chain, ideal):
    def cost(values):
        torso = abs(float(values[0]) - ideal)
        crowding = sum(max(0.0, 0.20 - min(v - lo, hi - v))
                       for v, (lo, hi) in zip(values, chain.limits))
        return 10.0 * torso + crowding
    return cost


def add_shelf(scene, face_x):
    """_add_shelf, with the default row heights."""
    centre_x = face_x + SHELF_DEPTH / 2.0 - 0.05
    placed = 0
    for index, height in enumerate(arena.ROW_HEIGHTS_BASE):
        placed += scene.add_box("shelf_board_%d" % index, "base_link",
                                (centre_x, 0.0, height - BOARD_DROP),
                                (SHELF_DEPTH, SHELF_WIDTH, 0.04))
    placed += scene.add_box("shelf_back", "base_link",
                            (face_x + SHELF_DEPTH, 0.0, 0.9), (0.04, SHELF_WIDTH, 1.8))
    return placed


def main():
    chain = ArmChain.from_urdf()
    rclpy.init()
    node = rclpy.create_node("shelftorso")
    scene = MoveItClient("shelftorso_scene")
    latest = {}

    def on_joints(msg):
        for name, pos in zip(msg.name, msg.position):
            latest[name] = pos

    node.create_subscription(JointState, "/joint_states", on_joints, 10)
    client = node.create_client(GetStateValidity, "/check_state_validity")
    deadline = time.time() + 30.0
    while time.time() < deadline and not client.service_is_ready():
        rclpy.spin_once(node, timeout_sec=0.2)
    if not client.service_is_ready() or not scene.apply_scene.wait_for_service(timeout_sec=30.0):
        print("no /check_state_validity or /apply_planning_scene: move_group is not running")
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

    def walk(start, pre, grasp, pin, refusals):
        seed = list(start)
        for step in range(1, REACH_STEPS + 1):
            point = pre + (step / float(REACH_STEPS)) * (grasp - pre)
            sol = chain.ik(point, seed=seed, approach=GRASP_APPROACH, closing=GRASP_CLOSING,
                           pin={"torso_lift_joint": pin})
            if sol is None:
                return (step - 1) / float(REACH_STEPS)
            ok, pairs = check(sol)
            if not ok:
                refusals.update(pairs)
                return (step - 1) / float(REACH_STEPS)
            seed = sol
        return 1.0

    try:
        for face_x in FACES:
            placed = add_shelf(scene, face_x)
            pre_x = max(face_x - STANDOFF, 0.34)
            grasp_x = face_x + GRASP_DEPTH
            print("", flush=True)
            print("book face %.3f: %d of %d shelf boxes in the scene; pre-grasp x %.3f, grasp x %.3f"
                  % (face_x, placed, len(BOXES), pre_x, grasp_x), flush=True)
            print("%-20s" % "height" + "".join("%10s" % ("torso %.2f" % t) for t in TORSOS),
                  flush=True)
            for row in ROWS:
                for offset in OFFSETS:
                    z = arena.ROW_HEIGHTS_BASE[row - 1] - BELOW_CENTRE + offset
                    pre = np.array([pre_x, Y, z])
                    grasp = np.array([grasp_x, Y, z])
                    refusals = collections.Counter()
                    cells = []
                    for t in TORSOS:
                        accepted = 0
                        best = None
                        for _ in range(TRIES):
                            sol = chain.ik(pre, seed=None, approach=GRASP_APPROACH,
                                           closing=GRASP_CLOSING, prefer=cost_towards(chain, t),
                                           pin={"torso_lift_joint": (t, SLACK)})
                            if sol is None:
                                continue
                            ok, pairs = check(sol)
                            if not ok:
                                refusals.update(pairs)
                                continue
                            accepted += 1
                            fraction = walk(sol, pre, grasp, (t, SLACK), refusals)
                            best = fraction if best is None else max(best, fraction)
                        reach = "  -" if best is None else "%3d" % round(best * 100)
                        cells.append("%10s" % ("%d/%d %s" % (accepted, TRIES, reach)))
                    hit = ", ".join("%s x%d" % (p, n) for p, n in refusals.most_common(2))
                    print("%-20s%s  %s" % ("row %d %+.2f z=%.3f" % (row, offset, z),
                                           "".join(cells), hit), flush=True)
    finally:
        removed = sum(scene.remove_object(name) for name in BOXES)
        print("", flush=True)
        print("shelf boxes removed from the scene: %d of %d" % (removed, len(BOXES)), flush=True)
        scene.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
