#!/usr/bin/env python3
"""Do validated seeds find a pre-grasp posture where the search's own starts do not?

    tools/in-sim seedtest.py [tries_per_seed]

The pre-grasp posture search seeds its first IK attempt from the folded arm and every later one
from nothing. On the thirteenth full run all six tries it had time for put the upper arm into the
torso -- torso_base_link against arm_left_3_link and arm_left_4_link -- for a row 3 pre-grasp at
(0.591, 0.158, 0.796), and the grasp gave up. Row 3 needed nine tries on the sixth run too.

This makes the same IK call the search makes -- approach and closing directions, the torso
preference and the torso pin for the row -- at every row's pre-grasp point, from three kinds of
start: none, the folded arm, and postures MoveIt has already accepted (the carry search), and asks
/check_state_validity about each answer. It reports, per row and per kind of start, how many tries
produced a valid posture and how long a try took.
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
                                      SHOULDER_BASE_Z, TUCK_POSE, TUCK_TORSO)
from avaa_solution.kinematics.arm_chain import ArmChain               # noqa: E402

TRIES = int(sys.argv[1]) if len(sys.argv) > 1 else 6
JOINTS = ["torso_lift_joint"] + ["arm_left_%d_joint" % i for i in range(1, 8)]
FACE_X = 0.741
STANDOFF = 0.15
Y = 0.158
BELOW_CENTRE = 0.045

# Postures /check_state_validity accepted in tools/carrysearch.py, arm reaching forward.
LIBRARY = [
    [0.0701, 4.6589, -2.1673, 2.2883, -2.3724, -1.2617, -1.6366, -1.9919],
    [0.1297, 4.6234, -2.2541, 2.2024, -2.2291, -1.2722, -1.5777, 1.0617],
    [0.0923, 1.0976, 1.0845, -0.7887, -2.3935, 0.9516, 2.4406, -2.3935],
    [0.049, 0.5533, 1.0845, -0.383, -2.3935, 1.3064, 2.952, 1.1324],
    [0.0579, 3.37, -1.1395, 2.568, -2.3935, -0.328, 2.952, -0.0522],
]


def posture_cost(chain, height):
    ideal = float(np.clip(height - SHOULDER_BASE_Z, 0.0, 0.35))

    def cost(values):
        torso = abs(float(values[0]) - ideal)
        crowding = sum(max(0.0, 0.20 - min(v - lo, hi - v))
                       for v, (lo, hi) in zip(values, chain.limits))
        return 10.0 * torso + crowding
    return cost


def torso_pin(height):
    ideal = float(np.clip(height - SHOULDER_BASE_Z + 0.25, 0.0, 0.35))
    return {"torso_lift_joint": (ideal, 0.10)}


def main():
    chain = ArmChain.from_urdf()
    rclpy.init()
    node = rclpy.create_node("seedtest")
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

    starts = {
        "unseeded": [None] * TRIES,
        "folded arm": [[TUCK_TORSO] + TUCK_POSE] * TRIES,
        "validated library": [LIBRARY[i % len(LIBRARY)] for i in range(TRIES)],
    }
    print("row  start               valid/tries  solved  sec per try")
    print("-" * 60)
    for row, row_height in enumerate(arena.ROW_HEIGHTS_BASE, start=1):
        z = row_height - BELOW_CENTRE
        target = [max(FACE_X - STANDOFF, 0.34), Y, z]
        for label, seeds in starts.items():
            good = 0
            solved = 0
            began = time.time()
            for seed in seeds:
                sol = chain.ik(target, seed=seed, approach=GRASP_APPROACH, closing=GRASP_CLOSING,
                               prefer=posture_cost(chain, z), pin=torso_pin(z))
                if sol is None:
                    continue
                solved += 1
                if valid(sol):
                    good += 1
            per = (time.time() - began) / max(len(seeds), 1)
            print("%3d  %-18s  %5d/%-5d  %6d  %6.1f" % (row, label, good, len(seeds), solved, per))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
