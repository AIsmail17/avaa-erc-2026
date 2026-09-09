#!/usr/bin/env python3
"""Find a folded posture for either arm, scored on where the arm IS.

    tools/in-sim find_arm_tuck.py right      -> RIGHT_TUCK, for approach_node
    tools/in-sim find_arm_tuck.py left       -> TUCK_POSE,  for grasp_node

Both tucks had the same fault and it is worth only writing the search once. The left
arm's TUCK_POSE folds arm_left_6_link into base_link at torso zero exactly as the old
RIGHT_TUCK folded arm_right_6_link into it -- the two constants are mirror images of each
other, so they share the flaw. See tools/tuckvalid.py for the measurement.


The first version of this scored candidates by the sum of their joint angles, which is a
measure of how close a posture is to all-zeros -- and all-zeros is the arm stretched
straight out. It duly returned a posture with the right gripper 0.88 m forward, inside the
shelf, and MoveIt then refused every left-arm goal because the robot was in collision.

This scores on the forward and lateral reach of the actual links, and checks the posture
with the torso RAISED, because that is when it matters: the torso goes to 0.35 for the top
rows and takes the whole upper body with it.
"""
import math
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.kinematics.arm_chain import ArmChain  # noqa: E402

URDF = "/opt/erc_ws/src/erc_description/urdf/tiago_pro.urdf"
SIDE = (sys.argv[1] if len(sys.argv) > 1 else "right").lower()
if SIDE not in ("left", "right"):
    sys.exit("usage: find_arm_tuck.py [left|right]")
ARM = ["arm_%s_%d_joint" % (SIDE, i) for i in range(1, 8)]
CONSTANT = "RIGHT_TUCK" if SIDE == "right" else "TUCK_POSE"
# 0.0 belongs here and its absence cost a week.
#
# The stow is sent BEFORE the torso is raised, so the height the arm is actually at when
# it folds is zero -- and zero was the one height this never checked. The posture it
# produced, RIGHT_TUCK, is valid at 0.05 and above and puts arm_right_6_link inside
# base_link at 0.00. Every plan made after the stow then starts from an invalid state and
# move_group refuses it before OMPL does any work, which reads as an arm that will not
# move. Measured with tools/tuckvalid.py.
TORSO_HEIGHTS = [0.0, 0.15, 0.35]


def main():
    limits = {}
    for joint in ET.parse(URDF).getroot().findall("joint"):
        limit = joint.find("limit")
        if limit is not None and joint.get("name") in ARM:
            limits[joint.get("name")] = (float(limit.get("lower")),
                                         float(limit.get("upper")))

    rclpy.init()
    node = rclpy.create_node("find_arm_tuck")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    latest = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: latest.__setitem__("js", m), 10)
    end = time.time() + 15
    while rclpy.ok() and time.time() < end and "js" not in latest:
        rclpy.spin_once(node, timeout_sec=0.2)
    js = latest.get("js")
    if js is None:
        print("no joint states")
        return 1

    client = node.create_client(GetStateValidity, "/check_state_validity")
    if not client.wait_for_service(timeout_sec=15.0):
        print("no /check_state_validity")
        return 1
    index = {n: i for i, n in enumerate(js.name)}

    # The right arm as a chain, so candidates can be scored on where the links end up
    # rather than on how small their angles are.
    chain = ArmChain.from_urdf(
        URDF, "base_link", "gripper_%s_grasping_link" % SIDE)
    names = chain.joint_names

    def valid(values, torso):
        state = JointState()
        state.name = list(js.name)
        state.position = list(js.position)
        for name, value in zip(ARM, values):
            if name in index:
                state.position[index[name]] = float(value)
        if "torso_lift_joint" in index:
            state.position[index["torso_lift_joint"]] = float(torso)
        request = GetStateValidity.Request()
        request.robot_state = RobotState()
        request.robot_state.joint_state = state
        request.robot_state.is_diff = False
        # An empty group means "check the whole robot". Naming a group here checked
        # only that group's links, which is how a posture that folds the arm into
        # base_link passed: the arm being searched was not in the group being checked.
        request.group_name = ""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        result = future.result()
        return None if result is None else bool(result.valid)

    def reach(values, torso):
        full = [torso] + list(values)
        if len(full) != len(names):
            full = list(values)
        points = chain.joint_origins(full)
        # Furthest forward any part of the arm gets, and how far out to the side.
        return max(float(p[0]) for p in points), max(abs(float(p[1])) for p in points)

    print("%s arm chain joints: %s" % (SIDE, names))
    print("searching, checking each candidate at torso %s" % TORSO_HEIGHTS)
    rng = np.random.default_rng(3)
    best = None
    checked = 0
    started = time.time()
    while time.time() - started < 260 and checked < 400:
        values = []
        for name in ARM:
            lo, hi = limits[name]
            centre = 0.5 * (lo + hi)
            values.append(float(np.clip(rng.normal(centre, 0.4 * (hi - lo)), lo, hi)))
        margin = min(min(v - limits[n][0], limits[n][1] - v)
                     for n, v in zip(ARM, values))
        if margin < 0.15:
            continue
        checked += 1
        forward, sideways = reach(values, 0.15)
        # Cheap rejection before spending a service call: the shelf face is about 0.53 m
        # ahead at grasping distance, so anything past 0.45 is not a stowed arm.
        if forward > 0.45:
            continue
        if not all(valid(values, t) for t in TORSO_HEIGHTS):
            continue
        score = forward + 0.3 * sideways
        if best is None or score < best[0]:
            best = (score, list(values), forward, sideways)
            print("   candidate: reaches %.3f m forward, %.3f sideways, margin %.2f"
                  % (forward, sideways, margin), flush=True)

    print()
    if best is None:
        print("nothing valid and folded in %d samples" % checked)
        return 1
    print("best of %d samples: %.3f m forward, %.3f sideways"
          % (checked, best[2], best[3]))
    print("%s = [%s]" % (CONSTANT, ", ".join("%.4f" % v for v in best[1])))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
