#!/usr/bin/env python3
"""Find a folded posture for the right arm, which this solution never uses but must stow.

The right arm is tucked by mirroring the left one, flipping two joints. That is a guess,
and it is wrong: arm_right_4_link ends up at x=0.491, y=-0.371, which is inside the shelf
once the base is close enough to grasp from, and MoveIt then refuses to plan for the left
arm because the robot as a whole is in collision.

Same method as the left arm: sample, ask /check_state_validity, keep the most compact
posture with room at every stop.
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

URDF = "/opt/erc_ws/src/erc_description/urdf/tiago_pro.urdf"
RIGHT = ["arm_right_%d_joint" % i for i in range(1, 8)]


def main():
    limits = {}
    for joint in ET.parse(URDF).getroot().findall("joint"):
        limit = joint.find("limit")
        if limit is not None and joint.get("name") in RIGHT:
            limits[joint.get("name")] = (float(limit.get("lower")),
                                         float(limit.get("upper")))

    rclpy.init()
    node = rclpy.create_node("find_right_tuck")
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

    def valid(values):
        state = JointState()
        state.name = list(js.name)
        state.position = list(js.position)
        for name, value in zip(RIGHT, values):
            if name in index:
                state.position[index[name]] = float(value)
        request = GetStateValidity.Request()
        request.robot_state = RobotState()
        request.robot_state.joint_state = state
        request.robot_state.is_diff = False
        request.group_name = "arm_left_torso"
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        result = future.result()
        if result is None:
            return None, []
        contacts = {(c.contact_body_1, c.contact_body_2) for c in result.contacts}
        return bool(result.valid), sorted(contacts)

    ok, contacts = valid([js.position[index[n]] for n in RIGHT if n in index])
    print("right arm as it stands: valid=%s" % ok)
    for pair in contacts[:4]:
        print("   %s <-> %s" % pair)
    print()
    print("searching...")

    rng = np.random.default_rng(1)
    best = None
    checked = 0
    started = time.time()
    while time.time() - started < 240 and checked < 300:
        values = []
        for name in RIGHT:
            lo, hi = limits[name]
            centre = 0.5 * (lo + hi)
            values.append(float(np.clip(rng.normal(centre, 0.35 * (hi - lo)), lo, hi)))
        margin = min(min(v - limits[n][0], limits[n][1] - v)
                     for n, v in zip(RIGHT, values))
        if margin < 0.15:
            continue
        checked += 1
        state_ok, _ = valid(values)
        if state_ok is not True:
            continue
        # Compact: judged on the joint angles rather than on a tip position, since the
        # analytic chain in this package only models the left arm.
        folded = sum(abs(v) for v in values)
        if best is None or folded < best[0]:
            best = (folded, list(values))
            print("   candidate: sum |angle| %.2f, margin %.2f" % (folded, margin),
                  flush=True)

    print()
    if best is None:
        print("nothing valid in %d samples" % checked)
        return 1
    print("best of %d samples:" % checked)
    print("RIGHT_TUCK = [%s]" % ", ".join("%.4f" % v for v in best[1]))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
