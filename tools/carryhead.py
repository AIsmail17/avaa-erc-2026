#!/usr/bin/env python3
"""Do the best carry postures stay valid at every head tilt delivery uses?

    tools/in-sim carryhead.py

The refused carry posture hit the head. carrysearch.py checked the candidates with the head where
it happened to be; delivery tilts it from looking down at the bin to level, and head_2_link moves
with it. So each candidate is checked again at the ends of the tilt range and in between.
"""
import sys
import time

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState

JOINTS = ["torso_lift_joint"] + ["arm_left_%d_joint" % i for i in range(1, 8)]
CANDIDATES = {
    "[0.38, 0.12, 1.00]": [0.1297, 4.6234, -2.2541, 2.2024, -2.2291, -1.2722, -1.5777, 1.0617],
    "[0.38, 0.12, 0.90]": [0.0701, 4.6589, -2.1673, 2.2883, -2.3724, -1.2617, -1.6366, -1.9919],
    "[0.46, 0.20, 1.00]": [0.0923, 1.0976, 1.0845, -0.7887, -2.3935, 0.9516, 2.4406, -2.3935],
    "[0.38, 0.26, 0.90]": [0.049, 0.5533, 1.0845, -0.383, -2.3935, 1.3064, 2.952, 1.1324],
}
TILTS = [-1.047, -0.60, -0.30, 0.0, 0.349]
PANS = [0.0, 0.5, -0.5]


def main():
    rclpy.init()
    node = rclpy.create_node("carryhead")
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

    def check(positions, pan, tilt):
        names = list(latest.keys())
        values = [latest[n] for n in names]
        overrides = dict(zip(JOINTS, positions))
        overrides["head_1_joint"] = pan
        overrides["head_2_joint"] = tilt
        for joint, value in overrides.items():
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
            return "?"
        result = future.result()
        if result.valid:
            return "ok"
        pairs = sorted({"%s/%s" % (c.contact_body_1, c.contact_body_2) for c in result.contacts})
        return "NO(" + pairs[0] + ")"

    print("tilt:%s" % "".join("%10.2f" % t for t in TILTS))
    for label, positions in CANDIDATES.items():
        for pan in PANS:
            row = [check(positions, pan, t) for t in TILTS]
            bad = [r for r in row if r != "ok"]
            print("%s pan %+.1f: %s" % (label, pan, "all valid" if not bad else " ".join(r[:40] for r in row)))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
