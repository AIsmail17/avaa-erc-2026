#!/usr/bin/env python3
"""Would MoveIt accept the carry posture, or refuse it for self-collision?

    tools/in-sim carryvalid.py

grasp_node.CARRY_POSTURE was found with pure kinematics -- reach, joint limits, how wide and how
low the arm goes -- and nothing there checks the robot against itself. A goal MoveIt refuses
costs a stow that hangs: the laptop run of 2026-09-10 waited out "no result within 240 s" stowing
to the tuck with a book in hand. So ask /check_state_validity, read-only, with the posture as it
stands, the tuck for comparison, and the tuck the old code used.
"""
import sys
import time

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.grasp_node import CARRY_POSTURE, TUCK_POSE, TUCK_TORSO   # noqa: E402

JOINTS = ["torso_lift_joint"] + ["arm_left_%d_joint" % i for i in range(1, 8)]
CASES = {
    "carry posture": CARRY_POSTURE,
    "the tuck (old stow)": [TUCK_TORSO] + TUCK_POSE,
}


def main():
    rclpy.init()
    node = rclpy.create_node("carryvalid")
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

    for label, positions in CASES.items():
        state = JointState()
        names = list(latest.keys())
        values = [latest[n] for n in names]
        for joint, value in zip(JOINTS, positions):
            if joint in names:
                values[names.index(joint)] = float(value)
            else:
                names.append(joint)
                values.append(float(value))
        state.name = names
        state.position = values
        request = GetStateValidity.Request()
        request.robot_state = RobotState(joint_state=state)
        request.group_name = ""
        future = client.call_async(request)
        end = time.time() + 15.0
        while time.time() < end and not future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not future.done():
            print("%-22s no answer" % label)
            continue
        result = future.result()
        pairs = sorted({"%s / %s" % (c.contact_body_1, c.contact_body_2) for c in result.contacts})
        print("%-22s %s%s" % (label, "VALID" if result.valid else "INVALID",
                               "" if result.valid else ": " + "; ".join(pairs[:6])))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
