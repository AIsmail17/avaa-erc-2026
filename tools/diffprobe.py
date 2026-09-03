#!/usr/bin/env python3
"""Is the shelf blocking the arm, or is it blocking the OTHER arm's default pose?

    tools/in-sim diffprobe.py

state_valid sends a RobotState naming only the eight left-arm joints with is_diff set to
False. False means "this is a complete state", so every joint NOT named takes its default
-- and the right arm's default is straight out in front, where ideal_grasp.py measured
gripper_right_base_link at x=+0.86 in base_link, which at a 0.68 m standoff is 0.18 m
inside the shelf.

That would make every posture invalid whenever the shelf is in the scene and valid
whenever it is not, which is exactly the pattern measured.

This asks the service the same question three ways: as a complete state of eight joints,
as a diff against the live robot, and as a complete state with every joint filled in from
/joint_states. If the first disagrees with the other two, the check has been answering a
question about a robot that does not exist.
"""
import sys

import numpy as np
import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.kinematics.arm_chain import ArmChain      # noqa: E402
from avaa_solution.moveit_client import MoveItClient         # noqa: E402

CHAIN = ["torso_lift_joint"] + ["arm_left_%d_joint" % i for i in range(1, 8)]
ROW_HEIGHTS = [1.391, 1.061, 0.731, 0.401]
BOARD_DROP = 0.145
SHELF_DEPTH = 0.30
SHELF_WIDTH = 4.8
BELOW_CENTRE = 0.045
STANDOFF = 0.15
FACE = 0.70
APPROACH = [1.0, 0.0, 0.0]
CLOSING = [0.0, 1.0, 0.0]


class Live(Node):
    def __init__(self):
        super().__init__("diffprobe_joints")
        self.joints = {}
        self.create_subscription(JointState, "/joint_states", self._on, 10)

    def _on(self, msg):
        for name, value in zip(msg.name, msg.position):
            self.joints[name] = value


def ask(client, names, values, is_diff, extra=None):
    state = RobotState()
    all_names = list(names)
    all_values = [float(v) for v in values]
    if extra:
        for name, value in extra.items():
            if name not in all_names:
                all_names.append(name)
                all_values.append(float(value))
    state.joint_state.name = all_names
    state.joint_state.position = all_values
    state.is_diff = is_diff
    request = GetStateValidity.Request()
    request.robot_state = state
    request.group_name = "arm_left_torso"
    result = client._wait(client.validity.call_async(request), timeout=10.0)
    if result is None:
        return None, []
    pairs = ["%s/%s" % (c.contact_body_1, c.contact_body_2)
             for c in getattr(result, "contacts", [])]
    return bool(result.valid), pairs


def main():
    rclpy.init()
    chain = ArmChain.from_urdf()
    node = Live()
    for _ in range(60):
        rclpy.spin_once(node, timeout_sec=0.1)
    client = MoveItClient("diffprobe")
    if not client.wait_until_ready(40.0):
        print("move_group is not running")
        return

    placed = []
    centre_x = FACE + SHELF_DEPTH / 2.0 - 0.05
    for j, board in enumerate(ROW_HEIGHTS):
        name = "shelf_board_%d" % j
        if client.add_box(name, "base_link", (centre_x, 0.0, board - BOARD_DROP),
                          (SHELF_DEPTH, SHELF_WIDTH, 0.04)):
            placed.append(name)
    if client.add_box("shelf_back", "base_link", (FACE + SHELF_DEPTH, 0.0, 0.9),
                      (0.04, SHELF_WIDTH, 1.8)):
        placed.append("shelf_back")
    print("shelf in the scene: %d boxes\n" % len(placed))

    right = {n: v for n, v in node.joints.items() if n.startswith("arm_right")}
    print("right arm now: %s" % " ".join("%+.2f" % v for v in right.values()))
    print()
    print("%-6s %-26s %-26s %-26s" % ("row", "8 joints, is_diff False",
                                      "8 joints, is_diff True",
                                      "every joint, is_diff False"))

    for index, height in enumerate(ROW_HEIGHTS):
        z = height - BELOW_CENTRE
        point = np.array([max(FACE - STANDOFF, 0.34), 0.16, z])
        counts = [0, 0, 0]
        seen = ["", "", ""]
        for attempt in range(8):
            seed = None if attempt == 0 else [
                float(np.random.uniform(lo, hi)) for lo, hi in chain.limits]
            solution = chain.ik(point, seed=seed, approach=APPROACH, closing=CLOSING)
            if solution is None:
                continue
            for slot, (is_diff, extra) in enumerate(
                    ((False, None), (True, None), (False, node.joints))):
                ok, pairs = ask(client, CHAIN, solution, is_diff, extra)
                if ok:
                    counts[slot] += 1
                elif pairs and not seen[slot]:
                    seen[slot] = pairs[0]
        print("%-6d %-26s %-26s %-26s"
              % (index + 1,
                 "%d clear  %s" % (counts[0], seen[0][:14]),
                 "%d clear  %s" % (counts[1], seen[1][:14]),
                 "%d clear  %s" % (counts[2], seen[2][:14])))

    for name in placed:
        client.remove_object(name)
    client.shutdown()
    node.destroy_node()
    rclpy.shutdown()


main()
