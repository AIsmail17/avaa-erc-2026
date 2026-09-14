#!/usr/bin/env python3
"""Check each leg of a right-arm stow route against the whole robot.

    tools/in-sim tuckpath_check.py

The route is PAL's own "home" motion for the right arm (out to the side, then in), ending at
the variant of its home pose that hometuck_search.py found clear at every torso height.
"""
import sys

import numpy as np
import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution import approach_node, grasp_node                  # noqa: E402

LEFT = ["arm_left_%d_joint" % i for i in range(1, 8)]
RIGHT = ["arm_right_%d_joint" % i for i in range(1, 8)]
POSES = {
    "zeros": [0.0] * 7,
    "RIGHT_TUCK_VIA": [-0.7194, -2.2867, -0.5064, 0.5221, 2.3399, 1.0503, 1.9772],
    "RIGHT_TUCK (now)": [-0.1641, -1.6339, -0.1826, 1.1345, 1.6524, 2.1592, 1.9100],
    "PAL out": [-1.8614, -1.6008, -0.34892, -1.9818, 0.10153, -1.5829, 0.0],
    "PAL in": [-0.26, -1.6008, -0.3489, -1.9818, 0.0, -1.5829, 0.0],
    "home variant": [-0.36, -1.7, -0.47, -2.35, 0.0, -1.2, 0.0],
}
LEGS = [("zeros", "PAL out"), ("RIGHT_TUCK_VIA", "PAL out"), ("RIGHT_TUCK (now)", "PAL out"),
        ("PAL out", "PAL in"), ("PAL in", "home variant"), ("PAL out", "home variant"),
        ("RIGHT_TUCK_VIA", "home variant"), ("zeros", "home variant")]


def main():
    rclpy.init()
    node = rclpy.create_node("tuckpath_check")
    client = node.create_client(GetStateValidity, "/check_state_validity")
    if not client.wait_for_service(timeout_sec=30.0):
        print("no /check_state_validity -- is move_group up?")
        return 1

    def valid(torso, left, right):
        state = RobotState()
        state.joint_state.name = ["torso_lift_joint"] + LEFT + RIGHT
        state.joint_state.position = [float(torso)] + [float(v) for v in left] + \
            [float(v) for v in right]
        request = GetStateValidity.Request()
        request.group_name = ""
        request.robot_state = state
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        result = future.result()
        if result is None:
            return None, []
        return result.valid, sorted({(c.contact_body_1, c.contact_body_2)
                                     for c in result.contacts})[:2]

    for a, b in LEGS:
        for torso, left, label in ((0.10, approach_node.TUCK_POSE, "approach tuck"),
                                   (0.02, grasp_node.TUCK_POSE, "grasp tuck")):
            bad = []
            for t in np.linspace(0.0, 1.0, 21):
                right = [x + t * (y - x) for x, y in zip(POSES[a], POSES[b])]
                ok, pairs = valid(torso, left, right)
                if ok is not True:
                    bad.append((round(float(t), 2), pairs))
            print("%-17s -> %-13s torso %.2f %-13s %s" % (
                a, b, torso, label,
                "clear" if not bad else "blocked %d/21 from t=%s %s" % (len(bad), bad[0][0], bad[0][1])))
    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
