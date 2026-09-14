#!/usr/bin/env python3
"""Find the right-arm tuck closest to PAL's own "home" pose that is clear at every torso height.

    tools/in-sim hometuck_search.py

PAL's motions file (tiago_pro_bringup/config/motions/tiago_pro_motions_general_spherical-wrist.yaml,
motion "home") tucks the right arm at [-0.36, -1.83, -0.47, -2.35, 0.0, -1.2, 0.0], the mirror
of the left tuck approach_node already uses. With the torso fully down MoveIt finds
arm_right_6_link against base_link, so this varies the shoulder lift, elbow and wrist a little
and asks move_group about the whole robot, with the left arm in each posture the solution holds,
and along the joint-space line from RIGHT_TUCK_VIA, which is how the stow moves the arm.
"""
import itertools
import sys

import numpy as np
import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution import approach_node, grasp_node                  # noqa: E402

LEFT = ["arm_left_%d_joint" % i for i in range(1, 8)]
RIGHT = ["arm_right_%d_joint" % i for i in range(1, 8)]
HOME = [-0.36, -1.83, -0.47, -2.35, 0.0, -1.2, 0.0]


def main():
    rclpy.init()
    node = rclpy.create_node("hometuck_search")
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
        return result is not None and result.valid

    # (left arm, torso heights it is held at). approach TUCK_POSE is itself invalid at 0.00.
    holds = [(approach_node.TUCK_POSE, (0.10, 0.15, 0.35)),
             (grasp_node.TUCK_POSE, (0.0, 0.05, 0.10, 0.15, 0.35)),
             (grasp_node.CARRY_POSTURE[1:], (grasp_node.CARRY_POSTURE[0], 0.0))]

    def clear(right):
        return all(valid(t, left, right) for left, heights in holds for t in heights)

    found = []
    for j1, j2, j4, j6 in itertools.product((-0.36, -0.5), (-1.83, -1.7, -1.55),
                                            (-2.35, -2.2, -2.05), (-1.2, -1.0, -0.8, -1.4)):
        right = [j1, j2, HOME[2], j4, HOME[4], j6, HOME[6]]
        if clear(right):
            found.append((float(np.linalg.norm(np.subtract(right, HOME))), right))
    found.sort()
    print("%d of 72 variants clear at every height" % len(found))
    for distance, right in found[:6]:
        line = [t for t in np.linspace(0.0, 1.0, 21)
                if not valid(0.10, approach_node.TUCK_POSE,
                             [v + t * (r - v) for v, r in zip(approach_node.RIGHT_TUCK_VIA, right)])]
        print("  %.2f rad from home  %s  line from RIGHT_TUCK_VIA at torso 0.10: %s"
              % (distance, right, "clear" if not line else "blocked at t=%s" %
                 [round(float(t), 2) for t in line[:4]]))
    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
