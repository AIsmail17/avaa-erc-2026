#!/usr/bin/env python3
"""Is a candidate right-arm tuck safe with the WHOLE robot, and on the way there?

    tools/in-sim righttuckcheck.py -0.1641 -1.6339 -0.1826 1.1345 1.6524 2.1592 1.9100

tools/tuck_search.py checks candidates for the arm_right_torso group, which says nothing
about the left arm. This asks move_group about the whole robot (an empty group) with the
left arm in each posture the solution holds while the right arm is tucked -- the driving
tuck from approach_node, the grasp's tuck, and the carry posture -- at every torso height,
and along a straight joint-space line from the current RIGHT_TUCK and from all zeros,
which is how the stow trajectory moves the arm. It also prints how far each tuck reaches
outside the base.
"""
import sys

import numpy as np
import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution import approach_node, grasp_node                  # noqa: E402
from avaa_solution.kinematics.arm_chain import ArmChain                # noqa: E402

LEFT = ["arm_left_%d_joint" % i for i in range(1, 8)]
RIGHT = ["arm_right_%d_joint" % i for i in range(1, 8)]
HALF_X, HALF_Y = 0.3585, 0.2485


def outside(values, torso=0.15):
    chain = ArmChain.from_urdf(tip="gripper_right_grasping_link")
    points = list(chain.joint_origins([torso] + list(values)))
    worst = 0.0
    for a, b in zip(points, points[1:]):
        for t in np.linspace(0.0, 1.0, 5):
            p = a + t * (b - a)
            worst = max(worst, abs(p[0]) - HALF_X, abs(p[1]) - HALF_Y)
    return worst


def main():
    candidate = [float(v) for v in sys.argv[1:8]]
    if len(candidate) != 7:
        print("give the seven right-arm joint values")
        return 1
    rclpy.init()
    node = rclpy.create_node("righttuckcheck")
    client = node.create_client(GetStateValidity, "/check_state_validity")
    if not client.wait_for_service(timeout_sec=30.0):
        print("no /check_state_validity -- is move_group up?")
        return 1

    def valid(torso, left, right):
        state = RobotState()
        state.joint_state.name = ["torso_lift_joint"] + LEFT + RIGHT
        state.joint_state.position = [float(torso)] + [float(v) for v in left] + \
            [float(v) for v in right]
        state.is_diff = False
        request = GetStateValidity.Request()
        request.group_name = ""
        request.robot_state = state
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        result = future.result()
        if result is None:
            return None, []
        pairs = sorted({(c.contact_body_1, c.contact_body_2) for c in result.contacts})
        return result.valid, pairs

    print("reach outside the base: current RIGHT_TUCK %+.0f mm, candidate %+.0f mm"
          % (outside(approach_node.RIGHT_TUCK) * 1000, outside(candidate) * 1000))

    lefts = {
        "approach TUCK_POSE": (list(approach_node.TUCK_POSE), None),
        "grasp TUCK_POSE": (list(grasp_node.TUCK_POSE), None),
        "CARRY_POSTURE": (list(grasp_node.CARRY_POSTURE[1:]), grasp_node.CARRY_POSTURE[0]),
    }
    failures = 0
    for label, (left, fixed_torso) in lefts.items():
        heights = (fixed_torso,) if fixed_torso is not None else (0.0, 0.10, 0.15, 0.35)
        for torso in heights:
            # A candidate fails only where the tuck in use passes: approach_node's left
            # TUCK_POSE touches base_link at torso 0.00 whatever the right arm does.
            baseline = None
            for name, right in (("current", approach_node.RIGHT_TUCK), ("candidate", candidate)):
                ok, pairs = valid(torso, left, right)
                if name == "current":
                    baseline = ok
                elif ok is not True and baseline is True:
                    failures += 1
                print("  left %-18s torso %.2f  right %-9s %s %s"
                      % (label, torso, name, {True: "VALID", False: "INVALID"}.get(ok, "?"),
                         pairs[:3] if pairs else ""))

    for start_label, start in (("from current RIGHT_TUCK", approach_node.RIGHT_TUCK),
                               ("from all zeros", [0.0] * 7)):
        bad = []
        for t in np.linspace(0.0, 1.0, 21):
            right = [s + t * (c - s) for s, c in zip(start, candidate)]
            # The left arm in a posture that is itself valid at that height.
            for torso, left in ((0.0, grasp_node.TUCK_POSE), (0.10, approach_node.TUCK_POSE)):
                ok, pairs = valid(torso, left, right)
                if ok is not True:
                    bad.append((round(float(t), 2), torso, pairs[:2]))
        print("  line %-24s %s" % (start_label, "clear" if not bad else "blocked at %s" % bad[:4]))
        if bad:
            failures += 1
    print("candidate %s" % ("PASSES" if failures == 0 else "FAILS %d check(s)" % failures))
    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
