#!/usr/bin/env python3
"""Ask move_group for one trivial joint move and say exactly what came back.

    tools/in-sim moveitping.py

grasp_node reports "error 99999" for both the torso raise and the pre-grasp, which its own
comment says means an exception was raised on the motion thread -- but the traceback that
should accompany it is not in the log, and MoveItClient.last_failure comes back empty. So
the two facts do not agree, and this asks the question with nothing else in the way: one
small move of the joints the arm is already at, on the same client the solution uses.

A code of 1 is SUCCESS. Anything else is printed with the client's own account of which
step failed, and an exception is printed with its traceback instead of being turned into a
number.
"""
import sys
import traceback

import rclpy
from sensor_msgs.msg import JointState

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.moveit_client import MoveItClient  # noqa: E402

CHAIN = ["torso_lift_joint"] + ["arm_left_%d_joint" % i for i in range(1, 8)]


def main():
    rclpy.init()
    node = rclpy.create_node("moveitping")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    joints = {}
    node.create_subscription(
        JointState, "/joint_states",
        lambda m: joints.update(dict(zip(m.name, m.position))), 10)
    for _ in range(120):
        rclpy.spin_once(node, timeout_sec=0.1)

    missing = [j for j in CHAIN if j not in joints]
    if missing:
        print("no joint state for: %s" % ", ".join(missing))
        return 1
    here = [joints[j] for j in CHAIN]
    print("the arm is at:")
    for name, value in zip(CHAIN, here):
        print("  %-22s %+.4f" % (name, value))

    client = MoveItClient("moveitping_client")
    print("\nwaiting for move_group")
    try:
        ok = client.move.wait_for_server(timeout_sec=30.0)
        print("  action server present: %s" % ok)
    except Exception as exc:  # noqa: BLE001
        print("  wait_for_server raised %r" % exc)
        traceback.print_exc()
        return 1

    # Ask for where it already is, plus a hair on one joint. If THIS fails, nothing about
    # the arm's reach or the shelf is involved.
    target = list(here)
    target[1] += 0.05
    print("\nasking for arm_left_1_joint %+.4f -> %+.4f" % (here[1], target[1]))
    try:
        code = client.move_to_joints(CHAIN, target, timeout=90.0)
        print("  returned %r" % (code,))
        print("  last_failure: %r" % (client.last_failure,))
        if code == 1:
            print("\n  MoveIt plans and executes here. The 99999 is something else.")
        else:
            print("\n  MoveIt is refusing a trivial move; that is the blocker, and it is")
            print("  nothing to do with the grasp geometry.")
    except Exception as exc:  # noqa: BLE001
        print("  RAISED %r" % exc)
        traceback.print_exc()
        print("\n  This is what grasp_node turns into 'error 99999'.")

    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
