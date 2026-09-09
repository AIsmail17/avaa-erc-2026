#!/usr/bin/env python3
"""Is RIGHT_TUCK a posture the planner will accept?

    tools/in-sim tuckvalid.py

A validity check after a bench grasp reported two collisions, and this is the one that is
not obviously a bench artifact:

    arm_right_6_link  vs  base_link

That is the stow posture the right arm is deliberately driven to, colliding with the robot
it is attached to. If it is real, then every plan made after the stow starts from an
invalid state and fails before OMPL does any work -- which presents as an arm that never
moves, with no explanation anywhere except a generic 99999.

This asks with NO shelf in the scene, so nothing else is involved, and it asks about the
exact RIGHT_TUCK values rather than whatever the arm happens to have reached.
"""
import sys

import rclpy
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.approach_node import RIGHT_TUCK  # noqa: E402

RIGHT = ["arm_right_%d_joint" % i for i in range(1, 8)]


def main():
    rclpy.init()
    node = rclpy.create_node("tuckvalid")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])

    latest = {}
    node.create_subscription(
        JointState, "/joint_states", lambda m: latest.update({"msg": m}), 10)
    for _ in range(120):
        rclpy.spin_once(node, timeout_sec=0.1)
        if "msg" in latest:
            break
    if "msg" not in latest:
        print("no /joint_states")
        return 1

    client = node.create_client(GetStateValidity, "/check_state_validity")
    if not client.wait_for_service(timeout_sec=20.0):
        print("no /check_state_validity -- is move_group up?")
        return 1

    def check(label, overrides, group=""):
        msg = latest["msg"]
        names = list(msg.name)
        positions = list(msg.position)
        for joint, value in overrides.items():
            if joint in names:
                positions[names.index(joint)] = float(value)
        state = RobotState()
        state.joint_state = JointState()
        state.joint_state.name = names
        state.joint_state.position = positions
        request = GetStateValidity.Request()
        request.group_name = group
        request.robot_state = state
        future = client.call_async(request)
        for _ in range(300):
            rclpy.spin_once(node, timeout_sec=0.1)
            if future.done():
                break
        if not future.done():
            print("  %-34s no answer" % label)
            return
        result = future.result()
        print("  %-34s %s" % (label, "VALID" if result.valid else "INVALID"))
        seen = set()
        for c in result.contacts:
            pair = (c.contact_body_1, c.contact_body_2)
            if pair not in seen:
                seen.add(pair)
                print("        %-32s vs %s" % pair)

    current = {j: latest["msg"].position[list(latest["msg"].name).index(j)]
               for j in RIGHT if j in latest["msg"].name}
    print("right arm now:")
    for j in RIGHT:
        print("  %-22s now %+.4f   RIGHT_TUCK %+.4f"
              % (j, current.get(j, float("nan")), RIGHT_TUCK[RIGHT.index(j)]))

    print("\nvalidity of the WHOLE robot (empty group means every group):")
    check("as it is now", {})
    check("right arm exactly at RIGHT_TUCK", dict(zip(RIGHT, RIGHT_TUCK)))
    check("right arm at all zeros", {j: 0.0 for j in RIGHT})

    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
