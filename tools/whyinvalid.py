#!/usr/bin/env python3
"""Why does MoveIt call the robot's CURRENT state invalid?

    tools/in-sim whyinvalid.py [group]

Both motions in a bench grasp came back 99999, which is not an exception sentinel -- it is
moveit_msgs/MoveItErrorCodes.FAILURE, a standard constant -- and move_group's own log says
what actually happened:

    [ompl] Skipping invalid start state (invalid state)
    [ompl] Motion planning start tree could not be initialized!
    [ompl] Unable to find solution by any of the threads in 0.000458 seconds

Nothing was planned. The START state was already in collision, so OMPL refused before it
began. That happens immediately after grasp_node adds its shelf boxes to the planning
scene, and if the robot is standing inside one of them then every plan fails and the arm
never moves -- which looks exactly like an arm that cannot reach.

This asks /check_state_validity for the current state and prints the colliding pairs.
"""
import sys

import rclpy
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

GROUP = sys.argv[1] if len(sys.argv) > 1 else "arm_left_torso"


def main():
    rclpy.init()
    node = rclpy.create_node("whyinvalid")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])

    latest = {}

    def on_joints(msg):
        latest["msg"] = msg

    node.create_subscription(JointState, "/joint_states", on_joints, 10)
    for _ in range(120):
        rclpy.spin_once(node, timeout_sec=0.1)
        if "msg" in latest:
            break
    if "msg" not in latest:
        print("no /joint_states")
        return 1

    client = node.create_client(GetStateValidity, "/check_state_validity")
    if not client.wait_for_service(timeout_sec=20.0):
        print("no /check_state_validity service -- is move_group up?")
        return 1

    request = GetStateValidity.Request()
    request.group_name = GROUP
    state = RobotState()
    state.joint_state = latest["msg"]
    request.robot_state = state

    future = client.call_async(request)
    for _ in range(300):
        rclpy.spin_once(node, timeout_sec=0.1)
        if future.done():
            break
    if not future.done():
        print("the validity check never answered")
        return 1

    result = future.result()
    print("group %s: the current state is %s"
          % (GROUP, "VALID" if result.valid else "INVALID"))

    if result.contacts:
        print("\ncolliding pairs:")
        seen = set()
        for c in result.contacts:
            pair = (c.contact_body_1, c.contact_body_2)
            if pair in seen:
                continue
            seen.add(pair)
            print("  %-38s  vs  %s" % (pair[0], pair[1]))
    else:
        print("\nno contacts reported")

    if result.cost_sources:
        print("\n%d cost sources" % len(result.cost_sources))
    if result.constraint_result:
        for cr in result.constraint_result:
            print("  constraint: %s" % cr)

    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
