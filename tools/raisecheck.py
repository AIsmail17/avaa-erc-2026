#!/usr/bin/env python3
"""Why does the torso raise come back "invalid motion plan"?

    tools/in-sim raisecheck.py

The first move of a grasp is to raise the torso with the arm still folded, and for a book
on row 1 it fails:

    scene -> raising
    raise returned -2 (moveit says '')
    could not raise the torso first (invalid motion plan); reaching from where we are
    ignoring a sighting 1121 mm from the shoulder; the arm reaches 1088 mm

The consequence is exact. The left shoulder sits at 0.677 + torso, so at torso zero it is
714 mm below a row 1 book, and the geometry then needs 1121 mm against a measured limit of
1088. Raise the torso to its 0.35 stop and the shoulder comes up to 1.027, the vertical
gap drops to 364 mm, and the same book is comfortably inside the arm.

So the whole of a row 1 grasp turns on this one motion, and -2 says only that a plan could
not be found -- not whether the fault is the state it starts in, the state it is asked to
end in, or the space between them. This asks /check_state_validity directly:

  - the state the robot is in now
  - the exact goal, torso at the clip with the left arm at TUCK_POSE
  - the goal at a ladder of torso heights, to find where validity changes

Whatever is in the planning scene stays there, because the shelf boxes grasp_node adds are
part of the question.
"""
import sys

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState

# Straight from grasp_node.py.
TUCK_POSE = [0.3877, -1.6152, 0.0717, 0.0408, 0.5074, -1.4708, 0.6063]
ARM_JOINTS = ["arm_left_%d_joint" % i for i in range(1, 8)]
TORSO = "torso_lift_joint"
SHOULDER_BASE_Z = 0.677
ROW_1_PREGRASP_Z = 1.346

GROUP = sys.argv[1] if len(sys.argv) > 1 else ""


def main():
    rclpy.init()
    node = rclpy.create_node("raisecheck")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])

    latest = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: latest.__setitem__("msg", m), 10)
    for _ in range(150):
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

    def check(label, overrides):
        """Ask about the current state with some joints replaced."""
        js = JointState()
        js.header = latest["msg"].header
        js.name = list(latest["msg"].name)
        js.position = list(latest["msg"].position)
        for name, value in overrides.items():
            if name in js.name:
                js.position[js.name.index(name)] = float(value)
            else:
                js.name.append(name)
                js.position.append(float(value))
        request = GetStateValidity.Request()
        request.group_name = GROUP
        state = RobotState()
        state.joint_state = js
        request.robot_state = state
        future = client.call_async(request)
        for _ in range(300):
            rclpy.spin_once(node, timeout_sec=0.1)
            if future.done():
                break
        if not future.done():
            print("  %-34s no answer" % label)
            return None
        result = future.result()
        pairs = []
        seen = set()
        for c in result.contacts:
            pair = (c.contact_body_1, c.contact_body_2)
            if pair not in seen:
                seen.add(pair)
                pairs.append(pair)
        print("  %-34s %s%s"
              % (label, "VALID" if result.valid else "INVALID",
                 "" if result.valid or not pairs else
                 "   " + ", ".join("%s vs %s" % p for p in pairs[:3])))
        return result.valid

    names = list(latest["msg"].name)
    now_torso = (latest["msg"].position[names.index(TORSO)]
                 if TORSO in names else float("nan"))
    now_arm = [latest["msg"].position[names.index(j)] if j in names else 0.0
               for j in ARM_JOINTS]
    ideal = min(max(ROW_1_PREGRASP_Z - SHOULDER_BASE_Z + 0.25, 0.0), 0.35)

    print("group checked: %r  (empty means the whole robot)" % GROUP)
    print("torso now      %+.4f" % now_torso)
    print("left arm now   [%s]" % ", ".join("%+.3f" % v for v in now_arm))
    print("raise goal     torso %.3f with the arm at TUCK_POSE" % ideal)
    print()

    print("the two states the raise is between:")
    check("current state, as it stands", {})
    check("goal: torso %.2f + TUCK_POSE" % ideal,
          dict({TORSO: ideal}, **dict(zip(ARM_JOINTS, TUCK_POSE))))
    print()

    print("the goal posture at a ladder of torso heights:")
    for height in (0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35):
        check("torso %.2f + TUCK_POSE" % height,
              dict({TORSO: height}, **dict(zip(ARM_JOINTS, TUCK_POSE))))
    print()

    print("the torso alone, arm left exactly where it is:")
    for height in (0.0, 0.10, 0.20, 0.30, 0.35):
        check("torso %.2f, arm unchanged" % height, {TORSO: height})

    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
