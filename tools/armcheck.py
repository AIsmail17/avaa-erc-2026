#!/usr/bin/env python3
"""Are the arms actually where the tuck asked them to be?

    tools/in-sim armcheck.py

Prints each arm joint against the driving posture, and where each gripper ends up in
base_link. The question is whether "stowing arms for driving" produced a stowed arm or
only a request for one.
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.kinematics.arm_chain import ArmChain  # noqa: E402

TUCK_LEFT = [2.1521, 0.3824, 1.2785, -2.1517, 0.8325, 0.1926, 1.3944]
TUCK_RIGHT = [-0.7194, -2.2867, -0.5064, 0.5221, 2.3399, 1.0503, 1.9772]
TUCK_TORSO = 0.15


class Watch(Node):
    def __init__(self):
        super().__init__("armcheck")
        self.joints = {}
        self.create_subscription(JointState, "/joint_states", self._on, 10)

    def _on(self, msg):
        for name, value in zip(msg.name, msg.position):
            self.joints[name] = value


def main():
    rclpy.init()
    node = Watch()
    end = time.time() + 12
    while time.time() < end and len(node.joints) < 10:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not node.joints:
        print("no joint states")
        return

    for side, wanted in (("left", TUCK_LEFT), ("right", TUCK_RIGHT)):
        names = ["arm_%s_%d_joint" % (side, i) for i in range(1, 8)]
        actual = [node.joints.get(n) for n in names]
        if any(a is None for a in actual):
            print("%s arm: joints not published" % side)
            continue
        gaps = [a - w for a, w in zip(actual, wanted)]
        worst = max(range(7), key=lambda i: abs(gaps[i]))
        print("%s arm" % side)
        print("   wanted %s" % " ".join("%+.2f" % v for v in wanted))
        print("   actual %s" % " ".join("%+.2f" % v for v in actual))
        print("   gap    %s   worst %s %+.2f rad (%.0f deg)"
              % (" ".join("%+.2f" % v for v in gaps), names[worst], gaps[worst],
                 abs(gaps[worst]) * 57.3))

    torso = node.joints.get("torso_lift_joint")
    print("torso  wanted %+.3f  actual %s"
          % (TUCK_TORSO, "?" if torso is None else "%+.3f" % torso))

    # Where the left gripper actually is, which is what hits the shelf.
    chain = ArmChain.from_urdf()
    values = [node.joints.get("torso_lift_joint")] + [
        node.joints.get("arm_left_%d_joint" % i) for i in range(1, 8)]
    if all(v is not None for v in values):
        here = chain.position(values)
        print("left gripper now      x=%+.3f y=%+.3f z=%+.3f (base_link)" % tuple(here))
        tucked = chain.position([TUCK_TORSO] + TUCK_LEFT)
        print("left gripper if tucked x=%+.3f y=%+.3f z=%+.3f"
              % tuple(tucked))
        print("difference %.0f mm" % (np.linalg.norm(here - tucked) * 1000))

    node.destroy_node()
    rclpy.shutdown()


main()
