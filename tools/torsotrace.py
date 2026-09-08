#!/usr/bin/env python3
"""Trace the torso lift against the mission phase, for as long as you leave it running.

    tools/in-sim torsotrace.py [seconds]

The torso is the cheapest height this robot has -- rated 2000 N against 26 Nm at the arm
joints -- and watching a run it is hard to tell whether it is being used at all, because
its rated velocity is 0.035 m/s. Full travel is 0.35 m, so ten simulated seconds end to
end, and at a real-time factor of 0.1 that is a hundred seconds of wall clock for a
motion of one hand-width. It looks stationary and is not.

So this prints where it is, where the grasp would want it for each row, and what phase
the mission is in, and lets the trace answer the question.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# From grasp_node: the shoulder sits this far up the torso, and the aim is a quarter of
# a metre above the target so the arm reaches DOWN.
SHOULDER_BASE_Z = 0.677
ROW_HEIGHTS = [1.391, 1.061, 0.731, 0.401]
TORSO_MIN, TORSO_MAX = 0.0, 0.35


def ideal_for(height):
    return min(max(height - SHOULDER_BASE_Z + 0.25, TORSO_MIN), TORSO_MAX)


class Trace(Node):
    def __init__(self):
        super().__init__("torsotrace")
        self.torso = None
        self.phase = "?"
        self.grasp = "?"
        self.create_subscription(JointState, "/joint_states", self._joints, 10)
        self.create_subscription(String, "/avaa/mission/phase",
                                 lambda m: setattr(self, "phase", m.data), 10)
        self.create_subscription(String, "/avaa/grasp/state",
                                 lambda m: setattr(self, "grasp", m.data), 10)

    def _joints(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name == "torso_lift_joint":
                self.torso = position


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 240.0
    rclpy.init()
    node = Trace()

    print("the grasp aims the torso here, per row:")
    for index, height in enumerate(ROW_HEIGHTS):
        print("  row %d  book at z=%.3f  ->  torso %.3f%s"
              % (index + 1, height, ideal_for(height),
                 "   (clipped at the top of its travel)"
                 if ideal_for(height) >= TORSO_MAX - 1e-9 else ""))
    print()
    print("  %-8s %-12s %-10s %s" % ("wall s", "phase", "grasp", "torso"))

    began = time.time()
    last = None
    while rclpy.ok() and (time.time() - began) < seconds:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.torso is None:
            continue
        moved = last is None or abs(node.torso - last) >= 0.002
        if moved:
            bar = "#" * int(round(40 * node.torso / TORSO_MAX))
            print("  %6.0f   %-12s %-10s %.3f  %s"
                  % (time.time() - began, node.phase, node.grasp, node.torso, bar),
                  flush=True)
            last = node.torso

    print()
    print("torso ended at %.3f" % (node.torso if node.torso is not None else float("nan")))
    node.destroy_node()
    rclpy.shutdown()


main()
