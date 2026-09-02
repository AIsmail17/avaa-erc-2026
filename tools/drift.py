#!/usr/bin/env python3
"""How far does the base move on its own, per SIMULATED second?

    tools/in-sim drift.py [seconds_of_wall_clock_per_window]

Every drift figure in this project until now was per second of wall clock, and that is
the wrong denominator. The simulator does not run at a fixed speed: measured on a fresh
launch it manages a real-time factor of 0.59, and after twenty minutes of experiments on
the same instance it manages 0.013. A robot that "drifts 4 mm/s" at one RTF and 4 mm/s
at the other is not doing the same thing at all -- the second is moving forty-five times
faster in the world the physics engine believes in, which is the world the controllers
have to work in.

So this reports millimetres per simulated second, prints the RTF alongside so the
reading can be judged, and refuses to report at all if the clock barely moved.
"""
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


def truth():
    out = subprocess.run(["gz", "model", "-m", "tiago_pro", "-p"],
                         capture_output=True, text=True, timeout=25).stdout
    lines = [l.strip() for l in out.splitlines()]
    for i, line in enumerate(lines):
        if line.startswith("[") and i + 1 < len(lines) and lines[i + 1].startswith("["):
            try:
                return ([float(v) for v in line.strip("[]").split()],
                        [float(v) for v in lines[i + 1].strip("[]").split()])
            except ValueError:
                return None, None
    return None, None


class Watch(Node):
    def __init__(self):
        super().__init__("drift_watch")
        self.now = None
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def _on_clock(self, msg):
        self.now = msg.clock.sec + msg.clock.nanosec / 1e9


def window(node, seconds, publishing):
    while node.now is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    start_p, start_r = truth()
    sim0, wall0 = node.now, time.time()
    while time.time() - wall0 < seconds:
        if publishing:
            node.pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.02)
    end_p, end_r = truth()
    sim = node.now - sim0
    wall = time.time() - wall0
    if start_p is None or end_p is None:
        return None
    dx, dy = end_p[0] - start_p[0], end_p[1] - start_p[1]
    dyaw = end_r[2] - start_r[2]
    travel = (dx * dx + dy * dy) ** 0.5
    return travel, sim, wall, dyaw


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    rclpy.init()
    node = Watch()
    time.sleep(1.0)

    print("%-26s %9s %8s %11s %11s" %
          ("", "travel", "rtf", "per sim s", "yaw/sim s"))
    for label, publishing in (("nothing commanded", False),
                              ("zero twist at 20 Hz", True),
                              ("nothing commanded again", False),
                              ("zero twist at 20 Hz again", True)):
        result = window(node, seconds, publishing)
        if result is None:
            print("%-26s   no ground truth" % label)
            continue
        travel, sim, wall, dyaw = result
        if sim < 0.5:
            print("%-26s %6.0f mm  %.3f   clock barely moved (%.2f s), not reporting"
                  % (label, travel * 1000, sim / wall, sim))
            continue
        print("%-26s %6.0f mm  %.3f  %7.1f mm %8.2f deg"
              % (label, travel * 1000, sim / wall, travel / sim * 1000,
                 abs(dyaw) / sim * 57.2958))

    node.destroy_node()
    rclpy.shutdown()


main()
