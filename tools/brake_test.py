#!/usr/bin/env python3
"""Does publishing a zero twist brake the drifting base, or is it already braked?

The base coasts at about 3 mm/s with nothing commanding it. Two very different causes:

  - the drive plugin holds the wheels at zero whenever it is asked to, and the robot
    slides across them anyway, in which case commanding zero changes nothing; or
  - the plugin only actuates when a Twist arrives, so an uncommanded robot has wheels
    that are entirely free, and the drift is momentum coasting on frictionless rollers.

The second is fixable in our own code for the price of one publisher. This measures both
in the same session, back to back, against Gazebo ground truth.

    tools/in-sim brake_test.py [seconds_per_window]
"""
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def truth():
    out = subprocess.run(["gz", "model", "-m", "tiago_pro", "-p"],
                         capture_output=True, text=True, timeout=25).stdout
    lines = [l.strip() for l in out.splitlines()]
    for i, line in enumerate(lines):
        if line.startswith("[") and i + 1 < len(lines) and lines[i + 1].startswith("["):
            return ([float(v) for v in line.strip("[]").split()],
                    [float(v) for v in lines[i + 1].strip("[]").split()])
    return None, None


def window(node, pub, seconds, publishing):
    start_p, start_r = truth()
    t0 = time.time()
    while time.time() - t0 < seconds:
        if publishing:
            pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.05)
    end_p, end_r = truth()
    if start_p is None or end_p is None:
        return None
    dx = end_p[0] - start_p[0]
    dy = end_p[1] - start_p[1]
    dyaw = end_r[2] - start_r[2]
    travel = (dx * dx + dy * dy) ** 0.5
    return travel, travel / seconds * 1000.0, abs(dyaw) / seconds * 57.2958


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    rclpy.init()
    node = Node("brake_test")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    time.sleep(1.0)

    print("each window is %.0f s of wall clock, judged against Gazebo\n" % seconds)
    print("%-28s %10s %12s %12s" % ("", "travel", "speed", "yaw rate"))
    for label, publishing in (("nothing commanded", False),
                              ("zero twist at 20 Hz", True),
                              ("nothing commanded again", False),
                              ("zero twist at 20 Hz again", True)):
        result = window(node, pub, seconds, publishing)
        if result is None:
            print("%-28s   no ground truth" % label)
            continue
        travel, speed, yaw = result
        print("%-28s %8.0f mm %9.1f mm/s %8.2f deg/s"
              % (label, travel * 1000, speed, yaw))

    node.destroy_node()
    rclpy.shutdown()


main()
