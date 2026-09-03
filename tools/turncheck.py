#!/usr/bin/env python3
"""When the base is told to turn, does it turn, and does odom say so honestly?

    tools/in-sim turncheck.py [rate ...]

The centring controller logs its command beside odom's measured rate, and in a stuck run
those two agreed perfectly -- commanded -0.28, odom -0.30 -- while the pixel error it
was correcting did not move at all across three samples three seconds apart. At 0.3
rad/s that is 51 degrees of sweep on a camera that sees about 60, so one of the two
numbers is not describing the world.

Either the base is not turning and odom is echoing the command, or the base is turning
and the bearing is frozen. This settles which, by putting Gazebo's answer beside both.
"""
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

ODOM = "/odom"


def _truth_once():
    out = subprocess.run(["gz", "model", "-m", "tiago_pro", "-p"],
                         capture_output=True, text=True, timeout=25).stdout
    lines = [line.strip() for line in out.splitlines()]
    for i, line in enumerate(lines):
        if line.startswith("[") and i + 1 < len(lines) and lines[i + 1].startswith("["):
            try:
                return [float(v) for v in lines[i + 1].strip("[]").split()]
            except ValueError:
                return None
    return None


def truth(tries=4):
    for _ in range(tries):
        answer = _truth_once()
        if answer is not None:
            return answer
        time.sleep(0.5)
    raise RuntimeError("Gazebo would not report the robot pose")


def main():
    rates = [float(v) for v in sys.argv[1:]] or [0.0, 0.15, 0.30, 0.45]
    span = 6.0

    rclpy.init()
    node = rclpy.create_node("turncheck")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    spin = {"w": 0.0}
    node.create_subscription(
        Odometry, ODOM,
        lambda m: spin.__setitem__("w", float(m.twist.twist.angular.z)), 10)

    def sim_now():
        return node.get_clock().now().nanoseconds * 1e-9

    for _ in range(60):
        rclpy.spin_once(node, timeout_sec=0.05)

    def unwrap(a, b):
        return (b - a + math.pi) % (2 * math.pi) - math.pi

    print("%-12s %-14s %-14s %s" % ("commanded", "odom says", "Gazebo says", "verdict"))
    for rate in rates:
        twist = Twist()
        twist.angular.z = rate
        start_t = sim_now()
        before = truth()[2]
        readings = []
        while sim_now() - start_t < span:
            pub.publish(twist)
            rclpy.spin_once(node, timeout_sec=0.05)
            readings.append(spin["w"])
        after = truth()[2]
        elapsed = max(sim_now() - start_t, 1e-6)
        real = unwrap(before, after) / elapsed
        heard = sum(readings) / max(len(readings), 1)
        if abs(rate) < 1e-6:
            verdict = "at rest"
        elif abs(real) < 0.3 * abs(rate):
            verdict = "NOT TURNING"
        elif abs(heard - real) > 0.3 * abs(rate):
            verdict = "odom disagrees"
        else:
            verdict = "both agree"
        print("%-12.2f %-14.3f %-14.3f %s" % (rate, heard, real, verdict))
        for _ in range(60):
            pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
