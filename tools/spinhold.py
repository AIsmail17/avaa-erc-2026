#!/usr/bin/env python3
"""Does driving against odom's yaw rate stop the base turning?

    tools/in-sim spinhold.py [gain ...]

The base coasts in rotation as well as in translation, and the rotation is doing more
damage. A point 0.8 m in front moves w*0.8 sideways per second, so half a degree a
second of unwanted spin looks like 7 mm/s of the book sliding across the frame -- the
same order as the whole translational coast, and it is why an estimate of that coast
built on the book alone came out ten times too large.

Odom is the right sensor for this one. It cannot see a slide, which is why nothing else
here trusts it, but turning genuinely rotates these wheels, so its yaw rate is real. The
approach already damps its own turns against it; the base hold does not, and publishes a
plain zero twist that the wheels have no friction to enforce.

Measured against Gazebo, per SIMULATED second, so the answer is about the robot rather
than about how long the instance has been up.
"""
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

ODOM = "/mobile_base_controller/odom"
MAX_RATE = 0.35


def truth(tries=4):
    """Gazebo's answer, retried: the service drops a request now and then, and a
    measurement tool that dies on one dropped reply wastes the whole window."""
    for attempt in range(tries):
        answer = _truth_once()
        if answer[0] is not None:
            return answer
        time.sleep(0.5)
    raise RuntimeError("Gazebo would not report the robot pose in %d tries" % tries)


def _truth_once():
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


def main():
    gains = [float(v) for v in sys.argv[1:]] or [0.0, 0.5, 1.0, 2.0]
    span = 15.0

    rclpy.init()
    node = rclpy.create_node("spinhold")
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

    def window(gain, active):
        start_t = sim_now()
        _, rpy0 = truth()
        while sim_now() - start_t < span:
            twist = Twist()
            if active:
                twist.angular.z = float(
                    max(-MAX_RATE, min(MAX_RATE, -gain * spin["w"])))
            pub.publish(twist)
            rclpy.spin_once(node, timeout_sec=0.05)
        where, rpy1 = truth()
        elapsed = max(sim_now() - start_t, 1e-6)
        turned = abs(unwrap(rpy0[2], rpy1[2]))
        return math.degrees(turned) / elapsed, where

    print("%-28s %s" % ("", "yaw per simulated second"))
    rate, _ = window(0.0, False)
    print("%-28s %.3f deg" % ("nothing commanded", rate))
    rate, _ = window(0.0, True)
    print("%-28s %.3f deg" % ("zero twist at 20 Hz", rate))
    for gain in gains:
        if gain == 0.0:
            continue
        rate, _ = window(gain, True)
        print("%-28s %.3f deg" % ("against odom, gain %.1f" % gain, rate))

    for _ in range(30):
        pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
