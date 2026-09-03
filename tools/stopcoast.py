#!/usr/bin/env python3
"""Can the base's coast be cancelled by driving against it?

    tools/in-sim stopcoast.py [gain ...]

coast.py established that the base holds one velocity indefinitely -- 7.7 mm per
simulated second, heading agreement 0.98 over eight windows -- and that a zero twist at
20 Hz does not touch it. That is the mecanum model doing what it was told: mu2 is 0
across the roller axis, so nothing damps a slide, and commanding zero wheel speed asks
the wheels not to turn rather than asking the base to stop.

Which leaves the obvious question, and it is worth asking before anything is built on
the answer: if the slide is measured and the opposite is commanded, does the base stop?
A counter-command of 8 mm/s is small enough that a controller deadband could swallow it
whole.

Measures the coast, drives against it at each gain, and measures what is left.
"""
import math
import subprocess
import sys

import rclpy
from geometry_msgs.msg import Twist


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


def main():
    gains = [float(v) for v in sys.argv[1:]] or [0.0, 1.0, 2.0, 4.0]
    span = 8.0

    rclpy.init()
    node = rclpy.create_node("stopcoast")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    pub = node.create_publisher(Twist, "/cmd_vel", 10)

    def sim_now():
        return node.get_clock().now().nanoseconds * 1e-9

    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.05)

    def measure(twist):
        """Drift over one window while ``twist`` is published at 20 Hz."""
        start_t = sim_now()
        start, _ = truth()
        while sim_now() - start_t < span:
            pub.publish(twist)
            rclpy.spin_once(node, timeout_sec=0.05)
        where, rpy = truth()
        elapsed = max(sim_now() - start_t, 1e-6)
        dx, dy = where[0] - start[0], where[1] - start[1]
        return (dx / elapsed, dy / elapsed, rpy[2])

    print("window is %.0f simulated seconds, all speeds mm per simulated second\n"
          % span)
    print("%-8s %-12s %-12s %s" % ("gain", "world vx", "world vy", "speed left"))

    vx, vy, yaw = measure(Twist())
    print("%-8s %-12.2f %-12.2f %.2f"
          % ("coast", vx * 1000, vy * 1000, math.hypot(vx, vy) * 1000))

    for gain in gains:
        # The slide, expressed in the base's own frame, negated.
        forward = vx * math.cos(yaw) + vy * math.sin(yaw)
        left = -vx * math.sin(yaw) + vy * math.cos(yaw)
        twist = Twist()
        twist.linear.x = -gain * forward
        twist.linear.y = -gain * left
        vx2, vy2, yaw = measure(twist)
        speed = math.hypot(vx2, vy2)
        print("%-8.1f %-12.2f %-12.2f %.2f  (commanded %+.1f, %+.1f mm/s in base)"
              % (gain, vx2 * 1000, vy2 * 1000, speed * 1000,
                 twist.linear.x * 1000, twist.linear.y * 1000))
        vx, vy = vx2, vy2

    # And leave it as still as it can be left.
    for _ in range(40):
        pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
