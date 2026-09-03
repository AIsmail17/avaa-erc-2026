#!/usr/bin/env python3
"""Is the base's drift a random wander, or is it coasting in a straight line?

    tools/in-sim coast.py [windows] [seconds_each]

The distinction decides whether a grasp may believe a sighting that moves the book a
long way. A wander averages out and a correction of 170 mm is a bad look; a coast does
not, and the same 170 mm is the robot genuinely having travelled.

Measured today the base moved 6.5, 7.1, 7.0 and 6.6 mm per simulated second across four
windows, with the arm still and with it swinging, with a zero twist at 20 Hz and with
nothing commanded at all -- four conditions that should differ and did not. That is what
a constant velocity looks like. This prints the direction as well as the distance, which
is what separates the two: a coast holds its heading.

Everything is per SIMULATED second and against Gazebo, not odom, which cannot see a
slide at all.
"""
import math
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def truth():
    out = subprocess.run(["gz", "model", "-m", "tiago_pro", "-p"],
                         capture_output=True, text=True, timeout=25).stdout
    lines = [l.strip() for l in out.splitlines()]
    for i, line in enumerate(lines):
        if line.startswith("[") and i + 1 < len(lines) and lines[i + 1].startswith("["):
            try:
                return [float(v) for v in line.strip("[]").split()]
            except ValueError:
                return None
    return None


def main():
    windows = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    span = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0

    rclpy.init()
    node = rclpy.create_node("coast")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    pub = node.create_publisher(Twist, "/cmd_vel", 10)

    def sim_now():
        return node.get_clock().now().nanoseconds * 1e-9

    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.05)

    print("%-8s %-10s %-10s %-11s %s"
          % ("window", "moved", "per sim s", "direction", "note"))
    where = truth()
    if where is None:
        print("no truth from Gazebo")
        return 1

    headings = []
    for index in range(windows):
        start_t = sim_now()
        start = where
        # A zero twist throughout, which is what the grasp does.
        while sim_now() - start_t < span:
            pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)
        where = truth()
        if where is None:
            continue
        dx, dy = where[0] - start[0], where[1] - start[1]
        moved = math.hypot(dx, dy)
        elapsed = sim_now() - start_t
        heading = math.degrees(math.atan2(dy, dx))
        headings.append(heading)
        print("%-8d %-10.1f %-10.2f %-11.0f %s"
              % (index + 1, moved * 1000, moved * 1000 / max(elapsed, 1e-6),
                 heading, "mm, mm/s, deg"))

    if len(headings) >= 3:
        # Circular spread. A coast holds one heading; a wander does not.
        sx = sum(math.sin(math.radians(h)) for h in headings) / len(headings)
        cx = sum(math.cos(math.radians(h)) for h in headings) / len(headings)
        r = math.hypot(sx, cx)
        print("\nheading agreement %.2f of 1.0 over %d windows" % (r, len(headings)))
        if r > 0.9:
            print("The base is COASTING: one direction, held. A zero twist does not")
            print("remove a velocity these wheels have no friction to shed, so a long")
            print("correction from perception may be the robot having really moved.")
        elif r < 0.5:
            print("The base is WANDERING: no single direction. A large correction is")
            print("a bad sighting rather than real travel.")
        else:
            print("Neither cleanly. Do not build a threshold on this.")

    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
