#!/usr/bin/env python3
"""Can the coast be measured from the book, and then driven out?

    tools/in-sim bookcoast.py [gain] [watch_s] [drive_s]

The pieces are established. coast.py: the base holds one velocity indefinitely, 7.7 mm
per simulated second over eight windows on one heading, agreement 0.98, and a zero twist
at 20 Hz does not touch it -- the wheels have no friction across the roller axis, so
commanding zero wheel speed asks them not to turn rather than asking the base to stop.
stopcoast.py: commanding the measured slide negated takes 7.1 mm/s down to 2.0 at a gain
of 2. But that measured the slide from Gazebo, which a scored run does not have.

The book is what a scored run has, but the book alone is not enough, and the first
version of this proved it: it read 61.9 and -76.9 mm/s against a true 6.0 and -6.0, and
driving on that took the drift from 8.5 mm/s to 22.3. The base ROTATES as well as
slides, and a rotation moves a point at 0.8 m far more than a slide does -- the terms are
w*py and w*px, so at the ranges here the turning swamps the sliding.

Odom can supply the missing half. It is blind to a slide, which is the whole problem with
it and why nothing else here trusts it, but turning is the one thing that genuinely
rotates these wheels, so its yaw rate is good. For a point fixed in the world, seen from
a base moving at v and turning at w,

    dp/dt = -v - w x p        so    vx = -dpx/dt + w*py
                                    vy = -dpy/dt - w*px

which is what this fits. A single difference of two sightings is mostly noise at these
speeds; a least-squares slope over ten seconds of them is not, which is the whole reason
for watching first.

Then it drives the estimate out and keeps driving it, because a constant velocity stays
cancelled only while something is cancelling it. That also means the estimate survives
the arm occluding the book, which it will: the velocity is constant, so a stale estimate
of it is still a good one.

Prints the estimate beside Gazebo's answer, then what is left after driving.
"""
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry

BOOK = "/avaa/perception/target_book_point"
ODOM = "/mobile_base_controller/odom"
MAX_SPEED = 0.06
# A coast is a few millimetres a second. An estimate an order of magnitude above that
# is not a coast, it is a bad fit, and driving on one is worse than doing nothing.
PLAUSIBLE = 0.030


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
    gain = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    watch = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
    drive = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0

    rclpy.init()
    node = rclpy.create_node("bookcoast")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    samples = []

    spin = {"w": 0.0}

    def on_book(msg):
        samples.append((node.get_clock().now().nanoseconds * 1e-9,
                        msg.point.x, msg.point.y, spin["w"]))

    node.create_subscription(PointStamped, BOOK, on_book, 10)
    node.create_subscription(
        Odometry, ODOM,
        lambda m: spin.__setitem__("w", float(m.twist.twist.angular.z)), 10)

    def sim_now():
        return node.get_clock().now().nanoseconds * 1e-9

    deadline = time.time() + 40
    while time.time() < deadline and len(samples) < 4:
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(samples) < 4:
        print("no book in view -- drive in front of one first")
        return 1

    # --- watch ------------------------------------------------------------------
    samples.clear()
    start_t = sim_now()
    before, _ = truth()
    while sim_now() - start_t < watch:
        pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.05)
    after, rpy = truth()
    span = sim_now() - start_t

    if len(samples) < 8:
        print("only %d sightings in %.0f s; not enough to fit a slope"
              % (len(samples), watch))
        return 1

    times = np.array([s[0] for s in samples]) - samples[0][0]
    px = np.array([s[1] for s in samples])
    py = np.array([s[2] for s in samples])
    spin_rate = float(np.mean([s[3] for s in samples]))
    # The book moving one way in base_link is the base moving the other, once the part
    # of that motion which is the base TURNING has been taken out.
    vx = -float(np.polyfit(times, px, 1)[0]) + spin_rate * float(np.mean(py))
    vy = -float(np.polyfit(times, py, 1)[0]) - spin_rate * float(np.mean(px))

    # Gazebo's answer, rotated into the base frame for comparison.
    yaw = rpy[2]
    dx, dy = (after[0] - before[0]) / span, (after[1] - before[1]) / span
    true_x = dx * math.cos(yaw) + dy * math.sin(yaw)
    true_y = -dx * math.sin(yaw) + dy * math.cos(yaw)

    print("%d sightings over %.0f simulated seconds\n" % (len(samples), span))
    print("%-12s %-12s %-12s" % ("", "forward", "left"))
    print("%-12s %-12.2f %-12.2f  mm/sim s" % ("from book", vx * 1000, vy * 1000))
    print("%-12s %-12.2f %-12.2f  mm/sim s" % ("from Gazebo", true_x * 1000,
                                               true_y * 1000))
    print("%-12s %-12.2f %-12.2f  mm/sim s\n"
          % ("error", (vx - true_x) * 1000, (vy - true_y) * 1000))

    # --- drive it out ------------------------------------------------------------
    twist = Twist()
    twist.linear.x = float(np.clip(-gain * vx, -MAX_SPEED, MAX_SPEED))
    twist.linear.y = float(np.clip(-gain * vy, -MAX_SPEED, MAX_SPEED))
    print("commanding %+.1f, %+.1f mm/s for %.0f simulated seconds"
          % (twist.linear.x * 1000, twist.linear.y * 1000, drive))

    start_t = sim_now()
    before, _ = truth()
    while sim_now() - start_t < drive:
        pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.05)
    after, _ = truth()
    span = max(sim_now() - start_t, 1e-6)
    moved = math.hypot(after[0] - before[0], after[1] - before[1])

    for _ in range(30):
        pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.05)

    print("\nwhile driving against it: %.1f mm in %.0f s, %.2f mm per simulated second"
          % (moved * 1000, span, moved * 1000 / span))
    print("(the coast it started from was %.2f)" % (math.hypot(true_x, true_y) * 1000))

    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
