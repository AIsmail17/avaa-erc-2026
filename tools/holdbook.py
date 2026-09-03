#!/usr/bin/env python3
"""Does holding the base against the BOOK stop the coast, where a zero twist does not?

    tools/in-sim holdbook.py [gain] [seconds]

coast.py: the base holds one velocity indefinitely, 7.7 mm per simulated second over
eight windows, heading agreement 0.98, and a zero twist at 20 Hz does not touch it. The
wheels have no friction across the roller axis, so commanding zero wheel speed asks them
not to turn rather than asking the base to stop.

stopcoast.py showed the slide can be driven out -- 7.1 mm/s down to 2.0 at a gain of 2 --
but it measured the slide from Gazebo, which a scored run does not have.

It does have the book. Perception puts it in base_link to 15-35 mm, and when the base
slides the book appears to move by exactly as much in the opposite sense. So this closes
a POSITION loop on it: remember where the book was, and drive to put it back there. No
differentiation, so perception's noise is not amplified, and the thing being held still
is the thing the arm is aiming at rather than a frame nobody can see.

Compares three windows against Gazebo: nothing commanded, a zero twist, and this.
"""
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist

BOOK = "/avaa/perception/target_book_point"
MAX_SPEED = 0.05
DEADBAND = 0.008


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
    gain = float(sys.argv[1]) if len(sys.argv) > 1 else 1.2
    span = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

    rclpy.init()
    node = rclpy.create_node("holdbook")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    seen = []
    node.create_subscription(
        PointStamped, BOOK,
        lambda m: seen.append([m.point.x, m.point.y, m.point.z]), 10)

    def sim_now():
        return node.get_clock().now().nanoseconds * 1e-9

    print("waiting for the book...")
    deadline = time.time() + 40
    while time.time() < deadline and len(seen) < 8:
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(seen) < 8:
        print("perception is not publishing a book point; is the solution running?")
        return 1
    print("book seen %d times, latest %s\n"
          % (len(seen), np.round(seen[-1], 3).tolist()))

    def window(label, hold):
        start_t = sim_now()
        start = truth()
        reference = np.median(np.array(seen[-6:]), axis=0)
        worst = 0.0
        while sim_now() - start_t < span:
            twist = Twist()
            if hold and len(seen) >= 6:
                now = np.median(np.array(seen[-6:]), axis=0)
                error = now - reference
                worst = max(worst, float(np.linalg.norm(error[:2])))
                # The book drifting AWAY in base_link means the base has fallen back,
                # so the correction is towards it, not away.
                for axis, value in ((0, error[0]), (1, error[1])):
                    if abs(value) > DEADBAND:
                        speed = float(np.clip(gain * value, -MAX_SPEED, MAX_SPEED))
                        if axis == 0:
                            twist.linear.x = speed
                        else:
                            twist.linear.y = speed
            pub.publish(twist)
            rclpy.spin_once(node, timeout_sec=0.05)
        where = truth()
        elapsed = max(sim_now() - start_t, 1e-6)
        moved = math.hypot(where[0] - start[0], where[1] - start[1])
        print("%-26s %7.1f mm   %6.2f mm/sim s   book wandered %.0f mm"
              % (label, moved * 1000, moved * 1000 / elapsed, worst * 1000))
        return moved / elapsed

    print("%-26s %10s   %14s   %s" % ("", "travel", "per sim s", ""))
    window("nothing commanded", False)
    window("zero twist at 20 Hz", False)
    held = window("held against the book", True)
    for _ in range(30):
        pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.05)

    print("\ngain %.1f, capped at %.0f mm/s, deadband %.0f mm"
          % (gain, MAX_SPEED * 1000, DEADBAND * 1000))
    print("held at %.2f mm per simulated second" % (held * 1000))

    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
