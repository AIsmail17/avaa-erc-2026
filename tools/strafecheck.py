#!/usr/bin/env python3
"""Can this base actually move sideways when it is asked to?

    tools/in-sim strafecheck.py [speed] [seconds]

The approach spent eight consecutive ticks with its lateral command saturated at
-0.100 m/s while the lateral error it was correcting GREW from -1.18 m to -1.70 m. That
is not a gain that needs tuning, that is a command doing nothing or doing the opposite,
and there is a reason to suspect the former: the wheels are mecanum, modelled with mu2 =
0 across the roller axis, and sideways is exactly the direction the model gives them no
friction in.

STATE.md has the same symptom recorded from a different run and a different controller,
where "the acquire strafe walked its error from 132 px to 315 px in one direction". It
was fixed by replacing the strafe with a pose controller rather than by asking whether
the strafe worked.

So this asks. Commands a pure lateral velocity, then a pure forward one, and reports
what the base actually did in each -- decomposed into its own forward and lateral axes
against Gazebo, not odom, which cannot see a slide.
"""
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist


def _truth_once():
    out = subprocess.run(["gz", "model", "-m", "tiago_pro", "-p"],
                         capture_output=True, text=True, timeout=25).stdout
    lines = [line.strip() for line in out.splitlines()]
    for i, line in enumerate(lines):
        if line.startswith("[") and i + 1 < len(lines) and lines[i + 1].startswith("["):
            try:
                return ([float(v) for v in line.strip("[]").split()],
                        [float(v) for v in lines[i + 1].strip("[]").split()])
            except ValueError:
                return None, None
    return None, None


def truth(tries=4):
    for _ in range(tries):
        answer = _truth_once()
        if answer[0] is not None:
            return answer
        time.sleep(0.5)
    raise RuntimeError("Gazebo would not report the robot pose")


def main():
    speed = float(sys.argv[1]) if len(sys.argv) > 1 else 0.10
    span = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0

    rclpy.init()
    node = rclpy.create_node("strafecheck")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    pub = node.create_publisher(Twist, "/cmd_vel", 10)

    def sim_now():
        return node.get_clock().now().nanoseconds * 1e-9

    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.05)

    def run(label, vx, vy):
        twist = Twist()
        twist.linear.x, twist.linear.y = vx, vy
        start_t = sim_now()
        before, rpy = truth()
        yaw = rpy[2]
        while sim_now() - start_t < span:
            pub.publish(twist)
            rclpy.spin_once(node, timeout_sec=0.05)
        after, _ = truth()
        elapsed = max(sim_now() - start_t, 1e-6)
        dx, dy = after[0] - before[0], after[1] - before[1]
        # Into the base's own axes, using the heading it started with.
        forward = (dx * math.cos(yaw) + dy * math.sin(yaw)) / elapsed
        left = (-dx * math.sin(yaw) + dy * math.cos(yaw)) / elapsed
        wanted = math.hypot(vx, vy)
        got = math.hypot(forward, left)
        print("%-22s %-11.3f %-11.3f %-11.3f %.0f%%"
              % (label, vx or vy, forward, left,
                 100.0 * got / wanted if wanted else 0.0))
        # Let it settle between trials, as far as it will.
        for _ in range(60):
            pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)
        return forward, left

    print("each command held for %.0f simulated seconds, speeds in m per simulated s\n"
          % span)
    print("%-22s %-11s %-11s %-11s %s"
          % ("commanded", "asked", "went fwd", "went left", "of what was asked"))
    run("forward  +x", speed, 0.0)
    run("backward -x", -speed, 0.0)
    run("left     +y", 0.0, speed)
    run("right    -y", 0.0, -speed)

    print()
    print("A base that cannot strafe will show near zero in the 'went left' column for")
    print("the two lateral trials while the forward ones track. A base that can will")
    print("show the commanded speed in the matching column.")

    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
