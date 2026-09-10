#!/usr/bin/env python3
"""What does the base actually do for a given cmd_vel, at the speeds a hold uses?

    tools/in-sim drivecurve.py [seconds] [speed ...]

The base hold is a proportional controller with a 40 mm/s clip, and it lost: over one
grasp the base ended 534 mm out of position, which put the book past the end of the arm
and the run clamped on air. The arithmetic says it should have won -- a 3 to 8 mm/s drift
against a controller that can ask for 40 -- so either the base does not do what it is
asked at these speeds, or the hold is not asking.

tools/hold_base.py already carries the suspicion, in a comment rather than a measurement:
"commanded 0.02 m/s the base simply sits there". Everything about the hold's gain, clip
and deadband depends on where that floor really is, so it is worth a number.

Each speed is commanded forward for a window and the true displacement measured, then the
same window with nothing commanded, so the base's own drift is subtracted rather than
attributed to the command. Reported as the fraction of the commanded speed that arrives.
"""
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist

WINDOW = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
SPEEDS = [float(a) for a in sys.argv[2:]] or [0.01, 0.02, 0.03, 0.04, 0.06, 0.10]


def gz(*args, timeout=20):
    try:
        return subprocess.run(["gz", *args], capture_output=True, text=True,
                              timeout=timeout).stdout
    except Exception:  # noqa: BLE001
        return ""


def truth():
    raw = gz("topic", "-e", "-t", "/world/erc_world/dynamic_pose/info", "-n", "1")
    name = None
    section = None
    fields = {}
    for line in raw.splitlines():
        line = line.strip()
        if line.startswith('name: "'):
            if name == "tiago_pro" and "qw" in fields:
                break
            name = line.split('"')[1]
            section = None
            fields = {}
        elif line.startswith("position"):
            section = "p"
        elif line.startswith("orientation"):
            section = "q"
        elif name and section and ":" in line:
            key, _, value = line.partition(":")
            key = key.strip()
            if key in ("x", "y", "z", "w"):
                tag = section + key
                if tag not in fields:
                    try:
                        fields[tag] = float(value)
                    except ValueError:
                        pass
    if name != "tiago_pro" or "qw" not in fields:
        return None
    yaw = math.atan2(2.0 * (fields["qw"] * fields["qz"] + fields["qx"] * fields["qy"]),
                     1.0 - 2.0 * (fields["qy"] ** 2 + fields["qz"] ** 2))
    return fields["px"], fields["py"], yaw


def run(node, pub, speed, seconds):
    """Command a forward speed for a window; return the displacement along the base's x."""
    start = truth()
    if start is None:
        return None
    began = node.get_clock().now().nanoseconds * 1e-9
    while node.get_clock().now().nanoseconds * 1e-9 - began < seconds:
        twist = Twist()
        twist.linear.x = float(speed)
        pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.03)
    pub.publish(Twist())
    elapsed = node.get_clock().now().nanoseconds * 1e-9 - began
    end = truth()
    if end is None:
        return None
    dx, dy = end[0] - start[0], end[1] - start[1]
    yaw = start[2]
    ahead = dx * math.cos(yaw) + dy * math.sin(yaw)
    return ahead, elapsed


def main():
    rclpy.init()
    node = rclpy.create_node("drivecurve")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    deadline = time.time() + 5.0
    while time.time() < deadline and node.get_clock().now().nanoseconds == 0:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("%.0f simulated seconds per trial, each command paired with a silent control"
          % WINDOW)
    print("")
    print("  commanded   ahead with   ahead with     arrived    fraction")
    print("   m/s        command mm   nothing mm      mm/s      of asked")
    print("  " + "-" * 62)

    for speed in SPEEDS:
        driven = run(node, pub, speed, WINDOW)
        idle = run(node, pub, 0.0, WINDOW)
        if driven is None or idle is None:
            print("  %-10.3f (no ground truth)" % speed)
            continue
        gained = driven[0] - idle[0]
        arrived = gained / max(driven[1], 1e-6)
        print("  %-10.3f %10.1f  %11.1f  %10.1f  %9.2f"
              % (speed, driven[0] * 1000, idle[0] * 1000, arrived * 1000,
                 arrived / speed if speed else 0.0))

    pub.publish(Twist())
    print("")
    print("The fraction is what a proportional gain is really multiplied by. A hold that")
    print("clips at 40 mm/s and gets a third of it is a hold with 13 mm/s of authority.")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
