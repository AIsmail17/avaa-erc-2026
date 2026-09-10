#!/usr/bin/env python3
"""How much of a turn command does the base actually turn, right now, as it stands?

    tools/in-sim turnresponse.py [rates ...]

Delivery turns in place towards the bin, and on the twelfth full run -- the book in the
gripper -- it commanded 0.18 rad/s and the bearing moved five degrees in five seconds, then
not at all. Seeking at 0.35 rad/s finds the bin every time. Whether that is a base that barely
answers small turn commands, or one that is blocked, is a measurement, not an argument.

Each rate is commanded for four seconds at 10 Hz, then zero for three, and the true heading is
read from Gazebo before and after each. Nothing else may be driving the base while this runs.
"""
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist

RATES = [float(a) for a in sys.argv[1:]] or [0.10, 0.18, 0.30, 0.35, -0.18, -0.35]
HOLD = 4.0
REST = 3.0


def pose_of(name):
    try:
        out = subprocess.run(["gz", "topic", "-e", "-t", "/world/erc_world/pose/info", "-n", "1"],
                             capture_output=True, text=True, timeout=20).stdout
    except Exception:  # noqa: BLE001
        return None
    lines = out.splitlines()
    for i, line in enumerate(lines):
        if line.strip() == 'name: "%s"' % name:
            vals = {}
            section = None
            for raw in lines[i + 1:i + 16]:
                s = raw.strip()
                if s.startswith("position"):
                    section = "p"
                elif s.startswith("orientation"):
                    section = "q"
                elif section and ":" in s:
                    k, _, v = s.partition(":")
                    if k in ("x", "y", "z", "w") and section + k not in vals:
                        try:
                            vals[section + k] = float(v)
                        except ValueError:
                            pass
            if "qw" not in vals:
                return None
            qx, qy, qz, qw = (vals.get("qx", 0.0), vals.get("qy", 0.0),
                              vals.get("qz", 0.0), vals["qw"])
            yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            return vals.get("px", 0.0), vals.get("py", 0.0), yaw
    return None


def main():
    rclpy.init()
    node = rclpy.create_node("turnresponse")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)

    robot = pose_of("tiago_pro")
    bin_ = pose_of("erc_collection_bin")
    if robot is None:
        print("no ground truth for the robot; is the simulator up?")
        return 1
    print("robot at (%.3f, %.3f) heading %+.1f deg" % (robot[0], robot[1], math.degrees(robot[2])))
    if bin_ is not None:
        d = math.hypot(bin_[0] - robot[0], bin_[1] - robot[1])
        b = math.degrees(math.atan2(bin_[1] - robot[1], bin_[0] - robot[0]) - robot[2])
        b = (b + 180.0) % 360.0 - 180.0
        print("bin at (%.3f, %.3f): %.2f m away, %+.1f deg off the nose" % (bin_[0], bin_[1], d, b))
    print("")
    print("  commanded   turned    effective   share   moved")
    print("  " + "-" * 50)

    def send(wz, seconds):
        msg = Twist()
        msg.angular.z = float(wz)
        end = time.time() + seconds
        while time.time() < end:
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.1)

    for rate in RATES:
        before = pose_of("tiago_pro")
        t0 = time.time()
        send(rate, HOLD)
        after = pose_of("tiago_pro")
        wall = time.time() - t0
        send(0.0, REST)
        if before is None or after is None:
            print("  %+.2f      (no pose)" % rate)
            continue
        turned = math.degrees(math.atan2(math.sin(after[2] - before[2]), math.cos(after[2] - before[2])))
        moved = math.hypot(after[0] - before[0], after[1] - before[1]) * 1000
        effective = math.radians(turned) / wall
        share = effective / rate if rate else float("nan")
        print("  %+.2f     %+7.1f deg  %+.3f r/s   %4.0f%%  %4.0f mm" % (rate, turned, effective, share * 100, moved))

    send(0.0, 1.0)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
