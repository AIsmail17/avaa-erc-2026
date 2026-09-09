#!/usr/bin/env python3
"""Does commanding linear.y strafe this base, or turn it?

    tools/in-sim strafe.py

grasp_node's base hold corrects a sideways book error by publishing linear.y, which is
right for a mecanum base and wrong for one that answers it with yaw. drive_to has believed
the latter since it was written -- "commanding pure vy yaws this base by roughly the
magnitude it strafes" -- and nothing has ever measured it.

It matters because the hold runs a proportional loop on that channel. If y turns the base,
the loop is positive feedback: turning moves the book sideways in base_link, which asks
for more y, which turns it further.
"""
import math
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist

WORLD = "erc_world"


def poses():
    raw = subprocess.run(["gz", "topic", "-e", "-t",
                          "/world/%s/dynamic_pose/info" % WORLD, "-n", "1"],
                         capture_output=True, text=True, timeout=25).stdout
    out, name, section, fields = {}, None, None, {}
    for line in raw.splitlines():
        line = line.strip()
        if line.startswith('name: "'):
            if name and "px" in fields:
                out[name] = dict(fields)
            name, section, fields = line.split('"')[1], None, {}
        elif line.startswith("position"):
            section = "p"
        elif line.startswith("orientation"):
            section = "q"
        elif name and section and ":" in line:
            k, _, v = line.partition(":")
            k = k.strip()
            if k in ("x", "y", "z", "w") and section + k not in fields:
                try:
                    fields[section + k] = float(v)
                except ValueError:
                    pass
    if name and "px" in fields:
        out[name] = dict(fields)
    return out


def base():
    r = poses().get("tiago_pro")
    if r is None:
        return None
    return (r["px"], r["py"],
            math.atan2(2.0 * (r["qw"] * r["qz"] + r["qx"] * r["qy"]),
                       1.0 - 2.0 * (r["qy"] ** 2 + r["qz"] ** 2)))


rclpy.init()
node = rclpy.create_node("strafe")
pub = node.create_publisher(Twist, "/cmd_vel", 10)


def drive(label, vx, vy, wz, seconds=6.0):
    start = base()
    t = Twist()
    t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
    end_at = time.time() + seconds
    while time.time() < end_at:
        pub.publish(t)
        rclpy.spin_once(node, timeout_sec=0.05)
    stop_at = time.time() + 3.0
    while time.time() < stop_at:
        pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.05)
    stop = base()
    if start is None or stop is None:
        print("  %-26s no pose" % label)
        return
    dx, dy = stop[0] - start[0], stop[1] - start[1]
    # In the frame the robot started in.
    ahead = dx * math.cos(-start[2]) - dy * math.sin(-start[2])
    across = dx * math.sin(-start[2]) + dy * math.cos(-start[2])
    turned = math.degrees(math.atan2(math.sin(stop[2] - start[2]),
                                     math.cos(stop[2] - start[2])))
    print("  %-26s ahead %+6.1f mm, across %+6.1f mm, turned %+6.2f deg"
          % (label, ahead * 1000, across * 1000, turned))


print("commanded for 6 s each, then stopped, measured against ground truth:")
drive("linear.x = +0.04", 0.04, 0.0, 0.0)
drive("linear.x = -0.04", -0.04, 0.0, 0.0)
drive("linear.y = +0.04", 0.0, 0.04, 0.0)
drive("linear.y = -0.04", 0.0, -0.04, 0.0)
drive("angular.z = +0.10", 0.0, 0.0, 0.10)
drive("angular.z = -0.10", 0.0, 0.0, -0.10)

node.destroy_node()
rclpy.shutdown()
