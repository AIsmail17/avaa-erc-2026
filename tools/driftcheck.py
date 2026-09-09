#!/usr/bin/env python3
"""Does the base drift with nothing commanding it, and does a zero command stop it?

    tools/in-sim driftcheck.py

Every arena grasp so far has ended with the pads short in depth and off to the side, and
the arena feed's ground truth said the book had moved several hundred millimetres in
base_link during the reach while the book itself never moved at all. That can only be the
base. This measures it: a stretch with nothing sent, then a stretch with zero Twist sent
at 20 Hz, then a stretch with nothing again.
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
    yaw = math.atan2(2.0 * (r["qw"] * r["qz"] + r["qx"] * r["qy"]),
                     1.0 - 2.0 * (r["qy"] ** 2 + r["qz"] ** 2))
    return (r["px"], r["py"], yaw)


rclpy.init()
node = rclpy.create_node("driftcheck")
pub = node.create_publisher(Twist, "/mobile_base_controller/cmd_vel_unstamped", 10)
alt = node.create_publisher(Twist, "/cmd_vel", 10)


def stretch(label, seconds, send_zero):
    start = base()
    end_at = time.time() + seconds
    while time.time() < end_at:
        if send_zero:
            pub.publish(Twist())
            alt.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.05)
    stop = base()
    if start is None or stop is None:
        print("  %-28s no pose" % label)
        return
    moved = math.hypot(stop[0] - start[0], stop[1] - start[1])
    turned = math.degrees(math.atan2(math.sin(stop[2] - start[2]),
                                     math.cos(stop[2] - start[2])))
    print("  %-28s moved %6.1f mm, turned %+6.2f deg  (%.1f mm/s)"
          % (label, moved * 1000, turned, moved * 1000 / seconds))


print("base drift, wall clock:")
stretch("nothing sent, 15 s", 15.0, False)
stretch("zero Twist at 20 Hz, 15 s", 15.0, True)
stretch("nothing sent again, 15 s", 15.0, False)

node.destroy_node()
rclpy.shutdown()
