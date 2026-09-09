#!/usr/bin/env python3
"""Does odometry see the drift, or does the base slide without the wheels knowing?

    tools/in-sim odomvstruth.py [seconds]

The base creeps and yaws with nothing commanding it. If the wheels are turning, odometry
sees it and everything downstream can compensate. If the base is SLIDING -- mu2 is 0.0 on
every wheel and no fdir1 is set, so lateral friction is whatever the engine picks -- then
odometry sees nothing, and any check of the form "the base cannot have carried the book
that far" is computed from a number that is too small.

grasp_node has exactly such a check, and it was rejecting true sightings:

    ignoring a sighting that moves the book 252 mm, over the 223 mm the base could have
    carried it
"""
import math
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry

WORLD = "erc_world"
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0


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


def yaw_of(q):
    return math.atan2(2.0 * (q["qw"] * q["qz"] + q["qx"] * q["qy"]),
                      1.0 - 2.0 * (q["qy"] ** 2 + q["qz"] ** 2))


def truth():
    r = poses().get("tiago_pro")
    return None if r is None else (r["px"], r["py"], yaw_of(r))


rclpy.init()
node = rclpy.create_node("odomvstruth")
odom = {}


def on_odom(m):
    p, o = m.pose.pose.position, m.pose.pose.orientation
    odom["v"] = (p.x, p.y, yaw_of({"qw": o.w, "qx": o.x, "qy": o.y, "qz": o.z}))


node.create_subscription(Odometry, "/odom", on_odom, 10)
for _ in range(80):
    rclpy.spin_once(node, timeout_sec=0.1)
    if "v" in odom:
        break
if "v" not in odom:
    print("no /odom")
    raise SystemExit(1)

t0, o0 = truth(), odom["v"]
start = time.time()
while time.time() - start < SECONDS:
    rclpy.spin_once(node, timeout_sec=0.1)

t1, o1 = truth(), odom["v"]


def delta(a, b):
    return (math.hypot(b[0] - a[0], b[1] - a[1]) * 1000,
            math.degrees(math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))))


tm, tt = delta(t0, t1)
om, ot = delta(o0, o1)
print("over %.0f s of wall clock, with nothing commanded:" % SECONDS)
print("  ground truth   moved %6.1f mm, turned %+6.2f deg" % (tm, tt))
print("  odometry       moved %6.1f mm, turned %+6.2f deg" % (om, ot))
print("  UNSEEN by odom %6.1f mm, %+6.2f deg" % (tm - om, tt - ot))
if tm - om > 5.0 or abs(tt - ot) > 1.0:
    print("\nThe base is moving in ways odometry cannot report. Anything that bounds a")
    print("correction by how far the base 'could have' moved is working from a number")
    print("that is too small.")

node.destroy_node()
rclpy.shutdown()
