#!/usr/bin/env python3
"""Watch the wheels and the base together, with nothing commanding either.

    tools/in-sim wheelwatch.py [seconds]

The base creeps at about 3 mm/s indefinitely and one wheel reads +0.4233 rad/s while the
other three read exactly zero. Two very different things look like that: a wheel the drive
plugin is still commanding, and a stale number in the joint state. A driven wheel wanders;
a latched one does not move at all.
"""
import math
import subprocess
import sys
import time

import rclpy
from sensor_msgs.msg import JointState

WORLD = "erc_world"
WHEELS = ["wheel_front_left_joint", "wheel_front_right_joint",
          "wheel_rear_left_joint", "wheel_rear_right_joint"]
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0


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
node = rclpy.create_node("wheelwatch")
latest = {}
node.create_subscription(
    JointState, "/joint_states",
    lambda m: latest.update(
        {"vel": dict(zip(m.name, m.velocity)), "pos": dict(zip(m.name, m.position))}),
    10)

for _ in range(60):
    rclpy.spin_once(node, timeout_sec=0.1)
    if "vel" in latest:
        break

print("%-7s %-42s %s" % ("t (s)", "wheel velocities (rad/s)", "base"))
start = time.time()
first = base()
while time.time() - start < SECONDS:
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    vel = latest.get("vel", {})
    pos = latest.get("pos", {})
    here = base()
    moved = ""
    if here and first:
        moved = "moved %5.1f mm, turned %+5.2f deg" % (
            math.hypot(here[0] - first[0], here[1] - first[1]) * 1000,
            math.degrees(math.atan2(math.sin(here[2] - first[2]),
                                    math.cos(here[2] - first[2]))))
    print("%-7.1f %s  %s"
          % (time.time() - start,
             " ".join("%+.4f" % vel.get(w, float("nan")) for w in WHEELS),
             moved))

print()
print("wheel POSITIONS at the end (a driven wheel has turned, a latched one has not):")
for w in WHEELS:
    print("  %-30s %+.4f rad" % (w, latest.get("pos", {}).get(w, float("nan"))))

node.destroy_node()
rclpy.shutdown()
