#!/usr/bin/env python3
"""How fast does the base YAW while nobody is driving it, per SIMULATED second?

    tools/in-sim yawrate.py [windows] [seconds each]

grasp_node's re-aim budget models the base coasting in a straight line -- 7 to 8 mm per
simulated second, measured, with a consistent heading. It does not model the base turning,
and turning is what actually moves a target: dp/dt = -v - w x p, so at 0.9 m a yaw rate of
0.01 rad/s carries the book 9 mm/s all on its own, as much again as the coast.
"""
import math
import subprocess
import sys

import rclpy
from rosgraph_msgs.msg import Clock

WORLD = "erc_world"
WINDOWS = int(sys.argv[1]) if len(sys.argv) > 1 else 5
EACH = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0


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
node = rclpy.create_node("yawrate")
sim = {}
node.create_subscription(
    Clock, "/clock",
    lambda m: sim.__setitem__("t", m.clock.sec + m.clock.nanosec * 1e-9), 10)
for _ in range(200):
    rclpy.spin_once(node, timeout_sec=0.05)
    if "t" in sim:
        break
if "t" not in sim:
    print("no /clock")
    raise SystemExit(1)

print("per SIMULATED second, nothing commanded:")
speeds, yaws = [], []
for i in range(WINDOWS):
    t0, p0 = sim["t"], base()
    while sim["t"] - t0 < EACH and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
    t1, p1 = sim["t"], base()
    if p0 is None or p1 is None:
        continue
    dt = t1 - t0
    moved = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    turned = math.atan2(math.sin(p1[2] - p0[2]), math.cos(p1[2] - p0[2]))
    speeds.append(moved / dt)
    yaws.append(turned / dt)
    print("  window %d over %.1f sim s: %5.1f mm/s, %+7.4f rad/s (%+5.2f deg/s)"
          % (i + 1, dt, moved / dt * 1000, turned / dt, math.degrees(turned / dt)))

if speeds:
    mean_v = sum(speeds) / len(speeds)
    mean_w = sum(yaws) / len(yaws)
    print()
    print("  mean %.1f mm/s and %+.4f rad/s (%+.2f deg/s)"
          % (mean_v * 1000, mean_w, math.degrees(mean_w)))
    for r in (0.7, 0.9, 1.1):
        print("  a book %.1f m out therefore moves %.1f mm/s: %.1f from the coast, "
              "%.1f from the turn"
              % (r, (mean_v + abs(mean_w) * r) * 1000, mean_v * 1000,
                 abs(mean_w) * r * 1000))

node.destroy_node()
rclpy.shutdown()
