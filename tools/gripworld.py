#!/usr/bin/env python3
"""Check the grasp fix's idea of where the gripper is, against Gazebo.

    tools/in-sim gripworld.py

sim_grasp_fix composes the gripper's world position from the robot's true pose and the
gripper's pose in base_link, then measures the distance to every book. When the jaws
closed around a book it reported the nearest one 667 mm away, so the composition was
wrong. This prints the composed position beside the true one for a link Gazebo also
knows about, which settles it without spending a twenty-minute run.
"""
import math
import subprocess
import sys

import rclpy
from tf2_ros import Buffer, TransformListener

GRASP_LINK = "gripper_left_grasping_link"
KNOWN_LINK = "gripper_left_fingertip_left_link"   # Gazebo keeps this one


def gz(*args, timeout=20):
    try:
        return subprocess.run(["gz", *args], capture_output=True, text=True,
                              timeout=timeout).stdout
    except Exception:  # noqa: BLE001
        return ""


def poses():
    raw = gz("topic", "-e", "-t", "/world/erc_world/dynamic_pose/info", "-n", "1")
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
            key, _, value = line.partition(":")
            key = key.strip()
            if key in ("x", "y", "z", "w") and section + key not in fields:
                try:
                    fields[section + key] = float(value)
                except ValueError:
                    pass
    if name and "px" in fields:
        out[name] = dict(fields)
    return out


def main():
    rclpy.init()
    node = rclpy.create_node("gripworld")
    buf = Buffer()
    listener = TransformListener(buf, node)
    _ = listener
    for _ in range(80):
        rclpy.spin_once(node, timeout_sec=0.1)

    p = poses()
    robot = p.get("tiago_pro")
    if not robot:
        print("no robot pose")
        return 1
    yaw = math.atan2(2.0 * (robot["qw"] * robot["qz"] + robot["qx"] * robot["qy"]),
                     1.0 - 2.0 * (robot["qy"] ** 2 + robot["qz"] ** 2))
    print("robot at (%.3f, %.3f, %.3f), world yaw %+.1f deg"
          % (robot["px"], robot["py"], robot["pz"], math.degrees(yaw)))

    def compose(link):
        tf = buf.lookup_transform("base_link", link, rclpy.time.Time())
        t = tf.transform.translation
        return (robot["px"] + t.x * math.cos(yaw) - t.y * math.sin(yaw),
                robot["py"] + t.x * math.sin(yaw) + t.y * math.cos(yaw),
                robot["pz"] + t.z)

    for link in (GRASP_LINK, KNOWN_LINK):
        try:
            got = compose(link)
        except Exception as exc:  # noqa: BLE001
            print("  %-40s no transform (%s)" % (link, exc))
            continue
        print("  %-40s composed (%.3f, %.3f, %.3f)"
              % (link, got[0], got[1], got[2]))

    books = {k: (v["px"], v["py"], v["pz"]) for k, v in p.items()
             if k.startswith("book_")}
    try:
        here = compose(GRASP_LINK)
    except Exception:  # noqa: BLE001
        print("no gripper transform")
        return 1
    ranked = sorted(((math.dist(here, xyz), k) for k, xyz in books.items()))
    print("\nnearest books to the composed gripper position:")
    for gap, name in ranked[:3]:
        print("  %-34s %6.0f mm" % (name, gap * 1000))

    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
