#!/usr/bin/env python3
"""How far above the floor is base_link, really?

    tools/in-sim baseheight.py

Everything in this solution is quoted in base_link and checked against Gazebo, which
speaks world -- and the bridge between them is one number, BASE_LINK_Z = 0.186, written
into six different files. If that number is wrong then every row height is wrong by the
same amount, the arm is commanded to the wrong place at every row, and every tool that
compares a reach against ground truth agrees with the mistake instead of catching it.

This asks three independent sources and prints them side by side:

  * TF, base_footprint to base_link, which is the URDF's answer
  * Gazebo's own pose for the tiago_pro model, which is where the model origin sits
  * the wheel geometry, via the lowest point of base_link's own frame against the floor

A disagreement here is worth more than it looks: a constant vertical offset between the
camera's fixes and ground truth was measured at +115 mm, and this is one of the two
places that could come from.
"""
import subprocess
import sys
import time

import rclpy
from tf2_ros import Buffer, TransformListener

FRAMES = ["base_link", "base_footprint", "torso_lift_link", "wheel_front_left_link",
          "suspension_front_left_link"]


def gz(*args, timeout=20):
    try:
        return subprocess.run(["gz", *args], capture_output=True, text=True,
                              timeout=timeout).stdout
    except Exception:  # noqa: BLE001
        return ""


def model_pose(model, attempts=6):
    for _ in range(attempts):
        lines = [l.strip() for l in gz("model", "-m", model, "-p").splitlines()]
        for i, line in enumerate(lines):
            if line.startswith("[") and i + 1 < len(lines) and lines[i + 1].startswith("["):
                try:
                    return [float(v) for v in line.strip("[]").split()]
                except ValueError:
                    return None
        time.sleep(0.3)
    return None


def link_poses(model):
    """Every link of a model with its world pose, from gz model -m <name>."""
    out = gz("model", "-m", model)
    poses = {}
    name = None
    lines = out.splitlines()
    for i, raw in enumerate(lines):
        line = raw.strip()
        if line.startswith("- Name:"):
            name = line.split(":", 1)[1].strip()
        elif line.startswith("[") and name and i + 1 < len(lines) \
                and lines[i + 1].strip().startswith("["):
            try:
                poses.setdefault(name, [float(v) for v in line.strip("[]").split()])
            except ValueError:
                pass
    return poses


def main():
    rclpy.init()
    node = rclpy.create_node("baseheight")
    buf = Buffer()
    TransformListener(buf, node)
    deadline = time.time() + 6.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("TF, from base_footprint:")
    for frame in FRAMES:
        try:
            t = buf.lookup_transform("base_footprint", frame,
                                     rclpy.time.Time()).transform
        except Exception as exc:  # noqa: BLE001
            print("  %-30s (%s)" % (frame, str(exc)[:46]))
            continue
        print("  %-30s x %+0.3f  y %+0.3f  z %+0.3f"
              % (frame, t.translation.x, t.translation.y, t.translation.z))

    p = model_pose("tiago_pro")
    print("")
    print("Gazebo model origin for tiago_pro: %s"
          % (["%+0.3f" % v for v in p[:3]] if p else "unreadable"))

    poses = link_poses("tiago_pro")
    print("")
    print("Gazebo link world z, for the links TF also knows:")
    for frame in FRAMES:
        if frame in poses:
            print("  %-30s z %+0.3f" % (frame, poses[frame][2]))
    if "base_link" in poses:
        print("")
        print("  so base_link stands %+0.3f m above the floor in Gazebo"
              % poses["base_link"][2])

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
