#!/usr/bin/env python3
"""Does TF agree with Gazebo about where the robot's links actually are?

    tools/in-sim worldtf.py [link ...]

Every height in this project is quoted in base_link and every check of one is made
against Gazebo, which speaks world. The bridge is a single constant. If that constant is
wrong, the error is invisible: the tool that measures the miss uses the same conversion
as the code that caused it, so they agree and the reach looks perfect.

This compares the two without using the constant at all. Gazebo's pose/info topic gives
the world pose of every link; TF gives the same link relative to base_footprint. On a
robot standing on flat ground those two numbers are the same number, and the difference
between TF's base_link and Gazebo's base_link is the constant, measured rather than
assumed.
"""
import subprocess
import sys
import time

import rclpy
from tf2_ros import Buffer, TransformListener

LINKS = sys.argv[1:] or [
    "base_footprint", "base_link", "torso_lift_link",
    "head_front_camera_depth_optical_frame", "gripper_left_grasping_link",
]


def world_poses(timeout=25):
    """Every link's world pose, from one dynamic_pose sample plus the static one."""
    poses = {}
    for topic in ("/world/erc_world/pose/info", "/world/erc_world/dynamic_pose/info"):
        try:
            out = subprocess.run(
                ["gz", "topic", "-e", "-t", topic, "-n", "1"],
                capture_output=True, text=True, timeout=timeout).stdout
        except Exception:  # noqa: BLE001
            continue
        # A pose block carries a position AND an orientation, and both have an x, a y
        # and a z. Only the three inside "position {" are wanted, so track the block.
        name = None
        inside = False
        pending = {}
        for raw in out.splitlines():
            line = raw.strip()
            if line.startswith("name:"):
                name = line.split(":", 1)[1].strip().strip('"')
                inside = False
                pending = {}
            elif line.startswith("position"):
                inside = True
                pending = {}
            elif line.startswith("orientation"):
                inside = False
            elif name and inside and line[:2] in ("x:", "y:", "z:"):
                pending[line[0]] = float(line.split(":", 1)[1])
                if len(pending) == 3:
                    poses.setdefault(name, (pending["x"], pending["y"], pending["z"]))
                    name = None
                    inside = False
    return poses


def main():
    poses = world_poses()
    if not poses:
        print("no pose topic; is the simulator up?")
        return 1

    rclpy.init()
    node = rclpy.create_node("worldtf")
    buf = Buffer()
    TransformListener(buf, node)
    deadline = time.time() + 6.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("%-42s %8s %8s %8s" % ("link", "gz z", "tf z", "diff"))
    print("-" * 70)
    for link in LINKS:
        gz_z = poses.get(link, (None, None, None))[2]
        try:
            tf_z = buf.lookup_transform("base_footprint", link,
                                        rclpy.time.Time()).transform.translation.z
        except Exception:  # noqa: BLE001
            tf_z = None
        print("%-42s %8s %8s %8s"
              % (link,
                 "%+0.4f" % gz_z if gz_z is not None else "    -",
                 "%+0.4f" % tf_z if tf_z is not None else "    -",
                 "%+0.4f" % (gz_z - tf_z) if (gz_z is not None and tf_z is not None)
                 else "    -"))

    print("")
    print("Sampled %d link poses from Gazebo." % len(poses))
    missing = [l for l in LINKS if l not in poses]
    if missing:
        print("Not published as links (TF-only frames): %s" % ", ".join(missing))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
