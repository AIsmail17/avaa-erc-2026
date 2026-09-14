#!/usr/bin/env python3
"""Where does perception put what it sees, against where Gazebo says it is?

    tools/in-sim lateralprobe.py

The laptop run of 2026-09-14 closed the jaws 144 mm to the left of its book: perception had
placed the book at y +0.24 in base_link from the first close fix, before the arm moved, and
Gazebo put it near +0.10. Run 23 missed by 84 mm the same way.

This takes one colour and one depth frame, finds every book-shaped blob and the bin the way
perception does, deprojects each box centre with the same depth sampling and TF, turns it
into world coordinates with the robot's true pose from Gazebo, and prints it beside the
nearest real object of that colour: the error along the robot's own axes, ahead and across.
"""
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.vision import book_detector as bd                   # noqa: E402
from avaa_solution.vision import depth_locator as dl                   # noqa: E402

RGB = "/head_front_camera/head_front_camera/color/image_raw"
DEPTH = "/head_front_camera/head_front_camera/depth/image_rect_raw"
INFO = "/head_front_camera/head_front_camera/depth/camera_info"


def poses():
    out = subprocess.run(["gz", "topic", "-e", "-t", "/world/erc_world/pose/info", "-n", "1"],
                         capture_output=True, text=True, timeout=40).stdout
    found, name, section, vals = {}, None, None, {}
    for line in out.splitlines() + ['name: "__end__"']:
        s = line.strip()
        if s.startswith('name: "'):
            if name is not None and name not in found and "px" in vals:
                found[name] = vals
            name, section, vals = s.split('"')[1], None, {}
        elif s.startswith("position"):
            section = "p"
        elif s.startswith("orientation"):
            section = "q"
        elif section and ":" in s:
            key, _, value = s.partition(":")
            key = key.strip()
            if key in ("x", "y", "z", "w") and section + key not in vals:
                try:
                    vals[section + key] = float(value)
                except ValueError:
                    pass
    for v in found.values():
        for k in ("px", "py", "pz", "qx", "qy", "qz"):
            v.setdefault(k, 0.0)
        v.setdefault("qw", 1.0)
    return found


def main():
    rclpy.init()
    node = rclpy.create_node("lateralprobe")
    bridge = CvBridge()
    got = {}
    node.create_subscription(Image, RGB, lambda m: got.__setitem__("rgb", m),
                             qos_profile_sensor_data)
    node.create_subscription(Image, DEPTH, lambda m: got.__setitem__("depth", m),
                             qos_profile_sensor_data)
    node.create_subscription(CameraInfo, INFO, lambda m: got.__setitem__("info", m),
                             qos_profile_sensor_data)
    buf = Buffer()
    TransformListener(buf, node)
    end = time.time() + 30
    tf = None
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if {"rgb", "depth", "info"} <= set(got):
            try:
                tf = buf.lookup_transform("base_link", got["depth"].header.frame_id,
                                          rclpy.time.Time())
                break
            except Exception:  # noqa: BLE001
                pass
    if tf is None:
        print("no frames or no transform after 30 s: %s" % sorted(got))
        return
    world = poses()
    robot = world["tiago_pro"]
    yaw = math.atan2(2 * (robot["qw"] * robot["qz"] + robot["qx"] * robot["qy"]),
                     1 - 2 * (robot["qy"] ** 2 + robot["qz"] ** 2))
    c, s = math.cos(yaw), math.sin(yaw)
    t = tf.transform.translation
    print("robot at (%.3f, %.3f) yaw %+.1f deg; depth frame %s at [%.3f, %.3f, %.3f] in base_link"
          % (robot["px"], robot["py"], math.degrees(yaw), got["depth"].header.frame_id,
             t.x, t.y, t.z))
    print("colour frame %s, %dx%d; depth %dx%d; fx %.1f cx %.1f"
          % (got["rgb"].header.frame_id, got["rgb"].width, got["rgb"].height,
             got["depth"].width, got["depth"].height, got["info"].k[0], got["info"].k[2]))

    bgr = bridge.imgmsg_to_cv2(got["rgb"], desired_encoding="bgr8")
    depth = bridge.imgmsg_to_cv2(got["depth"], desired_encoding="passthrough")
    intr = dl.Intrinsics.from_k(got["info"].k)
    blobs = [(b, "book_") for b in bd.detect_books(bgr)]
    blobs += [(b, "erc_collection_bin") for b in bd.detect_bin_candidates(bgr)[:1]]
    for blob, prefix in blobs:
        optical = dl.locate(blob.bbox, depth, intr)
        if optical is None:
            print("%-6s box at %3.0f px: no depth" % (blob.colour, blob.cx))
            continue
        p = dl.transform_point(optical, tf.transform.rotation, tf.transform.translation)
        wx = robot["px"] + c * p[0] - s * p[1]
        wy = robot["py"] + s * p[0] + c * p[1]
        if prefix == "book_":
            names = [n for n in world if n.startswith("book_") and n.endswith(blob.colour)]
        else:
            names = [prefix] if prefix in world else []
        if not names:
            continue
        best = min(names, key=lambda n: math.hypot(world[n]["px"] - wx, world[n]["py"] - wy))
        dx, dy = world[best]["px"] - wx, world[best]["py"] - wy
        ahead, across = c * dx + s * dy, -s * dx + c * dy
        print("%-6s box %3dx%-3d at %3.0f px, range %.2f: seen at base [%.3f, %+.3f, %.3f], "
              "%s is %+.0f mm ahead and %+.0f mm across of that"
              % (blob.colour, blob.w, blob.h, blob.cx, optical[2], p[0], p[1], p[2], best,
                 ahead * 1000, across * 1000))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
