#!/usr/bin/env python3
"""Does the depth-based shelf-face fit agree with the truth?

    tools/in-sim planecheck.py [row_height]

The laser version was wired into a controller before this question was asked, and the
answer turned out to be that the laser cannot see the shelf at all. So this comes first
this time: drive nothing, move nothing, just compare the fit against Gazebo from wherever
the robot is standing.

Prints the fitted distance and yaw beside the true ones, and how many points the fit used.
"""
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.vision import depth_locator as dl        # noqa: E402
from avaa_solution.vision.shelf_plane import face_from_depth  # noqa: E402

DEPTH = "/head_front_camera/head_front_camera/depth/image_rect_raw"
INFO = "/head_front_camera/head_front_camera/depth/camera_info"

SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        durability=QoSDurabilityPolicy.VOLATILE,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=1)

SHELF_FRONT_X = 2.755      # measured from erc_base_shelf.STL and the world pose
BASE_Z = 0.186


def truth():
    out = subprocess.run(["gz", "model", "-m", "tiago_pro", "-p"],
                         capture_output=True, text=True, timeout=25).stdout
    lines = [l.strip() for l in out.splitlines()]
    for i, line in enumerate(lines):
        if line.startswith("[") and i + 1 < len(lines) and lines[i + 1].startswith("["):
            try:
                return ([float(v) for v in line.strip("[]").split()],
                        [float(v) for v in lines[i + 1].strip("[]").split()])
            except ValueError:
                return None, None
    return None, None


class Look(Node):
    def __init__(self):
        super().__init__("planecheck")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.depth = None
        self.frame = ""
        self.intr = None
        self.create_subscription(Image, DEPTH, self._on_depth, SENSOR_QOS)
        self.create_subscription(CameraInfo, INFO, self._on_info, SENSOR_QOS)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_depth(self, msg):
        self.depth = np.frombuffer(msg.data, np.float32).reshape(
            msg.height, msg.width).copy()
        self.frame = msg.header.frame_id

    def _on_info(self, msg):
        self.intr = dl.Intrinsics.from_k(msg.k)


def main():
    row_height = float(sys.argv[1]) if len(sys.argv) > 1 else 1.061
    rclpy.init()
    node = Look()
    end = time.time() + 25
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.depth is not None and node.intr is not None and node.frame:
            break
    if node.depth is None or node.intr is None:
        print("no depth stream")
        return
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.1)

    try:
        tf = node.tf_buffer.lookup_transform("base_link", node.frame,
                                             rclpy.time.Time()).transform
    except Exception as exc:  # noqa: BLE001
        print("no transform base_link <- %s: %r" % (node.frame, exc))
        return
    q, t = tf.rotation, tf.translation
    xx, yy, zz = q.x * q.x, q.y * q.y, q.z * q.z
    xy, xz, yz = q.x * q.y, q.x * q.z, q.y * q.z
    wx, wy, wz = q.w * q.x, q.w * q.y, q.w * q.z
    rotation = np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)]])
    translation = np.array([t.x, t.y, t.z])

    print("row height %.3f m in base_link, band +/- 0.20 m\n" % row_height)
    print("%-10s %-10s %-10s %-10s %-10s %s"
          % ("true dist", "fit dist", "error", "true yaw", "fit yaw", "points"))

    for _ in range(6):
        for _ in range(25):
            rclpy.spin_once(node, timeout_sec=0.05)
        where, rpy = truth()
        result = face_from_depth(node.depth, node.intr, rotation, translation,
                                 row_height)
        if where is None:
            print("no truth")
            continue
        # No cos(yaw). The perpendicular distance from a point to a plane is a
        # geometric quantity and does not depend on how the robot is turned; the
        # cosine was an error that made a correct fit look 0.49 m wrong.
        true_perp = SHELF_FRONT_X - where[0]
        if result is None:
            print("%-10.3f %-10s %-10s %-+10.1f" % (true_perp, "no fit", "-",
                                                    math.degrees(rpy[2])))
            continue
        distance, yaw, points = result
        print("%-10.3f %-10.3f %-+10.3f %-+10.1f %-+10.1f %d"
              % (true_perp, distance, distance - true_perp,
                 math.degrees(rpy[2]), math.degrees(yaw), points))
        time.sleep(1.0)

    node.destroy_node()
    rclpy.shutdown()


main()
