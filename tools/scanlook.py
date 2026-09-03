#!/usr/bin/env python3
"""What is the forward LiDAR actually seeing, and is it the robot's own arm?

    tools/in-sim scanlook.py

The approach stopped for an obstacle at 0.54 m while its 3D fix put the book at 1.11 m.
Something much nearer than the shelf is in the laser and it matters what: the stowed arm
sits in the LiDAR plane, and the self filter only discards returns closer than 0.45 m, so
a driving posture whose elbow reaches a little further out reads as an obstacle and stops
the robot in open floor.

Prints the nearest forward returns with their bearings, in base_footprint, alongside where
each arm link is, so the two can be compared directly.
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        durability=QoSDurabilityPolicy.VOLATILE,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=1)

LINKS = ["arm_left_%d_link" % i for i in range(1, 8)] + \
        ["arm_right_%d_link" % i for i in range(1, 8)] + \
        ["gripper_left_base_link", "gripper_right_base_link"]


class Look(Node):
    def __init__(self):
        super().__init__("scanlook")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.scan = None
        self.create_subscription(LaserScan, "/scan_front_raw", self._on, SENSOR_QOS)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on(self, msg):
        self.scan = msg


def main():
    rclpy.init()
    node = Look()
    end = time.time() + 20
    while time.time() < end and node.scan is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.scan is None:
        print("no scan on /scan_front_raw")
        return
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)

    scan = node.scan
    print("laser frame: %s, %d beams, range %.2f..%.2f"
          % (scan.header.frame_id, len(scan.ranges),
             scan.range_min, scan.range_max))

    # Where the laser sits, so its returns can be put in base_footprint.
    try:
        tf = node.tf_buffer.lookup_transform(
            "base_footprint", scan.header.frame_id, rclpy.time.Time())
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        print("laser at base_footprint [%+.3f, %+.3f, %+.3f], yaw %+.1f deg"
              % (t.x, t.y, t.z, math.degrees(yaw)))
    except Exception as exc:  # noqa: BLE001
        print("no laser transform: %r" % exc)
        return

    rows = []
    for index, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or distance <= scan.range_min:
            continue
        angle = scan.angle_min + index * scan.angle_increment
        # into base_footprint
        x = t.x + distance * math.cos(angle + yaw)
        y = t.y + distance * math.sin(angle + yaw)
        if abs(math.atan2(y, x)) > 0.35:
            continue
        rows.append((math.hypot(x, y), x, y, math.degrees(math.atan2(y, x))))
    rows.sort()
    print("\nnearest 8 returns in the forward cone, in base_footprint:")
    print("%8s %8s %8s %9s" % ("range", "x", "y", "bearing"))
    for r, x, y, bearing in rows[:8]:
        print("%8.3f %8.3f %8.3f %8.1f deg" % (r, x, y, bearing))

    print("\narm links, for comparison:")
    for name in LINKS:
        try:
            lt = node.tf_buffer.lookup_transform(
                "base_footprint", name, rclpy.time.Time()).transform.translation
        except Exception:  # noqa: BLE001
            continue
        flat = math.hypot(lt.x, lt.y)
        near = " <-- inside the laser plane?" if abs(lt.z - t.z) < 0.12 else ""
        print("  %-24s [%+.3f, %+.3f, %+.3f]  flat range %.3f%s"
              % (name, lt.x, lt.y, lt.z, flat, near))

    node.destroy_node()
    rclpy.shutdown()


main()
