#!/usr/bin/env python3
"""What does the forward laser actually hit, by distance?

    tools/in-sim ranges.py

The shelf-relative design assumes the shelf front is a large flat surface in the laser's
view. The fit disagrees: standing about a metre from the shelf it reported a plane at
4.4 m -- correctly oriented, parallel to the shelf, and far too far away to be it.

If most beams pass straight THROUGH the shelf, the front face is not a large surface at
this height at all, and the assumption is wrong rather than the fit. A histogram settles
it.
"""
import math
import sys
import time
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import LaserScan

SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        durability=QoSDurabilityPolicy.VOLATILE,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=1)
SELF_FILTER = 0.45


class Look(Node):
    def __init__(self):
        super().__init__("ranges")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.scan = None
        self.create_subscription(LaserScan, "/scan_front_raw", self._on, SENSOR_QOS)

    def _on(self, msg):
        self.scan = msg


def main():
    half = float(sys.argv[1]) if len(sys.argv) > 1 else 0.45
    rclpy.init()
    node = Look()
    end = time.time() + 20
    while time.time() < end and node.scan is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.scan is None:
        print("no scan")
        return
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)

    lx, ly, lyaw = 0.275, -0.183, -math.pi / 4.0
    buckets = Counter()
    total = 0
    for index, distance in enumerate(node.scan.ranges):
        if not math.isfinite(distance) or distance <= node.scan.range_min:
            continue
        angle = node.scan.angle_min + index * node.scan.angle_increment
        x = lx + distance * math.cos(angle + lyaw)
        y = ly + distance * math.sin(angle + lyaw)
        flat = math.hypot(x, y)
        if flat < SELF_FILTER or abs(math.atan2(y, x)) > half:
            continue
        total += 1
        buckets[round(flat * 4) / 4.0] += 1

    print("forward cone +/- %.2f rad, %d returns" % (half, total))
    print("%8s %6s  %s" % ("range", "count", ""))
    for key in sorted(buckets):
        count = buckets[key]
        print("%8.2f %6d  %s" % (key, count, "#" * min(60, count)))

    node.destroy_node()
    rclpy.shutdown()


main()
