#!/usr/bin/env python3
"""How far outside the base does the tucked arm reach?

    tools/in-sim footprint.py

An arm that is "tucked" is not necessarily an arm that is inside the robot. This reads
every arm link's position in base_link from TF and reports the extremes, against the
base's own half-width. Anything beyond that is a part that can catch on a shelf edge or a
table while the base drives past it.

The TIAGo Pro base is 0.54 m across, so 0.27 m is the line.
"""
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

BASE_HALF_WIDTH = 0.27
BASE_HALF_LENGTH = 0.27

LINKS = []
for side in ("left", "right"):
    LINKS += ["arm_%s_%d_link" % (side, i) for i in range(1, 8)]
    LINKS += ["gripper_%s_base_link" % side]


def main():
    rclpy.init()
    node = Node("footprint")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    end = time.time() + 15
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if buffer.can_transform("base_link", LINKS[0], rclpy.time.Time()):
            break
    time.sleep(2.0)
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)

    print("%-26s %8s %8s %8s   %s" % ("link", "x", "y", "z", "outside the base?"))
    worst = 0.0
    worst_name = ""
    for name in LINKS:
        try:
            tf = buffer.lookup_transform("base_link", name, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            print("%-26s   no transform" % name)
            continue
        t = tf.transform.translation
        over_y = abs(t.y) - BASE_HALF_WIDTH
        over_x = abs(t.x) - BASE_HALF_LENGTH
        over = max(over_y, over_x)
        flag = ""
        if over > 0:
            flag = "yes, by %3.0f mm (%s)" % (over * 1000,
                                              "side" if over_y > over_x else "front/back")
            if over > worst:
                worst, worst_name = over, name
        print("%-26s %+8.3f %+8.3f %+8.3f   %s" % (name, t.x, t.y, t.z, flag))

    print()
    if worst_name:
        print("worst overhang: %s, %.0f mm outside the base" % (worst_name, worst * 1000))
    else:
        print("every arm link is inside the base footprint")

    node.destroy_node()
    rclpy.shutdown()


main()
