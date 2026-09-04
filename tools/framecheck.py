#!/usr/bin/env python3
"""Are the camera frames actually changing, or is the same picture arriving over again?

    tools/in-sim framecheck.py [seconds_between_grabs] [grabs]

'ros2 topic hz' answers "are messages arriving", which is a different question from "is
the camera looking". A gz-sim camera sensor that is not re-rendering will happily
republish the last image it made, at full rate, for ever.

The distinction matters because it explains a symptom nothing else did. Watched in a
run: the base turned 22 degrees against Gazebo's own ground truth -- odom agreeing to
within 5 -- while the steering bearing perception published moved from 286 px to 286 px.
A marker fixed in the world should have swept 136 px in that time. Either the detector
is wrong about which marker it is looking at, or it is looking at the same photograph
each time.

Prints the mean absolute difference between consecutive frames. A moving robot in front
of a shelf gives tens of levels; a frozen stream gives exactly zero.
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import Image

RGB = "/head_front_camera/head_front_camera/color/image_raw"
SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        durability=QoSDurabilityPolicy.VOLATILE,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=1)


def main():
    gap = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0
    grabs = int(sys.argv[2]) if len(sys.argv) > 2 else 6

    rclpy.init()
    node = rclpy.create_node("framecheck")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    latest = {}

    def on_image(msg):
        latest["frame"] = np.frombuffer(msg.data, np.uint8).copy()
        latest["stamp"] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        latest["count"] = latest.get("count", 0) + 1

    node.create_subscription(Image, RGB, on_image, SENSOR_QOS)

    deadline = time.time() + 30
    while time.time() < deadline and "frame" not in latest:
        rclpy.spin_once(node, timeout_sec=0.2)
    if "frame" not in latest:
        print("no colour frames at all")
        return 1

    print("%-8s %-12s %-14s %s"
          % ("grab", "arrived", "header stamp", "mean |difference| from the last"))
    previous = None
    previous_stamp = None
    identical = 0
    for index in range(grabs):
        start = time.time()
        while time.time() - start < gap:
            rclpy.spin_once(node, timeout_sec=0.1)
        frame = latest["frame"]
        stamp = latest["stamp"]
        if previous is None or frame.shape != previous.shape:
            print("%-8d %-12d %-14.2f %s"
                  % (index + 1, latest.get("count", 0), stamp, "-"))
        else:
            diff = float(np.mean(np.abs(frame.astype(np.int16)
                                        - previous.astype(np.int16))))
            same_stamp = (previous_stamp is not None
                          and abs(stamp - previous_stamp) < 1e-6)
            if diff == 0.0:
                identical += 1
            print("%-8d %-12d %-14.2f %.3f%s"
                  % (index + 1, latest.get("count", 0), stamp, diff,
                     "   (and the same header stamp)" if same_stamp else ""))
        previous = frame
        previous_stamp = stamp

    print()
    if identical >= grabs - 1:
        print("FROZEN. Messages are arriving and the picture is not changing.")
        print("Everything downstream is looking at one photograph.")
    elif identical:
        print("Intermittent: %d of %d consecutive grabs were byte-identical."
              % (identical, grabs - 1))
    else:
        print("The camera is live; every grab differs from the one before it.")

    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
