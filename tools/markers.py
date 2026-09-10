#!/usr/bin/env python3
"""Which digit is painted above which column, and does the reader agree?

    tools/in-sim markers.py

The digits are shuffled at spawn and the mapping lives only in the launch process, so
there is no way to look it up afterwards -- which makes every perception experiment start
by guessing which marker number to ask for. This reads one frame with the same reader the
solution uses and prints what it finds, left to right, beside the column each marker
model actually sits above according to Gazebo.

Agreement between the two is the marker reader working. Disagreement is worth more: it
says the digit recognition is wrong, and a wrong digit sends the robot to the wrong
column, which loses the identification points and everything downstream of them.
"""
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from sensor_msgs.msg import Image
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       QoSHistoryPolicy)

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.vision import marker_reader as mr      # noqa: E402

TOPIC_RGB = "/head_front_camera/head_front_camera/color/image_raw"
SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        durability=QoSDurabilityPolicy.VOLATILE,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=1)


def gz(*args, timeout=20):
    try:
        return subprocess.run(["gz", *args], capture_output=True, text=True,
                              timeout=timeout).stdout
    except Exception:  # noqa: BLE001
        return ""


def marker_positions():
    """World y of each number_marker model, keyed by the column in its name."""
    out = {}
    for column in range(1, 6):
        name = "number_marker_col_%d" % column
        for _ in range(4):
            lines = [l.strip() for l in gz("model", "-m", name, "-p").splitlines()]
            found = False
            for i, line in enumerate(lines):
                if line.startswith("[") and i + 1 < len(lines) \
                        and lines[i + 1].startswith("["):
                    try:
                        out[column] = [float(v) for v in line.strip("[]").split()]
                        found = True
                    except ValueError:
                        pass
                    break
            if found:
                break
            time.sleep(0.3)
    return out


def to_bgr(msg):
    data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    if msg.encoding == "rgb8":
        return data[:, :, ::-1].copy()
    return data.copy()


def main():
    truth = marker_positions()
    if not truth:
        print("no number markers in the world")
        return 1

    rclpy.init()
    node = rclpy.create_node("markers")
    latest = {"rgb": None}
    node.create_subscription(Image, TOPIC_RGB,
                             lambda m: latest.__setitem__("rgb", m), SENSOR_QOS)
    deadline = time.time() + 12.0
    while time.time() < deadline and latest["rgb"] is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    if latest["rgb"] is None:
        print("no camera frame in 12 s")
        return 1

    found = sorted(mr.read_markers(to_bgr(latest["rgb"])), key=lambda m: m.cx)
    print("Gazebo, left to right along the shelf (world y descending):")
    for column in sorted(truth, key=lambda c: -truth[c][1]):
        print("    number_marker_col_%d at y=%+0.3f" % (column, truth[column][1]))
    print("")
    print("The reader, left to right in the image:")
    if not found:
        print("    nothing. Stand further back: the plates are at 2.26 m and the head")
        print("    has to be able to see all five at once.")
    for marker in found:
        print("    digit %s at cx=%.0f, %s"
              % (marker.digit, marker.cx,
                 "confident" if marker.confident else "NOT confident"))
    print("")
    if len(found) == len(truth):
        columns = sorted(truth, key=lambda c: -truth[c][1])
        print("So the mapping, if the reader is right:")
        for column, marker in zip(columns, found):
            print("    shelf column %d carries digit %s" % (column, marker.digit))
        digits = [m.digit for m in found]
        if sorted(d for d in digits if d) != list(range(1, len(truth) + 1)):
            print("")
            print("    -- but %s is not a permutation of 1-%d, so at least one digit is"
                  % (digits, len(truth)))
            print("       misread and this mapping cannot be trusted.")
    else:
        print("Only %d of %d markers are in frame, so the mapping cannot be completed."
              % (len(found), len(truth)))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
