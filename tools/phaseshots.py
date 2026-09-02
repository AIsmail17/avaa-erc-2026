#!/usr/bin/env python3
"""Save one still per phase of a grasp, from the spectator camera.

Hours went into inferring jaw geometry from numbers without ever looking at the
robot, and it did not converge. A picture at each phase -- and several through the
clamp, where it actually fails -- costs one run and shows what the numbers cannot.

    tools/in-sim phaseshots.py            # writes /tmp/shots/NN_state.png
"""
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import Image
from std_msgs.msg import String

TOPIC = "/spectator/image"
OUT = "/tmp/shots"
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 700.0
# Through the clamp, take a burst: that is where it goes wrong and one frame of it
# is not enough to see whether the jaws are around the book or beside it.
BURST = {"clamping", "advancing", "lifting"}

QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                 durability=QoSDurabilityPolicy.VOLATILE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)

os.makedirs(OUT, exist_ok=True)
for old in os.listdir(OUT):
    os.remove(os.path.join(OUT, old))

rclpy.init()
node = rclpy.create_node("phaseshots")
state = {"s": "start", "n": 0, "last": None, "at": 0.0, "img": None}
node.create_subscription(String, "/avaa/grasp/state",
                         lambda m: state.__setitem__("s", m.data), 10)


def on_image(msg):
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    try:
        img = buf.reshape(msg.height, msg.width, -1)
    except ValueError:
        return
    img = (cv2.cvtColor(img[:, :, :3], cv2.COLOR_RGB2BGR)
           if msg.encoding in ("rgb8", "rgba8")
           else np.ascontiguousarray(img[:, :, :3]))
    state["img"] = img


node.create_subscription(Image, TOPIC, on_image, QOS)

began = time.time()
print("watching; writing stills to %s" % OUT, flush=True)
while rclpy.ok() and time.time() - began < SECONDS:
    rclpy.spin_once(node, timeout_sec=0.1)
    now = time.time()
    changed = state["s"] != state["last"]
    burst = state["s"] in BURST and now - state["at"] > 4.0
    if state["img"] is not None and (changed or burst):
        state["last"] = state["s"]
        state["at"] = now
        state["n"] += 1
        img = state["img"].copy()
        label = "%02d  %-11s  t+%3.0fs" % (state["n"], state["s"], now - began)
        cv2.rectangle(img, (0, 0), (img.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(img, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2, cv2.LINE_AA)
        name = "%02d_%s.png" % (state["n"], state["s"].replace(" ", "_"))
        cv2.imwrite(os.path.join(OUT, name), img)
        print("  %s" % name, flush=True)
    if state["s"] in ("done", "failed"):
        t = time.time()
        while time.time() - t < 4:
            rclpy.spin_once(node, timeout_sec=0.1)
        break

print("wrote %d stills" % state["n"], flush=True)
node.destroy_node()
rclpy.shutdown()
