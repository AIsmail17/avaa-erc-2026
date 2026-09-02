#!/usr/bin/env python3
"""Save one frame from the spectator camera as a PNG."""
import sys, time
import numpy as np, cv2, rclpy
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSHistoryPolicy, QoSDurabilityPolicy)
from sensor_msgs.msg import Image

TOPIC = sys.argv[1] if len(sys.argv) > 1 else "/spectator/image"
OUT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/frame.png"
QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                 durability=QoSDurabilityPolicy.VOLATILE,
                 history=QoSHistoryPolicy.KEEP_LAST, depth=1)
rclpy.init()
n = rclpy.create_node("grab")
got = {}
def cb(m):
    if "img" in got:
        return
    buf = np.frombuffer(m.data, dtype=np.uint8)
    try:
        img = buf.reshape(m.height, m.width, -1)
    except ValueError:
        return
    img = cv2.cvtColor(img[:, :, :3], cv2.COLOR_RGB2BGR) if m.encoding in ("rgb8", "rgba8") \
        else np.ascontiguousarray(img[:, :, :3])
    got["img"] = img
n.create_subscription(Image, TOPIC, cb, QOS)
t = time.time()
while rclpy.ok() and time.time() - t < 25 and "img" not in got:
    rclpy.spin_once(n, timeout_sec=0.1)
if "img" in got:
    cv2.imwrite(OUT, got["img"])
    print("saved %s %dx%d" % (OUT, got["img"].shape[1], got["img"].shape[0]))
else:
    print("no frame on %s" % TOPIC)
n.destroy_node(); rclpy.shutdown()
