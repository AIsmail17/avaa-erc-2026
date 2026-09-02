#!/usr/bin/env python3
"""Does a short cmd_vel burst actually move the base?"""
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


def pose():
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


class N(Node):
    def __init__(self):
        super().__init__("cmdtest")
        self.now = None
        self.create_subscription(Clock, "/clock", self._c, 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)

    def _c(self, m):
        self.now = m.clock.sec + m.clock.nanosec / 1e9


rclpy.init()
n = N()
while n.now is None:
    rclpy.spin_once(n, timeout_sec=0.2)

for label, sim_seconds in (("0.8 sim s", 0.8), ("3.0 sim s", 3.0)):
    before, rpy0 = pose()
    until = n.now + sim_seconds
    t = Twist()
    t.angular.z = 0.35
    published = 0
    started = n.now
    guard = time.time() + 60
    while n.now < until and time.time() < guard:
        n.cmd.publish(t)
        published += 1
        rclpy.spin_once(n, timeout_sec=0.02)
    for _ in range(20):
        n.cmd.publish(Twist())
        rclpy.spin_once(n, timeout_sec=0.02)
    time.sleep(1.0)
    after, rpy1 = pose()
    print("%s: published %d, sim advanced %.2f, yaw %+.1f -> %+.1f deg"
          % (label, published, n.now - started,
             rpy0[2] * 57.3, rpy1[2] * 57.3))

n.destroy_node()
rclpy.shutdown()
