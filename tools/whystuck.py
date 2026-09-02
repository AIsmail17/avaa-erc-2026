#!/usr/bin/env python3
"""Why is the approach commanding a turn that does not happen?

    tools/in-sim whystuck.py [seconds]

Watches /cmd_vel, /odom and the true pose together for a few seconds and prints them
side by side. The question it answers is which link in the chain is broken: the
controller not publishing, the bridge not delivering, the wheels not turning, or the
base not moving when they do.
"""
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32, String


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


class Watch(Node):
    def __init__(self):
        super().__init__("why_stuck")
        self.cmd = None
        self.cmd_count = 0
        self.odom_yaw_rate = None
        self.bearing = None
        self.state = None
        self.sim = None
        self.create_subscription(Twist, "/cmd_vel", self._cmd, 10)
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.create_subscription(
            Float32, "/avaa/perception/target_column_x", self._bearing, 10)
        self.create_subscription(String, "/avaa/approach/state", self._state, 10)
        self.create_subscription(Clock, "/clock", self._on_clock, 10)

    def _cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.angular.z)
        self.cmd_count += 1

    def _odom(self, m):
        self.odom_yaw_rate = m.twist.twist.angular.z

    def _bearing(self, m):
        self.bearing = m.data

    def _state(self, m):
        self.state = m.data

    def _on_clock(self, m):
        self.sim = m.clock.sec + m.clock.nanosec / 1e9


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0
    rclpy.init()
    node = Watch()
    started = time.time()
    while time.time() - started < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("%-6s %-11s %-22s %-11s %-9s %s"
          % ("sim s", "state", "cmd_vel (x,y,wz)", "odom wz", "bearing", "true yaw"))
    last_count = node.cmd_count
    while time.time() - started < seconds + 3.0:
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
        _, rpy = truth()
        rate = (node.cmd_count - last_count)
        last_count = node.cmd_count
        print("%-6s %-11s %-22s %-11s %-9s %s"
              % ("?" if node.sim is None else "%.1f" % node.sim,
                 node.state or "-",
                 "none (%d msgs)" % rate if node.cmd is None
                 else "%+.2f %+.2f %+.2f (%d)" % (node.cmd + (rate,)),
                 "-" if node.odom_yaw_rate is None else "%+.3f" % node.odom_yaw_rate,
                 "-" if node.bearing is None else "%.0f px" % node.bearing,
                 "-" if rpy is None else "%+.1f deg" % math.degrees(rpy[2])))

    node.destroy_node()
    rclpy.shutdown()


main()
