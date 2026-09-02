#!/usr/bin/env python3
"""Measure the real-time factor, and say what is subscribed to the cameras.

    tools/in-sim rtf.py [seconds]

RTF is the number that decides whether a run means anything. At 0.5 a grasp takes a
couple of minutes; at 0.02 the same grasp takes an hour and every timeout in the system
fires for the wrong reason. It is worth measuring before blaming a controller.

The camera subscriber count is printed with it because gz-sim renders a camera only
while something is listening, so a node that merely subscribes can change the RTF by an
order of magnitude on a machine with no working GPU.
"""
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

RGB = "/head_front_camera/head_front_camera/color/image_raw"
DEPTH = "/head_front_camera/head_front_camera/depth/image_rect_raw"


class Watch(Node):
    def __init__(self):
        super().__init__("rtf_watch")
        self.first = None
        self.last = None
        self.create_subscription(Clock, "/clock", self._on_clock, 10)

    def _on_clock(self, msg):
        seconds = msg.clock.sec + msg.clock.nanosec / 1e9
        if self.first is None:
            self.first = seconds
        self.last = seconds


def subscribers(topic):
    try:
        out = subprocess.run(["ros2", "topic", "info", topic], capture_output=True,
                             text=True, timeout=20).stdout
    except Exception:  # noqa: BLE001
        return "?"
    for line in out.splitlines():
        if "Subscription count" in line:
            return line.split(":")[1].strip()
    return "?"


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rclpy.init()
    node = Watch()
    started = time.time()
    while time.time() - started < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    wall = time.time() - started
    if node.first is None:
        print("no /clock at all — the simulator is not running")
        return
    simulated = node.last - node.first
    print("simulated %.2f s in %.2f s of wall clock" % (simulated, wall))
    print("real-time factor: %.3f" % (simulated / wall))
    print("colour camera subscribers: %s" % subscribers(RGB))
    print("depth camera subscribers : %s" % subscribers(DEPTH))
    node.destroy_node()
    rclpy.shutdown()


main()
