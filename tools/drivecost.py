#!/usr/bin/env python3
"""Does DRIVING the base cost what teleporting it costs?

    tools/in-sim drivecost.py

Any call to /world/erc_world/set_pose on the robot takes the real-time factor from about
0.48 to about 0.04 and it never recovers -- at every height tried, with the robot resting
level and unpenetrated afterwards. That matters enormously, because tools/place_robot.py
teleports and it sets up every grasp experiment in this project, while a scored run never
teleports at all: the robot spawns once and drives.

If driving is cheap, then the arm behaviour measured in those experiments -- trajectories
that never complete, a controller that cannot follow, a base that slides -- was measured
on a simulator running at a fifteenth speed, and needs measuring again.

Run immediately after `tools/sim restart`.
"""
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class Rig(Node):
    def __init__(self):
        super().__init__("drive_cost")
        self.now = None
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)

    def _on_clock(self, msg):
        self.now = msg.clock.sec + msg.clock.nanosec / 1e9

    def measure(self, seconds=8.0):
        while self.now is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        sim0, wall0 = self.now, time.time()
        while time.time() - wall0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)
        return (self.now - sim0) / (time.time() - wall0)

    def drive(self, vx, wz, sim_seconds):
        while self.now is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        until = self.now + sim_seconds
        command = Twist()
        command.linear.x = vx
        command.angular.z = wz
        while self.now < until:
            self.cmd.publish(command)
            rclpy.spin_once(self, timeout_sec=0.02)
        stop = Twist()
        for _ in range(40):
            self.cmd.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.02)


def where():
    out = subprocess.run(["gz", "model", "-m", "tiago_pro", "-p"],
                         capture_output=True, text=True, timeout=25).stdout
    lines = [l.strip() for l in out.splitlines()]
    for i, line in enumerate(lines):
        if line.startswith("[") and i + 1 < len(lines) and lines[i + 1].startswith("["):
            try:
                return [float(v) for v in line.strip("[]").split()]
            except ValueError:
                return None
    return None


def main():
    rclpy.init()
    node = Rig()
    time.sleep(1.0)

    print("baseline, untouched            rtf %.3f  at %s"
          % (node.measure(), where()), flush=True)

    node.drive(0.25, 0.0, 8.0)
    print("after driving forward 8 sim s  rtf %.3f  at %s"
          % (node.measure(), where()), flush=True)

    node.drive(0.0, 0.4, 6.0)
    print("after turning 6 sim s          rtf %.3f  at %s"
          % (node.measure(), where()), flush=True)

    node.drive(0.25, 0.0, 8.0)
    print("after driving forward again    rtf %.3f  at %s"
          % (node.measure(), where()), flush=True)

    node.destroy_node()
    rclpy.shutdown()


main()
