#!/usr/bin/env python3
"""What does a teleport cost, and is there a height that costs nothing?

    tools/in-sim teleportcost.py

Placing the robot with /world/erc_world/set_pose takes the real-time factor from about
0.48 to about 0.03 and it never recovers. Since tools/place_robot.py sets up every grasp
experiment in this project, every trial has been run on a simulator fifteen times slower
than a fresh one -- which is why trajectories appeared not to complete, why wall-clock
waits expired halfway, and why a fifteen minute experiment took four hours.

The suspicion is the floor: set_pose puts the model origin exactly at z=0 with whatever
velocity it already had, and if that leaves the wheels a millimetre inside the ground
plane the contact solver has something to grind on for the rest of the session. This
tries several heights and reports the factor after each.

Run immediately after `tools/sim restart`.
"""
import subprocess
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class Watch(Node):
    def __init__(self):
        super().__init__("teleport_cost")
        self.now = None
        self.create_subscription(Clock, "/clock", self._on_clock, 10)

    def _on_clock(self, msg):
        self.now = msg.clock.sec + msg.clock.nanosec / 1e9

    def measure(self, seconds=8.0):
        while self.now is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        sim0, wall0 = self.now, time.time()
        while time.time() - wall0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)
        return (self.now - sim0) / (time.time() - wall0)

    def settle(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def teleport(x, y, z):
    return subprocess.run(
        ["gz", "service", "-s", "/world/erc_world/set_pose",
         "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
         "--timeout", "3000", "--req",
         'name: "tiago_pro", position: {x: %f, y: %f, z: %f}, '
         'orientation: {x: 0, y: 0, z: 0, w: 1}' % (x, y, z)],
        capture_output=True, text=True, timeout=25).stdout.strip()


def height():
    out = subprocess.run(["gz", "model", "-m", "tiago_pro", "-p"],
                         capture_output=True, text=True, timeout=25).stdout
    lines = [l.strip() for l in out.splitlines()]
    for i, line in enumerate(lines):
        if line.startswith("[") and i + 1 < len(lines) and lines[i + 1].startswith("["):
            try:
                values = [float(v) for v in line.strip("[]").split()]
                rpy = [float(v) for v in lines[i + 1].strip("[]").split()]
                return values[2], rpy[1]
            except ValueError:
                return None, None
    return None, None


def main():
    rclpy.init()
    node = Watch()
    time.sleep(1.0)
    print("baseline, untouched                rtf %.3f" % node.measure(), flush=True)

    for z in (0.0, 0.001, 0.01, 0.05, 0.15):
        teleport(2.22, -0.108, z)
        node.settle(15)
        z_now, pitch = height()
        print("set_pose z=%-5.3f -> rests at z=%s pitch %s   rtf %.3f"
              % (z,
                 "?" if z_now is None else "%+.4f" % z_now,
                 "?" if pitch is None else "%+.4f" % pitch,
                 node.measure()), flush=True)

    node.destroy_node()
    rclpy.shutdown()


main()
