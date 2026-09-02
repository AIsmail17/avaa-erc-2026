#!/usr/bin/env python3
"""Which step of the setup is it that makes the simulator ten times slower?

    tools/in-sim whatslow.py

The real-time factor falls from about 0.5 to about 0.03 at some point during a trial, and
everything downstream suffers for it: every wall-clock wait expires early, every trajectory
takes twenty times as long, and a fifteen minute experiment becomes four hours. Killing the
nodes does not bring it back and neither does teleporting the robot. Only relaunching does.

So this walks the setup one step at a time on a fresh instance, measuring the factor after
each, and prints where the cliff is. Run it immediately after `tools/sim restart`.
"""
import subprocess
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TUCK_LEFT = [2.1521, 0.3824, 1.2785, -2.1517, 0.8325, 0.1926, 1.3944]
TUCK_RIGHT = [-0.7194, -2.2867, -0.5064, 0.5221, 2.3399, 1.0503, 1.9772]
TUCK_TORSO = 0.15


class Rig(Node):
    def __init__(self):
        super().__init__("whatslow")
        self.now = None
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.arms = {
            side: self.create_publisher(
                JointTrajectory, "/arm_%s_controller/joint_trajectory" % side, 10)
            for side in ("left", "right")}
        self.torso = self.create_publisher(
            JointTrajectory, "/torso_controller/joint_trajectory", 10)

    def _on_clock(self, msg):
        self.now = msg.clock.sec + msg.clock.nanosec / 1e9

    def send(self, publisher, names, values, seconds=20.0):
        traj = JointTrajectory()
        traj.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in values]
        point.time_from_start = Duration(
            sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))
        traj.points = [point]
        publisher.publish(traj)

    def measure(self, seconds=8.0):
        while self.now is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        sim0, wall0 = self.now, time.time()
        while time.time() - wall0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)
        return (self.now - sim0) / (time.time() - wall0)

    def settle(self, seconds=25.0):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def teleport(x, y):
    subprocess.run(
        ["gz", "service", "-s", "/world/erc_world/set_pose",
         "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
         "--timeout", "3000", "--req",
         'name: "tiago_pro", position: {x: %f, y: %f, z: 0.0}, '
         'orientation: {x: 0, y: 0, z: 0, w: 1}' % (x, y)],
        capture_output=True, text=True, timeout=25)


def main():
    rclpy.init()
    node = Rig()
    time.sleep(1.0)

    def report(label):
        print("%-42s rtf %.3f" % (label, node.measure()), flush=True)

    report("fresh, nothing done")

    teleport(2.22, -0.108)
    node.settle(10)
    report("after a teleport to the shelf")

    node.send(node.torso, ["torso_lift_joint"], [TUCK_TORSO])
    node.settle(30)
    report("after raising the torso")

    node.send(node.arms["left"],
              ["arm_left_%d_joint" % i for i in range(1, 8)], TUCK_LEFT)
    node.settle(40)
    report("after tucking the LEFT arm")

    node.send(node.arms["right"],
              ["arm_right_%d_joint" % i for i in range(1, 8)], TUCK_RIGHT)
    node.settle(40)
    report("after tucking the RIGHT arm")

    end = time.time() + 30
    while time.time() < end:
        node.cmd.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.02)
    report("after 30 s of zero twist at 20 Hz")

    node.destroy_node()
    rclpy.shutdown()


main()
