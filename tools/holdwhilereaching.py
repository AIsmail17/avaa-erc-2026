#!/usr/bin/env python3
"""Does a zero twist still hold the base while the arm is swinging?

    tools/in-sim holdwhilereaching.py [seconds_per_window]

A base held still with the arm folded proves nothing: the arm is what disturbs it. This
swings the arm between a folded posture and a reaching one, on a fixed cycle, and
measures the base against Gazebo in each of four windows -- arm still and arm swinging,
each with and without a zero twist at 20 Hz.

Everything is per SIMULATED second. The simulator's real-time factor moves by a factor
of forty over a session, so a figure per wall second says as much about how long the
instance has been up as about the robot.
"""
import subprocess
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ["arm_left_%d_joint" % i for i in range(1, 8)]
FOLDED = [2.1521, 0.3824, 1.2785, -2.1517, 0.8325, 0.1926, 1.3944]
REACHING = [0.6, 0.2, 1.0, -1.2, 0.8, 0.2, 1.4]


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


class Rig(Node):
    def __init__(self):
        super().__init__("hold_while_reaching")
        self.now = None
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.arm = self.create_publisher(
            JointTrajectory, "/arm_left_controller/joint_trajectory", 10)

    def _on_clock(self, msg):
        self.now = msg.clock.sec + msg.clock.nanosec / 1e9

    def swing(self, values, seconds=4.0):
        traj = JointTrajectory()
        traj.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in values]
        point.time_from_start = Duration(
            sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))
        traj.points = [point]
        self.arm.publish(traj)


def window(node, seconds, holding, swinging):
    while node.now is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    start_p, start_r = truth()
    sim0, wall0 = node.now, time.time()
    last_swing = 0.0
    towards_reach = True
    while time.time() - wall0 < seconds:
        if holding:
            node.cmd.publish(Twist())
        if swinging and node.now - last_swing > 5.0:
            node.swing(REACHING if towards_reach else FOLDED)
            towards_reach = not towards_reach
            last_swing = node.now
        rclpy.spin_once(node, timeout_sec=0.02)
    end_p, end_r = truth()
    sim = node.now - sim0
    wall = time.time() - wall0
    if start_p is None or end_p is None:
        return None
    dx, dy = end_p[0] - start_p[0], end_p[1] - start_p[1]
    dyaw = end_r[2] - start_r[2]
    return ((dx * dx + dy * dy) ** 0.5, sim, wall, dyaw)


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rclpy.init()
    node = Rig()
    time.sleep(1.0)
    node.swing(FOLDED, 4.0)
    time.sleep(6.0)

    print("%-34s %9s %7s %11s %11s"
          % ("", "travel", "rtf", "per sim s", "yaw/sim s"))
    cases = (("arm still, nothing commanded", False, False),
             ("arm still, zero twist", True, False),
             ("arm swinging, nothing commanded", False, True),
             ("arm swinging, zero twist", True, True))
    for label, holding, swinging in cases:
        result = window(node, seconds, holding, swinging)
        if result is None:
            print("%-34s   no ground truth" % label)
            continue
        travel, sim, wall, dyaw = result
        if sim < 0.5:
            print("%-34s %6.0f mm %.3f  clock barely moved" % (label, travel * 1000,
                                                               sim / wall))
            continue
        print("%-34s %6.0f mm %.3f %7.1f mm %8.2f deg"
              % (label, travel * 1000, sim / wall, travel / sim * 1000,
                 abs(dyaw) / sim * 57.2958))

    node.destroy_node()
    rclpy.shutdown()


main()
