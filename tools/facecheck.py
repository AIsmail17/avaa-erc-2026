#!/usr/bin/env python3
"""Does the shelf-face fit agree with the truth, and at what ranges?

    tools/in-sim facecheck.py

The whole redesigned approach rests on this one measurement, so it is worth knowing its
error before building a controller on it. Drives the robot in towards the shelf with its
own wheels, sampling the fit against Gazebo at each step.

Prints, for each sample: what the fit says the shelf face is, what the truth says, and the
yaw error the fit reports against the true yaw.
"""
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")

SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        durability=QoSDurabilityPolicy.VOLATILE,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=1)

SHELF_FRONT_X = 2.755      # measured from the supplied mesh
SELF_FILTER = 0.45
FACE_TOL = 0.05
FACE_CONSENSUS = 0.35


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


def largest_collinear(xs, ys, tolerance=0.05, iterations=60):
    best = None
    rng = np.random.default_rng(3)
    n = len(xs)
    if n < 4:
        return None
    for _ in range(iterations):
        a, b = rng.choice(n, size=2, replace=False)
        if abs(ys[a] - ys[b]) < 1e-6:
            continue
        slope = (xs[a] - xs[b]) / (ys[a] - ys[b])
        intercept = xs[a] - slope * ys[a]
        residual = np.abs(xs - (slope * ys + intercept)) / math.sqrt(1 + slope * slope)
        inliers = residual < tolerance
        if best is None or inliers.sum() > best.sum():
            best = inliers
    return best


class Rig(Node):
    def __init__(self):
        super().__init__("facecheck")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.scan = None
        self.now = None
        self.create_subscription(LaserScan, "/scan_front_raw", self._scan, SENSOR_QOS)
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(Odometry, "/odom", lambda m: None, 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)

    def _scan(self, msg):
        self.scan = msg

    def _on_clock(self, msg):
        self.now = msg.clock.sec + msg.clock.nanosec / 1e9

    def points(self, half_angle=0.45):
        if self.scan is None:
            return []
        # The laser sits forward and right of base_footprint, rotated -45 degrees.
        lx, ly, lyaw = 0.275, -0.183, -math.pi / 4.0
        out = []
        for index, distance in enumerate(self.scan.ranges):
            if not math.isfinite(distance) or distance <= self.scan.range_min:
                continue
            angle = self.scan.angle_min + index * self.scan.angle_increment
            x = lx + distance * math.cos(angle + lyaw)
            y = ly + distance * math.sin(angle + lyaw)
            if math.hypot(x, y) < SELF_FILTER:
                continue
            if abs(math.atan2(y, x)) > half_angle:
                continue
            out.append((x, y))
        return out

    def face(self):
        pts = self.points()
        if len(pts) < 12:
            return None
        xs = np.array([p[0] for p in pts])
        ys = np.array([p[1] for p in pts])
        floor = max(12, int(FACE_CONSENSUS * len(xs)))
        best = None
        remaining = np.ones(len(xs), dtype=bool)
        for _ in range(3):
            if int(remaining.sum()) < floor:
                break
            sx, sy = xs[remaining], ys[remaining]
            inl = largest_collinear(sx, sy, tolerance=FACE_TOL)
            if inl is None or int(inl.sum()) < floor:
                break
            fx, fy = sx[inl], sy[inl]
            slope, intercept = np.polyfit(fy, fx, 1)
            resid = float(np.std(fx - (slope * fy + intercept)))
            if resid <= 0.05:
                dist = abs(float(intercept)) / math.sqrt(1 + slope * slope)
                if best is None or dist < best[0]:
                    best = (dist, float(math.atan(slope)), int(inl.sum()))
            idx = np.where(remaining)[0]
            remaining[idx[inl]] = False
        return best

    def turn(self, wz, sim_seconds):
        while self.now is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        until = self.now + sim_seconds
        c = Twist()
        c.angular.z = wz
        guard = time.time() + 90
        while self.now < until and time.time() < guard:
            self.cmd.publish(c)
            rclpy.spin_once(self, timeout_sec=0.02)
        stop = Twist()
        for _ in range(30):
            self.cmd.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.02)

    def drive(self, vx, sim_seconds):
        while self.now is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        until = self.now + sim_seconds
        c = Twist()
        c.linear.x = vx
        guard = time.time() + 90
        while self.now < until and time.time() < guard:
            self.cmd.publish(c)
            rclpy.spin_once(self, timeout_sec=0.02)
        stop = Twist()
        for _ in range(30):
            self.cmd.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.02)


def main():
    rclpy.init()
    node = Rig()
    end = time.time() + 20
    while time.time() < end and node.scan is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.scan is None:
        print("no scan")
        return

    # Point at the shelf first. The robot is left wherever the last run put it, and a
    # fit taken while it faces a side wall measures the side wall -- correctly, and
    # uselessly. Turned with its own wheels, using truth only to decide when to stop.
    for _ in range(60):
        where, rpy = truth()
        if where is None:
            break
        error = (-rpy[2] + math.pi) % (2 * math.pi) - math.pi
        if abs(error) < 0.03:
            break
        node.turn(math.copysign(min(0.4, 1.5 * abs(error)), error), 0.5)
    where, rpy = truth()
    if rpy is not None:
        print("facing the shelf: yaw %+.1f deg" % math.degrees(rpy[2]))
    print()

    print("%-9s %-9s %-9s %-9s %-9s %s"
          % ("true dist", "fit dist", "error", "true yaw", "fit yaw", "returns"))
    for step in range(8):
        for _ in range(40):
            rclpy.spin_once(node, timeout_sec=0.05)
        where, rpy = truth()
        face = node.face()
        if where is None:
            print("no truth")
        elif face is None:
            print("%-9.3f %-9s" % (SHELF_FRONT_X - where[0], "no fit"))
        else:
            true_dist = SHELF_FRONT_X - where[0]
            # The fit is a perpendicular distance; compare against the true
            # perpendicular distance, which is the x gap times cos(yaw).
            true_perp = true_dist * math.cos(rpy[2])
            print("%-9.3f %-9.3f %-+9.3f %-+9.1f %-+9.1f %d"
                  % (true_perp, face[0], face[0] - true_perp,
                     math.degrees(rpy[2]), math.degrees(face[1]), face[2]))
        if step < 7:
            node.drive(0.18, 1.6)

    node.destroy_node()
    rclpy.shutdown()


main()
