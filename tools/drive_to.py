#!/usr/bin/env python3
"""Put the robot in front of a chosen book by DRIVING it there.

    python3 drive_to.py <book_colour|book_name> [standoff] [shoulder_offset]

The replacement for tools/place_robot.py, which teleports.

Teleporting is not free. Measured on a freshly launched simulator, one call to
/world/erc_world/set_pose takes the real-time factor from 0.48 to 0.04 and it never
recovers -- at z=0.000, 0.001, 0.010, 0.050 and 0.150 alike, with the robot resting
level and unpenetrated afterwards, with no node running and nothing in contact on any
instrumented link. Killing every node does not bring it back. Teleporting the robot
somewhere else does not bring it back. Only relaunching does.

Driving costs nothing at all. Measured the same way, in one session: 0.471 untouched,
0.550 after driving eight simulated seconds forward, 0.551 after turning, 0.561 after
driving further.

This is still a fixture: it reads the true pose from Gazebo to decide when to stop,
which the robot cannot do for itself. But it moves the robot with its own wheels, so the
simulation it hands to the experiment is the one the competition will use.

How it is controlled, and why it is not a sequence of bursts
------------------------------------------------------------
The first version issued one open-loop burst per correction and stopped between them. It
oscillated about the heading and never arrived: this base has no friction across the
roller axis, so when a command stops the base keeps the rate it was given. Measured, it
turned past the goal every time -- bearing +1.7 degrees, then -6.5, then -14.5, then
-24.1 -- and after eighty corrections it had moved 90 mm.

So the control is a cascade instead. Reading the true pose costs about a second of wall
clock, so that is the outer loop and it runs in a background thread; the inner loop
publishes at 20 Hz off the newest reading it has, and damps the turn against the yaw
rate from odometry. Odom is blind to sliding, which is why nothing in the solution trusts
it for position, but turning is the one thing that genuinely rotates these wheels.
"""
import math
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

SHOULDER_OFFSET_Y = 0.159

TURN_SPEED = 0.40
DRIVE_SPEED = 0.30
CREEP_SPEED = 0.06
TURN_DAMPING = 0.55

BEARING_TOL = 0.03      # rad, while there is still distance to cover
YAW_TOL = 0.03          # rad, for the final squaring
DISTANCE_TOL = 0.020    # m


def gz(*args, timeout=25):
    return subprocess.run(["gz", *args], capture_output=True, text=True,
                          timeout=timeout).stdout


def pose(model):
    lines = [l.strip() for l in gz("model", "-m", model, "-p").splitlines()]
    for i, line in enumerate(lines):
        if line.startswith("[") and i + 1 < len(lines) and lines[i + 1].startswith("["):
            try:
                return ([float(v) for v in line.strip("[]").split()],
                        [float(v) for v in lines[i + 1].strip("[]").split()])
            except ValueError:
                return None, None
    return None, None


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Driver(Node):
    def __init__(self):
        super().__init__("drive_to")
        self.yaw_rate = 0.0
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)

    def _odom(self, msg):
        self.yaw_rate = float(msg.twist.twist.angular.z)

    def turn(self, error, gain, floor=0.06):
        """A damped turn command, so the base does not carry its rate past the goal."""
        wanted = gain * error
        if abs(wanted) > 1e-6:
            wanted += math.copysign(floor, wanted)
        command = wanted - TURN_DAMPING * self.yaw_rate
        return float(max(-TURN_SPEED, min(TURN_SPEED, command)))

    def send(self, vx, wz):
        message = Twist()
        message.linear.x = float(vx)
        message.angular.z = float(wz)
        self.cmd.publish(message)

    def hold(self, seconds):
        """Publish a zero twist, which is not the same as publishing nothing."""
        end = time.time() + seconds
        while time.time() < end:
            self.cmd.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)


class Truth:
    """The true pose, refreshed in the background because reading it costs a second."""

    def __init__(self):
        self.value = None
        self.stop = False
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while not self.stop:
            try:
                here, rpy = pose("tiago_pro")
                if here is not None:
                    self.value = (here[0], here[1], rpy[2])
            except Exception:  # noqa: BLE001 - a dropped query is not a failure
                pass
            time.sleep(0.1)


def find_book(colour):
    names = []
    for _ in range(8):
        names = [l.strip(" -") for l in gz("model", "--list").splitlines()
                 if "book_col" in l and (colour in l or colour == l.strip(" -"))]
        if names:
            break
        time.sleep(0.5)
    if not names:
        return None, None
    books = []
    for name in names:
        for _ in range(5):
            p, _ = pose(name)
            if p:
                books.append((abs(p[1]), name, p))
                break
            time.sleep(0.4)
    if not books:
        return None, None
    books.sort()
    return books[0][1], books[0][2]


def main():
    colour = sys.argv[1] if len(sys.argv) > 1 else "red"
    standoff = float(sys.argv[2]) if len(sys.argv) > 2 else 0.68
    shoulder = float(sys.argv[3]) if len(sys.argv) > 3 else SHOULDER_OFFSET_Y

    name, book = find_book(colour)
    if name is None:
        print("no %s books" % colour)
        return

    # The shelf faces -x, so the robot looks along +x at yaw 0, with the book lined up
    # on the left shoulder rather than on the middle of the robot.
    goal_x = book[0] - standoff
    goal_y = book[1] - shoulder
    print("driving to [%.3f, %.3f] yaw 0, in front of %s" % (goal_x, goal_y, name),
          flush=True)

    rclpy.init()
    node = Driver()
    truth = Truth()
    while truth.value is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    deadline = time.time() + 240
    arrived = False
    reported = 0.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        x, y, yaw = truth.value
        dx, dy = goal_x - x, goal_y - y
        distance = math.hypot(dx, dy)

        if distance <= DISTANCE_TOL:
            error = wrap(0.0 - yaw)
            if abs(error) <= YAW_TOL:
                arrived = True
                break
            node.send(0.0, node.turn(error, 1.2))
        else:
            bearing = wrap(math.atan2(dy, dx) - yaw)
            # Rotate then drive, never both and never sideways: commanding pure vy yaws
            # this base by roughly the magnitude it strafes.
            if abs(bearing) > BEARING_TOL:
                node.send(0.0, node.turn(bearing, 1.2))
            else:
                speed = min(DRIVE_SPEED, max(CREEP_SPEED, 0.8 * distance))
                node.send(speed, node.turn(bearing, 0.8, floor=0.0))

        if time.time() - reported > 3.0:
            reported = time.time()
            print("  at [%.2f, %.2f] yaw %+.0f deg, %.2f m to go"
                  % (x, y, math.degrees(yaw), distance), flush=True)

    node.hold(1.5)
    x, y, yaw = truth.value
    truth.stop = True
    print("%s at [%.3f, %.3f] yaw %+.1f deg"
          % ("arrived" if arrived else "GAVE UP", x, y, math.degrees(yaw)))
    print("book %s at [%.3f, %.3f, %.3f]" % (name, book[0], book[1], book[2]))
    print("so the book is %.3f m ahead and %+.3f m to the side"
          % (book[0] - x, book[1] - y))

    node.destroy_node()
    rclpy.shutdown()


main()
