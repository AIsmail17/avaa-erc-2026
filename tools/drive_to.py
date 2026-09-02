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

Driving costs nothing at all. Measured in the same way, in one session: 0.471 untouched,
0.550 after driving eight simulated seconds forward, 0.551 after turning, 0.561 after
driving further.

That matters more than it sounds. place_robot.py sets up every grasp experiment in this
project, so every one of them has run on a simulator at a fifteenth of the speed of the
one the scored run will use -- and a scored run never teleports, because the robot spawns
once and drives. Much of what was measured about the arm under those conditions (that
trajectories complete while the arm is still travelling, that the controller cannot
follow, that the base slides several centimetres during a reach) deserves measuring again
on a simulator that is running properly.

This is still a fixture: it reads the true pose from Gazebo to decide when to stop, which
the robot cannot do for itself. But it moves the robot with its own wheels, so the
simulation it hands to the experiment is the one the competition will use.
"""
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

SHOULDER_OFFSET_Y = 0.159

# Reading the true pose costs about a second of wall clock, so the loop runs near 1 Hz
# and every correction is a short burst rather than a continuous command. Gains that
# suit a 50 Hz loop would be wildly unstable here.
TURN_SPEED = 0.35
DRIVE_SPEED = 0.30
BURST_SIM_SEC = 0.8

BEARING_TOL = 0.02      # rad, while there is still distance to cover
YAW_TOL = 0.02          # rad, for the final squaring
DISTANCE_TOL = 0.015    # m


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
        self.now = None
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)

    def _on_clock(self, msg):
        self.now = msg.clock.sec + msg.clock.nanosec / 1e9

    def wait_for_clock(self):
        while self.now is None:
            rclpy.spin_once(self, timeout_sec=0.2)

    def burst(self, vx, wz, sim_seconds=BURST_SIM_SEC):
        """Command for a fixed span of SIMULATED time, then stop and hold."""
        self.wait_for_clock()
        until = self.now + sim_seconds
        command = Twist()
        command.linear.x = float(vx)
        command.angular.z = float(wz)
        guard = time.time() + 120
        while self.now < until and time.time() < guard:
            self.cmd.publish(command)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.hold(0.4)

    def hold(self, sim_seconds):
        """Publish a zero twist, which is not the same as publishing nothing.

        Measured per simulated second: a base with nothing on /cmd_vel drifts 6.8 mm/s
        and 0.81 deg/s with the arm still, and 2.1 mm/s and 0.43 deg/s with a zero twist
        at 20 Hz while the arm swings.
        """
        self.wait_for_clock()
        until = self.now + sim_seconds
        guard = time.time() + 120
        while self.now < until and time.time() < guard:
            self.cmd.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)


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
    print("driving to [%.3f, %.3f] yaw 0, in front of %s" % (goal_x, goal_y, name))

    rclpy.init()
    node = Driver()
    node.wait_for_clock()

    for attempt in range(80):
        here, rpy = pose("tiago_pro")
        if here is None:
            node.hold(0.5)
            continue
        yaw = rpy[2]
        dx, dy = goal_x - here[0], goal_y - here[1]
        distance = math.hypot(dx, dy)

        if distance <= DISTANCE_TOL:
            error = wrap(0.0 - yaw)
            if abs(error) <= YAW_TOL:
                print("arrived: [%.3f, %.3f] yaw %+.2f deg, %.0f mm from the goal"
                      % (here[0], here[1], math.degrees(yaw), distance * 1000))
                break
            node.burst(0.0, math.copysign(min(TURN_SPEED, 2.0 * abs(error)), error),
                       min(BURST_SIM_SEC, abs(error) / TURN_SPEED + 0.1))
            continue

        bearing = wrap(math.atan2(dy, dx) - yaw)
        if attempt < 6 or attempt % 20 == 0:
            print("  [%d] at [%.3f, %.3f] yaw %+.1f, %.2f m to go, bearing %+.1f deg"
                  % (attempt, here[0], here[1], math.degrees(yaw), distance,
                     math.degrees(bearing)), flush=True)
        # Rotate then drive, never both and never sideways: commanding pure vy yaws this
        # base by roughly the magnitude it strafes.
        if abs(bearing) > BEARING_TOL:
            node.burst(0.0, math.copysign(min(TURN_SPEED, 2.0 * abs(bearing)), bearing),
                       min(BURST_SIM_SEC, abs(bearing) / TURN_SPEED + 0.1))
            continue
        speed = min(DRIVE_SPEED, max(0.05, 0.8 * distance))
        node.burst(speed, 0.0, min(BURST_SIM_SEC, distance / speed))
    else:
        here, rpy = pose("tiago_pro")
        print("gave up after 80 corrections, at %s" % here)

    here, rpy = pose("tiago_pro")
    if here:
        print("robot at [%.3f, %.3f] yaw %+.1f deg"
              % (here[0], here[1], math.degrees(rpy[2])))
        print("book %s at [%.3f, %.3f, %.3f]" % (name, book[0], book[1], book[2]))
        print("so the book is %.3f m ahead and %+.3f m to the side"
              % (book[0] - here[0], book[1] - here[1]))

    node.hold(1.0)
    node.destroy_node()
    rclpy.shutdown()


main()
