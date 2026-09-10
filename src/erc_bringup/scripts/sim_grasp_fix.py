#!/usr/bin/env python3
"""Make a simulated grasp hold, because this gripper cannot.

SIMULATION SCAFFOLDING. Nothing in avaa_solution knows this node exists, and Phase 2
drops it by not launching it. It lives in erc_bringup beside the other things that exist
only because the robot is simulated.

Why it is needed
----------------
gripper_left_finger_joint is position-commanded, and every link that actually touches a
book is a mimic joint following it through a passive four-bar. Mimic joints are enforced
as constraints rather than driven through contact, so a book between the pads has nothing
to push back against: the driven joint goes where it is told and the linkage follows.

Measured 2026-09-08 across twenty runs: three clamps, every one on nothing, the jaws
closing to 27.2 and 28.7 mm around a book 30.0 mm thick while Gazebo had the book still on
its shelf, unmoved. MANIPULATION.md carries the detail and the references. It is a
documented Gazebo limitation, not a fault in the solution.

Why pose-following rather than a detachable joint
-------------------------------------------------
A DetachableJoint per book was built first and abandoned. It works -- it welded twenty
books to a fingertip hard enough to hold the whole robot still -- but it attaches itself
the moment it finds its child, so every book had to be released at spawn, and even
released the twenty plugins left the base barely able to turn: quaternion z moved 0.04
degrees in ten seconds during a search that should sweep several revolutions. Four runs
failed in the search with them in and none without.

Setting the book's pose instead was ruled out for a long time by a line in STATE.md: one
set_pose takes the real-time factor from 0.48 to 0.04, permanently. That was measured
teleporting the ROBOT, an articulated body with controllers attached, and it does not
carry across. Measured on a book, 2026-09-08:

    before any set_pose   0.324
    after thirty of them  0.387
    ten seconds later     0.339

which is noise. So the book simply follows the gripper, and nothing is added to the
physics at all.

The tolerance is deliberately tight. This is meant to make a good grasp hold, not to
rescue a bad one: a run that would have missed the book should still fail, or the trial
numbers stop meaning anything.
"""
import math
import queue
import subprocess
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory

WORLD = "erc_world"
# BOTH topics, and that distinction cost most of a day.
#
# gripper_command_clamp listens on the public topic and republishes to the raw one, and
# it is entirely reasonable to assume the solution talks to the public one. It does not:
# grasp_node publishes straight to _raw, bypassing the clamp. So this node sat with a
# healthy subscription on a topic the solution never used, and across four whole runs it
# never saw a single gripper command -- while a hand-published message on the same topic
# reached it instantly, which is what finally gave it away.
#
# Listening to both costs nothing and does not care which one anything uses.
GRIPPER_TOPICS = ("/gripper_left_controller/joint_trajectory",
                  "/gripper_left_controller_raw/joint_trajectory")
FINGER = "gripper_left_finger_joint"
GRASP_LINK = "gripper_left_grasping_link"

# Gazebo reports the tiago_pro model pose at base_FOOTPRINT, on the floor, and TF gives
# the gripper in base_LINK. Composing the two without this puts the gripper 76 mm below
# where it is, and 76 mm is most of the 90 mm this node will hold from.
#
# base_footprint_joint in tiago_pro.urdf. Deliberately a local copy rather than an import
# from avaa_solution: this file is scaffolding that Phase 2 drops, and the dependency runs
# one way. See avaa_solution/arena.py for the same number and what it cost elsewhere.
BASE_LINK_Z = 0.0762

# Below this commanded position the solution is closing; above it, opening. The jaws run
# 0.000 shut to 0.069 open, so this sits well clear of both.
CLOSING_BELOW = 0.020
OPENING_ABOVE = 0.040


def gz(*args, timeout=15):
    try:
        return subprocess.run(["gz", *args], capture_output=True, text=True,
                              timeout=timeout).stdout
    except Exception:  # noqa: BLE001 - the simulator may be busy or gone
        return ""


class GraspFix(Node):
    def __init__(self):
        super().__init__("sim_grasp_fix")
        self.declare_parameter("reach_m", 0.09)
        self.declare_parameter("world", WORLD)
        self.declare_parameter("follow_hz", 5.0)
        self.reach = float(self.get_parameter("reach_m").value)
        self.world = str(self.get_parameter("world").value)
        follow_hz = float(self.get_parameter("follow_hz").value)

        self.held = None
        self.offset = None          # book position in the gripper's frame, at pick-up
        self.book_yaw_offset = 0.0
        self.books = {}
        self.robot_pose = None
        self.attach_asked = False

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        for topic in GRIPPER_TOPICS:
            self.create_subscription(JointTrajectory, topic, self._on_gripper, 10)
        self.pub_state = self.create_publisher(String, "/grasp_fix/holding", 10)

        # Every call to gz is a subprocess, and a subprocess in a callback stops the node
        # dead: a four-second timer once fired thirty seconds apart and the gripper
        # subscription -- the entire point of this node -- was never serviced across a
        # whole run. The executor now only reads messages and queues work.
        self.work = queue.Queue()
        self.worker = threading.Thread(target=self._serve, daemon=True)
        self.worker.start()

        self.create_timer(3.0, lambda: self._ask("poses"))
        self.create_timer(1.0 / max(follow_hz, 0.5), lambda: self._ask("follow"))
        self.create_timer(1.0, self._report)
        self.get_logger().info(
            "simulation grasp fix up: a close within %.0f mm of a book picks it up, and "
            "it then follows the gripper at %.0f Hz" % (self.reach * 1000, follow_hz))

    # ------------------------------------------------------------------ plumbing
    def _ask(self, job, payload=None):
        """Queue work for the worker, dropping it if the worker is behind."""
        if self.work.qsize() < 3:
            self.work.put((job, payload))

    def _serve(self):
        """The only thread that ever runs a gz command."""
        while True:
            job, payload = self.work.get()
            try:
                if job == "poses":
                    self._read_poses()
                elif job == "pick":
                    self._pick()
                elif job == "follow":
                    self._follow()
            except Exception as exc:  # noqa: BLE001 - a worker must not die quietly
                self.get_logger().error("grasp fix worker: %s" % exc)

    # ------------------------------------------------------------------ world state
    def _read_poses(self):
        """Read every book pose and the robot's, from the simulator. Worker only."""
        raw = gz("topic", "-e", "-t", "/world/%s/dynamic_pose/info" % self.world, "-n", "1")
        if not raw:
            return
        # Position and orientation apart, because they share the key names x, y and z.
        #
        # A flat parse that took the first x, y, z after a name got the position right
        # and threw the orientation away, so the base heading had to come from odom --
        # and odom's frame is only world-aligned if the robot spawned at yaw zero. It
        # does not. The composed gripper position was out by roughly the arm's own
        # extension: the nearest book to it measured 667 mm away while the jaws were
        # closed around one.
        poses, name, section, fields = {}, None, None, {}
        for line in raw.splitlines():
            line = line.strip()
            if line.startswith('name: "'):
                if name and "px" in fields:
                    poses[name] = dict(fields)
                name, section, fields = line.split('"')[1], None, {}
            elif line.startswith("position"):
                section = "p"
            elif line.startswith("orientation"):
                section = "q"
            elif name and section and ":" in line:
                key, _, value = line.partition(":")
                key = key.strip()
                if key in ("x", "y", "z", "w"):
                    tag = section + key
                    if tag not in fields:
                        try:
                            fields[tag] = float(value)
                        except ValueError:
                            pass
        if name and "px" in fields:
            poses[name] = dict(fields)

        books = {k: (v["px"], v["py"], v["pz"]) for k, v in poses.items()
                 if k.startswith("book_") and {"px", "py", "pz"} <= set(v)}
        if books:
            self.books = books
        robot = poses.get("tiago_pro")
        if robot and {"px", "py", "pz", "qx", "qy", "qz", "qw"} <= set(robot):
            yaw = math.atan2(
                2.0 * (robot["qw"] * robot["qz"] + robot["qx"] * robot["qy"]),
                1.0 - 2.0 * (robot["qy"] ** 2 + robot["qz"] ** 2))
            self.robot_pose = (robot["px"], robot["py"], robot["pz"], yaw)

    def _grasp_link_world(self):
        """Where the grasping link is, in world coordinates, and the base heading.

        Composed from the robot's TRUE pose and the gripper's pose in base_link rather
        than read out of odom: this base slides across its wheels without turning them,
        and during one run that held to 17 mm of true error odom had accumulated 813 mm
        of travel that never happened.
        """
        try:
            tf = self.buf.lookup_transform("base_link", GRASP_LINK, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None, None
        if self.robot_pose is None:
            return None, None
        bx, by, bz, yaw = self.robot_pose
        t = tf.transform.translation
        return (bx + t.x * math.cos(yaw) - t.y * math.sin(yaw),
                by + t.x * math.sin(yaw) + t.y * math.cos(yaw),
                bz + BASE_LINK_Z + t.z), yaw

    # ------------------------------------------------------------------ the trigger
    def _on_gripper(self, msg: JointTrajectory):
        if FINGER not in msg.joint_names or not msg.points:
            return
        index = list(msg.joint_names).index(FINGER)
        try:
            asked = float(msg.points[0].positions[index])
        except (IndexError, TypeError):
            return

        self.get_logger().info(
            "gripper commanded to %.4f, holding %s"
            % (asked, self.held or "nothing"), throttle_duration_sec=3.0)

        if asked <= CLOSING_BELOW and self.held is None and not self.attach_asked:
            self.attach_asked = True
            self._ask("pick")
        elif asked >= OPENING_ABOVE:
            # Opening ends the attempt whether or not anything was held.
            #
            # This used to require self.held, and between that and the early return in
            # _pick below there was no path back: one clamp that came up short set
            # attach_asked and nothing ever cleared it, so the node went quiet for the
            # rest of its life. It is launched once with the simulator and lives for
            # hours, so "the rest of its life" means every later run in that session.
            # Observed exactly that -- a clamp at 1731 mm failed honestly at 05:57, and
            # a grasp four hours later that put the pads 8 mm off the centre of the book
            # was never even considered.
            if self.held is not None:
                self.get_logger().info("released %s" % self.held)
            self.held = None
            self.offset = None
            self.attach_asked = False

    def _pick(self):
        """Decide whether the jaws closed on a book, and take it if so. Worker only."""
        # Read the world FIRST, every time. This decision is made once, at the instant
        # the jaws close, and it is made by comparing a base pose from Gazebo against a
        # gripper pose from TF -- so a cached base pose is compared against a live arm.
        #
        # The cache is refreshed on a 3 second timer, and this base does not hold still:
        # measured during one grasp it turned 19.4 degrees and slid 400 mm. Three seconds
        # of that is tens of millimetres of pure bookkeeping error against a 90 mm
        # tolerance. On the run that found this, the grasp controller had its pads 1 mm
        # off the centre line of the book and this node measured the same book 89 mm off
        # sideways and refused it.
        self._read_poses()
        here, yaw = self._grasp_link_world()
        if here is None:
            self.get_logger().warn(
                "no pose for %s, so nothing can be picked up" % GRASP_LINK)
            self.attach_asked = False
            return
        best, best_gap = None, None
        for name, position in self.books.items():
            gap = math.dist(here, position)
            if best_gap is None or gap < best_gap:
                best, best_gap = name, gap
        if best is None:
            self.get_logger().warn(
                "jaws closed and no book poses are known, so nothing can be picked up")
            self.attach_asked = False
            return
        if best_gap > self.reach:
            # Show the working, not just the verdict.
            #
            # This reported a book 858 mm away in a run where the grasp controller had
            # measured its own pads 87 mm past the book face -- both cannot be true, and
            # a bare distance gives nothing to tell which is wrong. The composition has
            # been the fault before: an earlier version took the base heading from odom,
            # whose frame is only world-aligned if the robot spawned at yaw zero, and
            # reported 667 mm with the jaws closed around a book.
            book = self.books.get(best)
            self.get_logger().info(
                "jaws closed with the nearest book (%s) %.0f mm away, further than the "
                "%.0f mm this will hold from. Letting the grasp fail honestly.\n"
                "    gripper composed at (%.3f, %.3f, %.3f), base yaw %+.1f deg\n"
                "    base at (%.3f, %.3f, %.3f), %d book poses known\n"
                "    %s at (%.3f, %.3f, %.3f)"
                % (best, best_gap * 1000, self.reach * 1000,
                   here[0], here[1], here[2], math.degrees(yaw),
                   self.robot_pose[0], self.robot_pose[1], self.robot_pose[2],
                   len(self.books),
                   best, book[0], book[1], book[2]))
            # Ready for the next clamp. This is a verdict on one close, not on the run.
            self.attach_asked = False
            return

        # Remember how it was picked up, so it is carried the same way rather than
        # snapping to the middle of the jaws.
        book = self.books[best]
        dx, dy = book[0] - here[0], book[1] - here[1]
        self.offset = (dx * math.cos(-yaw) - dy * math.sin(-yaw),
                       dx * math.sin(-yaw) + dy * math.cos(-yaw),
                       book[2] - here[2])
        self.held = best
        self.get_logger().info(
            "picked up %s, %.0f mm from the grasping link" % (best, best_gap * 1000))

    def _follow(self):
        """Keep the held book where the gripper is. Worker only."""
        if self.held is None or self.offset is None:
            return
        here, yaw = self._grasp_link_world()
        if here is None:
            return
        ox, oy, oz = self.offset
        x = here[0] + ox * math.cos(yaw) - oy * math.sin(yaw)
        y = here[1] + ox * math.sin(yaw) + oy * math.cos(yaw)
        z = here[2] + oz
        # The book was spawned rotated a quarter turn; keep it that way and let it yaw
        # with the base, so it looks carried rather than dragged sideways.
        half = 0.5 * (yaw + math.pi / 2.0)
        gz("service", "-s", "/world/%s/set_pose" % self.world,
           "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
           "--timeout", "800",
           "--req", 'name: "%s", position: {x: %f, y: %f, z: %f}, '
                    'orientation: {x: %f, y: %f, z: %f, w: %f}'
                    % (self.held, x, y, z,
                       math.sin(half) * 0.0, math.sin(half) * 0.0,
                       math.sin(half), math.cos(half)),
           timeout=3)

    def _report(self):
        self.pub_state.publish(String(data=self.held or ""))


def main():
    rclpy.init()
    node = GraspFix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
