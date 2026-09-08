#!/usr/bin/env python3
"""Make a simulated grasp hold, because this gripper cannot.

SIMULATION SCAFFOLDING. Nothing in avaa_solution knows this node exists, and Phase 2
drops it by not launching it. It is deliberately in erc_bringup, beside the other things
that exist only because the robot is simulated.

Why it is needed
----------------
gripper_left_finger_joint is position-commanded, and every link that actually touches a
book is a mimic joint following it through a passive four-bar. Mimic joints are enforced
as constraints rather than driven through contact, so a book between the pads has nothing
to push back against: the driven joint goes where it is told and the linkage follows.

Measured on 2026-09-08, three clamps across twenty runs, every one of them on nothing --
the jaws closing to 28.7 mm around a book 30.0 mm thick while Gazebo had the book still
on its shelf at x = 2.900, unmoved. It is a documented Gazebo limitation, not a fault in
the solution; MANIPULATION.md carries the references.

What it does
------------
Watches the gripper command the solution already publishes. On a close, if the grasping
link is within reach of a book, it fires that book's attach topic and a detachable joint
welds the two together. On an open, it detaches.

It decides using Gazebo ground truth, which is legitimate for a simulator aid and would
be meaningless on hardware -- which is the point. The real gripper closes on the real
book and none of this is wanted.

WHERE THIS STANDS (2026-09-08)
------------------------------
The plumbing is in and verified: every book carries a DetachableJoint, sixty topics are
advertised, the attach topic has the plugin as a subscriber, and this node runs and finds
the books. The weld itself does not take. Attaching a live book and then raising the torso
270 mm moves the book 3 mm, which is settling, not carrying.

Ruled out on the way, so it does not have to be re-checked:

  - the plugin loads and subscribes (gz topic -i shows it on the attach topic);
  - the generated SDF is well formed and one file per book, so an attach cannot pick up
    the wrong one;
  - the child link exists in the SIMULATED model. gripper_left_grasping_link and
    gripper_left_base_link do not: they are massless frames and Gazebo collapses them,
    so gz model -m tiago_pro -l lists only the finger chain. Naming one of those made
    the plugin load, subscribe, and silently weld nothing;
  - the torso really moves, and the book name really exists in the current launch. The
    layout is re-randomised every launch, and two of the early tests fired at a book
    from the previous one.

The next thing to try is inverting the direction. The plugin currently lives on the book,
which makes a free rigid body the parent and a large articulated robot the child. Every
documented use has the carrier as the parent -- a vehicle carrying a payload -- and gz-sim
may simply decline to re-parent an articulation. That means putting the plugin on the
robot instead, with parent_link the fingertip and child_model the book, which needs
gazebo tags injected into robot_description rather than into the book SDF.

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
GRIPPER_TOPIC = "/gripper_left_controller/joint_trajectory"
FINGER = "gripper_left_finger_joint"
GRASP_LINK = "gripper_left_grasping_link"

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


def gz_publish_all(topics, timeout=15):
    """Publish an empty message to many topics at once.

    In series this took about 25 seconds for twenty books -- a process spawn each --
    and that is 25 seconds during which every book is welded to a fingertip while the
    robot is trying to tuck its arms and stand still. Started together they finish in
    about one.
    """
    running = []
    for topic in topics:
        try:
            running.append(subprocess.Popen(
                ["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty",
                 "-p", "unused: true"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        except Exception:  # noqa: BLE001
            pass
    for process in running:
        try:
            process.wait(timeout=timeout)
        except Exception:  # noqa: BLE001
            process.kill()
    return len(running)


class GraspFix(Node):
    def __init__(self):
        super().__init__("sim_grasp_fix")
        self.declare_parameter("reach_m", 0.09)
        self.declare_parameter("world", WORLD)
        self.reach = float(self.get_parameter("reach_m").value)
        self.world = str(self.get_parameter("world").value)

        self.held = None
        self.books = {}
        self.robot_pose = None
        self.attach_asked = False
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.create_subscription(
            JointTrajectory, GRIPPER_TOPIC, self._on_gripper, 10)
        self.pub_state = self.create_publisher(String, "/grasp_fix/holding", 10)
        self.create_timer(5.0, self._refresh_books)
        self.create_timer(1.0, self._report)

        # Keep releasing for the first half minute, not just once.
        #
        # DetachableJoint attaches as soon as it finds its child, and the books spawn
        # over several seconds. A single release at startup leaves every book that
        # spawns afterwards welded, and even the ones released leave a window: twenty
        # fixed joints between a shelf of books and one fingertip, all to be satisfied
        # at once, is a large disturbance to hand a physics solver during the seconds
        # when the robot is trying to stand still and tuck its arms.
        #
        # The books do not all exist yet either, so their topics do not all exist yet,
        # which is why this rescans rather than reusing a list.
        # A BOUNDED number of sweeps, not a timer that keeps going.
        #
        # Every gz call here is a subprocess, and this node has one thread. Sweeping
        # twenty books every half second means twenty process spawns of about half a
        # second each -- ten seconds of work asked for every half second -- and the
        # executor never gets back to the gripper subscription that is the entire point
        # of the node. Three sweeps, spaced, is enough: the books all spawn within a
        # second or two of each other.
        # Every call to gz is a subprocess, and a subprocess in a callback stops the
        # node dead. Measured: a sweep timer set to 4 seconds fired 25 and 31 seconds
        # apart, and the gripper subscription -- the entire point of this node -- was
        # never serviced at all across a whole run. The grasp clamped, the grip check
        # failed it honestly, and nothing here had so much as seen a gripper command.
        #
        # So the executor now does nothing but read messages and put work on a queue.
        # One worker thread owns every conversation with the simulator.
        self.work = queue.Queue()
        self.worker = threading.Thread(target=self._serve, daemon=True)
        self.worker.start()
        for _ in range(3):
            self.work.put(("sweep", None))
        self.get_logger().info(
            "simulation grasp fix up: a close within %.0f mm of a book will attach it"
            % (self.reach * 1000))

    def _release_everything(self):
        """Detach every book, because DetachableJoint starts ATTACHED.

        This is not defensive tidying. The plugin welds itself the moment it finds its
        child model, so all twenty books weld to the fingertip the instant they spawn --
        and twenty fixed joints to books bolted on a shelf pin the robot where it stands.

        That is exactly what happened on 2026-09-08. The base stopped moving: a drive
        commanded to a book reported "at [-0.58, 1.06] yaw -96 deg, 1.71 m to go" three
        times running without shifting a centimetre, and two runs before that failed in
        the search having seen 0 markers across 123 tallies, because the robot had never
        turned. It looked like a perception regression and it was a parking brake.

        So: release everything at startup, and hold nothing until a grasp asks for it.
        """
        released = 0
        for name in self._book_names():
            gz("topic", "-t", "/grasp_fix/%s/detach" % name,
               "-m", "gz.msgs.Empty", "-p", "unused: true")
            released += 1
        self.held = None
        self.get_logger().info(
            "released %d books that the detachable joints had welded on at spawn"
            % released)

    def _serve(self):
        """The only thread that ever runs a gz command."""
        while True:
            job, payload = self.work.get()
            try:
                if job == "sweep":
                    names = self._book_names()
                    fired = gz_publish_all(
                        ["/grasp_fix/%s/detach" % name for name in names])
                    self.get_logger().info("sweep released %d books" % fired)
                elif job == "poses":
                    self._read_poses()
                elif job == "attach":
                    self._do_attach()
                elif job == "detach":
                    gz("topic", "-t", "/grasp_fix/%s/detach" % payload,
                       "-m", "gz.msgs.Empty", "-p", "unused: true")
                    self.get_logger().info("released %s" % payload)
            except Exception as exc:  # noqa: BLE001 - a worker must not die quietly
                self.get_logger().error("grasp fix worker: %s" % exc)

    def _book_names(self):
        """Every book the simulator has a detach topic for."""
        raw = gz("topic", "-l")
        names = []
        for line in raw.splitlines():
            line = line.strip()
            if line.startswith("/grasp_fix/") and line.endswith("/detach"):
                names.append(line[len("/grasp_fix/"):-len("/detach")])
        return names

    # ---------------------------------------------------------------- ground truth
    def _refresh_books(self):
        """Ask the worker for fresh poses. Never blocks."""
        if self.work.qsize() < 3:
            self.work.put(("poses", None))

    def _read_poses(self):
        """Read every book pose from the simulator. Worker thread only."""
        raw = gz("topic", "-e", "-t", "/world/%s/dynamic_pose/info" % self.world, "-n", "1")
        if not raw:
            return
        poses, name, fields = {}, None, {}
        for line in raw.splitlines():
            line = line.strip()
            if line.startswith('name: "'):
                if name and len(fields) >= 3:
                    poses[name] = dict(fields)
                name, fields = line.split('"')[1], {}
            elif name and ":" in line:
                key, _, value = line.partition(":")
                key = key.strip()
                if key in ("x", "y", "z", "w") and key not in fields:
                    try:
                        fields[key] = float(value)
                    except ValueError:
                        pass
        if name and len(fields) >= 3:
            poses[name] = dict(fields)

        books = {k: (v["x"], v["y"], v["z"]) for k, v in poses.items()
                 if k.startswith("book_") and {"x", "y", "z"} <= set(v)}
        if books:
            self.books = books
        robot = poses.get("tiago_pro")
        if robot and {"x", "y", "z"} <= set(robot):
            # Yaw only; the quaternion's x and y are shared with the position keys in
            # this flat parse, so take the heading from the base_link TF instead when
            # it matters. Position is what the distance check needs.
            self.robot_pose = (robot["x"], robot["y"], robot["z"], self._base_yaw())

    def _base_yaw(self):
        """The base heading, from TF, which is exact for orientation even when odom
        position has drifted: odom rotation is not what slides."""
        try:
            tf = self.buf.lookup_transform("odom", "base_link", rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return 0.0
        q = tf.transform.rotation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _grasp_link_world(self):
        """Where the grasping link is, in world coordinates.

        Composed from the robot's TRUE pose and the gripper's pose in base_link, not
        read out of odom. odom would have been the obvious choice and is the wrong one:
        this base slides across its wheels without turning them, and during one run that
        held to 17 mm of true error odom had accumulated 813 mm of travel that never
        happened. A distance check built on that would drift out of usefulness over
        exactly the minutes a run takes.

        Using ground truth is fine here and nowhere else. This node exists only because
        the robot is simulated.
        """
        try:
            tf = self.buf.lookup_transform("base_link", GRASP_LINK, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        base = self.robot_pose
        if base is None:
            return None
        bx, by, bz, yaw = base
        t = tf.transform.translation
        # Planar rotation is enough: this base does not pitch or roll.
        return (bx + t.x * math.cos(yaw) - t.y * math.sin(yaw),
                by + t.x * math.sin(yaw) + t.y * math.cos(yaw),
                bz + t.z)

    # ---------------------------------------------------------------- the trigger
    def _on_gripper(self, msg: JointTrajectory):
        if FINGER not in msg.joint_names or not msg.points:
            return
        index = list(msg.joint_names).index(FINGER)
        try:
            asked = float(msg.points[0].positions[index])
        except (IndexError, TypeError):
            return

        self.get_logger().info(
            "gripper commanded to %.4f (closing below %.3f, opening above %.3f), "
            "currently holding %s"
            % (asked, CLOSING_BELOW, OPENING_ABOVE, self.held or "nothing"),
            throttle_duration_sec=2.0)
        if asked <= CLOSING_BELOW and self.held is None and not self.attach_asked:
            self.attach_asked = True
            self.work.put(("attach", None))
        elif asked >= OPENING_ABOVE and self.held is not None:
            name, self.held = self.held, None
            self.attach_asked = False
            self.books = {}
            self.work.put(("detach", name))

    def _do_attach(self):
        here = self._grasp_link_world()
        if here is None:
            self.get_logger().warn("no transform to %s, so nothing to attach" % GRASP_LINK)
            self.attach_asked = False
            return
        if not self.books:
            # Already on the worker thread, so read directly rather than queueing a
            # job behind ourselves and then finding the books still empty.
            self._read_poses()
        best, best_gap = None, None
        for name, (x, y, z) in self.books.items():
            gap = math.dist(here, (x, y, z))
            if best_gap is None or gap < best_gap:
                best, best_gap = name, gap
        if best is None:
            # Silence here cost a run: the jaws closed, this returned without a word,
            # and the log showed a grasp failing with no sign the fix had been asked.
            self.get_logger().warn(
                "jaws closed and this knows of no books at all, so nothing can be "
                "attached. %d pose(s) last read from the simulator." % len(self.books))
            self.attach_asked = False
            return
        if best_gap > self.reach:
            self.get_logger().info(
                "jaws closed with the nearest book (%s) %.0f mm away, further than the "
                "%.0f mm this will hold from. Letting the grasp fail honestly."
                % (best, best_gap * 1000, self.reach * 1000))
            return
        gz("topic", "-t", "/grasp_fix/%s/attach" % best,
           "-m", "gz.msgs.Empty", "-p", "unused: true")
        self.held = best
        self.get_logger().info(
            "attached %s, %.0f mm from the grasping link" % (best, best_gap * 1000))

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
