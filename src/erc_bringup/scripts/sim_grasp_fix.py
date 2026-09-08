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
import subprocess
import sys

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


class GraspFix(Node):
    def __init__(self):
        super().__init__("sim_grasp_fix")
        self.declare_parameter("reach_m", 0.09)
        self.declare_parameter("world", WORLD)
        self.reach = float(self.get_parameter("reach_m").value)
        self.world = str(self.get_parameter("world").value)

        self.held = None
        self.books = {}
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.create_subscription(
            JointTrajectory, GRIPPER_TOPIC, self._on_gripper, 10)
        self.pub_state = self.create_publisher(String, "/grasp_fix/holding", 10)
        self.create_timer(5.0, self._refresh_books)
        self.create_timer(1.0, self._report)
        self.get_logger().info(
            "simulation grasp fix up: a close within %.0f mm of a book will attach it"
            % (self.reach * 1000))

    # ---------------------------------------------------------------- ground truth
    def _refresh_books(self):
        """Read every book pose from the simulator. They do not move until grasped."""
        if self.held is not None:
            return
        raw = gz("topic", "-e", "-t", "/world/%s/dynamic_pose/info" % self.world, "-n", "1")
        if not raw:
            return
        found = {}
        name = None
        for line in raw.splitlines():
            line = line.strip()
            if line.startswith('name: "'):
                name = line.split('"')[1]
            elif name and name.startswith("book_") and line.startswith("x:"):
                found.setdefault(name, [None, None, None])[0] = float(line.split()[1])
            elif name and name.startswith("book_") and line.startswith("y:"):
                found.setdefault(name, [None, None, None])[1] = float(line.split()[1])
            elif name and name.startswith("book_") and line.startswith("z:"):
                found.setdefault(name, [None, None, None])[2] = float(line.split()[1])
                name = None
        complete = {k: v for k, v in found.items() if None not in v}
        if complete:
            self.books = complete

    def _grasp_link_world(self):
        """Where the grasping link is, in world coordinates."""
        try:
            tf = self.buf.lookup_transform("odom", GRASP_LINK, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        # odom starts coincident with world on this robot and the base is never
        # teleported during a run, so odom IS world here. Stated rather than assumed
        # because it is the one thing in this file that would silently rot.
        t = tf.transform.translation
        return (t.x, t.y, t.z)

    # ---------------------------------------------------------------- the trigger
    def _on_gripper(self, msg: JointTrajectory):
        if FINGER not in msg.joint_names or not msg.points:
            return
        index = list(msg.joint_names).index(FINGER)
        try:
            asked = float(msg.points[0].positions[index])
        except (IndexError, TypeError):
            return

        if asked <= CLOSING_BELOW and self.held is None:
            self._try_attach()
        elif asked >= OPENING_ABOVE and self.held is not None:
            self._detach()

    def _try_attach(self):
        here = self._grasp_link_world()
        if here is None:
            self.get_logger().warn("no transform to %s, so nothing to attach" % GRASP_LINK)
            return
        if not self.books:
            self._refresh_books()
        best, best_gap = None, None
        for name, (x, y, z) in self.books.items():
            gap = math.dist(here, (x, y, z))
            if best_gap is None or gap < best_gap:
                best, best_gap = name, gap
        if best is None:
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

    def _detach(self):
        gz("topic", "-t", "/grasp_fix/%s/detach" % self.held,
           "-m", "gz.msgs.Empty", "-p", "unused: true")
        self.get_logger().info("released %s" % self.held)
        self.held = None
        self.books = {}

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
