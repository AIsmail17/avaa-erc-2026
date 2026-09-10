#!/usr/bin/env python3
"""Hand the grasp controller the truth about a book, in the arena, and measure the reach.

    tools/in-sim arenafeed.py red            # the red book, whichever column it is in
    tools/in-sim arenafeed.py book_col_3_row_2_red 300

This is bookfeed's sibling for the real world. bookfeed exists on the bench, where there
is no shelf and it can put the book where it likes; here nothing is moved at all. The
book stays on its shelf, the robot stays where it drove to, and the only thing supplied
is what perception would have supplied if it were right: a row on
/avaa/perception/target_row and a point on /avaa/perception/target_book_point.

Why that is worth doing rather than just fixing perception first
---------------------------------------------------------------
Because they are separable faults and only one of them is understood. Measured on this
world on 2026-09-09: the red book of column 3 sat on row 1 and perception latched row 4,
having voted 15 times without a single vote for the right answer. The arm then reaches
accurately for the row it is given -- the bench measured +24 mm in depth and -5 mm in
height at the clamp -- so a run that ends with the hand a shelf or three below the book
says nothing about the arm.

Feeding the truth here answers the other half: whether the grasp works in the arena, with
the shelf present, at whatever standoff the drive actually achieved, once the row is
right. If it does, perception's row is the whole remaining fault.

It prints the true row beside whatever perception is publishing, so the two can be
compared in one line without another tool.
"""
import math
import subprocess
import sys

import rclpy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, Int32, String
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401

WORLD = "erc_world"
TIP_L = "gripper_left_fingertip_left_link"
TIP_R = "gripper_left_fingertip_right_link"
GRASP = "gripper_left_grasping_link"
PAD_LOCAL = (0.0042, 0.0187, 0.0000)
BOOK_DEPTH = 0.16
# base_link sits this far above the floor, so a height measured from the robot's model
# origin has to lose it before being compared with anything in base_link.
#
# This was 0.186 until 2026-09-10 and the URDF says 0.0762, which is why this tool
# used to report a reach as landing 0 mm from the book while the hand was 110 mm
# below it: the same wrong number aimed the arm and then measured the miss. See
# avaa_solution/arena.py. Row 1 is 1.577 in world and 1.5008 in base_link.
BASE_LINK_Z = 0.0762
# grasp_node's grasp_below_centre_m, so the expected height miss is not zero.
GRASP_BELOW_CENTRE = 0.045

# The four stocked rows, in world z. From simulation.launch.py: the shelf centre is at
# 1.1, the top board offset is 0.825, rows are 0.33 apart, and rows 0 and 5 are left
# empty -- so the stocked rows are indices 1 to 4 and these are their heights.
ROW_WORLD_Z = [1.1 + 0.825 - i * 0.33 for i in (1, 2, 3, 4)]

WANTED = sys.argv[1] if len(sys.argv) > 1 else "red"
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 300.0


def gz(*args, timeout=20):
    try:
        return subprocess.run(["gz", *args], capture_output=True, text=True,
                              timeout=timeout).stdout
    except Exception:  # noqa: BLE001
        return ""


def poses():
    raw = gz("topic", "-e", "-t", "/world/%s/dynamic_pose/info" % WORLD, "-n", "1")
    out, name, section, fields = {}, None, None, {}
    for line in raw.splitlines():
        line = line.strip()
        if line.startswith('name: "'):
            if name and "px" in fields:
                out[name] = dict(fields)
            name, section, fields = line.split('"')[1], None, {}
        elif line.startswith("position"):
            section = "p"
        elif line.startswith("orientation"):
            section = "q"
        elif name and section and ":" in line:
            k, _, v = line.partition(":")
            k = k.strip()
            if k in ("x", "y", "z", "w") and section + k not in fields:
                try:
                    fields[section + k] = float(v)
                except ValueError:
                    pass
    if name and "px" in fields:
        out[name] = dict(fields)
    return out


def true_row(world_z):
    """Which stocked row a height belongs to, 1-based from the top."""
    best = min(range(len(ROW_WORLD_Z)), key=lambda i: abs(ROW_WORLD_Z[i] - world_z))
    return best + 1, abs(ROW_WORLD_Z[best] - world_z)


def main():
    rclpy.init()
    node = rclpy.create_node("arenafeed")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    buf = Buffer()
    listener = TransformListener(buf, node)
    _ = listener

    pub_row = node.create_publisher(Int32, "/avaa/perception/target_row", 10)
    pub_point = node.create_publisher(
        PointStamped, "/avaa/perception/target_book_point", 10)
    pub_yaw = node.create_publisher(Float32, "/avaa/perception/shelf_yaw", 10)

    states = []
    at_clamp = {}
    seen_row = {}

    def on_state(m):
        if not states or states[-1] != m.data:
            states.append(m.data)
            print("  state: %s" % m.data, flush=True)
        if m.data == "clamping" and "pad" not in at_clamp:
            at_clamp["pending"] = True

    node.create_subscription(String, "/avaa/grasp/state", on_state, 10)

    def sim_now():
        return node.get_clock().now().nanoseconds * 1e-9

    def spin(seconds):
        while rclpy.ok() and sim_now() == 0.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        end = sim_now() + seconds
        while rclpy.ok() and sim_now() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    def at(link, local):
        p = PointStamped()
        p.header.frame_id = link
        p.point.x, p.point.y, p.point.z = (float(v) for v in local)
        out = buf.transform(p, "base_link",
                            timeout=rclpy.duration.Duration(seconds=2))
        return (out.point.x, out.point.y, out.point.z)

    spin(3.0)

    world = poses()
    robot = world.get("tiago_pro")
    if robot is None:
        print("no robot in the world")
        return 1
    if WANTED in world:
        book_name = WANTED
    else:
        named = sorted(k for k in world
                       if k.startswith("book_") and k.endswith("_" + WANTED))
        if not named:
            print("no book matching %r; the world has: %s"
                  % (WANTED, ", ".join(sorted(k for k in world
                                              if k.startswith("book_"))[:8])))
            return 1
        # Nearest of that colour, which is the one the robot has driven to.
        book_name = min(named, key=lambda k: math.hypot(world[k]["px"] - robot["px"],
                                                        world[k]["py"] - robot["py"]))
    book = world[book_name]

    def base_yaw(robot_pose):
        """World heading of the base, which is also its yaw error against the shelf.

        The shelf faces -x and a robot square on to it stands at world yaw zero, so the
        two are the same number. perception_node publishes it as shelf_yaw with exactly
        that meaning: zero when square on, positive yawed counter-clockwise.
        """
        return math.atan2(
            2.0 * (robot_pose["qw"] * robot_pose["qz"]
                   + robot_pose["qx"] * robot_pose["qy"]),
            1.0 - 2.0 * (robot_pose["qy"] ** 2 + robot_pose["qz"] ** 2))

    def in_base(robot_pose):
        """The book, in the base frame of a robot at this pose."""
        yaw = math.atan2(
            2.0 * (robot_pose["qw"] * robot_pose["qz"]
                   + robot_pose["qx"] * robot_pose["qy"]),
            1.0 - 2.0 * (robot_pose["qy"] ** 2 + robot_pose["qz"] ** 2))
        dx = book["px"] - robot_pose["px"]
        dy = book["py"] - robot_pose["py"]
        return (dx * math.cos(-yaw) - dy * math.sin(-yaw),
                dx * math.sin(-yaw) + dy * math.cos(-yaw),
                book["pz"] - robot_pose["pz"])

    bx, by, bz = in_base(robot)
    face_x = bx - BOOK_DEPTH / 2.0

    row, miss = true_row(book["pz"])

    node.create_subscription(
        Int32, "/avaa/perception/target_row",
        lambda m: seen_row.setdefault("perception", m.data), 10)

    print("book %s" % book_name)
    print("  world              (%.3f, %.3f, %.3f)" % (book["px"], book["py"], book["pz"]))
    print("  in base_link       (%.3f, %.3f, %.3f), face at x=%.3f" % (bx, by, bz, face_x))
    print("  true row           %d  (%.0f mm from the row height, of 330 mm spacing)"
          % (row, miss * 1000))
    print("  standoff           %.3f m from the face" % face_x)
    print("\nfeeding row %d for %.0f simulated seconds" % (row, SECONDS))

    # Recompute the point every second, because base_link MOVES.
    #
    # This published one figure worked out at the start and kept publishing it, which is
    # what bookfeed does on the bench and is safe there only because the bench pins the
    # base. In the arena grasp_node drives the base to hold it against the book, so a
    # point fixed in base_link walks away from the book at exactly the rate the robot
    # corrects itself. Measured 2026-09-09: a run that reached its target to 2 mm and ran
    # all the way to "done" closed the jaws at world y = -1.021 on a book at y = +0.056,
    # 1.08 m away, on air. sim_grasp_fix reported the nearest book as one from another
    # column entirely, and the book never moved.
    #
    # Every reading is a subprocess, so this holds to about 1 Hz. That is fine here: the
    # base moves at centimetres a second and nothing is being teleported.
    last_read = [0.0]
    yaw_now = [base_yaw(robot)]
    end = None
    while rclpy.ok():
        if end is None:
            end = sim_now() + SECONDS
        if sim_now() > end:
            break
        if sim_now() - last_read[0] > 1.0 and "pad" not in at_clamp:
            last_read[0] = sim_now()
            fresh = poses().get("tiago_pro")
            if fresh is not None:
                bx, by, bz = in_base(fresh)
                face_x = bx - BOOK_DEPTH / 2.0
                yaw_now[0] = base_yaw(fresh)
        pub_row.publish(Int32(data=int(row)))
        point = PointStamped()
        point.header.frame_id = "base_link"
        point.header.stamp = node.get_clock().now().to_msg()
        point.point.x, point.point.y, point.point.z = face_x, by, bz
        pub_point.publish(point)
        # The TRUE yaw error, not a zero.
        #
        # This published 0.0 -- "the shelf is square, nothing to correct" -- for as long
        # as it existed, and that is not a neutral placeholder. grasp_node's base hold
        # has two channels, a linear one from the book point and an angular one from
        # this, and a constant zero switches the angular one off while telling the
        # controller it is working. Measured on the run that found it: the base turned
        # 19.4 degrees during a single grasp, the hold never commanded a single
        # correction for it, and a book 0.72 m out therefore swung 240 mm sideways --
        # past the 200 mm the linear channel will admit, so that gave up too. The grasp
        # aid then measured the nearest book 101 mm from the jaws and refused it.
        #
        # A harness may leave a signal out. It may not feed a wrong one: the whole point
        # of arenafeed is to supply what perception WOULD supply if it were right, and
        # perception would supply this.
        pub_yaw.publish(Float32(data=float(yaw_now[0])))
        if at_clamp.pop("pending", False):
            try:
                pa, pb = at(TIP_L, PAD_LOCAL), at(TIP_R, PAD_LOCAL)
                at_clamp["pad"] = tuple((pa[i] + pb[i]) / 2.0 for i in range(3))
                at_clamp["grasp"] = at(GRASP, (0.0, 0.0, 0.0))
            except Exception as exc:  # noqa: BLE001
                print("  could not read the arrival (%s)" % exc)
        if states and states[-1] in ("done", "failed"):
            spin(2.0)
            break
        spin(0.2)

    print("\nstates: %s" % " -> ".join(states) if states else "\nthe controller never ran")

    if "pad" in at_clamp:
        pad = at_clamp["pad"]
        # The book moves once it is grasped, so compare against where it was when fed.
        print("\nAT THE CLAMP, in base_link:")
        print("  pad middle      (%+.3f, %+.3f, %+.3f)" % pad)
        print("  grasping frame  (%+.3f, %+.3f, %+.3f)" % at_clamp["grasp"])
        # bz is the book's height above the ROBOT ORIGIN, which is on the floor, while
        # the pads are in base_link, 0.0762 m up. Comparing them directly reported a
        # 234 mm height error on a reach that was three millimetres out.
        book_in_base_z = bz - BASE_LINK_Z
        print("  book centre     (%+.3f, %+.3f, %+.3f)" % (bx, by, book_in_base_z))
        print("  MISS: %+.0f mm depth, %+.0f mm sideways, %+.0f mm in HEIGHT"
              % ((pad[0] - bx) * 1000, (pad[1] - by) * 1000,
                 (pad[2] - book_in_base_z) * 1000))
        print("  (the grasp aims %.0f mm below centre on purpose)"
              % (GRASP_BELOW_CENTRE * 1000))
    else:
        print("\nthe clamp was never reached, so there is no arrival to report")

    after = poses().get(book_name)
    if after is not None:
        moved = math.dist((after["px"], after["py"], after["pz"]),
                          (book["px"], book["py"], book["pz"]))
        print("\nthe book moved %.0f mm during the run" % (moved * 1000))

    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
