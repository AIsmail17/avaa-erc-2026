#!/usr/bin/env python3
"""Feed the real grasp controller a book, on the bench, and measure where the pads stop.

    tools/in-sim bookfeed.py [seconds]

grasp_node needs exactly two things to start: a row on /avaa/perception/target_row and a
book point on /avaa/perception/target_book_point. It takes the point's x as the book face
in base_link and ignores the frame, so the bench can hand it the truth from Gazebo instead
of a perception stack, and the whole grasp runs without a camera, a marker, a shelf or a
thirteen minute drive.

This publishes those two, watches /avaa/grasp/state, and at the end reports the number the
argument is actually about: how far the PADS ended up from the book face along the
approach axis.

There is no shelf in the lab world, and that is the experiment. If a reach that stops
short in the arena completes here, then the shelf collision geometry is what stops it --
which is a different fault from the arm being unable to reach, and has a different fix.
"""
import math
import os
import subprocess
import sys

import rclpy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, Int32, String
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401

WORLD = "erc_world"
ROW_HEIGHTS = [1.391, 1.061, 0.731, 0.401]
TIP_L = "gripper_left_fingertip_left_link"
TIP_R = "gripper_left_fingertip_right_link"
GRASP = "gripper_left_grasping_link"
PAD_LOCAL = (0.0042, 0.0187, 0.0000)
BOOK_DEPTH = 0.16
BOOK_TALL = 0.25

# The post the launch put under the book. Its height is decided from ERC_LAB_Z, and it was
# placed standing on the floor, so its centre is at half its height.
LAB_Z = float(os.environ.get("ERC_LAB_Z", "1.00"))
STAND_HEIGHT = LAB_Z - BOOK_TALL / 2.0
STAND = "book_stand"

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 240.0
ROW = int(sys.argv[2]) if len(sys.argv) > 2 else 1

# Where the book is PUT, in base_link, before anything is fed.
#
# The launch places it in world coordinates, and the base does not land in the same spot
# twice: the same ERC_LAB_X=0.75 put the book at base_link x=0.448 on one run and 0.691 on
# the next, because this base coasts on its mecanum wheels. A bench that moves the target
# between runs cannot compare two runs, so the book is teleported to a fixed spot relative
# to the base here instead.
#
# 0.80 m out is where the arm works: measured, it holds all four rows to 6 mm at that
# distance. The height is the one the controller itself aims at for the row -- it reported
# "row 1 at z=1.346" while DEFAULT_ROW_HEIGHTS[1] is 1.061, so there is an offset applied
# somewhere and the book has to be where the controller will actually go, not where the
# table says.
PLACE_X = 0.80
PLACE_Z = {0: 1.676, 1: 1.346, 2: 1.016, 3: 0.686}


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


def main():
    rclpy.init()
    node = rclpy.create_node("bookfeed")
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
    # Where the gripper is AT THE CLAMP, not where it ends up.
    #
    # These numbers used to be read once the feed finished, which is after the arm has
    # lifted, withdrawn and stowed -- so a grasp that ran to "done" reported the pads
    # 698 mm short of the book and 1312 mm out in height, describing a folded arm rather
    # than a reach. That is useless for the one question being asked of it, which is
    # whether the arm arrives at the right HEIGHT.
    at_clamp = {}

    def on_state(m):
        if not states or states[-1] != m.data:
            states.append(m.data)
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
        p.point.x = float(local[0])
        p.point.y = float(local[1])
        p.point.z = float(local[2])
        out = buf.transform(p, "base_link",
                            timeout=rclpy.duration.Duration(seconds=2))
        return (out.point.x, out.point.y, out.point.z)

    spin(3.0)

    world = poses()
    robot = world.get("tiago_pro")
    book_name = next((k for k in world if k.startswith("book_")), None)
    if robot is None or book_name is None:
        print("no robot or no book in the world")
        return 1
    book = world[book_name]

    # The book in base_link: undo the base's world pose.
    yaw = math.atan2(2.0 * (robot["qw"] * robot["qz"] + robot["qx"] * robot["qy"]),
                     1.0 - 2.0 * (robot["qy"] ** 2 + robot["qz"] ** 2))
    dx, dy = book["px"] - robot["px"], book["py"] - robot["py"]
    bx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
    by = dx * math.sin(-yaw) + dy * math.cos(-yaw)
    bz = book["pz"] - robot["pz"]

    # Perception reports the FACE of the book, not its centre.
    face_x = bx - BOOK_DEPTH / 2.0
    row = ROW

    # Put the book where this bench always puts it, then re-read the truth.
    target = (PLACE_X, 0.0, PLACE_Z.get(ROW, 1.346))
    wx = robot["px"] + target[0] * math.cos(yaw) - target[1] * math.sin(yaw)
    wy = robot["py"] + target[0] * math.sin(yaw) + target[1] * math.cos(yaw)
    wz = robot["pz"] + target[2]
    print("placing %s at base_link (%.3f, %.3f, %.3f)" % (book_name, *target))

    # Move the POST under it, and let the book rest on the post.
    #
    # Holding the book in mid-air by repeated set_pose does not work and the numbers say
    # why: every call is a subprocess, so the loop runs at one or two hertz, and a book
    # falls 0.78 m in the 0.4 simulated seconds between placements. One run had the arm
    # arrive within 30 mm of its target while the book was 640 mm below and 550 mm to the
    # side, and the grasp aid duly reported it 847 mm away. The post is static and does
    # not fall, which is what the shelf board does in the arena.
    stand_top = wz - BOOK_TALL / 2.0
    gz("service", "-s", "/world/%s/set_pose" % WORLD,
       "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
       "--timeout", "800",
       "--req", 'name: "%s", position: {x: %f, y: %f, z: %f}'
                % (STAND, wx, wy, stand_top - STAND_HEIGHT / 2.0), timeout=4)
    spin(0.5)
    for _ in range(6):
        gz("service", "-s", "/world/%s/set_pose" % WORLD,
           "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
           "--timeout", "800",
           "--req", 'name: "%s", position: {x: %f, y: %f, z: %f}'
                    % (book_name, wx, wy, wz), timeout=3)
        spin(0.3)

    world = poses()
    robot = world.get("tiago_pro", robot)
    book = world.get(book_name, book)
    yaw = math.atan2(2.0 * (robot["qw"] * robot["qz"] + robot["qx"] * robot["qy"]),
                     1.0 - 2.0 * (robot["qy"] ** 2 + robot["qz"] ** 2))
    dx, dy = book["px"] - robot["px"], book["py"] - robot["py"]
    bx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
    by = dx * math.sin(-yaw) + dy * math.cos(-yaw)
    bz = book["pz"] - robot["pz"]
    face_x = bx - BOOK_DEPTH / 2.0

    print("book %s" % book_name)
    print("  centre in base_link  (%.3f, %.3f, %.3f)" % (bx, by, bz))
    print("  face at x=%.3f, feeding row %d" % (face_x, row))

    print("\nfeeding the controller for %.0f simulated seconds" % SECONDS)
    # Once the controller says it has the book, stop propping it up, so a real hold can
    # be told apart from this script holding it.
    held = [False]
    ticks = [0]
    node.create_subscription(
        String, "/grasp_fix/holding",
        lambda m: held.__setitem__(0, bool(m.data)), 10)

    end = None
    while rclpy.ok():
        if end is None:
            end = sim_now() + SECONDS
        if sim_now() > end:
            break
        pub_row.publish(Int32(data=int(row)))
        point = PointStamped()
        point.header.frame_id = "base_link"
        point.header.stamp = node.get_clock().now().to_msg()
        point.point.x, point.point.y, point.point.z = face_x, by, bz
        pub_point.publish(point)
        pub_yaw.publish(Float32(data=0.0))
        # Snapshot on the tick after the clamp is announced, from this thread, where the
        # transforms and the pose feed are already being read.
        if at_clamp.pop("pending", False):
            try:
                pa, pb = at(TIP_L, PAD_LOCAL), at(TIP_R, PAD_LOCAL)
                at_clamp["pad"] = tuple((pa[i] + pb[i]) / 2.0 for i in range(3))
                at_clamp["grasp"] = at(GRASP, (0.0, 0.0, 0.0))
            except Exception as exc:  # noqa: BLE001
                print("  could not read the arrival (%s)" % exc)
        # The post carries the book now, so this only steps in if it has been knocked
        # off -- and never once the gripper is holding it, or a real pick would be
        # fought by this script.
        ticks[0] += 1
        if not held[0] and ticks[0] % 15 == 0:
            where = poses().get(book_name)
            if where is not None:
                slipped = math.dist((where["px"], where["py"], where["pz"]),
                                    (wx, wy, wz))
                if slipped > 0.05:
                    print("  the book slipped %.0f mm; putting it back" % (slipped * 1000))
                    gz("service", "-s", "/world/%s/set_pose" % WORLD,
                       "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                       "--timeout", "600",
                       "--req", 'name: "%s", position: {x: %f, y: %f, z: %f}'
                                % (book_name, wx, wy, wz), timeout=3)
        spin(0.2)
        if states and states[-1] in ("done", "failed", "FAILED", "DONE"):
            print("  controller reached %s" % states[-1])
            break

    print("\nstates seen: %s" % (" -> ".join(states) if states else "none"))

    if "pad" in at_clamp:
        pad = at_clamp["pad"]
        print("\nAT THE CLAMP, in base_link:")
        print("  pad middle      (%+.3f, %+.3f, %+.3f)" % pad)
        print("  grasping frame  (%+.3f, %+.3f, %+.3f)" % at_clamp["grasp"])
        print("  book fed at     (%+.3f, %+.3f, %+.3f)" % (bx, by, bz))
        print("  MISS: %+.0f mm depth, %+.0f mm sideways, %+.0f mm in HEIGHT"
              % ((pad[0] - bx) * 1000, (pad[1] - by) * 1000, (pad[2] - bz) * 1000))
        if abs(pad[2] - bz) > 0.15:
            print("  shelf rows are 330 mm apart, so that is %.1f rows out"
                  % ((pad[2] - bz) / 0.33))
    else:
        print("\nthe clamp was never reached, so there is no arrival to report")

    print("\nwhere the gripper finished (AFTER stowing -- expect a folded arm):")
    try:
        g = at(GRASP, (0.0, 0.0, 0.0))
        a, b = at(TIP_L, PAD_LOCAL), at(TIP_R, PAD_LOCAL)
        mid = tuple((a[i] + b[i]) / 2.0 for i in range(3))
        print("  grasping frame x = %+.3f, pad middle x = %+.3f" % (g[0], mid[0]))
        print("  the book face is at x = %+.3f" % face_x)
        print("  pads are %+.0f mm past the face (the book is %.0f mm deep, so the"
              % ((mid[0] - face_x) * 1000, BOOK_DEPTH * 1000))
        print("  middle of it is %+.0f mm)" % (BOOK_DEPTH / 2.0 * 1000))
        if mid[0] < face_x:
            print("  SHORT: the pads stopped in front of the book.")
        elif mid[0] > face_x + BOOK_DEPTH:
            print("  PAST: the pads went out the back of the book.")
        else:
            print("  the pads are inside the book's depth.")
        print("  sideways error %+.0f mm, height error %+.0f mm"
              % ((mid[1] - by) * 1000, (mid[2] - bz) * 1000))
    except Exception as exc:  # noqa: BLE001
        print("  could not read the gripper (%s)" % exc)

    after = poses().get(book_name)
    if after:
        moved = math.dist((book["px"], book["py"], book["pz"]),
                          (after["px"], after["py"], after["pz"]))
        print("\n  the book moved %.0f mm during all this" % (moved * 1000))

    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
