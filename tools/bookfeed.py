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
    node.create_subscription(
        String, "/avaa/grasp/state",
        lambda m: (states.append(m.data)
                   if not states or states[-1] != m.data else None), 10)

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
        # Hold the book up.
        #
        # In the arena a shelf board carries it. Here it hangs in mid-air, and a book that
        # falls during the reach makes every arrival number meaningless -- one run
        # reported a 193 mm height error against a book that was on the floor by then.
        # Re-placing it each tick is what the shelf does.
        if not held[0]:
            gz("service", "-s", "/world/%s/set_pose" % WORLD,
               "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
               "--timeout", "400",
               "--req", 'name: "%s", position: {x: %f, y: %f, z: %f}'
                        % (book_name, wx, wy, wz), timeout=2)
        spin(0.2)
        if states and states[-1] in ("done", "failed", "FAILED", "DONE"):
            print("  controller reached %s" % states[-1])
            break

    print("\nstates seen: %s" % (" -> ".join(states) if states else "none"))

    print("\nwhere the gripper finished:")
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
