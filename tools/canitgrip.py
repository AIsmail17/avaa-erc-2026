#!/usr/bin/env python3
"""Can these jaws touch a 30 mm book at all, even shut?

    tools/in-sim canitgrip.py

Every grasp test so far has opened the jaws, put a book in, and closed them, and every one
reported no contact on the pads' own sensors. tools/padspan.py says why that might be: at
the fully closed command the fingertip PAD centres are 35.5 mm apart while the book is
30.0 mm thick. The link origins are 27.1 mm apart at the same moment, which is the number
the span table in MANIPULATION.md recorded, and it is measuring the wrong thing.

This removes the closing motion from the question. Shut the jaws first, leave them shut,
and only then put the book between the pads. If a closed gripper cannot touch a book
sitting in the middle of it, no amount of force, gain, physics engine or grasp planning
will ever pick that book up, and the fix has to be geometric.
"""
import io
import math
import subprocess
import sys

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_geometry_msgs  # noqa: F401

WORLD = "erc_world"
FINGER = "gripper_left_finger_joint"
TOPIC = "/gripper_left_controller_raw/joint_trajectory"
TIP_L = "gripper_left_fingertip_left_link"
TIP_R = "gripper_left_fingertip_right_link"
PAD_LOCAL = (0.0042, 0.0187, 0.0000)
BOOK_THIN = 0.030

PADS = (TIP_L, TIP_R,
        "gripper_left_inner_finger_left_link",
        "gripper_left_inner_finger_right_link")


def gz(*args, timeout=15):
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
    node = rclpy.create_node("canitgrip")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    state = {}
    node.create_subscription(
        JointState, "/joint_states",
        lambda m: state.update(dict(zip(m.name, m.position))), 10)
    pub = node.create_publisher(JointTrajectory, TOPIC, 10)
    buf = Buffer()
    listener = TransformListener(buf, node)
    _ = listener

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

    print("1. shutting the jaws and leaving them shut")
    traj = JointTrajectory()
    traj.joint_names = [FINGER]
    point = JointTrajectoryPoint()
    point.positions = [0.0]
    point.velocities = [0.0]
    point.time_from_start = Duration(sec=3, nanosec=0)
    traj.points = [point]
    pub.publish(traj)
    spin(6.0)
    print("   finger at %+.4f" % state.get(FINGER, float("nan")))

    left_pad, right_pad = at(TIP_L, PAD_LOCAL), at(TIP_R, PAD_LOCAL)
    gap = math.dist(left_pad, right_pad)
    print("   pad centres %.1f mm apart, book %.1f mm thick -> %s"
          % (gap * 1000, BOOK_THIN * 1000,
             "the book is wider, so shutting must squeeze it" if gap < BOOK_THIN
             else "THE GAP IS WIDER THAN THE BOOK"))

    print("\n2. watching the pads")
    procs, paths = [], {}
    for link in PADS:
        topic = ("/world/%s/model/tiago_pro/link/%s/sensor/%s_contact/contact"
                 % (WORLD, link, link))
        path = "/tmp/canitgrip_%s.txt" % link
        handle = io.open(path, "w")
        procs.append((subprocess.Popen(["gz", "topic", "-e", "-t", topic],
                                       stdout=handle, stderr=subprocess.DEVNULL),
                      handle))
        paths[link] = path
    spin(2.0)

    print("\n3. putting the book in the middle of the shut jaws and holding it there")
    world = poses()
    robot = world.get("tiago_pro")
    book = next((k for k in world if k.startswith("book_")), None)
    if robot is None or book is None:
        print("   no robot or no book")
        return 1
    yaw = math.atan2(2.0 * (robot["qw"] * robot["qz"] + robot["qx"] * robot["qy"]),
                     1.0 - 2.0 * (robot["qy"] ** 2 + robot["qz"] ** 2))
    mid = tuple((left_pad[i] + right_pad[i]) / 2.0 for i in range(3))
    wx = robot["px"] + mid[0] * math.cos(yaw) - mid[1] * math.sin(yaw)
    wy = robot["py"] + mid[0] * math.sin(yaw) + mid[1] * math.cos(yaw)
    wz = robot["pz"] + mid[2]
    print("   pad middle in the world: (%.3f, %.3f, %.3f)" % (wx, wy, wz))
    for _ in range(30):
        gz("service", "-s", "/world/%s/set_pose" % WORLD,
           "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
           "--timeout", "500",
           "--req", 'name: "%s", position: {x: %f, y: %f, z: %f}' % (book, wx, wy, wz),
           timeout=3)
        spin(0.3)

    print("\n4. what the shut pads felt")
    total = 0
    for proc, handle in procs:
        try:
            proc.terminate()
            proc.wait(timeout=3)
        except Exception:  # noqa: BLE001
            pass
        handle.close()
    for link, path in paths.items():
        try:
            text = io.open(path, encoding="utf-8", errors="replace").read()
        except Exception:  # noqa: BLE001
            text = ""
        hits = text.count("book")
        total += hits
        print("   %-38s %5d contact lines naming the book"
              % (link.replace("gripper_left_", ""), hits))

    print("")
    print("5. letting go -- the jaws are shut around it, so does it stay?")
    before = poses().get(book)
    spin(5.0)
    after = poses().get(book)
    if before and after:
        fell = before["pz"] - after["pz"]
        print("   the book moved %+.0f mm down in five seconds" % (fell * 1000))
        if fell < 0.02:
            print("   HELD -- shut jaws around a book at the pad middle do hold it.")
        else:
            print("   NOT HELD -- it slipped out from between shut jaws, so the contact")
            print("   is there but the friction or the squeeze is not.")

    print()
    if total == 0:
        print("   A SHUT GRIPPER CANNOT TOUCH THE BOOK.")
        print("   The gap between the pads is wider than the book, so no force, gain,")
        print("   physics engine or plan will ever pick it up. The fix is geometric.")
        return 2
    print("   The shut jaws do touch it (%d lines), so the gap is not the whole story."
          % total)
    return 0


sys.exit(main())
