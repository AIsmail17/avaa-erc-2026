#!/usr/bin/env python3
"""Where do the jaws stop, with a book between them and without?

    tools/in-sim clampcurve.py

This settles a contradiction that GRIPPER_CLAMP rests on. The comment above it records
that at position_proportional_gain 5 "the finger stops ON the book -- commanded to a span
of 29.2 mm it settles at 29.6 with the book undisturbed", and on that basis the clamp asks
for -0.0010, the joint's lower limit. The bench says otherwise: with the pads measured
80 mm into the book and 2 mm off its centre, the jaws still closed to 27.4 mm and the
grasp was refused for closing on nothing.

Both cannot be right, and the difference matters because the grip check fails a grasp
whose span is under 28.0 mm while the clamp commands a span of about 27.7 mm. As written,
the jaws are driven to a width the check reads as proof of failure.

So: for each candidate command, close the jaws with the book present and with it absent,
and print both. If the finger stops on the book, the two columns differ. If they do not,
the book is not stopping anything and the clamp value cannot be chosen on that basis.

The book is placed by first closing the jaws to the command under test, remembering where
the pads end up, then opening and putting the book exactly there -- the pads ride a
four-bar, so the middle of an OPEN gripper is not where a closing one arrives. The lab
book has gravity off (ERC_LAB_FLOAT), so one placement holds.
"""
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
OPEN = 0.055

COMMANDS = (0.0030, 0.0020, 0.0015, 0.0009, 0.0000, -0.0010)
AWAY = (3.0, 3.0, 0.5)          # where the book goes when it is meant to be out of the way


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
    node = rclpy.create_node("clampcurve")
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

    def send(value, secs=3):
        traj = JointTrajectory()
        traj.joint_names = [FINGER]
        p = JointTrajectoryPoint()
        p.positions = [float(value)]
        p.velocities = [0.0]
        p.time_from_start = Duration(sec=secs, nanosec=0)
        traj.points = [p]
        pub.publish(traj)

    def pad(link):
        p = PointStamped()
        p.header.frame_id = link
        p.point.x = float(PAD_LOCAL[0])
        p.point.y = float(PAD_LOCAL[1])
        p.point.z = float(PAD_LOCAL[2])
        out = buf.transform(p, "base_link",
                            timeout=rclpy.duration.Duration(seconds=2))
        return (out.point.x, out.point.y, out.point.z)

    def span():
        return math.dist(pad(TIP_L), pad(TIP_R))

    def put(name, xyz):
        gz("service", "-s", "/world/%s/set_pose" % WORLD,
           "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
           "--timeout", "800",
           "--req", 'name: "%s", position: {x: %f, y: %f, z: %f}'
                    % (name, xyz[0], xyz[1], xyz[2]), timeout=4)

    def to_world(base_point):
        robot = poses().get("tiago_pro")
        if robot is None:
            return None
        yaw = math.atan2(2.0 * (robot["qw"] * robot["qz"] + robot["qx"] * robot["qy"]),
                         1.0 - 2.0 * (robot["qy"] ** 2 + robot["qz"] ** 2))
        return (robot["px"] + base_point[0] * math.cos(yaw) - base_point[1] * math.sin(yaw),
                robot["py"] + base_point[0] * math.sin(yaw) + base_point[1] * math.cos(yaw),
                robot["pz"] + base_point[2])

    spin(3.0)
    world = poses()
    book = next((k for k in world if k.startswith("book_")), None)
    if book is None:
        print("no book in the world")
        return 1
    if FINGER not in state:
        print("no /joint_states")
        return 1
    print("book: %s, thickness %.1f mm\n" % (book, BOOK_THIN * 1000))

    # Report against the joint the finger REACHED, not the one it was asked for.
    #
    # A first run printed free-air spans that did not fall with the command -- 41.8, 40.9,
    # 35.5, 39.6 mm for successively tighter asks -- which reads as noise until you look
    # at what the joint actually did. It tracks badly: tools/padspan.py had it commanded
    # to 0.010 and settling at 0.0046, then commanded to 0.020 and settling at 0.0240.
    # Span is a clean function of where the joint IS and a poor one of where it was sent.
    print("  commanded   reached    span   |  reached    span   |  gained   book moved")
    print("             --- free air ---   |  --- with the book ---")
    print("  " + "-" * 76)
    rows = []
    for command in COMMANDS:
        # --- free air: book well out of the way ------------------------------------
        put(book, AWAY)
        spin(1.0)
        send(OPEN)
        spin(4.0)
        send(command)
        spin(8.0)
        free = span()
        free_joint = state.get(FINGER, float("nan"))
        pads_here = tuple((pad(TIP_L)[i] + pad(TIP_R)[i]) / 2.0 for i in range(3))

        # --- with the book, placed where the CLOSING jaws actually arrive -----------
        send(OPEN)
        spin(4.0)
        target = to_world(pads_here)
        if target is None:
            print("  %.4f      could not read the base pose" % command)
            continue
        # Seat the book, let it settle, seat it again, and only then close. With
        # gravity off it stays where it is put, but the OPENING jaws sweep through the
        # same space and knock it on the way -- a first run had it 1189 mm away by the
        # time the close happened, which measures nothing.
        put(book, target)
        spin(1.5)
        put(book, target)
        spin(0.5)
        before = poses().get(book)
        send(command)
        spin(8.0)
        loaded = span()
        loaded_joint = state.get(FINGER, float("nan"))
        after = poses().get(book)
        moved = (math.dist((before["px"], before["py"], before["pz"]),
                           (after["px"], after["py"], after["pz"]))
                 if before and after else float("nan"))

        print("  %+.4f    %+.4f  %6.1f   |  %+.4f  %6.1f   |  %+5.1f mm  %5.0f mm"
              % (command, free_joint, free * 1000, loaded_joint, loaded * 1000,
                 (loaded - free) * 1000, moved * 1000))
        rows.append((command, free, loaded, moved, free_joint, loaded_joint))

    print()
    # A gain in span only counts as the book stopping the jaws if the joint went to
    # about the same place both times. If the joint itself landed somewhere else, the
    # wider span is the joint's tracking, not the book.
    stopped = [r for r in rows
               if (r[2] - r[1]) > 0.001 and abs(r[5] - r[4]) < 0.0008]
    if not stopped:
        print("  The book never stopped the jaws: every command closed to the same width")
        print("  with the book there as without it. GRIPPER_CLAMP cannot be chosen on")
        print("  'the finger stops ON the book', because it does not.")
    else:
        print("  The book stops the jaws at: %s"
              % ", ".join("%+.4f" % r[0] for r in stopped))
        print("  Those are the commands where a grip is being resolved, and the grip")
        print("  check has to sit below the loaded span, not below the free-air one.")

    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
