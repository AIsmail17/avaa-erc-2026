#!/usr/bin/env python3
"""Can this gripper hold anything at all? Put a book in the jaws and let go.

    tools/in-sim jawtest.py [--lift 0.10]

Every grasp attempt so far has confounded two questions: did the arm reach the right
place, and can the gripper hold a book once it is there. This answers the second on its
own and in about a minute, with no planning, no IK and no approach.

    1. open the jaws
    2. teleport the book to the midpoint between the fingertips, thin side across the
       closing axis, and keep it there
    3. close the jaws
    4. stop holding it -- let go of the puppet strings
    5. raise the torso 100 mm, which moves the whole arm vertically with no plan
    6. report where the book went

A book that stays between the pads and rises with the torso is held. A book that drops
to the floor the moment step 4 stops teleporting it is not, however tightly the jaws
report closing.

The orientation is taken from the fingertips rather than assumed. The book's thin 30 mm
side is its own local y in every case, so the placement rotation is built to put local y
along the line between the two fingertip links: that works whatever pose the arm is in,
and the arm is not moved at all here.
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
import tf2_geometry_msgs  # noqa: F401  (registers the PointStamped transform)

# The finger links each carry their own contact sensor -- generate_urdf.py injects one
# per link -- and they publish to
#   /world/erc_world/model/tiago_pro/link/<link>/sensor/<link>_contact/contact
# not to a single /contacts topic. An earlier version of this test subscribed to /contacts
# through the bridge, received nothing, and printed "0 contacts", which read as "the
# fingers never touched the book" when it only meant "nobody publishes on that topic".
PAD_LINKS = (
    "gripper_left_fingertip_left_link",
    "gripper_left_fingertip_right_link",
    "gripper_left_inner_finger_left_link",
    "gripper_left_inner_finger_right_link",
)


def watch_pads():
    """Start one gz subscriber per pad link. Returns (procs, paths)."""
    procs, paths = [], {}
    for link in PAD_LINKS:
        topic = ("/world/%s/model/tiago_pro/link/%s/sensor/%s_contact/contact"
                 % (WORLD, link, link))
        path = "/tmp/jawtest_%s.txt" % link
        try:
            handle = io.open(path, "w")
            procs.append((subprocess.Popen(["gz", "topic", "-e", "-t", topic],
                                           stdout=handle, stderr=subprocess.DEVNULL),
                          handle))
            paths[link] = path
        except Exception:  # noqa: BLE001
            pass
    return procs, paths


def read_pads(procs, paths):
    for proc, handle in procs:
        try:
            proc.terminate()
            proc.wait(timeout=3)
        except Exception:  # noqa: BLE001
            pass
        try:
            handle.close()
        except Exception:  # noqa: BLE001
            pass
    out = {}
    for link, path in paths.items():
        try:
            text = io.open(path, encoding="utf-8", errors="replace").read()
        except Exception:  # noqa: BLE001
            text = ""
        out[link] = (text.count("collision2"), text.count("book"))
    return out

WORLD = "erc_world"
FINGER = "gripper_left_finger_joint"
TORSO = "torso_lift_joint"
TIP_L = "gripper_left_fingertip_left_link"
TIP_R = "gripper_left_fingertip_right_link"

# Where the book goes.
#
# This used to be the midpoint between the two fingertip LINK ORIGINS, and that is not
# where the pads are. The fingertip collision is the full fingertip.stl, whose bounding
# box is centred 18.7 mm off the link origin in local y; measured in the world, a contact
# on that pad lands at z=0.176 while its link origin sits at z=0.119. Placing the book at
# the origin midpoint put it about 58 mm clear of the jaws, and every "the jaws closed to
# 27.4 mm around a 30.0 mm book" reading was taken with the book beside the gripper rather
# than in it. tools/sensorcheck.sh is what caught it: driven onto the pad deliberately,
# the same sensor produced 323347 lines naming the book.
#
# gripper_left_grasping_link is the frame the robot description provides for the purpose,
# and it is only 8 mm from the true middle of the pads -- but the book still felt nothing
# there. tools/canitgrip.py found what does work: the middle of the two PAD centres, with
# the book left in the orientation it spawned in. Shut jaws around a book placed that way
# produced 11553 contacts across three pads, so contact is available and every earlier
# "the pads felt nothing" was this test aiming badly, not the gripper failing to close.
GRASP_LINK = "gripper_left_grasping_link"

# fingertip.stl bounding-box centre in link coordinates, from tools/stlbox.py.
PAD_LOCAL = (0.0042, 0.0187, 0.0000)
GRIPPER_TOPIC = "/gripper_left_controller_raw/joint_trajectory"
TORSO_TOPIC = "/torso_controller/joint_trajectory"

OPEN, SHUT = 0.060, 0.000
BOOK_THIN = 0.03

LIFT = 0.10
for i, a in enumerate(sys.argv):
    if a == "--lift" and i + 1 < len(sys.argv):
        LIFT = float(sys.argv[i + 1])


def gz(*args, timeout=10):
    try:
        return subprocess.run(["gz", *args], capture_output=True, text=True,
                              timeout=timeout).stdout
    except Exception:  # noqa: BLE001
        return ""


def poses():
    """Every model's world pose, position and orientation kept apart."""
    raw = gz("topic", "-e", "-t", "/world/%s/dynamic_pose/info" % WORLD, "-n", "1",
             timeout=15)
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
            key, _, value = line.partition(":")
            key = key.strip()
            if key in ("x", "y", "z", "w") and section + key not in fields:
                try:
                    fields[section + key] = float(value)
                except ValueError:
                    pass
    if name and "px" in fields:
        out[name] = dict(fields)
    return out


def quat_from_columns(i, j, k):
    """A quaternion from three orthonormal column vectors."""
    m = [[i[0], j[0], k[0]], [i[1], j[1], k[1]], [i[2], j[2], k[2]]]
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
                (m[1][0] - m[0][1]) / s, 0.25 * s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        return (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s,
                (m[2][1] - m[1][2]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        return ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s,
                (m[0][2] - m[2][0]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
    return ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s,
            (m[1][0] - m[0][1]) / s)


def main():
    rclpy.init()
    node = rclpy.create_node("jawtest")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    state = {}
    node.create_subscription(
        JointState, "/joint_states",
        lambda m: state.update(dict(zip(m.name, m.position))), 10)
    grip = node.create_publisher(JointTrajectory, GRIPPER_TOPIC, 10)

    # Whether the pads ever TOUCH the book is a different question from whether they end
    # up around it, and the two have been confounded all week. A closed span of 27.1 mm
    # around a 30 mm book has two readings that call for opposite fixes: the fingers
    # contacted it and squeezed past, or they never contacted it at all.
    pad_procs, pad_paths = watch_pads()
    torso = node.create_publisher(JointTrajectory, TORSO_TOPIC, 10)
    buf = Buffer()
    listener = TransformListener(buf, node)
    _ = listener

    def sim_now():
        return node.get_clock().now().nanoseconds * 1e-9

    def spin(seconds, each=None):
        while rclpy.ok() and sim_now() == 0.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        end = sim_now() + seconds
        while rclpy.ok() and sim_now() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if each:
                each()

    def send(pub, joint, value, secs=3):
        traj = JointTrajectory()
        traj.joint_names = [joint]
        point = JointTrajectoryPoint()
        point.positions = [float(value)]
        point.velocities = [0.0]
        point.time_from_start = Duration(sec=secs, nanosec=0)
        traj.points = [point]
        pub.publish(traj)

    spin(3.0)
    if FINGER not in state:
        print("no /joint_states -- are the controllers up?")
        return 1

    world = poses()
    book = next((k for k in world if k.startswith("book_")), None)
    if book is None:
        print("no book in the world")
        return 1
    print("book: %s" % book)

    def tips():
        try:
            a = buf.lookup_transform("world", TIP_L, rclpy.time.Time()).transform.translation
            b = buf.lookup_transform("world", TIP_R, rclpy.time.Time()).transform.translation
        except Exception:  # noqa: BLE001
            # No world frame published on the bench: compose through the robot's true
            # pose instead, exactly as sim_grasp_fix does.
            robot = poses().get("tiago_pro")
            if robot is None:
                return None, None
            yaw = math.atan2(
                2.0 * (robot["qw"] * robot["qz"] + robot["qx"] * robot["qy"]),
                1.0 - 2.0 * (robot["qy"] ** 2 + robot["qz"] ** 2))
            out = []
            for link in (TIP_L, TIP_R):
                t = buf.lookup_transform(
                    "base_link", link, rclpy.time.Time()).transform.translation
                out.append((robot["px"] + t.x * math.cos(yaw) - t.y * math.sin(yaw),
                            robot["py"] + t.x * math.sin(yaw) + t.y * math.cos(yaw),
                            robot["pz"] + t.z))
            return out[0], out[1]
        return (a.x, a.y, a.z), (b.x, b.y, b.z)

    def pad_in_base(link):
        """The centre of a pad's collision mesh, in base_link."""
        point = PointStamped()
        point.header.frame_id = link
        point.point.x = float(PAD_LOCAL[0])
        point.point.y = float(PAD_LOCAL[1])
        point.point.z = float(PAD_LOCAL[2])
        out = buf.transform(point, "base_link",
                            timeout=rclpy.duration.Duration(seconds=2))
        return (out.point.x, out.point.y, out.point.z)

    # Where the pads will be once they are SHUT, in base_link. Filled in by calibrate().
    #
    # The pads ride a four-bar, so closing does not just narrow the gap -- it carries the
    # middle of the gap somewhere else. Placing the book at the middle of the OPEN jaws
    # and then closing them swings the pads past it entirely, which is why every run
    # reported the pads feeling nothing while tools/canitgrip.py, which shuts the jaws
    # first and only then puts the book in, got 11553 contacts across three pads.
    #
    # Measured on one bench: the open middle sat at (1.221, 0.482, 0.115) in the world and
    # the shut middle at (1.105, 0.554, 0.103).
    shut_mid = {}

    def pads_mid_in_base():
        a, b = pad_in_base(TIP_L), pad_in_base(TIP_R)
        return tuple((a[i] + b[i]) / 2.0 for i in range(3))

    def calibrate():
        """Shut the jaws, remember where the pads meet, then open again."""
        send(grip, FINGER, SHUT)
        spin(6.0)
        try:
            shut_mid["p"] = pads_mid_in_base()
            shut_mid["gap"] = math.dist(pad_in_base(TIP_L), pad_in_base(TIP_R))
        except Exception as exc:  # noqa: BLE001
            print("   could not read the shut pads (%s)" % exc)
            return False
        print("   shut, the pad centres are %.1f mm apart at base_link "
              "(%+.3f, %+.3f, %+.3f)"
              % (shut_mid["gap"] * 1000, *shut_mid["p"]))
        return True

    def grasp_point():
        """Where the SHUT pads will be, in world coordinates."""
        robot = poses().get("tiago_pro")
        if robot is None or "p" not in shut_mid:
            return None
        yaw = math.atan2(
            2.0 * (robot["qw"] * robot["qz"] + robot["qx"] * robot["qy"]),
            1.0 - 2.0 * (robot["qy"] ** 2 + robot["qz"] ** 2))
        mid = shut_mid["p"]
        return (robot["px"] + mid[0] * math.cos(yaw) - mid[1] * math.sin(yaw),
                robot["py"] + mid[0] * math.sin(yaw) + mid[1] * math.cos(yaw),
                robot["pz"] + mid[2])

    def place():
        """Put the book at the grasp point, thin side across the closing axis."""
        left, right = tips()
        if left is None:
            return None
        mid = grasp_point()
        if mid is None:
            mid = tuple((left[n] + right[n]) / 2.0 for n in range(3))
        axis = [right[n] - left[n] for n in range(3)]
        span = math.sqrt(sum(c * c for c in axis))
        if span < 1e-6:
            return None
        j = [c / span for c in axis]
        _ = j  # kept for the span report; the book is no longer re-oriented
        # Any direction across the closing axis will do for the other two; taking it
        # from world up keeps the book standing rather than lying on its side.
        up = [0.0, 0.0, 1.0]
        i = [j[1] * up[2] - j[2] * up[1],
             j[2] * up[0] - j[0] * up[2],
             j[0] * up[1] - j[1] * up[0]]
        n = math.sqrt(sum(c * c for c in i))
        if n < 1e-6:
            i, n = [1.0, 0.0, 0.0], 1.0
        i = [c / n for c in i]
        k = [i[1] * j[2] - i[2] * j[1],
             i[2] * j[0] - i[0] * j[2],
             i[0] * j[1] - i[1] * j[0]]
        gz("service", "-s", "/world/%s/set_pose" % WORLD,
           "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
           "--timeout", "600",
           "--req", 'name: "%s", position: {x: %f, y: %f, z: %f}'
                    % (book, mid[0], mid[1], mid[2]),
           timeout=3)
        return mid, span

    print("\n1. finding where the jaws actually meet, then opening them")
    if not calibrate():
        return 1
    send(grip, FINGER, OPEN)
    spin(5.0)
    print("   opened, finger at %.4f" % state.get(FINGER, float("nan")))

    print("\n2. placing the book between the pads and holding it there")
    got = place()
    if got is None:
        print("   could not find the fingertips")
        return 1
    mid, span = got
    print("   link origins %.1f mm apart, book %.1f mm thick, placed where the SHUT "
          "pads will be (%.3f, %.3f, %.3f)"
          % (span * 1000, BOOK_THIN * 1000, mid[0], mid[1], mid[2]))

    print("\n3. closing the jaws while still holding the book in place")
    send(grip, FINGER, SHUT)
    # Keep re-placing the book for the WHOLE close, not just after it.
    #
    # This used to place the book once and then spin(5.0) while the jaws shut. The book
    # is held in mid-air by nothing but these set_pose calls, so five unattended seconds
    # is five seconds of free fall: it was metres away before the pads arrived, and the
    # test reported "the pads felt nothing" every single time. tools/canitgrip.py teleports
    # every 0.3 s without a gap and gets 11553 contacts.
    for _ in range(28):
        place()
        spin(0.3)
    print("   finger at %.4f" % state.get(FINGER, float("nan")))
    left, right = tips()
    closed_span = math.dist(left, right)
    print("   pads now %.1f mm apart" % (closed_span * 1000))
    pads = read_pads(pad_procs, pad_paths)
    total = sum(c for c, _ in pads.values())
    books = sum(b for _, b in pads.values())
    print("   what the pads felt while closing:")
    for link, (contacts, book_hits) in sorted(pads.items()):
        print("     %-38s %4d contacts, %3d naming a book"
              % (link.replace("gripper_left_", ""), contacts, book_hits))
    if total == 0:
        print("     nothing at all -- the pads closed through empty space")
    elif books == 0:
        print("     contacts, but none against the book: the jaws missed it")
    else:
        print("     %d of %d contacts were against the book" % (books, total))

    print("\n4. letting go -- nothing is holding the book now but the gripper")
    spin(4.0)
    here = poses().get(book)
    if here is None:
        print("   the book vanished from the pose feed")
        return 1
    drop = mid[2] - here["pz"]
    print("   book at (%.3f, %.3f, %.3f), %.0f mm below where it was placed"
          % (here["px"], here["py"], here["pz"], drop * 1000))
    if drop > 0.05:
        print("\n   NOT HELD -- it fell straight through the closed jaws.")
        return 2

    print("\n5. raising the torso %.0f mm" % (LIFT * 1000))
    start_torso = state.get(TORSO, 0.0)
    send(torso, TORSO, min(start_torso + LIFT, 0.35), secs=6)
    spin(9.0)
    after = poses().get(book)
    rose = after["pz"] - here["pz"]
    moved_torso = state.get(TORSO, 0.0) - start_torso
    print("   torso moved %.0f mm, book moved %+.0f mm"
          % (moved_torso * 1000, rose * 1000))

    if moved_torso > 0.02 and rose > moved_torso * 0.6:
        print("\n   HELD -- the book came up with the arm.")
        return 0
    print("\n   NOT HELD -- the arm rose and the book stayed behind.")
    return 2


sys.exit(main())
