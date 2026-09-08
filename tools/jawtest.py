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
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

WORLD = "erc_world"
FINGER = "gripper_left_finger_joint"
TORSO = "torso_lift_joint"
TIP_L = "gripper_left_fingertip_left_link"
TIP_R = "gripper_left_fingertip_right_link"
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

    def place():
        """Put the book between the pads, thin side across the closing axis."""
        left, right = tips()
        if left is None:
            return None
        mid = tuple((left[n] + right[n]) / 2.0 for n in range(3))
        axis = [right[n] - left[n] for n in range(3)]
        span = math.sqrt(sum(c * c for c in axis))
        if span < 1e-6:
            return None
        j = [c / span for c in axis]
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
        qx, qy, qz, qw = quat_from_columns(i, j, k)
        gz("service", "-s", "/world/%s/set_pose" % WORLD,
           "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
           "--timeout", "600",
           "--req", 'name: "%s", position: {x: %f, y: %f, z: %f}, '
                    'orientation: {x: %f, y: %f, z: %f, w: %f}'
                    % (book, mid[0], mid[1], mid[2], qx, qy, qz, qw),
           timeout=3)
        return mid, span

    print("\n1. opening the jaws to %.3f" % OPEN)
    send(grip, FINGER, OPEN)
    spin(4.0)
    print("   finger at %.4f" % state.get(FINGER, float("nan")))

    print("\n2. placing the book between the pads and holding it there")
    got = place()
    if got is None:
        print("   could not find the fingertips")
        return 1
    mid, span = got
    print("   pads %.1f mm apart, book %.1f mm thick, midpoint (%.3f, %.3f, %.3f)"
          % (span * 1000, BOOK_THIN * 1000, mid[0], mid[1], mid[2]))

    print("\n3. closing the jaws while still holding the book in place")
    send(grip, FINGER, SHUT)
    spin(5.0, each=lambda: None)
    for _ in range(10):
        place()
        spin(0.3)
    print("   finger at %.4f" % state.get(FINGER, float("nan")))
    left, right = tips()
    closed_span = math.dist(left, right)
    print("   pads now %.1f mm apart" % (closed_span * 1000))

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
