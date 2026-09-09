#!/usr/bin/env python3
"""How wrong is a book's measured HEIGHT, and is the error the same at every row?

    tools/in-sim bookheight.py [standoff] [column] [tilt ...]

The grasp does not use the measured height at all. It uses the row, looks that row up in
a table, and reaches there -- so a row that is one out puts the hand 330 mm from the
book and nothing downstream can notice. The row itself comes from grouping books under
column markers in the image, which is exactly the fragile step.

Perception does hold a height cross-check, but it only overrules the markers when the two
disagree by two rows or more, on the grounds that one row is "within what the height bias
can explain". That bias is a single constant, DEPTH_HEIGHT_BIAS = 0.152 m, and this asks
whether one constant is the right shape for it. A bias that comes from geometry -- the
camera looking DOWN at the lower rows and UP at the top one -- cannot have one sign, and
a bias that comes from pixels cannot have one value in metres at every range.

Nothing here goes through the perception node. It teleports the base, aims the head, runs
the same detector and deprojection on one frame, and compares every book it finds against
Gazebo ground truth. Two estimators are reported for each: the bounding box CENTRE, which
is what the pipeline uses, and the bounding box TOP EDGE less half a book, which cannot
be dragged upward by the top face of the book coming into view.
"""
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       QoSHistoryPolicy)

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.vision import book_detector as bd      # noqa: E402
from avaa_solution.vision import depth_locator as dl      # noqa: E402

STANDOFF = float(sys.argv[1]) if len(sys.argv) > 1 else 1.50
COLUMN = int(sys.argv[2]) if len(sys.argv) > 2 else 3
TILTS = [float(a) for a in sys.argv[3:]] or [-0.35]

BASE_LINK_Z = 0.0762
BOOK_HEIGHT = 0.25          # erc_book.sdf box 0.25 x 0.03 x 0.16, pitched 90 deg
BOOK_DEPTH = 0.16           # so the visible face sits half of this in front of centre
ROW_HEIGHTS_BASE = [1.391, 1.061, 0.731, 0.401]
SHOULDER_OFFSET_Y = 0.159
TOPIC_RGB = "/head_front_camera/head_front_camera/color/image_raw"
TOPIC_DEPTH = "/head_front_camera/head_front_camera/depth/image_rect_raw"
TOPIC_INFO = "/head_front_camera/head_front_camera/depth/camera_info"

SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        durability=QoSDurabilityPolicy.VOLATILE,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=1)


def gz(*args, timeout=20):
    try:
        return subprocess.run(["gz", *args], capture_output=True, text=True,
                              timeout=timeout).stdout
    except Exception:  # noqa: BLE001
        return ""


def pose(model, attempts=6):
    for _ in range(attempts):
        lines = [l.strip() for l in gz("model", "-m", model, "-p").splitlines()]
        for i, line in enumerate(lines):
            if line.startswith("[") and i + 1 < len(lines) and lines[i + 1].startswith("["):
                try:
                    return ([float(v) for v in line.strip("[]").split()],
                            [float(v) for v in lines[i + 1].strip("[]").split()])
                except ValueError:
                    return None, None
        time.sleep(0.3)
    return None, None


def all_books():
    for _ in range(8):
        names = sorted(l.strip(" -") for l in gz("model", "--list").splitlines()
                       if "book_col" in l)
        if names:
            return names
        time.sleep(0.5)
    return []


def teleport(x, y):
    request = ('name: "tiago_pro", position: {x: %f, y: %f, z: 0.0}, '
               'orientation: {x: 0, y: 0, z: 0, w: 1}' % (x, y))
    gz("service", "-s", "/world/erc_world/set_pose", "--reqtype", "gz.msgs.Pose",
       "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", request)


def to_bgr(msg):
    data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    if msg.encoding == "rgb8":
        return data[:, :, ::-1].copy()
    return data.copy()


def to_depth(msg):
    if msg.encoding == "32FC1":
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    if msg.encoding == "16UC1":
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        return raw.astype(np.float32) / 1000.0
    raise ValueError("unexpected depth encoding %s" % msg.encoding)


def main():
    names = all_books()
    if not names:
        print("no books; is the simulator up?")
        return 1
    wanted = "book_col_%d_" % COLUMN
    column_books = [n for n in names if n.startswith(wanted)]
    if not column_books:
        print("no column %d" % COLUMN)
        return 1

    truth = {}
    for name in names:
        p, _ = pose(name, attempts=3)
        if p:
            truth[name] = p
    middle = [truth[n] for n in column_books if n in truth]
    if not middle:
        print("could not read column %d back from Gazebo" % COLUMN)
        return 1
    centre_y = float(np.mean([p[1] for p in middle]))
    shelf_x = float(np.mean([p[0] for p in middle]))

    rclpy.init()
    node = rclpy.create_node("bookheight")
    frames = {"rgb": None, "depth": None,
              "intr": dl.Intrinsics(337.2096, 337.2096, 320.0, 180.0),
              "frame": "head_front_camera_depth_optical_frame"}

    node.create_subscription(Image, TOPIC_RGB,
                             lambda m: frames.__setitem__("rgb", m), SENSOR_QOS)
    node.create_subscription(Image, TOPIC_DEPTH,
                             lambda m: frames.__setitem__("depth", m), SENSOR_QOS)

    def on_info(m):
        frames["intr"] = dl.Intrinsics.from_k(m.k)
        frames["frame"] = m.header.frame_id
    node.create_subscription(CameraInfo, TOPIC_INFO, on_info, SENSOR_QOS)

    head = node.create_publisher(JointTrajectory, "/head_controller/joint_trajectory", 10)
    buf = Buffer()
    TransformListener(buf, node)

    x = shelf_x - STANDOFF
    y = centre_y - SHOULDER_OFFSET_Y
    teleport(x, y)
    print("robot at [%.3f, %.3f], column %d centred at y=%.3f, standoff %.2f m"
          % (x, y, COLUMN, centre_y, STANDOFF))

    for tilt in TILTS:
        traj = JointTrajectory()
        traj.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [0.0, float(tilt)]
        point.time_from_start = Duration(sec=2)
        traj.points = [point]
        for _ in range(6):
            head.publish(traj)
            rclpy.spin_once(node, timeout_sec=0.2)
        deadline = time.time() + 5.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        # A settled frame, not the one in flight while the head was still moving.
        frames["rgb"] = None
        frames["depth"] = None
        deadline = time.time() + 10.0
        while time.time() < deadline and (frames["rgb"] is None or frames["depth"] is None):
            rclpy.spin_once(node, timeout_sec=0.1)
        if frames["rgb"] is None or frames["depth"] is None:
            print("  tilt %+0.2f: no camera frame" % tilt)
            continue

        robot, rpy = pose("tiago_pro", attempts=3)
        try:
            tf = buf.lookup_transform("base_link", frames["frame"],
                                      rclpy.time.Time()).transform
        except Exception as exc:  # noqa: BLE001
            print("  tilt %+0.2f: no TF to %s (%s)" % (tilt, frames["frame"], exc))
            continue
        cam_z = tf.translation.z

        bgr = to_bgr(frames["rgb"])
        depth = to_depth(frames["depth"])
        books = bd.detect_books(bgr)

        print("")
        print("  tilt %+0.2f rad, camera %.3f m up in base_link, %d blob(s)"
              % (tilt, cam_z, len(books)))
        print("    book                    range   dx      dy      dz     top dz   px h")
        print("    " + "-" * 76)
        errors_centre = []
        errors_top = []
        errors_x = []
        errors_y = []
        for b in books:
            bbox = (b.x, b.y, b.w, b.h)
            d = dl.sample_depth(depth, bbox)
            if d is None:
                continue
            intr = frames["intr"]
            p_centre = dl.deproject(b.cx, b.cy, d, intr)
            p_top = dl.deproject(b.cx, float(b.y), d, intr)
            z_centre = dl.transform_point(p_centre, tf.rotation, tf.translation)
            z_top = dl.transform_point(p_top, tf.rotation, tf.translation)

            # Which book is this? Nearest true book in base_link, sideways and up.
            best = None
            best_gap = 1e9
            for name, p in truth.items():
                dx = p[0] - robot[0]
                dy = p[1] - robot[1]
                yaw = rpy[2]
                bx = dx * math.cos(yaw) + dy * math.sin(yaw)
                by = -dx * math.sin(yaw) + dy * math.cos(yaw)
                bz = p[2] - BASE_LINK_Z
                gap = math.hypot(by - z_centre[1], bz - z_centre[2])
                if gap < best_gap:
                    best = (name, bx, by, bz)
                    best_gap = gap
            if best is None or best_gap > 0.45:
                continue
            name, true_x, true_y, true_z = best
            # The fix lands on the FACE of the book; the truth is its centre, and the
            # book is 0.16 m deep. Compare like with like.
            true_face_x = true_x - BOOK_DEPTH / 2.0
            top_estimate = z_top[2] - BOOK_HEIGHT / 2.0
            errors_centre.append(z_centre[2] - true_z)
            errors_top.append(top_estimate - true_z)
            errors_x.append(z_centre[0] - true_face_x)
            errors_y.append(z_centre[1] - true_y)
            print("    %-22s %6.3f %+6.3f  %+6.3f  %+6.3f  %+6.3f  %4d"
                  % (name.replace("book_", ""), d,
                     z_centre[0] - true_face_x, z_centre[1] - true_y,
                     z_centre[2] - true_z, top_estimate - true_z, b.h))
        if errors_centre:
            print("    bias    dx %+0.3f (+-%0.3f)   dy %+0.3f (+-%0.3f)   "
                  "dz %+0.3f (+-%0.3f)   top dz %+0.3f (+-%0.3f)"
                  % (float(np.mean(errors_x)), float(np.std(errors_x)),
                     float(np.mean(errors_y)), float(np.std(errors_y)),
                     float(np.mean(errors_centre)), float(np.std(errors_centre)),
                     float(np.mean(errors_top)), float(np.std(errors_top))))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
