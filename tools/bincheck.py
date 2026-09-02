#!/usr/bin/env python3
"""Can the robot see the collection bin, and where does it say it is?

    python3 bincheck.py [standoff] [head_tilt_deg]

Teleports the base to the delivery pose, tilts the head down, and runs the bin detector
over live frames. Prints what vision says and what Gazebo says, so the error is measured
rather than assumed. Saves an annotated frame to /tmp/bincheck.png.

A fixture: the teleport and the ground truth are both things the solution cannot do.
"""
import math
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.vision import book_detector as bd     # noqa: E402
from avaa_solution.vision import depth_locator as dl     # noqa: E402

RGB = "/head_front_camera/head_front_camera/color/image_raw"
DEPTH = "/head_front_camera/head_front_camera/depth/image_rect_raw"
INFO = "/head_front_camera/head_front_camera/depth/camera_info"
CAMERA_FRAME = "head_front_camera_depth_optical_frame"

SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        durability=QoSDurabilityPolicy.VOLATILE,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=1)


def gz(*args, timeout=25):
    return subprocess.run(["gz", *args], capture_output=True, text=True,
                          timeout=timeout).stdout


def pose(model):
    lines = [l.strip() for l in gz("model", "-m", model, "-p").splitlines()]
    for i, line in enumerate(lines):
        if line.startswith("[") and i + 1 < len(lines) and lines[i + 1].startswith("["):
            try:
                return ([float(v) for v in line.strip("[]").split()],
                        [float(v) for v in lines[i + 1].strip("[]").split()])
            except ValueError:
                return None, None
    return None, None


class Look(Node):
    def __init__(self):
        super().__init__("bincheck")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.rgb = None
        self.depth = None
        self.intr = None
        self.create_subscription(Image, RGB, self._on_rgb, SENSOR_QOS)
        self.create_subscription(Image, DEPTH, self._on_depth, SENSOR_QOS)
        self.create_subscription(CameraInfo, INFO, self._on_info, SENSOR_QOS)
        self.head = self.create_publisher(
            JointTrajectory, "/head_controller/joint_trajectory", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_rgb(self, msg):
        self.rgb = np.frombuffer(msg.data, np.uint8).reshape(
            msg.height, msg.width, 3)[:, :, ::-1].copy()

    def _on_depth(self, msg):
        self.depth = np.frombuffer(msg.data, np.float32).reshape(
            msg.height, msg.width).copy()

    def _on_info(self, msg):
        self.intr = dl.Intrinsics.from_k(msg.k)

    def tilt(self, radians):
        traj = JointTrajectory()
        traj.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [0.0, float(radians)]
        point.time_from_start = Duration(sec=2, nanosec=0)
        traj.points = [point]
        for _ in range(6):
            self.head.publish(traj)
            time.sleep(0.2)


def main():
    standoff = float(sys.argv[1]) if len(sys.argv) > 1 else 0.65
    tilt_deg = float(sys.argv[2]) if len(sys.argv) > 2 else -35.0

    bin_p, _ = pose("erc_collection_bin")
    if bin_p is None:
        print("cannot read the bin pose from Gazebo")
        return
    # The bin model is rotated: its mesh +y is world +z, so the rim sits 0.105 above
    # the model origin and the floor 0.105 below it.
    print("bin (truth)  : [%.3f, %.3f, %.3f], rim at z=%.3f"
          % (bin_p[0], bin_p[1], bin_p[2], bin_p[2] + 0.105))

    x = bin_p[0] + standoff
    request = ('name: "tiago_pro", position: {x: %f, y: %f, z: 0.0}, '
               'orientation: {x: 0, y: 0, z: 1, w: 0}' % (x, bin_p[1]))
    gz("service", "-s", "/world/erc_world/set_pose", "--reqtype", "gz.msgs.Pose",
       "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", request)
    robot, rpy = pose("tiago_pro")
    print("robot placed : [%.3f, %.3f] yaw %.1f deg"
          % (robot[0], robot[1], math.degrees(rpy[2])))

    rclpy.init()
    node = Look()
    node.tilt(math.radians(tilt_deg))
    deadline = time.time() + 25.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.rgb is not None and node.depth is not None and node.intr is not None:
            break
    if node.rgb is None or node.intr is None:
        print("no frames arrived")
        return
    time.sleep(3.0)
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.1)

    found = bd.detect_bin(node.rgb)
    reds = [b for b in bd.detect_books(node.rgb) if b.colour == "red"]
    print("red book-shaped blobs also in view: %d" % len(reds))
    if found is None:
        cv2.imwrite("/tmp/bincheck.png", node.rgb[:, :, ::-1])
        print("BIN NOT DETECTED — frame saved to /tmp/bincheck.png")
        return
    print("bin blob     : %dx%d at (%d,%d), area %.0f, aspect %.2f, fill %.2f"
          % (found.w, found.h, found.x, found.y, found.area, found.aspect, found.fill))

    point = dl.locate(found.bbox, node.depth, node.intr)
    if point is None:
        print("no depth on the bin")
        return
    try:
        tf = node.tf_buffer.lookup_transform("base_link", CAMERA_FRAME,
                                             rclpy.time.Time())
    except Exception as exc:  # noqa: BLE001
        print("no transform: %r" % exc)
        return
    q, t = tf.transform.rotation, tf.transform.translation
    xx, yy, zz = q.x * q.x, q.y * q.y, q.z * q.z
    xy, xz, yz = q.x * q.y, q.x * q.z, q.y * q.z
    wx, wy, wz = q.w * q.x, q.w * q.y, q.w * q.z
    rot = np.array([[1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
                    [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
                    [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)]])
    in_base = rot @ np.asarray(point) + np.array([t.x, t.y, t.z])

    robot, rpy = pose("tiago_pro")
    yaw = rpy[2]
    dx, dy = bin_p[0] - robot[0], bin_p[1] - robot[1]
    truth = np.array([dx * math.cos(-yaw) - dy * math.sin(-yaw),
                      dx * math.sin(-yaw) + dy * math.cos(-yaw),
                      bin_p[2] - 0.186])
    print("bin in base  : vision [%+.3f, %+.3f, %+.3f]"
          % (in_base[0], in_base[1], in_base[2]))
    print("               truth  [%+.3f, %+.3f, %+.3f]"
          % (truth[0], truth[1], truth[2]))
    print("               error  [%+.0f, %+.0f, %+.0f] mm"
          % ((in_base[0] - truth[0]) * 1000, (in_base[1] - truth[1]) * 1000,
             (in_base[2] - truth[2]) * 1000))

    # The other way of reading it: bearing only, against a known height.
    #
    # Depth is the wrong instrument for this. The bin is 560 mm deep and 210 mm tall,
    # so a depth patch over its bounding box averages the near rim, the far rim and the
    # floor between them, and the answer lands wherever that mixture falls. Height is
    # not measured at all: the rules fix the bin on a table at a known height, exactly
    # as they fix the shelf rows, and the row heights have been treated as known since
    # the first grasp for the same reason.
    #
    # So take the camera ray through the middle of the blob and intersect it with the
    # plane of the rim. One measurement, from the part of the sensor that is accurate.
    rim_base = (bin_p[2] + 0.105) - 0.186
    ray = dl.deproject(found.cx, found.cy, 1.0, node.intr)
    direction = rot @ np.asarray(ray)
    origin = np.array([t.x, t.y, t.z])
    if abs(direction[2]) < 1e-6:
        print("the camera is looking along the rim plane; no intersection")
    else:
        scale = (rim_base - origin[2]) / direction[2]
        aimed = origin + scale * direction
        print("bin by bearing: [%+.3f, %+.3f, %+.3f] against a rim known at z=%+.3f"
              % (aimed[0], aimed[1], aimed[2], rim_base))
        print("               error  [%+.0f, %+.0f] mm in x and y"
              % ((aimed[0] - truth[0]) * 1000, (aimed[1] - truth[1]) * 1000))

    shown = node.rgb.copy()
    cv2.rectangle(shown, (found.x, found.y),
                  (found.x + found.w, found.y + found.h), (0, 255, 0), 2)
    cv2.imwrite("/tmp/bincheck.png", shown[:, :, ::-1])
    print("annotated frame saved to /tmp/bincheck.png")

    node.destroy_node()
    rclpy.shutdown()


main()
