#!/usr/bin/env python3
"""Where does TF say the head camera is, and where is it really?

    tools/in-sim camframe.py [tilt]

Every book fix comes back 115 mm too high in base_link, at every row, every head tilt and
every range -- a constant metric offset, which is the signature of the depth image being
rendered from somewhere other than the frame it is stamped with. This prints the frames
in the camera chain so the 115 mm can be matched against an actual joint.
"""
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TILT = float(sys.argv[1]) if len(sys.argv) > 1 else -0.35
FRAMES = [
    "head_1_link",
    "head_2_link",
    "head_front_camera_bottom_screw_frame",
    "head_front_camera_link",
    "head_front_camera_depth_frame",
    "head_front_camera_depth_optical_frame",
    "head_front_camera_color_optical_frame",
    "torso_lift_link",
]


def main():
    rclpy.init()
    node = rclpy.create_node("camframe")
    head = node.create_publisher(JointTrajectory, "/head_controller/joint_trajectory", 10)
    buf = Buffer()
    TransformListener(buf, node)

    traj = JointTrajectory()
    traj.joint_names = ["head_1_joint", "head_2_joint"]
    point = JointTrajectoryPoint()
    point.positions = [0.0, TILT]
    point.time_from_start = Duration(sec=2)
    traj.points = [point]
    for _ in range(6):
        head.publish(traj)
        rclpy.spin_once(node, timeout_sec=0.2)
    deadline = time.time() + 6.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("head tilt %+0.2f rad, frames in base_link:" % TILT)
    print("  %-42s %8s %8s %8s" % ("frame", "x", "y", "z"))
    print("  " + "-" * 70)
    for frame in FRAMES:
        try:
            t = buf.lookup_transform("base_link", frame, rclpy.time.Time()).transform
        except Exception as exc:  # noqa: BLE001
            print("  %-42s  (%s)" % (frame, str(exc)[:40]))
            continue
        print("  %-42s %+8.3f %+8.3f %+8.3f"
              % (frame, t.translation.x, t.translation.y, t.translation.z))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
