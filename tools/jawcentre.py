#!/usr/bin/env python3
"""Where is the gap between the pads, really?

    tools/in-sim jawcentre.py

Three different points have been used as "where the book goes" and none of them is
between the jaws:

  - the midpoint of the two fingertip LINK ORIGINS
  - gripper_left_grasping_link, which is the frame provided for the purpose
  - and, in the solution, whatever grasp_node aims at

Closing on a book at any of them produced zero contacts on the pads' own sensors, while
driving the book deliberately onto a pad produced 323347 lines of them, so the sensors
work and the placements are wrong.

The pads are meshes, and a mesh is not centred on its link origin. fingertip.stl has its
bounding box centred at (0.0042, 0.0187, 0.0000) in link coordinates, 57 x 34 x 31 mm.
This transforms that box centre through TF for both fingertips and prints the true middle
of the jaws, next to the three candidates, so the difference is a number rather than a
guess.
"""
import math
import subprocess
import sys

import rclpy
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401  (registers the PointStamped transform)

WORLD = "erc_world"
TIP_L = "gripper_left_fingertip_left_link"
TIP_R = "gripper_left_fingertip_right_link"
INNER_L = "gripper_left_inner_finger_left_link"
INNER_R = "gripper_left_inner_finger_right_link"
GRASP = "gripper_left_grasping_link"

# Bounding-box centres of the collision meshes, in link coordinates, from tools/stlbox.py.
PAD_CENTRE = {
    TIP_L: (0.0042, 0.0187, 0.0000),
    TIP_R: (0.0042, 0.0187, 0.0000),
    INNER_L: (0.0000, 0.0245, 0.0000),
    INNER_R: (0.0000, 0.0245, 0.0000),
}


def main():
    rclpy.init()
    node = rclpy.create_node("jawcentre")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    buf = Buffer()
    listener = TransformListener(buf, node)
    _ = listener
    for _ in range(90):
        rclpy.spin_once(node, timeout_sec=0.1)

    def in_base(link, local=(0.0, 0.0, 0.0)):
        point = PointStamped()
        point.header.frame_id = link
        point.point.x, point.point.y, point.point.z = local
        try:
            out = buf.transform(point, "base_link", timeout=rclpy.duration.Duration(seconds=2))
        except Exception as exc:  # noqa: BLE001
            print("  %-42s no transform (%s)" % (link, exc))
            return None
        return (out.point.x, out.point.y, out.point.z)

    print("in base_link coordinates:\n")

    origins = {}
    pads = {}
    for link in (TIP_L, TIP_R, INNER_L, INNER_R):
        o = in_base(link)
        p = in_base(link, PAD_CENTRE[link])
        if o is None or p is None:
            continue
        origins[link] = o
        pads[link] = p
        print("  %-40s origin (%+.3f, %+.3f, %+.3f)"
              % (link.replace("gripper_left_", ""), *o))
        print("  %-40s pad    (%+.3f, %+.3f, %+.3f)   %.0f mm from the origin"
              % ("", *p, math.dist(o, p) * 1000))

    grasp = in_base(GRASP)
    if grasp:
        print("\n  %-40s        (%+.3f, %+.3f, %+.3f)"
              % (GRASP.replace("gripper_left_", ""), *grasp))

    if TIP_L in pads and TIP_R in pads:
        a, b = pads[TIP_L], pads[TIP_R]
        mid = tuple((a[i] + b[i]) / 2.0 for i in range(3))
        gap = math.dist(a, b)
        print("\n  TRUE MIDDLE OF THE FINGERTIP PADS  (%+.3f, %+.3f, %+.3f)" % mid)
        print("  pad centres %.1f mm apart" % (gap * 1000))
        if TIP_L in origins and TIP_R in origins:
            omid = tuple((origins[TIP_L][i] + origins[TIP_R][i]) / 2.0 for i in range(3))
            print("  the link-origin midpoint is %.0f mm away from it"
                  % (math.dist(omid, mid) * 1000))
        if grasp:
            print("  gripper_left_grasping_link is %.0f mm away from it"
                  % (math.dist(grasp, mid) * 1000))

    if INNER_L in pads and INNER_R in pads:
        a, b = pads[INNER_L], pads[INNER_R]
        print("\n  inner-finger pad centres %.1f mm apart, middle (%+.3f, %+.3f, %+.3f)"
              % (math.dist(a, b) * 1000,
                 *[(a[i] + b[i]) / 2.0 for i in range(3)]))

    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
