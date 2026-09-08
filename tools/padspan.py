#!/usr/bin/env python3
"""How far apart are the PADS, as the finger joint closes?

    tools/in-sim padspan.py

MANIPULATION.md carries a span table -- 0.000 gives 28.0 mm, 0.040 gives 60.5 mm -- and
concluded that a 30 mm book fits and can be clamped. That table is the distance between
the fingertip LINK ORIGINS, and the pads are not at their link origins: fingertip.stl's
bounding box is centred 18.7 mm away in local y, and the offset points inward, so the
origins and the pads do not even move the same distance.

Measured once at rest, the origins were 29 mm apart while the pad centres were 37.9 mm
apart. If that holds as the jaws close, the closed gap is wider than the table says and a
30 mm book cannot be gripped at all -- which would explain why the pads' contact sensors
stay silent through every close.

This sweeps the finger joint and reports both, so the two can be compared directly.
"""
import math
import sys

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_geometry_msgs  # noqa: F401

FINGER = "gripper_left_finger_joint"
TOPIC = "/gripper_left_controller_raw/joint_trajectory"
TIP_L = "gripper_left_fingertip_left_link"
TIP_R = "gripper_left_fingertip_right_link"
PAD_LOCAL = (0.0042, 0.0187, 0.0000)     # fingertip.stl bounding-box centre, tools/stlbox.py
BOOK_THIN = 0.030

STEPS = (0.000, 0.010, 0.020, 0.030, 0.040, 0.050, 0.060, 0.070)


def main():
    rclpy.init()
    node = rclpy.create_node("padspan")
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

    def send(value):
        traj = JointTrajectory()
        traj.joint_names = [FINGER]
        point = JointTrajectoryPoint()
        point.positions = [float(value)]
        point.velocities = [0.0]
        point.time_from_start = Duration(sec=3, nanosec=0)
        traj.points = [point]
        pub.publish(traj)

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
    print("  commanded    reached    origins apart    PADS apart    a 30 mm book")
    print("  " + "-" * 68)
    for target in STEPS:
        send(target)
        spin(4.5)
        try:
            origins = math.dist(at(TIP_L, (0, 0, 0)), at(TIP_R, (0, 0, 0)))
            pads = math.dist(at(TIP_L, PAD_LOCAL), at(TIP_R, PAD_LOCAL))
        except Exception as exc:  # noqa: BLE001
            print("  %.3f        transform failed (%s)" % (target, exc))
            continue
        verdict = "fits" if pads > BOOK_THIN else "GRIPPED"
        print("  %.3f        %+.4f    %6.1f mm       %6.1f mm      %s"
              % (target, state.get(FINGER, float("nan")),
                 origins * 1000, pads * 1000, verdict))

    print("\n  'GRIPPED' means the pad centres are closer together than the book is")
    print("  thick, which is the only way a 30 mm book gets squeezed.")

    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
