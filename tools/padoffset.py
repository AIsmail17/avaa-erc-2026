#!/usr/bin/env python3
"""How far behind the grasping frame are the pads, along the approach axis?

    tools/in-sim padoffset.py

grasp_node sets grasp_depth_m = 0.11 to drive the GRASPING FRAME past the book face, and
its comment justifies that with "they sit 29.7 mm behind that frame, measured from TF".
Everything about the reach depth rests on that one number, and tools/jawcentre.py measured
8 mm, not 29.7. Both cannot be right.

The two are not measuring the same thing. 29.7 mm is presumably between link ORIGINS; the
pads are 19 mm off their origins because the fingertip collision is its own visual mesh.
And the offset changes with the jaw opening, because the pads ride a four-bar -- so the
number that matters is the one at the opening the reach actually uses, not at rest.

This reports the offset along the approach axis (base_link +x) at several openings.
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
GRASP = "gripper_left_grasping_link"
PAD_LOCAL = (0.0042, 0.0187, 0.0000)

# 0.068 is what grasp_node opens to for the reach; 0.060 and 0.040 bracket it.
OPENINGS = (0.000, 0.040, 0.060, 0.068)


def main():
    rclpy.init()
    node = rclpy.create_node("padoffset")
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
        p = JointTrajectoryPoint()
        p.positions = [float(value)]
        p.velocities = [0.0]
        p.time_from_start = Duration(sec=3, nanosec=0)
        traj.points = [p]
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
    print("  all distances in base_link; +x is the approach direction\n")
    print("  opening   reached   grasping frame x   pad middle x   pads BEHIND by")
    print("  " + "-" * 70)
    for target in OPENINGS:
        send(target)
        spin(4.5)
        try:
            g = at(GRASP, (0.0, 0.0, 0.0))
            a, b = at(TIP_L, PAD_LOCAL), at(TIP_R, PAD_LOCAL)
        except Exception as exc:  # noqa: BLE001
            print("   %.3f     transform failed (%s)" % (target, exc))
            continue
        mid = tuple((a[i] + b[i]) / 2.0 for i in range(3))
        behind_x = g[0] - mid[0]
        print("   %.3f    %+.4f    %+.4f          %+.4f       %+6.1f mm"
              % (target, state.get(FINGER, float("nan")), g[0], mid[0],
                 behind_x * 1000))

    print("\n  grasp_node assumes 29.7 mm and adds it into grasp_depth_m = 0.11.")
    print("  If the real offset is smaller, the grasping frame is driven too deep and")
    print("  the pads end up past the middle of the book; if larger, they stop short.")

    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
