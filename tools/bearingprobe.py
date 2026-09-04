#!/usr/bin/env python3
"""Does the steering bearing follow the robot when it turns, and how far behind is it?

    tools/in-sim bearingprobe.py [rate] [seconds]

The centring state logged a pixel error of +225, +225, +225 across samples three seconds
apart while both odom and Gazebo said the base was turning at 0.3 rad/s. That is 51
degrees of sweep on a camera that sees about 60, so the marker should have crossed the
frame and left it. Either the bearing is not describing the present, or it is not
describing the marker.

This turns the base at a steady, gentle rate and logs every bearing that arrives beside
the yaw the base has actually turned through, both stamped on the simulator's clock.
A bearing that tracks will fall at focal * tan(yaw); one that lags will fall late; one
that is frozen will not fall at all.

Yaw comes from odom, which tools/turncheck.py verified against Gazebo for commanded
turns: 0.15, 0.30 and 0.45 rad/s commanded read 0.142, 0.270 and 0.407 on odom against
0.142, 0.235 and 0.356 true. Sampling Gazebo directly would cost a second per reading
and this needs many.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

BEARING = "/avaa/perception/target_column_x"
ODOM = "/odom"
FOCAL_PX = 337.2      # from the head camera's CameraInfo
IMAGE_WIDTH = 640


def main():
    rate = float(sys.argv[1]) if len(sys.argv) > 1 else 0.10
    span = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0

    rclpy.init()
    node = rclpy.create_node("bearingprobe")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    pub = node.create_publisher(Twist, "/cmd_vel", 10)

    state = {"yaw": None, "seen": []}

    def on_odom(msg):
        q = msg.pose.pose.orientation
        state["yaw"] = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_bearing(msg):
        now = node.get_clock().now().nanoseconds * 1e-9
        state["seen"].append((now, float(msg.data), state["yaw"]))

    node.create_subscription(Odometry, ODOM, on_odom, 10)
    node.create_subscription(Float32, BEARING, on_bearing, 10)

    def sim_now():
        return node.get_clock().now().nanoseconds * 1e-9

    # Level the head first. Nothing else is running, and the approach node is what
    # normally aims it -- left where it spawns, the markers at 2.26 m are out of frame
    # and perception sees nothing at all, which is not the question being asked here.
    head = node.create_publisher(
        JointTrajectory, "/head_controller/joint_trajectory", 10)
    deadline = time.time() + 10
    while time.time() < deadline and head.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    aim = JointTrajectory()
    aim.joint_names = ["head_1_joint", "head_2_joint"]
    point = JointTrajectoryPoint()
    point.positions = [0.0, 0.0]
    point.time_from_start = Duration(sec=3)
    aim.points = [point]
    head.publish(aim)
    settle = sim_now()
    while sim_now() - settle < 5.0:
        pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.05)

    # The robot spawns facing a wall, so sweep until the marker comes into view --
    # slowly, since the whole point of this tool is that a fast turn loses it.
    print("sweeping for the marker (the robot spawns facing away from the shelf)")
    hunt = Twist()
    hunt.angular.z = 0.20
    deadline = time.time() + 180
    while time.time() < deadline and len(state["seen"]) < 3:
        pub.publish(hunt)
        rclpy.spin_once(node, timeout_sec=0.05)
    for _ in range(40):
        pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.05)
    if len(state["seen"]) < 3:
        print("no bearing published at all while sweeping a full circle")
        return 1
    print("marker in view at %.0f px; settling, then turning"
          % state["seen"][-1][1])
    settle = sim_now()
    while sim_now() - settle < 3.0:
        pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.05)

    state["seen"].clear()
    while len(state["seen"]) < 1:
        rclpy.spin_once(node, timeout_sec=0.1)
    start_yaw = state["yaw"]
    start_t = sim_now()
    twist = Twist()
    twist.angular.z = rate
    while sim_now() - start_t < span:
        pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.02)
    for _ in range(40):
        pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.05)

    samples = list(state["seen"])
    if len(samples) < 4:
        print("only %d bearings in %.0f simulated seconds -- that is the finding"
              % (len(samples), span))
        return 0

    def unwrap(a, b):
        return (b - a + math.pi) % (2 * math.pi) - math.pi

    print("\nturned at %+.2f rad/s for %.0f simulated seconds, %d bearings arrived\n"
          % (rate, span, len(samples)))
    print("%-8s %-10s %-12s %-12s %s"
          % ("t", "bearing", "turned", "predicted", "gap"))

    distinct = 0
    previous_value = None
    for stamp, value, yaw in samples:
        if yaw is None:
            continue
        turned = unwrap(start_yaw, yaw)
        # Turning left moves a fixed marker right in the image, and back again.
        predicted = (IMAGE_WIDTH / 2.0) + FOCAL_PX * math.tan(
            math.atan2(samples[0][1] - IMAGE_WIDTH / 2.0, FOCAL_PX) + turned)
        if previous_value is None or abs(value - previous_value) > 0.5:
            distinct += 1
        previous_value = value
        print("%-8.1f %-10.1f %-12.3f %-12.1f %+.0f px"
              % (stamp - start_t, value, turned, predicted, value - predicted))

    first, last = samples[0], samples[-1]
    moved_px = last[1] - first[1]
    turned = unwrap(start_yaw, last[2]) if last[2] is not None else 0.0
    print("\n%d of %d bearings were a NEW number" % (distinct, len(samples)))
    print("bearing moved %+.0f px while the base turned %+.1f degrees"
          % (moved_px, math.degrees(turned)))
    expected = FOCAL_PX * math.tan(turned)
    print("a marker fixed in the world would have moved about %+.0f px" % -expected)
    if abs(moved_px) < 0.2 * abs(expected):
        print("\nThe bearing is NOT following the robot. It is either frozen or it is")
        print("being measured against something that moves with the base.")
    elif distinct < len(samples) / 3:
        print("\nThe bearing follows, but most arrivals repeat the previous number:")
        print("perception is republishing faster than it looks.")
    else:
        print("\nThe bearing follows the turn. The centring fault is elsewhere.")

    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
