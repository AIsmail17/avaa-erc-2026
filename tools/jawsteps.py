#!/usr/bin/env python3
"""Command the finger joint to a series of values and see which it reaches.

    tools/in-sim jawsteps.py [--raw]

Closing was commanded and the joint stayed near 0.070. Before blaming the close
specifically, find out whether the joint tracks anything at all: step it down in stages
and watch. A joint that follows 0.05 and 0.03 and then stalls at 0.01 is a different
fault from one that never leaves 0.070.

--raw publishes straight to gripper_left_controller_raw, bypassing the clamp node, which
separates "the controller will not do it" from "something in front of the controller is
not passing it on".
"""
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT = "gripper_left_finger_joint"
RAW = "--raw" in sys.argv
TOPIC = ("/gripper_left_controller_raw/joint_trajectory" if RAW
         else "/gripper_left_controller/joint_trajectory")


def main():
    rclpy.init()
    node = rclpy.create_node("jawsteps")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    state = {}
    node.create_subscription(
        JointState, "/joint_states",
        lambda m: state.update(dict(zip(m.name, m.position))), 10)
    pub = node.create_publisher(JointTrajectory, TOPIC, 10)

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
        traj.joint_names = [JOINT]
        point = JointTrajectoryPoint()
        point.positions = [float(value)]
        point.velocities = [0.0]
        point.time_from_start = Duration(sec=4, nanosec=0)
        traj.points = [point]
        pub.publish(traj)

    print("publishing to %s" % TOPIC)
    spin(2.0)
    print("  %-10s %-10s %s" % ("asked", "settled", "reached?"))
    for value in (0.060, 0.045, 0.030, 0.015, 0.000):
        send(value)
        spin(8.0)
        got = state.get(JOINT, float("nan"))
        print("  %-10.3f %-10.4f %s"
              % (value, got, "yes" if abs(got - value) < 0.006 else "NO"))

    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
