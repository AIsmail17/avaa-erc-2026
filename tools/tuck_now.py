#!/usr/bin/env python3
"""Command the driving posture and wait for the arms to actually get there.

    tools/in-sim tuck_now.py [seconds]

Reads the poses from the solution itself, so it cannot drift out of step with what the
robot really does. Waits on the joints rather than on a stopwatch: the simulator's
real-time factor moves by a factor of forty over a session, and a wall-clock wait for a
trajectory expressed in simulated seconds is how the grasp fixture spent weeks measuring
an arm that was still halfway through its tuck.
"""
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.approach_node import RIGHT_TUCK, TUCK_POSE, TUCK_TORSO  # noqa: E402


class Tucker(Node):
    def __init__(self):
        super().__init__("tuck_now")
        self.joints = {}
        self.create_subscription(JointState, "/joint_states", self._on, 10)
        self.arms = {
            side: self.create_publisher(
                JointTrajectory, "/arm_%s_controller/joint_trajectory" % side, 10)
            for side in ("left", "right")}
        self.torso = self.create_publisher(
            JointTrajectory, "/torso_controller/joint_trajectory", 10)

    def _on(self, msg):
        for name, value in zip(msg.name, msg.position):
            self.joints[name] = value

    def send(self, publisher, names, values, seconds):
        traj = JointTrajectory()
        traj.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in values]
        point.time_from_start = Duration(sec=int(seconds))
        traj.points = [point]
        publisher.publish(traj)


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0
    rclpy.init()
    node = Tucker()
    deadline = time.time() + 10
    while time.time() < deadline and not node.joints:
        rclpy.spin_once(node, timeout_sec=0.2)

    wanted = {}
    for side, pose in (("left", TUCK_POSE), ("right", RIGHT_TUCK)):
        names = ["arm_%s_%d_joint" % (side, i) for i in range(1, 8)]
        node.send(node.arms[side], names, pose, seconds)
        wanted.update(zip(names, pose))
    node.send(node.torso, ["torso_lift_joint"], [TUCK_TORSO], seconds)
    wanted["torso_lift_joint"] = TUCK_TORSO
    print("commanded the driving posture", flush=True)

    def worst():
        gaps = [abs(node.joints[n] - v) for n, v in wanted.items() if n in node.joints]
        return max(gaps) if len(gaps) == len(wanted) else None

    end = time.time() + 300
    settled = 0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        gap = worst()
        if gap is None:
            continue
        if gap < 0.03:
            settled += 1
            if settled > 25:
                print("tucked, worst joint %.3f rad out" % gap)
                break
        else:
            settled = 0
    else:
        print("did NOT reach the posture; worst joint %s rad out"
              % ("unknown" if worst() is None else "%.3f" % worst()))

    node.destroy_node()
    rclpy.shutdown()


main()
