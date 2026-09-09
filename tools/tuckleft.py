#!/usr/bin/env python3
"""Fold the left arm to the tuck, straight through the controller.

    tools/in-sim tuckleft.py

For getting the robot out of a state a previous failed run left it in. The arm spawns
straight out, and a run that fails before it stows leaves it wherever it stopped; driving
the base with it extended sweeps it through the shelf.

No planning, no collision checking -- this is the same raw trajectory publish grasp_node
uses for its own stow, and it is only ever safe in the retracting direction.
"""
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TUCK_POSE = [0.3877, -1.6152, 0.0717, 0.0408, 0.5074, -1.4708, 0.6063]
NAMES = ["arm_left_%d_joint" % i for i in range(1, 8)]
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0

rclpy.init()
node = rclpy.create_node("tuckleft")
node.set_parameters([rclpy.parameter.Parameter(
    "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
pub = node.create_publisher(JointTrajectory, "/arm_left_controller/joint_trajectory", 10)

seen = {}
node.create_subscription(
    JointState, "/joint_states",
    lambda m: seen.update(dict(zip(m.name, m.position))), 10)

for _ in range(40):
    rclpy.spin_once(node, timeout_sec=0.1)

print("left arm before: [%s]"
      % ", ".join("%+.3f" % seen.get(j, float("nan")) for j in NAMES))

traj = JointTrajectory()
traj.joint_names = NAMES
point = JointTrajectoryPoint()
point.positions = [float(v) for v in TUCK_POSE]
point.time_from_start = Duration(sec=int(SECONDS), nanosec=0)
traj.points = [point]
for _ in range(3):
    pub.publish(traj)
    time.sleep(0.3)

deadline = time.time() + SECONDS + 12.0
while rclpy.ok() and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    worst = max(abs(seen.get(j, 99.0) - t) for j, t in zip(NAMES, TUCK_POSE))
    if worst < 0.12:
        break

print("left arm after : [%s]"
      % ", ".join("%+.3f" % seen.get(j, float("nan")) for j in NAMES))
print("worst joint off the tuck: %.3f rad"
      % max(abs(seen.get(j, 99.0) - t) for j, t in zip(NAMES, TUCK_POSE)))
node.destroy_node()
rclpy.shutdown()
