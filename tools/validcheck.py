#!/usr/bin/env python3
"""Does state_valid say yes to a posture we know is fine?"""
import sys

import rclpy

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.moveit_client import MoveItClient  # noqa: E402

CHAIN = ["torso_lift_joint"] + ["arm_left_%d_joint" % i for i in range(1, 8)]
CURRENT_TUCK = [0.10, 0.36, -1.83, 0.47, -2.35, 0.0, -1.2, 0.0]
FOUND = [0.15, 3.4413, -2.4214, 0.7426, 1.1345, -3.4231, -1.8850, -1.7096]
ZERO = [0.15, 0, 0, 0, 0, 0, 0, 0]

rclpy.init()
client = MoveItClient("validcheck")
print("move_group ready:", client.wait_until_ready(30.0))
for label, values in (("PAL home (left)", CURRENT_TUCK),
                      ("search result", FOUND),
                      ("all zeros", ZERO)):
    print("%-14s -> %r" % (label, client.state_valid(CHAIN, values)))
client.shutdown()
rclpy.shutdown()
