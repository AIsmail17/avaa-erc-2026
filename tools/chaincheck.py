#!/usr/bin/env python3
"""Is the posture the reach ARRIVES at as good as the one it could have chosen?

    tools/in-sim chaincheck.py [row_height] [pre_x] [grasp_x]

sagcheck says the arm holds the failing target to 3 mm, so standing closer is not the
answer and neither is the torque estimate that read 112 per cent while it held. What is
left is which of the many postures that reach the point the arm ends up in: this arm has
seven joints for three constraints, so a fingertip position does not determine a posture,
and _straight_path seeds each waypoint from the last one, so the reach arrives wherever
the chain happens to walk.

This compares the two. Both are commanded to the same point and given the same time.
"""
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.kinematics.arm_chain import ArmChain  # noqa: E402
from avaa_solution.moveit_client import MoveItClient  # noqa: E402

ARM = ["arm_left_%d_joint" % i for i in range(1, 8)]
CHAIN = ["torso_lift_joint"] + ARM
SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        durability=QoSDurabilityPolicy.VOLATILE,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=1)
GRASP_APPROACH = [1.0, 0.0, 0.0]
GRASP_CLOSING = [0.0, 1.0, 0.0]
SHOULDER_BASE_Z = 0.677
SHOULDER_Y = 0.159


def main():
    height = float(sys.argv[1]) if len(sys.argv) > 1 else 1.346
    pre_x = float(sys.argv[2]) if len(sys.argv) > 2 else 0.653
    grasp_x = float(sys.argv[3]) if len(sys.argv) > 3 else 0.935

    rclpy.init()
    node = rclpy.create_node("chaincheck")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    pub_arm = node.create_publisher(
        JointTrajectory, "/arm_left_controller/joint_trajectory", 10)
    pub_torso = node.create_publisher(
        JointTrajectory, "/torso_controller/joint_trajectory", 10)
    state = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: state.__setitem__("js", m), SENSOR_QOS)
    client = MoveItClient("chaincheck_moveit")
    if not client.wait_until_ready(40.0):
        print("move_group not up")
        return 1
    chain = ArmChain.from_urdf()

    deadline = time.time() + 10
    while time.time() < deadline and pub_arm.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)

    def sim_now():
        return node.get_clock().now().nanoseconds * 1e-9

    def wait(seconds):
        start = sim_now()
        while sim_now() - start < seconds:
            rclpy.spin_once(node, timeout_sec=0.05)

    def joints():
        for _ in range(80):
            rclpy.spin_once(node, timeout_sec=0.05)
            js = state.get("js")
            if js is None:
                continue
            index = {n: i for i, n in enumerate(js.name)}
            if all(n in index for n in CHAIN):
                return [js.position[index[n]] for n in CHAIN]
        return None

    def command(values, seconds=16):
        for pub, names, vals in ((pub_torso, ["torso_lift_joint"], values[:1]),
                                 (pub_arm, ARM, values[1:])):
            traj = JointTrajectory()
            traj.joint_names = list(names)
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in vals]
            point.time_from_start = Duration(sec=int(seconds))
            traj.points = [point]
            pub.publish(traj)

    def torso_for(z):
        return {"torso_lift_joint": (float(np.clip(z - SHOULDER_BASE_Z + 0.25,
                                                   0.0, 0.35)), 0.10)}

    def free_solve(point, tries=16):
        for _ in range(tries):
            candidate = chain.ik(point, approach=GRASP_APPROACH,
                                 closing=GRASP_CLOSING, pin=torso_for(point[2]))
            if candidate is None:
                continue
            if client.state_valid(CHAIN, candidate) is not False:
                return candidate
        return None

    def report(label, solution):
        command(solution)
        wait(20.0)
        actual = joints()
        where = chain.fk(actual)[:3, 3]
        delta = where - np.array([grasp_x, SHOULDER_Y, height])
        gaps = [a - b for a, b in zip(actual, solution)]
        worst = max(range(len(gaps)), key=lambda i: abs(gaps[i]))
        print("%-10s miss %4.0f mm (dx %+4.0f dz %+4.0f), worst joint %s off %+.3f"
              % (label, float(np.linalg.norm(delta)) * 1000,
                 delta[0] * 1000, delta[2] * 1000, CHAIN[worst], gaps[worst]))
        return actual

    pre_point = np.array([pre_x, SHOULDER_Y, height])
    grasp_point = np.array([grasp_x, SHOULDER_Y, height])

    pre = free_solve(pre_point)
    if pre is None:
        print("no clear pre-grasp posture at x=%.3f" % pre_x)
        return 1

    # 1. Walk in the way the reach does: seeded, waypoint by waypoint.
    print("building the seeded chain from the pre-grasp, 9 steps")
    seeded = None
    seed = list(pre)
    for step in range(1, 10):
        point = pre_point + (step / 9.0) * (grasp_point - pre_point)
        solution = chain.ik(point, seed=seed, approach=GRASP_APPROACH,
                            closing=GRASP_CLOSING, pin=torso_for(point[2]))
        if solution is None:
            print("  the chain broke %d/9 of the way in" % step)
            break
        seed = solution
        seeded = solution

    free = free_solve(grasp_point)
    if free is None:
        print("no clear posture at the grasp point")
        return 1

    print()
    if seeded is not None:
        print("seeded endpoint %s" % np.round(seeded, 3).tolist())
    print("free   endpoint %s" % np.round(free, 3).tolist())
    if seeded is not None:
        print("they differ by %s"
              % np.round(np.array(seeded) - np.array(free), 3).tolist())
    print()

    # Command each from the same starting place, so neither is helped by where it began.
    if seeded is not None:
        command(pre)
        wait(18.0)
        report("seeded", seeded)
    command(pre)
    wait(18.0)
    report("free", free)

    client.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
