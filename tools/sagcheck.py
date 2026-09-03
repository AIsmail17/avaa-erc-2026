#!/usr/bin/env python3
"""How far out can the arm hold a grasp posture before it sags?

    tools/in-sim sagcheck.py [row_height] [x ...]

reach.py says the failed grasp asked for 85 per cent of the arm's maximum extension. That
is a number, not a verdict: 85 per cent might be perfectly holdable. This finds out where
the line actually is, by commanding the posture and measuring where the gripper settles.

The base does not move. Reach is a property of the arm, so the sweep is over the TARGET
in base_link, and one parked robot answers for every standoff at once -- which is also
the only affordable way to do it, a base move being minutes and an arm move being
seconds.

Every posture is checked against MoveIt before it is commanded, so a target the shelf is
standing in is skipped rather than driven into.
"""
import math
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
SHOULDER_BASE_Z = 0.677
SHOULDER_Y = 0.159
GRASP_APPROACH = [1.0, 0.0, 0.0]
GRASP_CLOSING = [0.0, 1.0, 0.0]
SETTLE = 22.0


def main():
    height = float(sys.argv[1]) if len(sys.argv) > 1 else 1.346
    xs = [float(v) for v in sys.argv[2:]] or [
        0.55, 0.62, 0.69, 0.76, 0.83, 0.90]

    rclpy.init()
    node = rclpy.create_node("sagcheck")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    arm = node.create_publisher(
        JointTrajectory, "/arm_left_controller/joint_trajectory", 10)
    torso = node.create_publisher(
        JointTrajectory, "/torso_controller/joint_trajectory", 10)
    state = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: state.__setitem__("js", m), SENSOR_QOS)

    client = MoveItClient("sagcheck_moveit")
    if not client.wait_until_ready(40.0):
        print("move_group not up")
        return 1
    chain = ArmChain.from_urdf()

    deadline = time.time() + 10
    while time.time() < deadline and arm.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)

    def send(values, seconds=18):
        for pub, names, vals in ((torso, ["torso_lift_joint"], values[:1]),
                                 (arm, ARM, values[1:])):
            traj = JointTrajectory()
            traj.joint_names = list(names)
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in vals]
            point.time_from_start = Duration(sec=int(seconds), nanosec=0)
            traj.points = [point]
            pub.publish(traj)

    def measured():
        for _ in range(60):
            rclpy.spin_once(node, timeout_sec=0.1)
            js = state.get("js")
            if js is None:
                continue
            index = {n: i for i, n in enumerate(js.name)}
            if all(n in index for n in CHAIN):
                return [js.position[index[n]] for n in CHAIN]
        return None

    reach = 1.088   # measured by reach.py, torso-independent
    torso_pin = float(np.clip(height - SHOULDER_BASE_Z + 0.25, 0.0, 0.35))
    shoulder = chain.joint_origins([torso_pin] + [0.0] * 7)[1]
    print("row height %.3f, torso pinned to %.3f, shoulder at z %.3f\n"
          % (height, torso_pin, float(shoulder[2])))
    limits = chain.effort_limits()
    names = CHAIN
    print("%-7s %-9s %-8s %-9s %-9s %-9s %-9s %s"
          % ("x", "of reach", "torque", "settled dx", "dy", "dz", "miss",
             "worst joint"))

    for x in xs:
        target = np.array([x, SHOULDER_Y, height])
        need = float(np.linalg.norm(target - shoulder))
        solution = None
        for _ in range(12):
            candidate = chain.ik(
                target, approach=GRASP_APPROACH, closing=GRASP_CLOSING,
                pin={"torso_lift_joint": (torso_pin, 0.10)})
            if candidate is None:
                continue
            if client.state_valid(CHAIN, candidate) is False:
                continue
            solution = candidate
            break
        if solution is None:
            print("%-7.3f %-9s %s" % (x, "%.0f%%" % (100 * need / reach),
                                      "no clear posture"))
            continue
        torques = chain.gravity_torque(solution)
        share, worst = max((t / l, n) for n, t, l in zip(names, torques, limits)
                           if n != "torso_lift_joint" and l > 0.0)

        send(solution)
        # Settle in SIMULATED time. Wall clock says nothing here: the real-time factor
        # moves by a factor of forty over a session, and a fixed wall-clock wait is a
        # different amount of arm motion every time it is used.
        start = node.get_clock().now().nanoseconds * 1e-9
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = node.get_clock().now().nanoseconds * 1e-9
            if now - start >= SETTLE:
                break

        actual = measured()
        if actual is None:
            print("%-7.2f  no joint states" % x)
            continue
        where = chain.fk(actual)[:3, 3]
        delta = where - target
        print("%-7.3f %-9s %-8s %-+9.0f %-+9.0f %-+9.0f %-9s %s"
              % (x, "%.0f%%" % (100 * need / reach),
                 "%.0f%%" % (share * 100.0),
                 delta[0] * 1000, delta[1] * 1000, delta[2] * 1000,
                 "%.0f mm" % (float(np.linalg.norm(delta)) * 1000), worst))

    print("\nsag is the settled gap with the posture still commanded: the arm is not")
    print("moving towards it any more, this is where it stops.")
    client.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
