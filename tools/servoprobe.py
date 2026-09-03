#!/usr/bin/env python3
"""Does the servo loop converge, or does it drive the arm away from the book?

    tools/in-sim servoprobe.py [x] [z] [ticks]

sagcheck.py ruled out the obvious explanation for the 445 mm collapse: commanded as one
trajectory and given time to settle, the arm holds an 0.88 m reach at the top row to
within 4 mm. So the arm can hold the posture. Something about the way the SERVO state
asks for it is what fails.

This runs the servo's own control law -- the same step, gain, clip and 0.35 s command --
in free space, and prints per tick what it asked for against what the arm did. If the
commanded position is running away from the measured one, the loop is the fault and no
amount of standing closer will fix it.
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

ARM = ["arm_left_%d_joint" % i for i in range(1, 8)]
CHAIN = ["torso_lift_joint"] + ARM
SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        durability=QoSDurabilityPolicy.VOLATILE,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=1)
GRASP_APPROACH = [1.0, 0.0, 0.0]
GRASP_CLOSING = [0.0, 1.0, 0.0]
SHOULDER_BASE_Z = 0.677
SHOULDER_Y = 0.159

# The servo's own settings, copied from grasp_node so this measures what runs.
SERVO_STEP = 0.012
SERVO_GAIN = 1.6
SERVO_COMMAND = 0.35
SERVO_MAX_JOINT = 0.25


def main():
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.935
    z = float(sys.argv[2]) if len(sys.argv) > 2 else 1.346
    ticks = int(sys.argv[3]) if len(sys.argv) > 3 else 90

    rclpy.init()
    node = rclpy.create_node("servoprobe")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    pub_arm = node.create_publisher(
        JointTrajectory, "/arm_left_controller/joint_trajectory", 10)
    pub_torso = node.create_publisher(
        JointTrajectory, "/torso_controller/joint_trajectory", 10)
    state = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: state.__setitem__("js", m), SENSOR_QOS)
    chain = ArmChain.from_urdf()

    deadline = time.time() + 10
    while time.time() < deadline and pub_arm.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)

    def joints():
        for _ in range(60):
            rclpy.spin_once(node, timeout_sec=0.05)
            js = state.get("js")
            if js is None:
                continue
            index = {n: i for i, n in enumerate(js.name)}
            if all(n in index for n in CHAIN):
                return [js.position[index[n]] for n in CHAIN]
        return None

    def sim_now():
        return node.get_clock().now().nanoseconds * 1e-9

    def wait(seconds):
        start = sim_now()
        while sim_now() - start < seconds:
            rclpy.spin_once(node, timeout_sec=0.05)

    target = np.array([x, SHOULDER_Y, z])
    torso_pin = float(np.clip(z - SHOULDER_BASE_Z + 0.25, 0.0, 0.35))

    # Start where the reach would have left it: a posture whose gripper is a little
    # short of the book, which is what the servo is for.
    start_point = target - np.array([0.06, 0.0, 0.0])
    seeded = None
    for _ in range(20):
        candidate = chain.ik(start_point, approach=GRASP_APPROACH,
                             closing=GRASP_CLOSING,
                             pin={"torso_lift_joint": (torso_pin, 0.10)})
        if candidate is not None:
            seeded = candidate
            break
    if seeded is None:
        print("could not solve the starting posture")
        return 1

    for pub, names, vals in ((pub_torso, ["torso_lift_joint"], seeded[:1]),
                             (pub_arm, ARM, seeded[1:])):
        traj = JointTrajectory()
        traj.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in vals]
        point.time_from_start = Duration(sec=14)
        traj.points = [point]
        pub.publish(traj)
    print("moving to the starting posture, 60 mm short of the target")
    wait(18.0)

    now = joints()
    where = chain.fk(now)[:3, 3]
    print("started %.0f mm from the target\n"
          % (float(np.linalg.norm(target - where)) * 1000))
    print("%-5s %-9s %-9s %-9s %-9s %s"
          % ("tick", "to go", "lag", "push", "torso", "note"))

    rejected = 0
    for tick in range(ticks):
        now = joints()
        if now is None:
            break
        here = chain.fk(now)[:3, 3]
        error = target - here
        distance = float(np.linalg.norm(error))

        solution = None
        for scale in (1.0, 0.5, 0.25):
            step = min(SERVO_STEP * scale, distance)
            goal = here + error * (step / distance)
            candidate = chain.ik(
                goal, seed=now, approach=GRASP_APPROACH, closing=GRASP_CLOSING,
                pin={"torso_lift_joint": (now[0], 0.004)})
            if candidate is None:
                continue
            if max(abs(a - b) for a, b in zip(candidate, now)) <= SERVO_MAX_JOINT:
                solution = candidate
                break
        if solution is None:
            rejected += 1
            print("%-5d %-9.0f %-9s %-9s %-9.3f %s"
                  % (tick, distance * 1000, "-", "-", now[0], "REJECTED"))
            wait(0.2)
            continue

        command = []
        for value, actual, (lo, hi) in zip(solution, now, chain.limits):
            pushed = value + SERVO_GAIN * (value - actual)
            pushed = min(max(pushed, value - SERVO_MAX_JOINT),
                         value + SERVO_MAX_JOINT)
            command.append(float(np.clip(pushed, lo, hi)))

        lag = max(abs(a - b) for a, b in zip(solution, now))
        push = max(abs(a - b) for a, b in zip(command, now))

        traj = JointTrajectory()
        traj.joint_names = list(ARM)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in command[1:]]
        point.time_from_start = Duration(
            sec=int(SERVO_COMMAND), nanosec=int((SERVO_COMMAND % 1.0) * 1e9))
        traj.points = [point]
        pub_arm.publish(traj)

        if tick % 5 == 0 or distance > 0.30:
            print("%-5d %-9.0f %-9.3f %-9.3f %-9.3f %s"
                  % (tick, distance * 1000, lag, push, now[0],
                     "" if distance < 0.30 else "DIVERGING"))
        wait(0.2)

    now = joints()
    here = chain.fk(now)[:3, 3]
    delta = here - target
    print("\nfinished %.0f mm out (dx %+.0f dy %+.0f dz %+.0f), %d rejected of %d"
          % (float(np.linalg.norm(delta)) * 1000, delta[0] * 1000,
             delta[1] * 1000, delta[2] * 1000, rejected, ticks))
    print("torso ended at %.3f, pinned at %.3f" % (now[0], torso_pin))

    node.destroy_node()
    rclpy.shutdown()
    return 0


sys.exit(main())
