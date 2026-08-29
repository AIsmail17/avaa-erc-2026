#!/usr/bin/env python3
"""Can this gripper hold this book at all, with targeting taken out of the question?

Every failed run so far has ended the same way: the gripper arrives within a millimetre
of where it was aimed, the jaws close, and the book is not held. That has two very
different explanations -- the book is not where we aimed, or the jaws cannot grip it --
and no amount of accuracy fixes the second one.

So this removes aiming entirely. It puts the arm in the posture a real grasp ends in,
reads where the jaws actually are, teleports the book exactly between them, closes, and
then lifts. If the book comes up, the mechanics are sound. If it does not, no targeting
work will ever produce a pick.

    python3 gripcheck.py [book_model]
"""
import math, subprocess, sys, time
import numpy as np, rclpy
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

GRASP_POSTURE = [0.25, 3.14, -2.30, -0.68, -2.07, 1.57, -0.28, -1.80]
ARM = ["arm_left_%d_joint" % i for i in range(1, 8)]
GRASP_LINK = "gripper_left_grasping_link"
TIPS = ["gripper_left_fingertip_left_link", "gripper_left_fingertip_right_link"]
BASE_Z = 0.186


def gz(*a, t=25):
    return subprocess.run(["gz", *a], capture_output=True, text=True, timeout=t).stdout


def pose(model, attempts=8):
    for _ in range(attempts):
        ls = [l.strip() for l in gz("model", "-m", model, "-p").splitlines()]
        for i, l in enumerate(ls):
            if l.startswith("[") and i + 1 < len(ls) and ls[i + 1].startswith("["):
                return ([float(v) for v in l.strip("[]").split()],
                        [float(v) for v in ls[i + 1].strip("[]").split()])
        time.sleep(0.3)
    return None, None


def put(model, x, y, z):
    return gz("service", "-s", "/world/erc_world/set_pose",
              "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
              "--timeout", "3000",
              "--req", 'name: "%s", position: {x: %f, y: %f, z: %f}, '
                       'orientation: {x: 0, y: 0.7071068, z: 0, w: 0.7071068}'
                       % (model, x, y, z))


def main():
    book = sys.argv[1] if len(sys.argv) > 1 else "book_col_3_row_3_red"
    rclpy.init()
    n = rclpy.create_node("gripcheck")
    n.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    buf = Buffer(); TransformListener(buf, n)
    st = {}
    n.create_subscription(JointState, "/joint_states",
                          lambda m: st.update(zip(m.name, m.position)), 10)
    arm = n.create_publisher(JointTrajectory, "/arm_left_controller/joint_trajectory", 10)
    torso = n.create_publisher(JointTrajectory, "/torso_controller/joint_trajectory", 10)
    grip = n.create_publisher(JointTrajectory,
                              "/gripper_left_controller/joint_trajectory", 10)
    t = time.time()
    while time.time() - t < 8:
        rclpy.spin_once(n, timeout_sec=0.1)

    def send(pub, names, values, secs):
        tr = JointTrajectory(); tr.joint_names = names
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in values]
        p.time_from_start = Duration(sec=secs)
        tr.points = [p]; pub.publish(tr)

    def settle(secs):
        t = time.time()
        while time.time() - t < secs:
            rclpy.spin_once(n, timeout_sec=0.1)

    print("posing the arm as a real grasp ends, jaws open...")
    send(torso, ["torso_lift_joint"], [GRASP_POSTURE[0]], 18)
    send(arm, ARM, GRASP_POSTURE[1:], 18)
    send(grip, ["gripper_left_finger_joint"], [0.055], 6)
    settle(50)

    tips = []
    for link in TIPS:
        tf = buf.lookup_transform("base_link", link, rclpy.time.Time()).transform.translation
        tips.append(np.array([tf.x, tf.y, tf.z]))
    mid = (tips[0] + tips[1]) / 2.0
    robot, rpy = pose("tiago_pro")
    if robot is None:
        print("no robot pose"); return
    yaw = rpy[2]
    wx = robot[0] + mid[0] * math.cos(yaw) - mid[1] * math.sin(yaw)
    wy = robot[1] + mid[0] * math.sin(yaw) + mid[1] * math.cos(yaw)
    wz = mid[2] + BASE_Z
    print("jaws are at world (%.3f, %.3f, %.3f); gap %.1f mm"
          % (wx, wy, wz, float(np.linalg.norm(tips[0] - tips[1])) * 1000))
    print("teleporting %s exactly between them" % book)
    put(book, wx, wy, wz)
    settle(12)

    before, _ = pose(book)
    print("book now at (%.3f, %.3f, %.3f)" % tuple(before))
    print("closing...")
    send(grip, ["gripper_left_finger_joint"], [-0.001], 8)
    for i in range(8):
        settle(5)
        here, _ = pose(book)
        print("  t+%2ds finger=%+.4f  book z=%.3f" % (
            5 * (i + 1), st.get("gripper_left_finger_joint", float("nan")),
            here[2] if here else float("nan")))

    held, _ = pose(book)
    print()
    print("lifting the torso by 100 mm...")
    send(torso, ["torso_lift_joint"], [GRASP_POSTURE[0] + 0.10], 10)
    settle(35)
    after, after_rpy = pose(book)
    if after is None or held is None:
        print("lost the book pose"); return
    print("book rose %.3f m while the torso rose 0.100 m" % (after[2] - held[2]))
    print("finger ended at %+.4f" % st.get("gripper_left_finger_joint", float("nan")))
    if after[2] - held[2] > 0.05:
        print("RESULT: HELD — the gripper can carry this book")
    elif after[2] < held[2] - 0.05:
        print("RESULT: DROPPED — it fell out")
    else:
        print("RESULT: NOT HELD — it stayed behind")
    n.destroy_node(); rclpy.shutdown()


main()
