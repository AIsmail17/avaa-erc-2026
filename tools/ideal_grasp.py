#!/usr/bin/env python3
"""Can the arm pick up a book when the target is perfect?

    python3 ideal_grasp.py <book_colour>

Perception is now close: measured against Gazebo the robot ends square, the row is right
and the grasp point lands inside the book. The book still does not move. That leaves two
very different causes -- a targeting error of a few centimetres, or an arm that never
does what it is told -- and the fix for one is nothing like the fix for the other.

So this removes perception from the loop. It reads the target book straight out of
Gazebo, publishes that as the row and the book point, and runs the real grasp controller
against it while watching the arm. If the book moves, the mechanics are sound and the
remaining work is in perception. If it does not, the mechanics are the problem and no
amount of perception accuracy would have helped.

Run it with the robot already standing in front of the shelf.
"""
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, String

sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
from avaa_solution.kinematics.arm_chain import ArmChain  # noqa: E402

ARM_JOINTS = ["arm_left_%d_joint" % i for i in range(1, 8)]
CHAIN_JOINTS = ["torso_lift_joint"] + ARM_JOINTS
FINGER = "gripper_left_finger_joint"
BASE_Z = 0.186          # base_link sits this far above the world origin
BOOK_HALF_DEPTH = 0.08  # book centre to its front face

SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        durability=QoSDurabilityPolicy.VOLATILE,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=1)


def gz_pose(model):
    out = subprocess.run(["gz", "model", "-m", model, "-p"],
                         capture_output=True, text=True, timeout=25).stdout
    lines = [l.strip() for l in out.splitlines()]
    for i, line in enumerate(lines):
        if line.startswith("[") and i + 1 < len(lines) and lines[i + 1].startswith("["):
            return ([float(v) for v in line.strip("[]").split()],
                    [float(v) for v in lines[i + 1].strip("[]").split()])
    return None, None


def nearest_book(colour):
    listing = subprocess.run(["gz", "model", "--list"], capture_output=True,
                             text=True, timeout=25).stdout
    names = [l.strip(" -") for l in listing.splitlines()
             if "book_col" in l and colour in l]
    base, _ = gz_pose("tiago_pro")
    best = None
    for name in names:
        p, r = gz_pose(name)
        if p is None:
            continue
        d = math.hypot(p[0] - base[0], p[1] - base[1])
        if best is None or d < best[0]:
            best = (d, name, p, r)
    return best


def run_grasp():
    cmd = ("source /opt/erc_ws/install/setup.bash && "
           "exec python3 -u /opt/erc_ws/install/avaa_solution/lib/avaa_solution/"
           "grasp --ros-args -p use_sim_time:=true > /tmp/ideal_grasp.log 2>&1")
    return subprocess.Popen(["/entrypoint.sh", "bash", "-c", cmd])


def main():
    colour = sys.argv[1] if len(sys.argv) > 1 else "red"
    chain = ArmChain.from_urdf()

    found = nearest_book(colour)
    if found is None:
        print("no %s book found" % colour)
        return
    distance, name, truth, truth_rpy = found
    robot, rpy = gz_pose("tiago_pro")
    yaw = rpy[2]

    # The book in base_link, from ground truth: rotate the world offset by -yaw.
    dx, dy = truth[0] - robot[0], truth[1] - robot[1]
    bx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
    by = dx * math.sin(-yaw) + dy * math.cos(-yaw)
    face_x = bx - BOOK_HALF_DEPTH
    bz = truth[2] - BASE_Z

    # Which competition row that height is, against the heights the grasp node uses.
    heights = [1.391, 1.061, 0.731, 0.401]
    row = min(range(4), key=lambda i: abs(heights[i] - bz)) + 1

    print("target      : %s at %.2f m" % (name, distance))
    print("robot yaw   : %+.1f deg" % math.degrees(yaw))
    print("book in base: face x=%.3f  y=%+.3f  z=%.3f" % (face_x, by, bz))
    print("row          : %d (nominal z=%.3f, truth z=%.3f, error %+.3f m)"
          % (row, heights[row - 1], bz, bz - heights[row - 1]))
    print()

    rclpy.init()
    node = rclpy.create_node("ideal_grasp")
    pub_row = node.create_publisher(Int32, "/avaa/perception/target_row", 10)
    pub_point = node.create_publisher(
        PointStamped, "/avaa/perception/target_book_point", 10)
    state = {"joints": None, "grasp": "?"}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: state.__setitem__("joints", m), SENSOR_QOS)
    node.create_subscription(String, "/avaa/grasp/state",
                             lambda m: state.__setitem__("grasp", m.data), 10)

    grasp = run_grasp()
    time.sleep(4)

    print("%-11s %6s  %-40s  %8s %8s %8s  %6s" %
          ("state", "torso", "arm 1..7", "fk x", "fk y", "fk z", "finger"))
    seen = set()
    start = time.time()
    try:
        while time.time() - start < 170:
            msg = PointStamped()
            msg.header.frame_id = "base_link"
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.point.x, msg.point.y, msg.point.z = face_x, by, bz
            pub_point.publish(msg)
            pub_row.publish(Int32(data=row))

            rclpy.spin_once(node, timeout_sec=0.2)
            js = state["joints"]
            if js is not None:
                index = {n: i for i, n in enumerate(js.name)}
                if all(n in index for n in CHAIN_JOINTS):
                    actual = [js.position[index[n]] for n in CHAIN_JOINTS]
                    finger = js.position[index[FINGER]] if FINGER in index else float("nan")
                    fk = chain.fk(actual)[:3, 3]
                    now = state["grasp"]
                    stamp = "%.0f" % (time.time() - start)
                    key = (now, stamp)
                    if now not in seen or key not in seen:
                        print("%-11s %6.3f  %-40s  %8.3f %8.3f %8.3f  %6.4f" %
                              (now, actual[0],
                               " ".join("%+.2f" % v for v in actual[1:]),
                               fk[0], fk[1], fk[2], finger), flush=True)
                        seen.add(now)
                        seen.add(key)
            if state["grasp"] in ("done", "failed"):
                break
            time.sleep(1.0)
    finally:
        grasp.terminate()
        subprocess.run(["pkill", "-f", "avaa_solution/lib"], capture_output=True)
        node.destroy_node()
        rclpy.shutdown()

    time.sleep(2)
    after, after_rpy = gz_pose(name)
    moved = math.dist(truth[:3], after[:3]) if after else float("nan")
    tipped = max(abs(a - b) for a, b in zip(truth_rpy[:2], after_rpy[:2]))
    print()
    print("=== judged against Gazebo ===")
    print("%s moved %.3f m, tipped %.2f rad" % (name, moved, tipped))
    # Displacement is not a pick. A book swept onto its side travelled 0.14 m with
    # the fingers fully closed on air, and in the competition that is a penalty.
    if moved <= 0.02:
        print("RESULT: NOT MOVED")
    elif tipped > 0.35:
        print("RESULT: KNOCKED OVER — swept, not grasped")
    elif moved < 0.05:
        print("RESULT: NUDGED — touched but not taken")
    else:
        print("RESULT: PICKED UP")


if __name__ == "__main__":
    main()
