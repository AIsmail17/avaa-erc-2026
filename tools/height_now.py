#!/usr/bin/env python3
"""Where is the gripper, in world height, against the book? One clean reading.

Only meaningful with the arm stationary: it combines a TF lookup with two Gazebo
queries that are a second apart, so anything moving smears across them.
"""
import math, subprocess, sys, time
import numpy as np, rclpy
from tf2_ros import Buffer, TransformListener

BASE_Z = 0.186
TIPS = ["gripper_left_fingertip_left_link", "gripper_left_fingertip_right_link"]
GRASP = "gripper_left_grasping_link"

def pose(model, tries=6):
    for _ in range(tries):
        out = subprocess.run(["gz", "model", "-m", model, "-p"],
                             capture_output=True, text=True, timeout=20).stdout
        lines = [l.strip() for l in out.splitlines()]
        for i, l in enumerate(lines):
            if l.startswith("[") and i + 1 < len(lines) and lines[i + 1].startswith("["):
                return ([float(v) for v in l.strip("[]").split()],
                        [float(v) for v in lines[i + 1].strip("[]").split()])
        time.sleep(0.3)
    return None, None

book = sys.argv[1] if len(sys.argv) > 1 else "book_col_3_row_3_green"
rclpy.init()
n = rclpy.create_node("height_now")
n.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
buf = Buffer(); TransformListener(buf, n)
t = time.time()
while time.time() - t < 8:
    rclpy.spin_once(n, timeout_sec=0.1)

pts = {}
for link in TIPS + [GRASP]:
    tr = buf.lookup_transform("base_link", link, rclpy.time.Time()).transform.translation
    pts[link] = np.array([tr.x, tr.y, tr.z])
mid = (pts[TIPS[0]] + pts[TIPS[1]]) / 2.0

robot, rpy = pose("tiago_pro")
bk, _ = pose(book)
if robot is None or bk is None:
    print("could not read poses"); sys.exit(1)

print("base_link z in world (model z + %.3f): %.3f" % (BASE_Z, robot[2] + BASE_Z))
print("robot roll %.2f deg, pitch %.2f deg" % (math.degrees(rpy[0]), math.degrees(rpy[1])))
yaw = rpy[2]
for name, p in (("grasping frame", pts[GRASP]), ("jaw midpoint", mid)):
    wx = robot[0] + p[0] * math.cos(yaw) - p[1] * math.sin(yaw)
    wy = robot[1] + p[0] * math.sin(yaw) + p[1] * math.cos(yaw)
    wz = p[2] + robot[2] + BASE_Z
    dx, dy, dz = wx - bk[0], wy - bk[1], wz - bk[2]
    print("%-15s world (%.3f, %+.3f, %.3f)" % (name, wx, wy, wz))
    print("%-15s vs book centre: depth %+.0f mm, sideways %+.0f mm, height %+.0f mm"
          % ("", dx * 1000, dy * 1000, dz * 1000))
    print("%-15s book spans depth %.3f..%.3f, width +/-15 mm, height %.3f..%.3f"
          % ("", bk[0] - 0.08, bk[0] + 0.08, bk[2] - 0.125, bk[2] + 0.125))
n.destroy_node(); rclpy.shutdown()
