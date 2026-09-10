#!/usr/bin/env python3
"""Is the carried book touching the base -- the wheels, or the floor?

    tools/in-sim bookgap.py [book_model_name]

With a book in the gripper the base turns at a capped 0.04 rad/s whatever it is asked for, and
drifts sideways as it turns. The grasp stows the arm to the same tuck it uses empty-handed,
which leaves the gripper low and beside the base; worked out from ground truth, the book then
sits about 0.27 m to the left and 0.15 m up, next to the front-left wheel.

This reads world poses for the book, the robot and every wheel link from Gazebo, puts the book
and wheels in the robot's frame, and reports the book's lowest corner above the floor and the
gap from the book's footprint to each wheel.
"""
import math
import subprocess
import sys

BOOK = sys.argv[1] if len(sys.argv) > 1 else None
BOOK_SIZE = (0.25, 0.03, 0.16)   # local x, y, z of erc_book.sdf
WHEEL_RADIUS = 0.0762            # base_link height, which is the axle height
WHEEL_HALF_WIDTH = 0.025


def read_poses():
    out = ""
    for topic in ("/world/erc_world/pose/info", "/world/erc_world/dynamic_pose/info"):
        try:
            out += subprocess.run(["gz", "topic", "-e", "-t", topic, "-n", "1"],
                                  capture_output=True, text=True, timeout=25).stdout + "\n"
        except Exception:  # noqa: BLE001
            pass
    poses = {}
    lines = out.splitlines()
    i = 0
    while i < len(lines):
        s = lines[i].strip()
        if s.startswith('name: "'):
            name = s.split('"')[1]
            vals = {}
            section = None
            j = i + 1
            while j < len(lines) and not lines[j].strip().startswith('name: "'):
                t = lines[j].strip()
                if t.startswith("position"):
                    section = "p"
                elif t.startswith("orientation"):
                    section = "q"
                elif section and ":" in t:
                    k, _, v = t.partition(":")
                    if k in ("x", "y", "z", "w") and section + k not in vals:
                        try:
                            vals[section + k] = float(v)
                        except ValueError:
                            pass
                j += 1
            if name not in poses and "qw" in vals:
                poses[name] = vals
            i = j
        else:
            i += 1
    return poses


def rot(q):
    x, y, z, w = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def main():
    poses = read_poses()
    robot = poses.get("tiago_pro")
    if robot is None:
        print("no robot pose")
        return 1
    books = [n for n in poses if n.startswith("book_col")]
    book = BOOK
    if book is None:
        # The one off its shelf: lowest and nearest the robot.
        book = min(books, key=lambda n: math.hypot(poses[n]["px"] - robot["px"], poses[n]["py"] - robot["py"]))
    b = poses.get(book)
    if b is None:
        print("no pose for %s" % book)
        return 1
    yaw = math.atan2(2 * (robot["qw"] * robot["qz"] + robot["qx"] * robot["qy"]),
                     1 - 2 * (robot["qy"] ** 2 + robot["qz"] ** 2))

    def to_base(px, py, pz):
        dx, dy = px - robot["px"], py - robot["py"]
        return (dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw), pz)

    # The book's eight corners, in world, then base.
    r = rot((b["qx"], b["qy"], b["qz"], b["qw"]))
    corners = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                lx, ly, lz = sx * BOOK_SIZE[0] / 2, sy * BOOK_SIZE[1] / 2, sz * BOOK_SIZE[2] / 2
                wx = b["px"] + r[0][0] * lx + r[0][1] * ly + r[0][2] * lz
                wy = b["py"] + r[1][0] * lx + r[1][1] * ly + r[1][2] * lz
                wz = b["pz"] + r[2][0] * lx + r[2][1] * ly + r[2][2] * lz
                corners.append(to_base(wx, wy, wz))
    xs = [c[0] for c in corners]
    ys = [c[1] for c in corners]
    zs = [c[2] for c in corners]
    centre = to_base(b["px"], b["py"], b["pz"])
    print("book %s, centre in base frame (%.3f, %+.3f, %.3f)" % (book, centre[0], centre[1], centre[2]))
    print("  footprint x %.3f..%.3f  y %+.3f..%+.3f   lowest corner %.3f m above the floor"
          % (min(xs), max(xs), min(ys), max(ys), min(zs)))

    wheels = sorted(n for n in poses if "wheel" in n and n.endswith("_link"))
    if not wheels:
        print("  (no wheel link poses published; wheel positions from the URDF instead)")
        wheels_base = {"wheel_front_left": (0.244, 0.223), "wheel_front_right": (0.244, -0.223),
                       "wheel_rear_left": (-0.244, 0.223), "wheel_rear_right": (-0.244, -0.223)}
    else:
        wheels_base = {}
        for n in wheels:
            w = poses[n]
            c = to_base(w["px"], w["py"], w["pz"])
            wheels_base[n] = (c[0], c[1])
    for name, (wx, wy) in wheels_base.items():
        gx = max(0.0, max(min(xs) - (wx + WHEEL_RADIUS), (wx - WHEEL_RADIUS) - max(xs)))
        gy = max(0.0, max(min(ys) - (wy + WHEEL_HALF_WIDTH), (wy - WHEEL_HALF_WIDTH) - max(ys)))
        gz = max(0.0, min(zs) - 2 * WHEEL_RADIUS)
        gap = math.sqrt(gx * gx + gy * gy + gz * gz)
        print("  %-22s at (%+.3f, %+.3f): gap from the book %.0f mm%s"
              % (name, wx, wy, gap * 1000, "   <-- TOUCHING" if gap < 0.005 else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
