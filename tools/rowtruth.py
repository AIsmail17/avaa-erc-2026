#!/usr/bin/env python3
"""Watch what perception says the row is, beside what the row actually is.

    tools/in-sim rowtruth.py book_col_3_row_5_red 60

The row is the one number in the whole pipeline that cannot be recovered from later.
Every other quantity the grasp uses is measured continuously and corrected -- the book's
x and y come from depth every frame -- but the HEIGHT comes from a lookup table indexed
by the row, so a row that is one out puts the hand 330 mm from the book and nothing
downstream can tell.

This does not touch the robot. It reads Gazebo ground truth for the base and for every
book, converts each book into base_link, and prints that beside whatever perception is
publishing on target_row, target_column and target_book_point. Two answers to the same
question, arrived at completely differently.

The columns of the report:

    row      what perception latched, and what the book's model name says it is
    z        the measured height of the fix, and the true height, both in base_link
    implied  the row that measured height alone would choose, bias-corrected
    y        measured sideways position against the truth, as a check that the fix is
             even on the right book

A run where "row" disagrees but "implied" is right means the marker grouping is at
fault and the height already knows better. A run where both are wrong means the fix is
on a different book altogether.
"""
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32

WANTED = sys.argv[1] if len(sys.argv) > 1 else "red"
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0

WORLD = "erc_world"
BASE_LINK_Z = 0.0762
ROW_HEIGHTS_BASE = [1.391, 1.061, 0.731, 0.401]
# perception_node.DEPTH_HEIGHT_BIAS: the fix reads high because the book's lower edge is
# behind the shelf lip, so the bounding box centre is above the book centre.
DEPTH_HEIGHT_BIAS = 0.152


def gz(*args, timeout=20):
    try:
        return subprocess.run(["gz", *args], capture_output=True, text=True,
                              timeout=timeout).stdout
    except Exception:  # noqa: BLE001
        return ""


def pose(model, attempts=6):
    """Position and rpy of a model, or (None, None). Gazebo drops queries when busy."""
    for _ in range(attempts):
        lines = [l.strip() for l in gz("model", "-m", model, "-p").splitlines()]
        for i, line in enumerate(lines):
            if line.startswith("[") and i + 1 < len(lines) and lines[i + 1].startswith("["):
                try:
                    return ([float(v) for v in line.strip("[]").split()],
                            [float(v) for v in lines[i + 1].strip("[]").split()])
                except ValueError:
                    return None, None
        time.sleep(0.3)
    return None, None


def books():
    for _ in range(8):
        names = sorted(l.strip(" -") for l in gz("model", "--list").splitlines()
                       if "book_col" in l)
        if names:
            return names
        time.sleep(0.5)
    return []


def in_base(robot, yaw, point):
    """A world point in base_link."""
    dx, dy = point[0] - robot[0], point[1] - robot[1]
    return (dx * math.cos(yaw) + dy * math.sin(yaw),
            -dx * math.sin(yaw) + dy * math.cos(yaw),
            point[2] - BASE_LINK_Z)


def implied_row(z):
    """The row a measured height alone would choose."""
    corrected = z - DEPTH_HEIGHT_BIAS
    return 1 + min(range(len(ROW_HEIGHTS_BASE)),
                   key=lambda i: abs(ROW_HEIGHTS_BASE[i] - corrected))


def main():
    names = books()
    if not names:
        print("no books; is the simulator up?")
        return 1
    matches = [n for n in names if WANTED == n or WANTED in n]
    if not matches:
        print("no book matching %r; the world has %d" % (WANTED, len(names)))
        return 1
    # The colour alone matches one book per column. Take the one nearest the middle,
    # which is what place_robot picks, so the two tools agree on the target.
    target = matches[0]
    if len(matches) > 1:
        scored = []
        for n in matches:
            p, _ = pose(n)
            if p:
                scored.append((abs(p[1]), n))
        if scored:
            target = sorted(scored)[0][1]
    true_row = int(target.split("_row_")[1].split("_")[0]) - 1

    rclpy.init()
    node = rclpy.create_node("rowtruth")
    seen = {"row": None, "column": None, "point": None, "at": None}

    node.create_subscription(Int32, "/avaa/perception/target_row",
                             lambda m: seen.__setitem__("row", m.data), 10)
    node.create_subscription(Int32, "/avaa/perception/target_column",
                             lambda m: seen.__setitem__("column", m.data), 10)

    def on_point(msg: PointStamped):
        seen["point"] = (msg.point.x, msg.point.y, msg.point.z)
        seen["at"] = time.time()

    node.create_subscription(PointStamped, "/avaa/perception/target_book_point",
                             on_point, 10)

    print("target %s -- grasp row %d, true height %.3f in base_link"
          % (target, true_row, ROW_HEIGHTS_BASE[true_row - 1]))
    print()
    print("  t     row  true | fix z   true z  implied | fix y   true y | col")
    print("  " + "-" * 68)

    started = time.time()
    rows_reported = []
    implied_reported = []
    while time.time() - started < SECONDS:
        rclpy.spin_once(node, timeout_sec=0.2)
        if time.time() - started < 1.0:
            continue
        robot, rpy = pose("tiago_pro", attempts=2)
        book, _ = pose(target, attempts=2)
        if robot is None or book is None:
            continue
        tx, ty, tz = in_base(robot, rpy[2], book)

        row = seen["row"]
        point = seen["point"]
        fresh = point is not None and seen["at"] and time.time() - seen["at"] < 3.0
        if row is not None:
            rows_reported.append(row)
        if fresh:
            implied_reported.append(implied_row(point[2]))

        print("  %4.0f   %s   %d  | %s  %6.3f  %s    | %s  %+6.3f | %s"
              % (time.time() - started,
                 "%3d" % row if row is not None else "  -",
                 true_row,
                 "%6.3f" % point[2] if fresh else "     -",
                 tz,
                 "%3d" % implied_row(point[2]) if fresh else "  -",
                 "%+6.3f" % point[1] if fresh else "     -",
                 ty,
                 "%2d" % seen["column"] if seen["column"] is not None else " -"))
        time.sleep(1.0)

    print()
    if rows_reported:
        latched = rows_reported[-1]
        print("perception settled on row %d; the truth is row %d -- %s"
              % (latched, true_row,
                 "correct" if latched == true_row
                 else "%+d rows, %.0f mm of height"
                      % (latched - true_row,
                         abs(ROW_HEIGHTS_BASE[min(latched, 4) - 1]
                             - ROW_HEIGHTS_BASE[true_row - 1]) * 1000)))
    else:
        print("perception never published a row")
    if implied_reported:
        from collections import Counter
        tally = Counter(implied_reported)
        print("the measured height alone implies %s over %d fixes"
              % (dict(tally), len(implied_reported)))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
