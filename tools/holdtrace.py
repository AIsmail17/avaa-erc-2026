#!/usr/bin/env python3
"""Watch the base hold work, or fail to, while a grasp runs.

    tools/in-sim holdtrace.py [seconds]

The hold is a proportional controller on the book's position in base_link, clipped at
40 mm/s, and on paper it should settle a 3 to 8 mm/s coast at twenty or thirty
millimetres of error. Measured during a real grasp the base finished 534 mm out of
position, which put the book past the end of the arm. Those two cannot both be right, and
the difference is not something to reason about from the source.

So this records what actually happened, second by second, in one table: what the hold
commanded, what the book point it was correcting from said, and where the base truly was.
Run it alongside a grasp and read the row where the three stop agreeing.
"""
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import String

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0


def gz(*args, timeout=20):
    try:
        return subprocess.run(["gz", *args], capture_output=True, text=True,
                              timeout=timeout).stdout
    except Exception:  # noqa: BLE001
        return ""


def truth():
    raw = gz("topic", "-e", "-t", "/world/erc_world/dynamic_pose/info", "-n", "1")
    name = None
    section = None
    fields = {}
    for line in raw.splitlines():
        line = line.strip()
        if line.startswith('name: "'):
            if name == "tiago_pro" and "qw" in fields:
                break
            name = line.split('"')[1]
            section = None
            fields = {}
        elif line.startswith("position"):
            section = "p"
        elif line.startswith("orientation"):
            section = "q"
        elif name and section and ":" in line:
            key, _, value = line.partition(":")
            key = key.strip()
            if key in ("x", "y", "z", "w"):
                tag = section + key
                if tag not in fields:
                    try:
                        fields[tag] = float(value)
                    except ValueError:
                        pass
    if name != "tiago_pro" or "qw" not in fields:
        return None
    yaw = math.atan2(2.0 * (fields["qw"] * fields["qz"] + fields["qx"] * fields["qy"]),
                     1.0 - 2.0 * (fields["qy"] ** 2 + fields["qz"] ** 2))
    return fields["px"], fields["py"], yaw


def main():
    rclpy.init()
    node = rclpy.create_node("holdtrace")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])

    seen = {"cmd": None, "cmd_n": 0, "book": None, "state": "?",
            "peak_x": 0.0, "peak_y": 0.0, "peak_z": 0.0}

    def on_cmd(msg):
        seen["cmd"] = msg
        seen["cmd_n"] += 1
        seen["peak_x"] = max(seen["peak_x"], abs(msg.linear.x))
        seen["peak_y"] = max(seen["peak_y"], abs(msg.linear.y))
        seen["peak_z"] = max(seen["peak_z"], abs(msg.angular.z))

    node.create_subscription(Twist, "/cmd_vel", on_cmd, 10)
    node.create_subscription(
        PointStamped, "/avaa/perception/target_book_point",
        lambda m: seen.__setitem__("book", (m.point.x, m.point.y, m.point.z)), 10)
    node.create_subscription(
        String, "/avaa/grasp/state",
        lambda m: seen.__setitem__("state", m.data), 10)

    start = truth()
    if start is None:
        print("no ground truth; is the simulator up?")
        return 1
    print("base starts at (%.3f, %.3f) yaw %+.1f deg" %
          (start[0], start[1], math.degrees(start[2])))
    print("")
    print("   t    state        cmd x    cmd y    cmd wz   msgs/s |  book x  book y "
          "|  drift mm  turn deg")
    print("  " + "-" * 92)

    began = time.time()
    last = began
    last_n = 0
    while time.time() - began < SECONDS:
        deadline = time.time() + 1.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        here = truth()
        cmd = seen["cmd"]
        book = seen["book"]
        rate = (seen["cmd_n"] - last_n) / max(time.time() - last, 1e-6)
        last_n = seen["cmd_n"]
        last = time.time()
        if here is None:
            continue
        drift = math.hypot(here[0] - start[0], here[1] - start[1])
        turn = math.atan2(math.sin(here[2] - start[2]), math.cos(here[2] - start[2]))
        print("  %4.0f  %-11s %+7.3f  %+7.3f  %+7.3f  %6.1f | %s %s | %8.0f  %+8.2f"
              % (time.time() - began, seen["state"][:11],
                 cmd.linear.x if cmd else 0.0,
                 cmd.linear.y if cmd else 0.0,
                 cmd.angular.z if cmd else 0.0,
                 rate,
                 "%7.3f" % book[0] if book else "      -",
                 "%7.3f" % book[1] if book else "      -",
                 drift * 1000, math.degrees(turn)))
        if seen["state"] in ("done", "failed"):
            break

    print("")
    print("largest command seen: x %.3f, y %.3f, wz %.3f m/s"
          % (seen["peak_x"], seen["peak_y"], seen["peak_z"]))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
