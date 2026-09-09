#!/usr/bin/env python3
"""Does the base hold still, and where is the book from it? Ground truth, twice over."""
import math
import subprocess
import time

WORLD = "erc_world"
BOOK = "book_col_3_row_2_red"


def poses():
    raw = subprocess.run(["gz", "topic", "-e", "-t",
                          "/world/%s/dynamic_pose/info" % WORLD, "-n", "1"],
                         capture_output=True, text=True, timeout=25).stdout
    out, name, section, fields = {}, None, None, {}
    for line in raw.splitlines():
        line = line.strip()
        if line.startswith('name: "'):
            if name and "px" in fields:
                out[name] = dict(fields)
            name, section, fields = line.split('"')[1], None, {}
        elif line.startswith("position"):
            section = "p"
        elif line.startswith("orientation"):
            section = "q"
        elif name and section and ":" in line:
            k, _, v = line.partition(":")
            k = k.strip()
            if k in ("x", "y", "z", "w") and section + k not in fields:
                try:
                    fields[section + k] = float(v)
                except ValueError:
                    pass
    if name and "px" in fields:
        out[name] = dict(fields)
    return out


for i in range(3):
    p = poses()
    r, b = p.get("tiago_pro"), p.get(BOOK)
    if r is None or b is None:
        print("sample %d: robot=%s book=%s" % (i, r is not None, b is not None))
        time.sleep(6)
        continue
    yaw = math.atan2(2.0 * (r["qw"] * r["qz"] + r["qx"] * r["qy"]),
                     1.0 - 2.0 * (r["qy"] ** 2 + r["qz"] ** 2))
    dx, dy = b["px"] - r["px"], b["py"] - r["py"]
    bx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
    by = dx * math.sin(-yaw) + dy * math.cos(-yaw)
    print("sample %d: base world (%.3f, %.3f) yaw %+.1f deg | book in base (%.3f, %.3f) "
          "| face x %.3f"
          % (i, r["px"], r["py"], math.degrees(yaw), bx, by, bx - 0.08))
    time.sleep(6)
