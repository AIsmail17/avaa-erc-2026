#!/usr/bin/env python3
"""Bounding box and triangle count of the gripper's collision meshes.

    tools/in-sim stlbox.py

The fingertip and inner finger use their full visual STL as COLLISION geometry. That is
worth measuring rather than assuming: a detailed mesh is both expensive and unreliable to
collide, and replacing pads with a box primitive is the usual fix when a gripper closes
through things. This prints what those meshes actually are.
"""
import struct
import sys

MESHES = "/opt/erc_ws/src/pal_pro_gripper/pal_pro_gripper_description/meshes"
FILES = ("fingertip.stl", "inner_finger.stl", "outer_finger.stl")


def read_stl(path):
    with open(path, "rb") as f:
        head = f.read(84)
        count = struct.unpack("<I", head[80:84])[0]
        lo = [float("inf")] * 3
        hi = [float("-inf")] * 3
        for _ in range(count):
            chunk = f.read(50)
            if len(chunk) < 50:
                break
            for v in range(3):
                x, y, z = struct.unpack_from("<3f", chunk, 12 + v * 12)
                for i, c in enumerate((x, y, z)):
                    lo[i] = min(lo[i], c)
                    hi[i] = max(hi[i], c)
    return count, lo, hi


for name in FILES:
    try:
        count, lo, hi = read_stl("%s/%s" % (MESHES, name))
    except Exception as exc:  # noqa: BLE001
        print("%-18s could not read (%s)" % (name, exc))
        continue
    size = [hi[i] - lo[i] for i in range(3)]
    print("%-18s %6d triangles   size %.3f x %.3f x %.3f m"
          % (name, count, size[0], size[1], size[2]))
    print("%-18s   centred at (%.4f, %.4f, %.4f)"
          % ("", (hi[0] + lo[0]) / 2, (hi[1] + lo[1]) / 2, (hi[2] + lo[2]) / 2))

sys.exit(0)
