#!/usr/bin/env python3
"""Which books are lying in the collection bin?

    tools/in-sim binjudge.py

The delivery controller's "book delivered" says the jaws opened over where it aimed; it does
not say where the book went. This reads every book's pose and the bin's from Gazebo, and the
bin's extent from its mesh, and reports each book whose centre sits inside the bin's box, with
how high above the bin's bottom it rests -- and, when there is none, the book nearest the bin.

It used to find the book by the grasp aid's last "picked up" line, and that was wrong twice:
on the sixteenth full run, which grasped nothing, it judged a blue book from a morning session,
and on the laptop, where the aid's lines are not in sim.log at all, it found nothing to judge.
Every book is in the world's pose list; no log is needed. An argument is accepted and ignored,
so the watchers that pass a start time keep working.
"""
import struct
import subprocess

import numpy as np

MESH = "/opt/erc_ws/src/erc_description/models/collection_bin/meshes/erc_base_collection_bin.STL"


def poses():
    out = subprocess.run(["gz", "topic", "-e", "-t", "/world/erc_world/pose/info", "-n", "1"],
                         capture_output=True, text=True, timeout=40).stdout
    found = {}
    name, section, vals = None, None, {}
    for line in out.splitlines() + ['name: "__end__"']:
        s = line.strip()
        if s.startswith('name: "'):
            if name is not None and name not in found and "px" in vals:
                found[name] = vals
            name, section, vals = s.split('"')[1], None, {}
        elif s.startswith("position"):
            section = "p"
        elif s.startswith("orientation"):
            section = "q"
        elif section and ":" in s:
            key, _, value = s.partition(":")
            key = key.strip()
            if key in ("x", "y", "z", "w") and section + key not in vals:
                try:
                    vals[section + key] = float(value)
                except ValueError:
                    pass
    return found


def rotation(p):
    x, y, z, w = p.get("qx", 0.0), p.get("qy", 0.0), p.get("qz", 0.0), p.get("qw", 1.0)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mesh_bounds():
    data = open(MESH, "rb").read()
    count = struct.unpack("<I", data[80:84])[0]
    raw = np.frombuffer(data, dtype=np.uint8, count=count * 50, offset=84).reshape(count, 50)
    verts = np.frombuffer(raw[:, 12:48].tobytes(), dtype="<f4").reshape(-1, 3).astype(float)
    return verts.min(axis=0), verts.max(axis=0)


def main():
    found = poses()
    if "erc_collection_bin" not in found:
        print("no pose for the bin")
        return
    lo, hi = mesh_bounds()
    bin_pose = found["erc_collection_bin"]
    R = rotation(bin_pose)
    origin = np.array([bin_pose["px"], bin_pose["py"], bin_pose["pz"]])
    up = R.T @ np.array([0.0, 0.0, 1.0])
    axis = int(np.argmax(np.abs(up)))
    books = sorted(n for n in found if n.startswith("book_"))
    inside, nearest = [], None
    for name in books:
        p = found[name]
        centre = np.array([p["px"], p["py"], p["pz"]])
        local = R.T @ (centre - origin)
        gap = np.maximum(0.0, np.maximum(lo - local, local - hi))
        distance = float(np.linalg.norm(gap))
        if distance == 0.0:
            bottom = lo[axis] if up[axis] > 0 else -hi[axis]
            height = (local[axis] if up[axis] > 0 else -local[axis]) - bottom
            inside.append((name, height))
        elif nearest is None or distance < nearest[1]:
            nearest = (name, distance, centre)
    print("%d books in the world" % len(books))
    if inside:
        for name, height in inside:
            print("IN THE BIN: %s -- centre %.0f mm above the bin's bottom" % (name, height * 1000))
    else:
        print("IN THE BIN: none")
        if nearest is not None:
            print("nearest book to the bin: %s, %.0f mm outside its box, at (%.3f, %.3f, %.3f)"
                  % ((nearest[0], nearest[1] * 1000) + tuple(nearest[2])))


if __name__ == "__main__":
    main()
