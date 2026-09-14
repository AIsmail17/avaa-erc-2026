#!/usr/bin/env python3
"""How far does a teleported book fall between teleports, at 5 Hz and faster?

    tools/in-sim holdprobe.py [book name]

sim_grasp_fix holds a book by setting its pose with one gz CLI call at a time, five times
a second. Between calls the book is a free body. The laptop run of 2026-09-14 (marker 2,
blue) had its book hit the bin during the move across and land on the table, and the
user watched books drop and snap back into the hand.

This lifts one book 0.5 m into the air and holds it there with set_pose through the
Gazebo transport Python bindings, at several rates, reading the book's pose from
dynamic_pose/info just before each teleport. It prints how far below the hold point the
book had fallen, second by second, then puts the book back where it was.
"""
import sys
import threading
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

WORLD = "erc_world"


def main():
    node = Node()
    latest = {}
    lock = threading.Lock()

    def on_poses(msg):
        with lock:
            for p in msg.pose:
                # Models only: each book's link is listed too, as book_base_link at the
                # model's origin, and set_pose on that name times out.
                if p.name.startswith("book_col_"):
                    latest[p.name] = (p.position.x, p.position.y, p.position.z,
                                      p.orientation.x, p.orientation.y, p.orientation.z,
                                      p.orientation.w)

    node.subscribe(Pose_V, "/world/%s/dynamic_pose/info" % WORLD, on_poses)
    end = time.time() + 10
    while time.time() < end and not latest:
        time.sleep(0.05)
    if not latest:
        print("no book poses on dynamic_pose/info")
        return 1
    with lock:
        name = sys.argv[1] if len(sys.argv) > 1 else sorted(latest)[0]
        start = latest[name]
    print("holding %s 0.5 m above where it stands, at (%.3f, %.3f, %.3f)"
          % (name, start[0], start[1], start[2]))

    def set_pose(x, y, z, q, timeout_ms=500):
        req = Pose()
        req.name = name
        req.position.x, req.position.y, req.position.z = x, y, z
        req.orientation.x, req.orientation.y, req.orientation.z, req.orientation.w = q
        t0 = time.time()
        ok, _ = node.request("/world/%s/set_pose" % WORLD, req, Pose, Boolean, timeout_ms)
        return ok, time.time() - t0

    hold = (start[0], start[1], start[2] + 0.5)
    q = start[3:]
    # A reply that is slow to come back does not mean the request was not carried out,
    # so the faster rates also run with a timeout shorter than the period.
    for rate, timeout_ms in ((5.0, 500), (20.0, 500), (20.0, 15), (50.0, 8)):
        set_pose(*hold, q)
        time.sleep(0.3)
        period = 1.0 / rate
        began = time.time()
        samples, calls, worst_call, acked = [], 0, 0.0, 0
        while time.time() - began < 4.0:
            tick = time.time()
            with lock:
                z = latest[name][2]
            samples.append(hold[2] - z)
            ok, took = set_pose(*hold, q, timeout_ms)
            calls += 1
            acked += bool(ok)
            worst_call = max(worst_call, took)
            time.sleep(max(0.0, period - (time.time() - tick)))
        samples.sort()
        print("  %4.0f Hz, %3d ms timeout: fall median %.0f mm, 90th %.0f, worst %.0f; "
              "%d calls, %d answered, slowest %.0f ms"
              % (rate, timeout_ms, samples[len(samples) // 2] * 1000,
                 samples[int(len(samples) * 0.9)] * 1000, samples[-1] * 1000, calls, acked,
                 worst_call * 1000))
    set_pose(start[0], start[1], start[2], q)
    print("put %s back" % name)
    return 0


sys.exit(main())
