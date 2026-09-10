#!/usr/bin/env python3
"""Does teleporting the base leave it moving, and can that be settled out?

    tools/in-sim settlecheck.py [seconds]

tools/place_robot.py teleports the base into position so a grasp can be tested without
waiting for the approach. It also, apparently, hands it a velocity: measured, a freshly
spawned robot drifts 8 mm and 2 degrees every 30 seconds, and a teleported one drifts
140 mm and 21 degrees, and neither decays because nothing damps this base.

That matters more than a test-rig annoyance. It is ten times the real drift, and every
grasp measured after a teleport has been measured against it -- one run had the base
534 mm out of position by the clamp, which put the book past the end of the arm.

A set_pose in Gazebo sets a pose. It does not say anything about velocity, so whatever
the body had it keeps. But a pose held for a while is a velocity of zero by construction,
so this asks whether simply repeating the call settles it.

Three trials, each measuring the drift over the same window: no teleport at all, one
teleport, and one teleport followed by a second of repeated placements.
"""
import math
import subprocess
import sys
import time

REPEATS = 20
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0


def gz(*args, timeout=20):
    try:
        return subprocess.run(["gz", *args], capture_output=True, text=True,
                              timeout=timeout).stdout
    except Exception:  # noqa: BLE001
        return ""


def truth():
    """Base x, y, yaw from Gazebo, or None."""
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


def teleport(x, y, times=1, pause=0.05):
    request = ('name: "tiago_pro", position: {x: %f, y: %f, z: 0.0}, '
               'orientation: {x: 0, y: 0, z: 0, w: 1}' % (x, y))
    for _ in range(times):
        gz("service", "-s", "/world/erc_world/set_pose", "--reqtype", "gz.msgs.Pose",
           "--reptype", "gz.msgs.Boolean", "--timeout", "800", "--req", request,
           timeout=4)
        if times > 1:
            time.sleep(pause)


def measure(label, seconds):
    start = truth()
    if start is None:
        print("  %-32s no ground truth" % label)
        return
    time.sleep(seconds)
    end = truth()
    if end is None:
        print("  %-32s no ground truth at the end" % label)
        return
    moved = math.hypot(end[0] - start[0], end[1] - start[1])
    turned = math.atan2(math.sin(end[2] - start[2]), math.cos(end[2] - start[2]))
    print("  %-32s %6.0f mm  %+7.2f deg   (%.1f mm/s, %.2f deg/s)"
          % (label, moved * 1000, math.degrees(turned),
             moved * 1000 / seconds, abs(math.degrees(turned)) / seconds))


def main():
    here = truth()
    if here is None:
        print("no ground truth; is the simulator up?")
        return 1
    x, y = here[0], here[1]
    print("base at (%.3f, %.3f), %.0f s per trial. Wall seconds, not simulated:"
          % (x, y, SECONDS))
    print("")

    measure("left alone", SECONDS)

    teleport(x, y, times=1)
    time.sleep(1.0)
    measure("after one set_pose", SECONDS)

    teleport(x, y, times=REPEATS)
    time.sleep(1.0)
    measure("after %d set_pose calls" % REPEATS, SECONDS)

    print("")
    print("A teleport that leaves the base moving is a test rig manufacturing the very")
    print("fault it is being used to measure.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
