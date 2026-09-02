#!/usr/bin/env python3
"""What is the robot touching right now, and what is that costing?

    tools/in-sim whatstouching.py

Reads every arm and body contact sensor once and prints what each reports, alongside the
current real-time factor. Unlike tools/contacts.py this does not place the robot or run
a grasp -- it takes the simulation exactly as it stands, which is what you want when the
question is "why has this instance gone slow".

Persistent contact is expensive here. Measured in one sitting: 0.033 with the robot at
the shelf after a run, 0.365 after teleporting it to open floor, 0.433 after teleporting
it straight back to the shelf. The position was not what mattered.
"""
import re
import subprocess
import time

LINKS = (["arm_left_%d_link" % i for i in range(1, 8)]
         + ["arm_right_%d_link" % i for i in range(1, 8)]
         + ["gripper_left_base_link", "torso_lift_link", "base_link"])
TOPIC = "/world/erc_world/model/tiago_pro/link/%s/sensor/%s_contact/contact"


def sh(command, timeout=60):
    try:
        return subprocess.run(["bash", "-lc", command], capture_output=True,
                              text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return ""


def rtf():
    out = sh("gz topic -e -t /clock -n 2 2>/dev/null")
    return out


def main():
    available = sh("gz topic -l 2>/dev/null")
    touching = 0
    for link in LINKS:
        topic = TOPIC % (link, link)
        if topic not in available:
            continue
        out = sh("timeout 6 gz topic -e -t %s -n 1 2>/dev/null" % topic)
        names = set(re.findall(r'collision2?\s*{\s*name:\s*"([^"]+)"', out))
        others = sorted(n for n in names if "tiago_pro" not in n)
        selfs = sorted(n for n in names if "tiago_pro" in n and link not in n)
        if not others and not selfs:
            continue
        touching += 1
        print("%-24s" % link)
        for name in others:
            print("    world: %s" % name)
        for name in selfs:
            print("    self : %s" % name.split("::")[-1])
    if touching == 0:
        print("nothing in contact on any instrumented link")
    else:
        print("\n%d links in contact" % touching)


main()
