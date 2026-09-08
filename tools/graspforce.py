#!/usr/bin/env python3
"""Scale the left gripper's effort limits, to find one that grips instead of crushing.

    tools/graspforce.py 0.05      scale every left-finger effort limit by 0.05
    tools/graspforce.py --show    print the current limits
    tools/graspforce.py --restore put them back to the values in git

Runs on the HOST, not in the container: it edits the URDF in src/, which is mounted into
the container, so the next bench launch picks it up with no rebuild.

Why effort is worth sweeping
----------------------------
gz_ros2_control drives these joints by writing JointVelocityCmd -- "move at this velocity"
-- and dartsim honours the joint's effort limit when it does. Measured on the bench: with
the limits scaled by 0.001 the finger stalled at 0.0634 instead of reaching 0.0000, so the
ceiling is real. At the stock 10 N it is simply high enough to push a 30 mm book out from
between 28 mm jaws.

Somewhere between 0.001 and 1.0 there should be a scale that closes the jaws onto a book
and stops there. This is what finds it.
"""
import io
import re
import sys

URDF = "src/erc_description/urdf/tiago_pro.urdf"

JOINTS = ["gripper_left_finger_joint", "gripper_left_finger_right_joint"]
for _side in ("left", "right"):
    for _part in ("outer_finger", "inner_finger", "fingertip"):
        JOINTS.append("gripper_left_%s_%s_joint" % (_part, _side))


def blocks(text):
    for name in JOINTS:
        m = re.search(r'<joint name="%s" type="[a-z]+">.*?</joint>' % re.escape(name),
                      text, re.S)
        if m:
            yield name, m.group(0)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    arg = sys.argv[1]

    if arg == "--restore":
        import subprocess
        subprocess.run(["git", "checkout", "--", URDF], check=True)
        print("restored %s from git" % URDF)
        return 0

    text = io.open(URDF, encoding="utf-8").read()

    if arg == "--show":
        for name, block in blocks(text):
            m = re.search(r'effort="([0-9.eE+-]+)"', block)
            print("  %-42s effort=%s" % (name, m.group(1) if m else "?"))
        return 0

    factor = float(arg)
    changed = 0
    for name, block in blocks(text):
        new = re.sub(r'effort="([0-9.eE+-]+)"',
                     lambda m: 'effort="%g"' % (float(m.group(1)) * factor),
                     block, count=1)
        if new != block:
            text = text.replace(block, new, 1)
            changed += 1
    io.open(URDF, "w", encoding="utf-8", newline="\n").write(text)
    print("scaled %d left-gripper effort limits by %g" % (changed, factor))
    for name, block in blocks(text):
        m = re.search(r'effort="([0-9.eE+-]+)"', block)
        print("  %-42s effort=%s" % (name, m.group(1) if m else "?"))
    return 0


sys.exit(main())
