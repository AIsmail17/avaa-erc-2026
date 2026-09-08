#!/usr/bin/env python3
"""Switch the left gripper between position and effort command, and back.

    tools/grippereffort.py --on       effort command interface + PID gains
    tools/grippereffort.py --off      back to what git has
    tools/grippereffort.py --show     which mode the files are in

Runs on the HOST. It edits the URDF and the controller params in src/, both of which are
mounted into the container, so the next bench launch picks the change up with no rebuild.

Why
---
This is the configuration the working reference uses. gz_ros2_control ships a parallel
gripper with a mimic joint that grasps, and the difference from ours is one line:

    ours   <joint name="gripper_left_finger_joint"><command_interface name="position"/>
    demo   <joint name="right_finger_joint">       <command_interface name="effort"/>

(gz_ros2_control_demos/urdf/test_gripper_mimic_joint_effort.xacro.urdf, and the launch
file gripper_mimic_joint_example_effort.launch.py that drives it.)

The two paths through gz_system.cpp are not comparable. A position command becomes
"move at this velocity" -- JointVelocityCmd -- and the engine spends whatever force that
costs, up to the joint's effort limit, which here is 10 N against a book that needs 0.3 N
to hold. An effort command is applied to the joint as it is: the controller asks for a
squeeze of a particular strength and the physics resolves what that does to whatever is
between the pads. That is what a grasp needs.

The solution does not change. grasp_node keeps publishing position trajectories to
gripper_left_controller_raw; the trajectory controller now converts them to effort through
a PID, so nothing above the controller knows or cares.
"""
import io
import re
import subprocess
import sys

URDF = "src/erc_description/urdf/tiago_pro.urdf"
PARAMS = "src/erc_bringup/config/controller_params.yaml"

EFFORT_BLOCK = """gripper_left_controller_raw:
  ros__parameters:
    joints: [gripper_left_finger_joint]
    command_interfaces: [effort]
    state_interfaces: [position, velocity]

    # Effort command means the trajectory controller has to close the loop itself, so it
    # needs gains where the position path needed none. These are a starting point, not a
    # tuned answer: p is what decides how hard the jaws squeeze once they are on the book.
    gains:
      gripper_left_finger_joint:
        p: 50.0
        i: 0.0
        d: 1.0
        i_clamp: 1.0
"""

POSITION_BLOCK = """gripper_left_controller_raw:
  ros__parameters:
    joints: [gripper_left_finger_joint]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
"""


def read(path):
    return io.open(path, encoding="utf-8").read()


def write(path, text):
    io.open(path, "w", encoding="utf-8", newline="\n").write(text)


def urdf_mode(text):
    m = re.search(
        r'<joint name="gripper_left_finger_joint">\s*<command_interface name="(\w+)"',
        text)
    return m.group(1) if m else "unknown"


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else "--show"

    if arg == "--show":
        print("  URDF   gripper_left_finger_joint command interface: %s"
              % urdf_mode(read(URDF)))
        params = read(PARAMS)
        m = re.search(r"gripper_left_controller_raw:.*?command_interfaces: \[(\w+)\]",
                      params, re.S)
        print("  params gripper_left_controller_raw command_interfaces: [%s]"
              % (m.group(1) if m else "?"))
        return 0

    if arg == "--off":
        subprocess.run(["git", "checkout", "--", URDF, PARAMS], check=True)
        print("restored both files from git (position command)")
        return 0

    if arg != "--on":
        print(__doc__)
        return 1

    # ---- URDF: the one line that matters -----------------------------------------
    text = read(URDF)
    if urdf_mode(text) == "effort":
        print("URDF already in effort mode")
    else:
        old = ('<joint name="gripper_left_finger_joint">\n'
               '      <command_interface name="position"/>')
        new = ('<joint name="gripper_left_finger_joint">\n'
               '      <command_interface name="effort"/>')
        if text.count(old) != 1:
            print("could not find the left finger command interface (%d matches)"
                  % text.count(old))
            return 1
        write(URDF, text.replace(old, new, 1))
        print("URDF   gripper_left_finger_joint -> effort command interface")

    # ---- controller params -------------------------------------------------------
    params = read(PARAMS)
    if "command_interfaces: [effort]" in params:
        print("params already in effort mode")
        return 0
    if POSITION_BLOCK not in params:
        print("controller_params.yaml does not look the way this expects; not touching it")
        return 1
    write(PARAMS, params.replace(POSITION_BLOCK, EFFORT_BLOCK, 1))
    print("params gripper_left_controller_raw -> effort, with PID gains")
    return 0


sys.exit(main())
