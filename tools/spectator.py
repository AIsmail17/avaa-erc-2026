#!/usr/bin/env python3
"""Spawn a fixed camera that watches the robot work at the shelf.

There is no way to look at this simulation. The Gazebo GUI cannot get an OpenGL
context in the container, and on the host WSL drops the virtual GPU
("D3D12: Removing Device"). The robot's own head camera is not a substitute: it
looks straight at the shelf, so the arm entering from the side barely appears, and
every wobble of the base swings the whole picture.

The headless server still renders sensors through EGL. So this puts an ordinary
camera in the world, off to one side and above, pointed at the grasp. Spawned at
runtime through the create service, so no supplied file is touched and it vanishes
when the simulation restarts.

    python3 spectator.py            # default view of column 3
    python3 spectator.py 1.6 -1.8 1.9 2.75 0.0 1.15        # eye, then target
    python3 spectator.py 1.6 -1.8 1.9 2.75 0.0 1.15 0.9    # ...and a longer lens
"""
import math
import subprocess
import sys

NAME = "spectator_cam"
TOPIC = "spectator/image"


def look_at(eye, target):
    """Yaw and pitch that point a camera's +X axis from eye to target."""
    dx, dy, dz = (t - e for t, e in zip(target, eye))
    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(dz, math.hypot(dx, dy))
    return pitch, yaw


def sdf(eye, pitch, yaw, width=1280, height=720, fov=1.25):
    return f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{NAME}">
    <static>true</static>
    <pose>{eye[0]} {eye[1]} {eye[2]} 0 {pitch} {yaw}</pose>
    <link name="link">
      <sensor name="cam" type="camera">
        <topic>{TOPIC}</topic>
        <update_rate>15</update_rate>
        <always_on>1</always_on>
        <camera>
          <horizontal_fov>{fov}</horizontal_fov>
          <image><width>{width}</width><height>{height}</height><format>R8G8B8</format></image>
          <clip><near>0.1</near><far>50</far></clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>"""


def main():
    fov = 1.25
    if len(sys.argv) >= 8:
        fov = float(sys.argv[7])
    if len(sys.argv) >= 7:
        eye = tuple(float(v) for v in sys.argv[1:4])
        target = tuple(float(v) for v in sys.argv[4:7])
    else:
        # A stage view, not a close-up.
        #
        # The first framing looked from off the robot's shoulder straight at the shelf,
        # which put the shelf behind the subject and filling most of the picture: good
        # for watching fingers close on a book, useless for watching a robot drive,
        # because it leaves frame within a couple of metres and the shelf hides it when
        # it does not. This looks along the shelf's normal instead, from the open side,
        # far enough back that the whole working area is in shot -- the start zone, the
        # collection bin at x=-1, the robot, and the full width of the shelf at x=2.9 --
        # with the robot between the camera and the shelf rather than against it.
        eye, target = (1.05, -4.30, 2.85), (2.10, 0.00, 0.40)

    pitch, yaw = look_at(eye, target)
    print("camera at %s looking at %s (pitch %.3f, yaw %.3f)"
          % (eye, target, pitch, yaw))

    # Remove any previous one, so this can be re-run to reposition the view.
    subprocess.run(["gz", "service", "-s", "/world/erc_world/remove",
                    "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean",
                    "--timeout", "2000",
                    "--req", 'name: "%s", type: MODEL' % NAME],
                   capture_output=True, text=True)

    out = subprocess.run(
        ["gz", "service", "-s", "/world/erc_world/create",
         "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
         "--timeout", "5000",
         "--req", 'sdf: %r' % sdf(eye, pitch, yaw, fov=fov)],
        capture_output=True, text=True, timeout=30)
    print("create said:", (out.stdout or out.stderr).strip()[:200])


if __name__ == "__main__":
    main()
