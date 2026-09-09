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

    python3 spectator.py follow     # aimed at wherever the robot is now
    python3 spectator.py            # the fixed wide view of the room
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


def robot_pose():
    """Where the robot is, or None. Used to aim the camera at it rather than at a spot."""
    out = subprocess.run(["gz", "model", "-m", "tiago_pro", "-p"],
                         capture_output=True, text=True, timeout=25).stdout
    lines = [l.strip() for l in out.splitlines()]
    for i, line in enumerate(lines):
        if line.startswith("[") and i + 1 < len(lines) and lines[i + 1].startswith("["):
            try:
                return [float(v) for v in line.strip("[]").split()]
            except ValueError:
                return None
    return None


def framing_for(where, back=2.6, side=-2.2, up=1.9, look_at_z=0.85):
    """A camera pose that puts the robot in the middle of the picture.

    Aiming at a fixed spot in the room was the whole problem with this tool. The robot
    drives four metres during a trial, so any fixed aim is right for one moment of it:
    every framing tried by hand had the robot at the edge of the frame, half cut off, or
    out of it entirely, and each attempt cost a spawn, a bridge and a look.

    Aimed at the robot instead, one command always gives a usable shot. The offsets put
    the camera behind and to the robot's right, high enough to see over the base and
    down into the shelf it is working at.
    """
    x, y = where[0], where[1]
    return (x - back, y + side, up), (x, y, look_at_z)


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
    if len(sys.argv) >= 2 and sys.argv[1] == "follow":
        where = robot_pose()
        if where is None:
            print("cannot read the robot's pose; falling back to the fixed framing")
            eye, target = (1.05, -4.30, 2.85), (2.10, 0.00, 0.40)
        else:
            back = float(sys.argv[2]) if len(sys.argv) > 2 else 2.6
            eye, target = framing_for(where, back=back)
            print("following the robot at [%.2f, %.2f]" % (where[0], where[1]))
    elif len(sys.argv) >= 7:
        if len(sys.argv) >= 8:
            fov = float(sys.argv[7])
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

    # Bridge it into ROS, or nothing can see it.
    #
    # Creating the camera is only half the job. Gazebo publishes on its own transport and
    # tools/liveview.py subscribes in ROS, so without a bridge the topic exists, carries
    # frames -- measured at 148 in fifteen seconds -- and the viewer sits on "waiting for
    # spectator frames" for ever. That is what a frozen spectator view actually is, and
    # it is why the robot's head camera looked fine beside it: the head camera is bridged
    # by the launch and this one never was.
    # Always replace the bridge. Never keep one.
    #
    # This used to skip when a bridge was already running, and that is the wrong way
    # round: a bridge belongs to the Gazebo session it was started against, and once the
    # simulation restarts the old one is attached to a transport that no longer exists.
    # It keeps its ROS topic advertised and simply never publishes again, so the viewer
    # holds its last frame for ever.
    #
    # Measured: eleven frames pulled from the stream over eight minutes, every one of
    # them 25647 bytes -- the same picture, byte for byte, while the robot was moving.
    # That is exactly what "the spectator view is frozen" looks like, and there is no
    # error anywhere to find, because nothing failed.
    subprocess.run(["bash", "-c", "pkill -f 'parameter_bridge.*spectator'; sleep 1"],
                   capture_output=True, text=True)

    subprocess.Popen(
        ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
         "/%s@sensor_msgs/msg/Image[gz.msgs.Image" % TOPIC.lstrip("/"),
         "--ros-args", "-p", "use_sim_time:=true"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)
    print("bridging /%s into ROS" % TOPIC.lstrip("/"))


if __name__ == "__main__":
    main()
