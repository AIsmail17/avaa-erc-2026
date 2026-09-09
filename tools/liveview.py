#!/usr/bin/env python3
"""Watch the simulation live in a browser tab.

    tools/in-sim liveview.py            # then open http://localhost:8080
    tools/in-sim liveview.py 8090       # on a different port

Why this exists rather than the Gazebo window
---------------------------------------------
Because on this machine the Gazebo window is drawn correctly and Windows never shows it.

That is worth stating precisely, because the explanation carried here until 2026-09-09
was wrong in a way that stopped anyone fixing it. It read: no /dev/dri in WSL, so Mesa
falls back to software GLX, the drisw path fails, and "Failed to create OpenGL context"
kills the GUI. Measured inside the container, with no environment variables set at all:

    direct rendering: Yes
    Device: D3D12 (NVIDIA RTX A4500 Laptop GPU)
    Max core profile version: 4.2

WSLg routes GL through /dev/dxg to the real GPU, and gz sim -g renders the arena on it:
an X11 capture of the window shows the shelf, the books, the bin and the entity tree, at
about 49% of real time.

What fails is the last step, getting those pixels onto the Windows desktop. WSLg is in
its RAIL fallback -- every window title carries [WARN:COPY MODE] and /mnt/shared_memory
does not exist -- which is microsoft/wslg#1456, open, no fix. The application renders;
the compositor never presents it.

So the picture has to leave the container some other way, and HTTP is a fine way. This
serves MJPEG: the camera sensors the server is already rendering through EGL, and, where
ImageMagick is installed, the Gazebo window itself, captured from X11 where it is
perfectly intact.

It serves three views:

    spectator   a fixed camera placed in the world, watching the robot from above and
                to one side -- the view you want in order to see what the robot is doing
    head        the robot's own camera, which is what perception actually sees
    gui         the Gazebo window, captured from X11 -- the model tree, the controls
                and the free camera, exactly as drawn, just relayed

The spectator camera is spawned at runtime through the world's create service, so no
supplied file is touched and it disappears when the simulation restarts.
"""
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import Image

GUI_WINDOW_NAME = "Gazebo Sim"
GUI_FPS = 4.0

SPECTATOR_TOPIC = "/spectator/image"
HEAD_TOPIC = "/head_front_camera/head_front_camera/color/image_raw"

# Camera topics are best effort. A reliable subscription to them receives nothing at
# all -- not fewer frames, none -- which cost an afternoon the first time.
SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        durability=QoSDurabilityPolicy.VOLATILE,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=1)

PAGE = b"""<!doctype html>
<title>AVAA - live simulation</title>
<style>
  body { background:#111; color:#ddd; font:14px system-ui,sans-serif; margin:0; padding:16px; }
  h1 { font-size:15px; font-weight:600; margin:0 0 12px; letter-spacing:.02em; }
  .row { display:flex; flex-wrap:wrap; gap:16px; }
  figure { margin:0; }
  figcaption { padding:6px 2px; color:#8b8b8b; font-size:12px; }
  img { display:block; background:#000; border:1px solid #333; max-width:100%; height:auto; }
</style>
<h1>AVAA &mdash; Emirates Robotics Competition, live from Gazebo</h1>
<div class="row">
  <figure>
    <img src="/spectator" width="720">
    <figcaption>Spectator camera &mdash; watching the robot</figcaption>
  </figure>
  <figure>
    <img src="/head" width="480">
    <figcaption>Robot head camera &mdash; what perception sees</figcaption>
  </figure>
</div>
<div class="row" style="margin-top:16px">
  <figure>
    <img src="/gui" width="1000">
    <figcaption>The Gazebo window itself &mdash; relayed out of X11, because WSLg
      will not put it on the Windows desktop (microsoft/wslg#1456)</figcaption>
  </figure>
</div>
"""


class Frames(Node):
    def __init__(self):
        super().__init__("avaa_liveview")
        self.latest = {"spectator": None, "head": None}
        self.count = {"spectator": 0, "head": 0}
        self.lock = threading.Lock()
        self.create_subscription(
            Image, SPECTATOR_TOPIC,
            lambda m: self._store("spectator", m), SENSOR_QOS)
        self.create_subscription(
            Image, HEAD_TOPIC, lambda m: self._store("head", m), SENSOR_QOS)

    def _store(self, which, msg):
        try:
            frame = np.frombuffer(msg.data, np.uint8).reshape(
                msg.height, msg.width, 3)
        except ValueError:
            return
        # Published RGB, and OpenCV encodes BGR.
        ok, jpeg = cv2.imencode(".jpg", frame[:, :, ::-1],
                                [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return
        with self.lock:
            self.latest[which] = jpeg.tobytes()
            self.count[which] += 1

    def put(self, which, jpeg):
        with self.lock:
            self.latest[which] = jpeg

    def get(self, which):
        with self.lock:
            return self.latest.get(which)


PLACEHOLDER = None


def gui_window_id():
    """The X11 id of the Gazebo window, or None if there is no window to capture.

    Looked up every time rather than cached: the GUI is attached and detached freely
    while this server keeps running, and a stale id captures nothing without saying so.
    """
    try:
        tree = subprocess.run(["xwininfo", "-root", "-tree"],
                              capture_output=True, text=True, timeout=10).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    for line in tree.splitlines():
        if '"%s"' % GUI_WINDOW_NAME in line:
            word = line.strip().split()[0]
            if word.startswith("0x"):
                return word
    return None


def grab_gui(frames):
    """Relay the Gazebo window into the stream.

    ImageMagick's import reads the window's own pixels out of the X server, so this does
    not care whether the window is on top, behind a browser, or -- as it is here -- never
    presented to Windows at all. It is a copy and a process per frame, which is why it
    runs at a few frames a second rather than thirty.
    """
    if subprocess.run(["which", "import"], capture_output=True).returncode != 0:
        frames.put("gui", placeholder("no ImageMagick: apt-get install imagemagick"))
        return
    period = 1.0 / GUI_FPS
    while True:
        started = time.time()
        window = gui_window_id()
        if window is None:
            frames.put("gui", placeholder("no Gazebo window -- run: tools/sim gui"))
            time.sleep(2.0)
            continue
        try:
            shot = subprocess.run(["import", "-silent", "-window", window, "jpg:-"],
                                  capture_output=True, timeout=15)
        except subprocess.SubprocessError:
            time.sleep(1.0)
            continue
        if shot.returncode == 0 and shot.stdout:
            frames.put("gui", shot.stdout)
        else:
            frames.put("gui", placeholder("could not read the Gazebo window"))
            time.sleep(1.0)
        time.sleep(max(0.0, period - (time.time() - started)))


def placeholder(text):
    image = np.full((360, 640, 3), 24, np.uint8)
    cv2.putText(image, text, (24, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (180, 180, 180), 1, cv2.LINE_AA)
    return cv2.imencode(".jpg", image)[1].tobytes()


class Handler(BaseHTTPRequestHandler):
    frames = None

    def log_message(self, *args):
        pass  # the console belongs to the simulation, not to every GET

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(PAGE)))
            self.end_headers()
            self.wfile.write(PAGE)
            return

        which = self.path.strip("/")
        if which not in ("spectator", "head", "gui"):
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        waiting = placeholder("waiting for %s frames..." % which)
        try:
            while True:
                jpeg = self.frames.get(which) or waiting
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                 b"Content-Length: " + str(len(jpeg)).encode()
                                 + b"\r\n\r\n" + jpeg + b"\r\n")
                time.sleep(0.06)
        except (BrokenPipeError, ConnectionResetError):
            return  # the tab was closed, which is not an error


def bridge_spectator():
    """Bring the spectator camera's pictures across into ROS.

    The camera is spawned into the world at runtime, so it publishes on a GAZEBO topic
    and nothing carries it into ROS -- the simulation launch bridges a fixed list, and a
    camera that did not exist when it started is not on it. gz topic -l shows
    /spectator/image, ros2 topic list shows nothing, and the page sits on its placeholder
    looking like the camera failed.

    Started here rather than left to the caller, so that one command produces a working
    view. Harmless if a bridge is already running.
    """
    listed = subprocess.run(
        ["gz", "topic", "-l"], capture_output=True, text=True, timeout=25).stdout
    if SPECTATOR_TOPIC.lstrip("/") not in listed:
        print("no spectator camera in the world; run tools/in-sim spectator.py first",
              flush=True)
        return None
    print("bridging %s into ROS" % SPECTATOR_TOPIC, flush=True)
    return subprocess.Popen(
        ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
         "%s@sensor_msgs/msg/Image[gz.msgs.Image" % SPECTATOR_TOPIC],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    bridge = bridge_spectator()
    rclpy.init()
    node = Frames()

    thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    thread.start()

    threading.Thread(target=grab_gui, args=(node,), daemon=True).start()

    Handler.frames = node
    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    print("live view on http://localhost:%d" % port, flush=True)
    print("(from Windows, open that in a browser -- WSL forwards localhost)",
          flush=True)
    print("streams: /spectator, /head and /gui", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if bridge is not None:
            bridge.terminate()
        node.destroy_node()
        rclpy.shutdown()


main()
