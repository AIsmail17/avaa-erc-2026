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
  h1 { font-size:15px; font-weight:600; margin:0 0 4px; letter-spacing:.02em; }
  p { color:#8b8b8b; font-size:12px; margin:0 0 12px; }
  a { color:#7aa7d8; }
  img { display:block; background:#000; border:1px solid #333; max-width:100%; height:auto; }
</style>
<h1>AVAA &mdash; Emirates Robotics Competition, live from Gazebo</h1>
<p>One stream, three panels. Separate streams if you want them:
   <a href="/gui">/gui</a> &middot;
   <a href="/spectator">/spectator</a> &middot;
   <a href="/head">/head</a></p>
<img src="/all" width="1280">
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


def placeholder_image(text):
    image = np.full((360, 640, 3), 24, np.uint8)
    cv2.putText(image, text, (24, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (180, 180, 180), 1, cv2.LINE_AA)
    return image


def placeholder(text):
    return cv2.imencode(".jpg", placeholder_image(text))[1].tobytes()


def compose(frames, width=1280):
    """One picture with all three views in it.

    Three <img> tags pointed at three MJPEG endpoints is three HTTP connections that never
    end, and in practice one of them wins and the others sit blank -- reported as "only one
    view working", and reproducible by reloading the page, which leaves the old streams
    holding their sockets until they time out. A composite is one connection, so there is
    nothing to lose a race with, and every panel is from the same instant.
    """
    gui = frames.get("gui")
    spec = frames.get("spectator")
    head = frames.get("head")

    def decode(jpeg, fallback_text):
        if jpeg is None:
            return placeholder_image(fallback_text)
        image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        return image if image is not None else placeholder_image(fallback_text)

    def fit(image, w, h):
        out = np.zeros((h, w, 3), np.uint8)
        scale = min(w / image.shape[1], h / image.shape[0])
        small = cv2.resize(image, (max(1, int(image.shape[1] * scale)),
                                   max(1, int(image.shape[0] * scale))))
        y = (h - small.shape[0]) // 2
        x = (w - small.shape[1]) // 2
        out[y:y + small.shape[0], x:x + small.shape[1]] = small
        return out

    def label(image, text):
        cv2.rectangle(image, (0, 0), (image.shape[1], 22), (20, 20, 20), -1)
        cv2.putText(image, text, (8, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (210, 210, 210), 1, cv2.LINE_AA)
        return image

    half = width // 2
    top = label(fit(decode(gui, "no Gazebo window"), width, int(width * 0.60)),
                "Gazebo window (relayed from X11)")
    left = label(fit(decode(spec, "no spectator camera"), half, int(half * 0.62)),
                 "Spectator camera")
    right = label(fit(decode(head, "no head camera"), width - half, int(half * 0.62)),
                  "Robot head camera")
    bottom = np.hstack([left, right])
    return cv2.imencode(".jpg", np.vstack([top, bottom]),
                        [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()


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
        if which not in ("spectator", "head", "gui", "all"):
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
                if which == "all":
                    jpeg = compose(self.frames)
                    pause = 0.25
                else:
                    jpeg = self.frames.get(which) or waiting
                    pause = 0.06
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                 b"Content-Length: " + str(len(jpeg)).encode()
                                 + b"\r\n\r\n" + jpeg + b"\r\n")
                time.sleep(pause)
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
    print("streams: /all (everything, one connection), /spectator, /head, /gui",
          flush=True)
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
