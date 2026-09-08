#!/usr/bin/env bash
# Does the fingertip contact sensor work at all?
#
# jawtest reports "the pads felt nothing at all" while closing on a book. That has two
# readings and only one of them is about the grasp: the pads really never touched it, or
# the sensor never reports anything and the measurement is worthless. This checks the
# instrument before trusting it, by driving the book straight INTO a pad -- overlapping
# it, not near it -- where a working sensor cannot stay quiet.
#
# One invocation on purpose: on this workstation WSL shuts down whenever no command is
# running and takes the container with it.
set -u
cd ~/erc/erc_sim_2026

LINK=gripper_left_fingertip_left_link
TOPIC="/world/erc_world/model/tiago_pro/link/$LINK/sensor/${LINK}_contact/contact"

timeout 220 ./tools/lab up dart 2>&1 | grep -E "real-time|mimic"
sleep 15

echo
echo "=== is anyone publishing on the pad's contact topic?"
docker exec erc_sim /entrypoint.sh bash -c "timeout 12 gz topic -l 2>/dev/null | grep -c contact" \
    | sed 's/^/  contact topics in the world: /'
docker exec erc_sim /entrypoint.sh bash -c "timeout 12 gz topic -i -t '$TOPIC' 2>&1 | head -6" \
    | sed 's/^/  /'

echo
echo "=== where is that pad?"
POSE=$(docker exec erc_sim /entrypoint.sh bash -c \
  "source /opt/erc_ws/install/setup.bash && timeout 15 ros2 run tf2_ros tf2_echo base_link $LINK 2>/dev/null | grep -A1 Translation | head -2")
echo "$POSE" | sed 's/^/  /'

echo
echo "=== watching the pad, then driving the book into it"
docker exec -d erc_sim /entrypoint.sh bash -c \
    "timeout 60 gz topic -e -t '$TOPIC' > /tmp/sensorcheck.txt 2>&1"
sleep 3

# The pad's world position, composed the same way jawtest does it, then the book put
# exactly there so the two overlap.
docker exec -i erc_sim /entrypoint.sh bash -c \
  "source /opt/erc_ws/install/setup.bash && python3 -" <<'PY'
import math, subprocess, time
import rclpy
from tf2_ros import Buffer, TransformListener

LINK = "gripper_left_fingertip_left_link"
rclpy.init()
node = rclpy.create_node("sensorcheck")
buf = Buffer(); TransformListener(buf, node)
for _ in range(80):
    rclpy.spin_once(node, timeout_sec=0.1)

raw = subprocess.run(["gz","topic","-e","-t","/world/erc_world/dynamic_pose/info","-n","1"],
                     capture_output=True, text=True, timeout=20).stdout
poses, name, section, fields = {}, None, None, {}
for line in raw.splitlines():
    line = line.strip()
    if line.startswith('name: "'):
        if name and "px" in fields: poses[name] = dict(fields)
        name, section, fields = line.split('"')[1], None, {}
    elif line.startswith("position"): section = "p"
    elif line.startswith("orientation"): section = "q"
    elif name and section and ":" in line:
        k,_,v = line.partition(":"); k = k.strip()
        if k in ("x","y","z","w") and section+k not in fields:
            try: fields[section+k] = float(v)
            except ValueError: pass
if name and "px" in fields: poses[name] = dict(fields)

r = poses["tiago_pro"]
yaw = math.atan2(2*(r["qw"]*r["qz"]+r["qx"]*r["qy"]), 1-2*(r["qy"]**2+r["qz"]**2))
t = buf.lookup_transform("base_link", LINK, rclpy.time.Time()).transform.translation
x = r["px"] + t.x*math.cos(yaw) - t.y*math.sin(yaw)
y = r["py"] + t.x*math.sin(yaw) + t.y*math.cos(yaw)
z = r["pz"] + t.z
book = next(k for k in poses if k.startswith("book_"))
print("  pad at (%.3f, %.3f, %.3f); driving %s onto it" % (x, y, z, book))
for i in range(40):
    subprocess.run(["gz","service","-s","/world/erc_world/set_pose",
        "--reqtype","gz.msgs.Pose","--reptype","gz.msgs.Boolean","--timeout","500",
        "--req",'name: "%s", position: {x: %f, y: %f, z: %f}' % (book, x, y, z)],
        capture_output=True, timeout=3)
    time.sleep(0.25)
print("  held the book inside the pad for ten seconds")
PY

sleep 3
docker exec erc_sim bash -c 'pkill -f "gz topic -e"' 2>/dev/null || true
echo
echo "=== what the sensor said"
lines=$(docker exec erc_sim bash -c 'wc -l < /tmp/sensorcheck.txt' 2>/dev/null | tr -d "\r")
echo "  $lines lines of output"
docker exec erc_sim bash -c 'head -25 /tmp/sensorcheck.txt' 2>/dev/null | sed 's/^/  /'
