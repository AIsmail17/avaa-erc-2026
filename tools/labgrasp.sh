#!/usr/bin/env bash
# Run the REAL grasp controller against one book, on the bench.
#
#     tools/labgrasp.sh [book_z]        default 1.21, which puts it on row 1
#
# The arena costs thirteen minutes to reach the grasp and spends eleven of them on an
# approach that is understood. This brings up the lab world with move_group, starts
# avaa_solution's own grasp controller, hands it a row and a book point taken from
# Gazebo's ground truth, and reports where the pads ended up relative to the book face.
#
# The lab world has NO SHELF. That is the experiment: if a reach that stops short in the
# arena completes here, the shelf collision geometry is what stops it.
#
# One invocation start to finish, because WSL shuts down whenever no command is running
# and takes the container with it.
set -u
cd ~/erc/erc_sim_2026

BOOK_Z="${1:-1.21}"
FEED_SECONDS="${FEED_SECONDS:-240}"

echo "=== bench up, with move_group, book at z=$BOOK_Z"
ERC_LAB_Z="$BOOK_Z" MOVEIT=1 timeout 260 ./tools/lab up dart moveit 2>&1 \
    | grep -E "real-time|mimic|controllers"

# Say whether the grasp aid is running, rather than assuming it from the environment.
# A run was reported as "with ERC_GRASP_FIX=1" and the book still fell 559 mm, and there
# was no way to tell from the output whether the aid had engaged and failed or had never
# started at all.
echo
if [ "${ERC_GRASP_FIX:-0}" = "1" ]; then
    if docker exec erc_sim pgrep -f "[s]im_grasp_fix" >/dev/null 2>&1; then
        echo "=== the simulation grasp aid IS running"
    else
        echo "=== ERC_GRASP_FIX=1 was asked for but sim_grasp_fix is NOT running"
    fi
else
    echo "=== running WITHOUT the simulation grasp aid (ERC_GRASP_FIX is not 1)"
fi

echo
echo "=== waiting for move_group"
for i in $(seq 1 30); do
    if docker exec erc_sim /entrypoint.sh bash -c \
        'source /opt/erc_ws/install/setup.bash && timeout 5 ros2 node list 2>/dev/null' \
        | grep -q move_group; then
        echo "  move_group up after $((i * 4)) s"
        break
    fi
    sleep 4
done

echo
echo "=== putting the arm in the driving posture first"
# The arena does this during the approach; the bench has no approach controller, so the
# left arm sits where it spawned -- all-zero, which is fully extended, 0.838 m forward and
# well inside the shelf boxes grasp_node is about to add. Every plan then starts from a
# state move_group refuses, and the run fails for a reason that has nothing to do with
# the grasp. TUCK_POSE and TUCK_TORSO are read from the solution so this cannot drift
# away from what the arena actually uses.
docker exec erc_sim /entrypoint.sh bash -c '
source /opt/erc_ws/install/setup.bash
python3 - <<PY
import sys, time
sys.path.insert(0, "/opt/erc_ws/src/avaa_solution")
import rclpy
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from avaa_solution.grasp_node import TUCK_POSE, TUCK_TORSO

rclpy.init()
node = rclpy.create_node("labtuck")
arm = node.create_publisher(JointTrajectory, "/arm_left_controller/joint_trajectory", 10)
torso = node.create_publisher(JointTrajectory, "/torso_controller/joint_trajectory", 10)
time.sleep(2.0)

def send(pub, names, values, secs):
    t = JointTrajectory()
    t.joint_names = list(names)
    p = JointTrajectoryPoint()
    p.positions = [float(v) for v in values]
    p.velocities = [0.0] * len(values)
    p.time_from_start = Duration(sec=secs, nanosec=0)
    t.points = [p]
    for _ in range(3):
        pub.publish(t)
        time.sleep(0.2)

send(torso, ["torso_lift_joint"], [TUCK_TORSO], 6)
send(arm, ["arm_left_%d_joint" % i for i in range(1, 8)], TUCK_POSE, 8)
print("  sent TUCK_POSE and torso %.2f" % TUCK_TORSO)
node.destroy_node()
rclpy.shutdown()
PY'
sleep 25

echo
echo "=== pinning the base"
# The base will not stay put on its own, and that is not a bench artifact.
#
# The mecanum wheels are modelled with mu2 = 0 across the roller axis, so there is no
# friction in one direction at all, and the reaction from the reaching arm turns the whole
# robot. Measured here twice with the base holder switched off: the base finished at yaw
# -88 and -96 degrees, having started square, which put the book 775 mm to the side in
# base_link and made every arrival number meaningless.
#
# tools/pin_base.py already existed for exactly this and its docstring carries the
# measurement from a real grasp: 19 mm of error growing to 263 mm, and 2 degrees to 25,
# while the wheel-driven holder was commanding full correction the whole way. It is a
# fixture, not a fix -- a real robot cannot teleport -- and its only job is to take base
# drift out of an experiment about the arm.
BASE=$(docker exec erc_sim /entrypoint.sh bash -c \
    'timeout 15 gz topic -e -t /world/erc_world/dynamic_pose/info -n 1 2>/dev/null' \
    | python3 -c '
import math, sys
name, section, fields, out = None, None, {}, {}
for line in sys.stdin:
    line = line.strip()
    if line.startswith("name: \""):
        if name and "px" in fields:
            out[name] = dict(fields)
        name, section, fields = line.split("\"")[1], None, {}
    elif line.startswith("position"):
        section = "p"
    elif line.startswith("orientation"):
        section = "q"
    elif name and section and ":" in line:
        k, _, v = line.partition(":")
        k = k.strip()
        if k in ("x", "y", "z", "w") and section + k not in fields:
            try:
                fields[section + k] = float(v)
            except ValueError:
                pass
if name and "px" in fields:
    out[name] = dict(fields)
r = out.get("tiago_pro")
if r is None:
    print("")
else:
    yaw = math.atan2(2*(r["qw"]*r["qz"] + r["qx"]*r["qy"]),
                     1 - 2*(r["qy"]**2 + r["qz"]**2))
    print("%.4f %.4f %.2f" % (r["px"], r["py"], math.degrees(yaw)))
')
if [ -n "$BASE" ]; then
    echo "  holding the base at $BASE"
    docker exec -i erc_sim bash -c 'cat > /tmp/pin_base.py' < "$PWD/tools/pin_base.py"
    docker exec -d erc_sim /entrypoint.sh bash -c \
        "source /opt/erc_ws/install/setup.bash && \
         python3 /tmp/pin_base.py $((FEED_SECONDS + 200)) $BASE > /tmp/pinbase.log 2>&1"
    sleep 3
else
    echo "  could not read the base pose; it will drift"
fi

echo
echo "=== starting the grasp controller"
docker exec -d erc_sim /entrypoint.sh bash -c \
    "source /opt/erc_ws/install/setup.bash && ros2 run avaa_solution grasp --ros-args \
     -p use_sim_time:=true -p hold_base:=false > /tmp/labgrasp.log 2>&1"
sleep 8

echo
echo "=== feeding it the book"
timeout 900 ./tools/in-sim bookfeed.py "$FEED_SECONDS" 2>&1 | tail -40

echo
echo "=== what the grasp aid did"
docker exec erc_sim bash -c \
    'grep -iE "grasp fix|picked up|released|jaws closed|commanded to" /tmp/lab.log 2>/dev/null | tail -12' \
    2>/dev/null | sed 's/\x1b\[[0-9;]*m//g' | sed 's/^/  /'

echo
echo "=== what the controller said about the reach"
docker exec erc_sim bash -c \
    'grep -iE "reach|fraction|arriv|short|blocked|abandon|returned|raised|stow|-> " /tmp/labgrasp.log | tail -28' \
    2>/dev/null | sed 's/\x1b\[[0-9;]*m//g' | sed 's/^/  /'
