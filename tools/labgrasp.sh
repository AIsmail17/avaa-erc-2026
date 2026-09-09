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
echo "=== starting the grasp controller"
docker exec -d erc_sim /entrypoint.sh bash -c \
    "source /opt/erc_ws/install/setup.bash && ros2 run avaa_solution grasp --ros-args \
     -p use_sim_time:=true > /tmp/labgrasp.log 2>&1"
sleep 8

echo
echo "=== feeding it the book"
timeout 900 ./tools/in-sim bookfeed.py "$FEED_SECONDS" 2>&1 | tail -40

echo
echo "=== what the controller said about the reach"
docker exec erc_sim bash -c \
    'grep -iE "reach|fraction|arriv|short|blocked|abandon|-> " /tmp/labgrasp.log | tail -25' \
    2>/dev/null | sed 's/\x1b\[[0-9;]*m//g' | sed 's/^/  /'
