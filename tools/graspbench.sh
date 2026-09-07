#!/usr/bin/env bash
# Test the grasp on its own, without the approach's search and centring in front of it.
#
#     tools/graspbench.sh [colour] [standoff]
#
# A full run costs thirteen minutes and spends eleven on an approach that is already
# understood, to reach a grasp that is not.
#
# The order below is not arbitrary and the first version got it wrong. Driving straight
# to grasping range and then starting perception gives the grasp nothing to work with:
# from 0.68 m the column marker at 2.26 m sits 55 degrees above the camera axis, and the
# vertical field of view is 28. The marker CANNOT be seen from where the arm works, which
# is why the real pipeline identifies the column from four metres out and carries the
# identification in on the book tracker.
#
# So: stand back, let perception name the column, and only then close in -- with
# perception running throughout, exactly as it is during a real approach. Driving, never
# teleporting: one set_pose takes the real-time factor from 0.48 to 0.04 for good.
set -u
COLOUR="${1:-red}"
STANDOFF="${2:-0.65}"
FAR="${3:-3.0}"
cd ~/erc/erc_sim_2026

if docker exec erc_sim bash -c 'ps -eo args | grep -v grep | grep -q "[r]os2 launch avaa_solution solution.launch.py"'; then
    echo "a solution launch is already running; refusing to start another"
    exit 2
fi
if ./tools/sim status 2>&1 | grep -q 'SILENT'; then
    echo "the simulator is up but blind; restart it before benching"
    exit 1
fi

mkdir -p ~/erc/logs
docker exec erc_sim bash -c 'pkill -f moveit.launch.py; pkill -f "avaa_solution perception"; pkill -f "avaa_solution grasp"; sleep 2; true'

echo "=== 1. standing back at $FAR m, where the marker is visible"
timeout 420 ./tools/in-sim drive_to.py "$COLOUR" "$FAR" 2>&1 | tail -3

echo
echo "=== 2. move_group and perception up"
docker exec -d erc_sim /entrypoint.sh bash -c \
  "source /opt/erc_ws/install/setup.bash && ros2 launch avaa_solution moveit.launch.py \
   > /tmp/bench_moveit.log 2>&1"
docker exec -d erc_sim /entrypoint.sh bash -c \
  "source /opt/erc_ws/install/setup.bash && ros2 run avaa_solution perception --ros-args \
   -p use_sim_time:=true -p shelf_column_number:=3 -p book_colour:=$COLOUR \
   -p save_images:=false > /tmp/bench_perception.log 2>&1"
sleep 20

echo
echo "=== 2b. levelling the head"
# Nothing else aims it here. The approach node normally does, and whatever tilt the
# last run left is still there -- a head tilted down for grasping puts the column
# markers at 2.26 m clean out of the top of the picture. Measured from 2.0 m with a
# stale downward tilt: zero markers in view, on every frame, for two hundred seconds.
#
# Level, and far enough back that the geometry is not marginal either. The camera sits
# about 1.3 m up and its vertical half-angle is 28 degrees; a marker at 2.26 m is 26
# degrees above the axis from 2.0 m away, which is inside the frame only just, and 18
# degrees from 3.0 m, which is comfortable.
docker exec erc_sim /entrypoint.sh bash -c   "source /opt/erc_ws/install/setup.bash && ros2 topic pub --once    /head_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory    '{joint_names: [head_1_joint, head_2_joint], points: [{positions: [0.0, 0.0],      time_from_start: {sec: 2}}]}'" > /dev/null 2>&1
sleep 6

echo
echo "=== 2c. which marker is the robot actually parked under?"
# The marker digits are randomised every run, and drive_to.py goes to whichever book of
# the colour is nearest -- so the column in front of the robot is almost never the one
# the launch argument names. Watched: parked at book_col_3_row_5_red with perception
# hunting for marker 3, it read "2 x29 (best score 0.52)" and never identified anything,
# which looks exactly like a broken reader and is nothing of the kind.
#
# The grasp does not care which book it picks up. So read the digit that is actually
# overhead and tell perception to want that one.
DIGIT=$(docker exec erc_sim bash -c 'grep "digits read in that window"     /tmp/bench_perception.log 2>/dev/null | tail -3'     | grep -oE '[0-9]+ x[0-9]+' | sort -t'x' -k2 -rn | head -1 | cut -d' ' -f1)
if [ -z "${DIGIT:-}" ]; then
    echo "  no digit read at all; leaving the target at 3"
    DIGIT=3
else
    echo "  parked under marker $DIGIT; retargeting perception at it"
    docker exec erc_sim bash -c 'pkill -f "avaa_solution perception"; sleep 2; true'
    docker exec -d erc_sim /entrypoint.sh bash -c       "source /opt/erc_ws/install/setup.bash && ros2 run avaa_solution perception        --ros-args -p use_sim_time:=true -p shelf_column_number:=$DIGIT        -p book_colour:=$COLOUR -p save_images:=false        > /tmp/bench_perception.log 2>&1"
    sleep 15
fi

echo
echo "=== 3. letting perception settle on the column and row"
# No polling here. `ros2 topic echo --once` is a poor test -- it can miss a topic that
# is publishing perfectly well -- and the grasp already does this properly: it sits in
# IDLE until it has both a row and a book point, and starts by itself when it does. One
# waiting mechanism is enough, and the one inside the node is the one that matters.
sleep 25
docker exec erc_sim bash -c 'grep -E "identified on|row identified|target .* book is on row"     /tmp/bench_perception.log 2>/dev/null | tail -3' | sed 's/^/  /'

echo "=== 4. closing to $STANDOFF m, perception tracking throughout"
timeout 420 ./tools/in-sim drive_to.py "$COLOUR" "$STANDOFF" 2>&1 | tail -3

echo
echo "=== 5. the grasp"
LOG=~/erc/logs/bench.log
: > "$LOG"
timeout 300 docker exec erc_sim /entrypoint.sh bash -c \
  "source /opt/erc_ws/install/setup.bash && ros2 run avaa_solution grasp --ros-args \
   -p use_sim_time:=true" > "$LOG" 2>&1

echo
grep -vE 'throttle' "$LOG" | tail -40
mkdir -p ~/erc/runs
cp "$LOG" ~/erc/runs/"bench-$(date +%Y%m%d-%H%M%S).log"
