#!/usr/bin/env bash
# Test the grasp on its own, without the approach's search and centring in front of it.
#
#     tools/graspbench.sh [colour] [standoff] [far]
#
# A full run costs thirteen minutes and spends eleven on an approach that is already
# understood, to reach a grasp that is not.
#
# The order below is not arbitrary and the first version of this got it wrong. Driving
# straight to grasping range and starting perception there gives the grasp nothing to
# work with: from 0.65 m the column marker at 2.26 m sits 59 degrees above the camera
# axis and the vertical half-angle is 28. The marker CANNOT be seen from where the arm
# works, which is why the real pipeline identifies the column from three metres out and
# carries the identification in on the book tracker.
#
# So: stand back, let perception name the column and the row, and only then close in --
# with perception running throughout, exactly as it is during a real approach. Driving,
# never teleporting: one set_pose takes the real-time factor from 0.48 to 0.04 for good,
# and it never recovers without a relaunch.
set -u
COLOUR="${1:-red}"
STANDOFF="${2:-0.65}"
FAR="${3:-2.4}"
cd ~/erc/erc_sim_2026

if docker exec erc_sim bash -c 'ps -eo args | grep -v grep | grep -q "[r]os2 launch avaa_solution solution.launch.py"'; then
    echo "a solution launch is already running; refusing to start another"
    exit 2
fi
if ./tools/sim status 2>&1 | grep -q 'SILENT'; then
    echo "the simulator is up but blind; restart it before benching"
    exit 1
fi

mkdir -p ~/erc/logs ~/erc/runs
docker exec erc_sim bash -c \
    'pkill -f moveit.launch.py; pkill -f "avaa_solution perception"; pkill -f "avaa_solution grasp"; sleep 2; true'

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
sleep 25

echo
echo "=== 3. waiting for perception to name the row"
row=""
for i in $(seq 1 20); do
    row=$(docker exec erc_sim /entrypoint.sh bash -c \
        "source /opt/erc_ws/install/setup.bash && timeout 8 ros2 topic echo \
         /avaa/perception/target_row --once --field data 2>/dev/null" 2>/dev/null | head -1)
    if [ -n "${row:-}" ]; then echo "  row $row"; break; fi
    echo "  attempt $i: no row yet"
done
if [ -z "${row:-}" ]; then
    echo
    echo "perception never named a row from $FAR m. Last of its log:"
    docker exec erc_sim bash -c 'tail -12 /tmp/bench_perception.log'
    exit 1
fi

echo
echo "=== 4. closing to $STANDOFF m, perception tracking throughout"
timeout 420 ./tools/in-sim drive_to.py "$COLOUR" "$STANDOFF" 2>&1 | tail -3

echo
echo "=== 5. the grasp"
LOG=~/erc/logs/bench.log
: > "$LOG"
# trust_finger_span:=false, because this script only ever runs in the simulator.
#
# The parameter defaults to true, which is right for the real robot: jaws that close
# past the thickness of the book closed on nothing. In Gazebo they ALWAYS close past
# it. DART refuses the mimic constraints in this gripper, so the driven joint goes
# where it is told and the linkage follows through the book -- measured at 27.4 mm
# around a 30.0 mm book while the book was still sitting on its shelf.
#
# With the default, the grasp cannot pass its own check here whatever the arm does,
# and the run dies on a message about closing on nothing that says nothing about the
# grasp. Lean on the geometric check instead, which has something real to measure.
timeout 400 docker exec erc_sim /entrypoint.sh bash -c \
  "source /opt/erc_ws/install/setup.bash && ros2 run avaa_solution grasp --ros-args \
   -p use_sim_time:=true -p trust_finger_span:=${TRUST_SPAN:-false}" > "$LOG" 2>&1

echo
grep -vE 'throttle' "$LOG" | tail -45
cp "$LOG" ~/erc/runs/"bench-$(date +%Y%m%d-%H%M%S).log"
echo
echo "kept at ~/erc/runs/bench-*.log"
