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
