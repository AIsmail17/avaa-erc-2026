#!/usr/bin/env bash
# Do the pads ever touch the book?
#
#     tools/padcontact.sh
#
# The jaws close to 27-31 mm around a 30 mm book and the book does not move. That has two
# readings and they call for opposite fixes: the fingers contact the book and squeeze past
# it, or they never contact it at all. This watches the fingertips' own contact sensors
# while jawtest.py puts a book between them and closes.
#
# The contact sensors publish per link, not on a single /contacts topic -- generate_urdf.py
# injects one sensor per finger link -- so they are read straight from Gazebo here rather
# than through a bridge.
set -u
cd ~/erc/erc_sim_2026

BASE=/world/erc_world/model/tiago_pro/link
LINKS="gripper_left_fingertip_left_link gripper_left_fingertip_right_link
        gripper_left_inner_finger_left_link gripper_left_inner_finger_right_link"

docker exec erc_sim bash -c 'rm -f /tmp/pad_*.txt' 2>/dev/null

for link in $LINKS; do
    topic="$BASE/$link/sensor/${link}_contact/contact"
    docker exec -d erc_sim /entrypoint.sh bash -c \
        "timeout 120 gz topic -e -t '$topic' > /tmp/pad_${link}.txt 2>&1"
done
sleep 3
echo "watching ${#LINKS} fingertip and inner-finger contact sensors"
echo

timeout 280 ./tools/in-sim jawtest.py 2>&1 | tail -20

echo
echo "=== what the pads felt"
for link in $LINKS; do
    n=$(docker exec erc_sim bash -c "grep -c 'collision2' /tmp/pad_${link}.txt 2>/dev/null" 2>/dev/null || echo 0)
    hits=$(docker exec erc_sim bash -c "grep -A1 collision2 /tmp/pad_${link}.txt 2>/dev/null | grep -c book" 2>/dev/null || echo 0)
    printf '  %-40s %4s contacts, %4s against the book\n' "$link" "${n:-0}" "${hits:-0}"
done
docker exec erc_sim bash -c 'pkill -f "gz topic -e"' 2>/dev/null || true
