#!/usr/bin/env bash
# Find a gripper force that holds a book instead of crushing past it.
#
#     tools/forcesweep.sh [factor ...]      default: 1 0.2 0.1 0.05
#
# The stock left gripper is allowed 10 N on the driven prismatic. A 0.3 kg book with
# mu=5.0 needs about 0.3 N of pinch to stay put, so the jaws have roughly thirty times
# the force they need, and a light book squirts out rather than being held. Scaling by
# 0.001 was measured to stall the finger completely, so the useful range is in between.
#
# Everything for one factor happens in ONE invocation on purpose. On the workstation WSL
# shuts down whenever no command is running and takes Docker, the container and the bench
# with it; splitting bring-up from the test is what produced a run of "the simulator died
# on its own" that was nothing of the kind.
set -u
cd ~/erc/erc_sim_2026

FACTORS="${*:-1 0.2 0.1 0.05}"
RESULTS=/tmp/forcesweep.txt
: > "$RESULTS"

trap 'python3 tools/graspforce.py --restore >/dev/null 2>&1' EXIT

for f in $FACTORS; do
    echo
    echo "════════════════════════════════════════════════════════════"
    echo "  effort x $f"
    echo "════════════════════════════════════════════════════════════"
    python3 tools/graspforce.py --restore >/dev/null 2>&1
    if [ "$f" != "1" ]; then
        python3 tools/graspforce.py "$f" | head -1
    else
        echo "stock limits (10 N driven, 0.1 Nm linkage)"
    fi

    timeout 220 ./tools/lab up dart 2>&1 | grep -E "real-time|mimic|controllers"
    sleep 20

    out=$(timeout 280 ./tools/in-sim jawtest.py 2>&1)
    echo "$out" | grep -E "finger at|pads|book at|HELD|contacts|torso moved"

    span=$(echo "$out" | grep "pads now" | grep -oE "[0-9.]+ mm" | head -1)
    verdict=$(echo "$out" | grep -oE "NOT HELD|HELD" | head -1)
    printf '%-8s closed to %-10s %s\n' "$f" "${span:-?}" "${verdict:-no result}" >> "$RESULTS"
done

echo
echo "════════════════════════════════════════════════════════════"
echo "  summary  (book is 30.0 mm thick)"
echo "════════════════════════════════════════════════════════════"
printf '%-8s %s\n' "factor" "result"
cat "$RESULTS"
