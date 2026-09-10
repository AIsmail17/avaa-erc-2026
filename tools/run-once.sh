#!/usr/bin/env bash
set -u
cd ~/erc/erc_sim_2026

# The task, which the organisers randomise and which was hardcoded to "3 red" here.
#
#     tools/run-once.sh                # marker 3, red book
#     tools/run-once.sh 3 yellow       # marker 3, yellow book
#
# Worth having as an argument for two reasons. The report needs five trials and they
# should not all be the same task. And the layout is randomised per launch, so a fixed
# colour lands wherever it lands: on 2026-09-08 red was on the bottom shelf in four of
# the five columns and yellow was on the top two in four of them, which is the
# difference between exercising the whole pipeline and exercising the one reach that
# does not plan yet.
COLUMN="${1:-3}"
COLOUR="${2:-red}"
echo "=== task: marker $COLUMN, $COLOUR book"

# One run at a time. Two overlapping runs tore down each other's simulator on
# 2026-09-04 and produced a "blind simulator" that looked like a product fault.
# The pattern must not match the checking command itself, which "solution.launch.py"
# alone does -- pgrep sees its own bash -c argument and always reports a hit.
if docker exec erc_sim bash -c 'ps -eo args | grep -v grep | grep -q "[r]os2 launch avaa_solution solution.launch.py"'; then
  echo "a solution launch is already running; refusing to start another"
  exit 2
fi

# Headless, and not as an optimisation.
#
# Started with the GUI, this launch intermittently comes up with Gazebo never
# initialising: zero controllers reach "Configured and activated", all seven spawners die
# on "Could not successfully call service /controller_manager/list_controllers after 3
# attempts", and no colour frames are ever published. It happened on roughly a third of
# restarts over one evening, and the generous spawner timeouts in 15982e4 did not help,
# because the controller manager they wait for never exists.
#
# I very nearly reverted this. The first headless run looked like it had killed
# perception outright -- no frame-rate lines, no marker tallies -- and I had a revert
# written before checking the kept log rather than the thirty tail lines the task
# printed. The full log has perception running perfectly: 5.0 frames per simulated
# second, and one ten-second window with the target marker identified on 31 frames of
# 50, the best rate yet recorded. The evidence for the revert was an artefact of where I
# was reading.
#
# Nothing here needs watching. `sim gui` attaches a viewer to a running simulation when
# there is something worth seeing, and the GUI is still the way to record the video.
# With the simulation grasp aid, or no grasp can hold.
#
# simulation.launch.py only starts sim_grasp_fix when ERC_GRASP_FIX is 1, and tools/sim
# defaults it to 0. This line never set it -- so every run through here restarted the
# simulator WITHOUT the aid, and a position-driven gripper then closed through the book
# and pushed it. Measured 2026-09-10 on the first perception-driven run since the row
# heights were fixed: pads 1 mm off the book's centre line and 72 mm into it at the
# clamp, the book shoved 78 mm and twisted 55 degrees, and not one line from the aid,
# because there was no aid. The arena tests that lifted books had been run on a
# simulator started by hand with it on, which is why none of them saw this.
#
# ERC_GRASP_FIX=0 tools/run-once.sh ...   still runs without it, deliberately.
export ERC_GRASP_FIX="${ERC_GRASP_FIX:-1}"
echo "=== simulation grasp aid: ERC_GRASP_FIX=$ERC_GRASP_FIX"
# Under WSL, tools/sim restarts the simulator headless. On a Linux desktop -- the laptop --
# tools/linux-sim.sh restarts it with its window on that screen instead: headless does not
# render there at all (Mesa's EGL fails on its NVIDIA card, see linux-sim.sh), and the
# window is the reason for running it there. Everything after this is the same on both.
if grep -qi microsoft /proc/version 2>/dev/null; then
  ./tools/sim restart --fast --headless 2>&1 | tail -3
else
  ./tools/linux-sim.sh all 2>&1 | tail -3
fi
echo "=== waiting for the camera to actually stream"
ok=0
for i in $(seq 1 12); do
  sleep 8
  n=$(docker exec erc_sim /entrypoint.sh bash -c 'source /opt/erc_ws/install/setup.bash && timeout 10 ros2 topic hz /head_front_camera/head_front_camera/color/image_raw 2>&1 | grep -c "average rate"')
  echo "  colour frames flowing: $n"
  if [ "$n" -gt 0 ]; then ok=1; break; fi
  # A server that has died is not going to start streaming, so say so now rather than
  # after the rest of the tries. On 2026-09-10 the laptop's aborted 15 s in and this
  # waited out all twelve, three and a half minutes, to report a blind simulator.
  if ! docker exec erc_sim bash -c 'ps -eo args | grep "gz sim" | grep -qvE "^(bash|grep)"'; then
    echo "the Gazebo server is not running; see /tmp/sim.log in the container"
    break
  fi
done
if [ "$ok" -eq 0 ]; then echo "the simulator came up blind; not launching"; exit 1; fi
if [ "$ERC_GRASP_FIX" = "1" ] && ! docker exec erc_sim bash -c 'ps -eo args | grep -v grep | grep -q "sim_grasp_fix.py"'; then
  echo "ERC_GRASP_FIX=1 but the grasp aid is not running; not launching a run that cannot hold a book"
  exit 1
fi

# Long enough for the whole mission, and it kills the run rather than the log.
#
# Two faults in "timeout 800 docker exec", both seen in one run on 2026-09-07.
#
# 800 seconds of wall clock is not a mission. The mission took 157 simulated seconds to
# reach the delivery, and at the 0.3 to 0.5 real-time factor this machine manages that is
# already 300 to 500 seconds before the delivery has driven anywhere -- plus a minute of
# startup. The run that first completed a grasp and handed over to the delivery was cut
# off in the middle of it.
#
# And killing the docker exec CLIENT does not kill what it started. The ros2 launch went
# on running inside the container with nowhere to write: the log stopped at 12:34 while
# the nodes were still going at 12:40, the next run refused to start because a launch was
# already up, and the mission's own phase topic was the only way left to find out what
# had happened. So: kill by name inside the container, then wait for it to go.
LAUNCH_TIMEOUT=${LAUNCH_TIMEOUT:-2400}
docker exec erc_sim /entrypoint.sh bash -c \
  "source /opt/erc_ws/install/setup.bash && exec ros2 launch avaa_solution \
   solution.launch.py shelf_column_number:=$COLUMN book_colour:=$COLOUR" \
  > /tmp/run_raw.log 2>&1 &
CLIENT=$!
( sleep "$LAUNCH_TIMEOUT"
  if kill -0 "$CLIENT" 2>/dev/null; then
    echo "=== $LAUNCH_TIMEOUT s elapsed; stopping the launch" >> /tmp/run_raw.log
    docker exec erc_sim bash -c 'pkill -f "ros2 launch avaa_solution solution.launch.py"' \
      >/dev/null 2>&1
  # Its output goes nowhere, and it is killed with its children below. Otherwise its
  # sleep outlives it: killing the subshell leaves sleep holding this script's stdout,
  # and whatever pipes that stdout -- run-once.sh | tail, which is how it is always
  # run -- waits the full LAUNCH_TIMEOUT for a pipe nobody is writing to. Found
  # 2026-09-10 as a finished run still 'running' after its log had been kept.
  fi ) >/dev/null 2>&1 &
GUARD=$!
# Stop when the MISSION stops, not when the clock runs out.
#
# The launch never exits on its own: when the mission reaches done or failed, every node
# stays up waiting for a phase that will never come. So this used to sit on "wait $CLIENT"
# for the whole of LAUNCH_TIMEOUT -- forty minutes after a run that ended in five. Measured
# 2026-09-10: two finished runs were found still waiting, 36 and 20 minutes in, and each
# would have finished with the pkill below and taken down whatever run was going by then.
# That is the same trap as the orphaned drive_to in tools/in-sim, reached a different way.
#
# Watch the mission's own verdict instead, give the last lines ten seconds to land, and
# stop. LAUNCH_TIMEOUT stays as the upper bound for a mission that never decides.
while kill -0 "$CLIENT" 2>/dev/null; do
  if sed 's/\x1b\[[0-9;]*m//g' /tmp/run_raw.log 2>/dev/null \
       | grep -qE "avaa_mission.*phase [a-z]+ -> (done|failed)"; then
    echo "=== the mission has finished; stopping the launch"
    sleep 10
    docker exec erc_sim bash -c 'pkill -f "ros2 launch avaa_solution solution.launch.py"' \
      >/dev/null 2>&1
    # And its nodes, by PID. Stopping the launch does not stop them: on 2026-09-10 all
    # five were found alive twenty minutes after the launch was killed, and the client
    # this script waits on never returned. Ten seconds for a clean exit, then no choice.
    sleep 10
    # move_group too: it is started by the same launch but runs as its own binary, which the
    # patterns above miss. After the thirteenth run one was still up 565 s later, and starting
    # a fresh one for a measurement put two on the same planning scene.
    stragglers=$(docker exec erc_sim ps -eo pid=,args= \
      | grep -E "avaa_solution/lib/avaa_solution/|ros2 launch avaa_solution|moveit_ros_move_group/move_group" \
      | grep -v " ps -eo" | awk '{print $1}' | tr '\n' ' ')
    if [ -n "${stragglers// /}" ]; then
      echo "=== nodes still up after the launch stopped; killing: $stragglers"
      docker exec erc_sim kill -9 $stragglers >/dev/null 2>&1
    fi
    break
  fi
  sleep 15
done
wait "$CLIENT" 2>/dev/null || true
pkill -P "$GUARD" 2>/dev/null || true
kill "$GUARD" 2>/dev/null || true
# Whatever ended the launch, leave nothing of it behind for the next run to trip over.
docker exec erc_sim bash -c 'pkill -f "ros2 launch avaa_solution solution.launch.py"' \
  >/dev/null 2>&1 || true
# Keep it. /tmp is cleared out from under this often enough that two separate
# investigations lost the log they were reading halfway through, and a run costs
# thirteen minutes to reproduce.
mkdir -p ~/erc/runs
cp /tmp/run_raw.log ~/erc/runs/"run-$(date +%Y%m%d-%H%M%S)-$COLUMN-$COLOUR.log"
ls -t ~/erc/runs/*.log | tail -n +21 | xargs -r rm -f
echo "=== done, kept at ~/erc/runs/"
grep -E 'avaa_grasp|avaa_mission|avaa_approach' /tmp/run_raw.log | grep -v throttle | tail -30
exit 0
