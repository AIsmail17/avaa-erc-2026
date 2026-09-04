set -u
cd ~/erc/erc_sim_2026

# One run at a time. Two overlapping runs tore down each other's simulator on
# 2026-09-04 and produced a "blind simulator" that looked like a product fault.
# The pattern must not match the checking command itself, which "solution.launch.py"
# alone does -- pgrep sees its own bash -c argument and always reports a hit.
if docker exec erc_sim bash -c 'ps -eo args | grep -v grep | grep -q "[r]os2 launch avaa_solution"'; then
  echo "a solution launch is already running; refusing to start another"
  exit 2
fi

# Headless, and not as an optimisation.
#
# Started with the GUI, this launch intermittently comes up with Gazebo never
# initialising: zero controllers reach "Configured and activated", all seven spawners die
# with "Could not successfully call service /controller_manager/list_controllers after 3
# attempts", and no colour frames are ever published. Counted over one evening it
# happened on roughly a third of restarts, and the generous spawner timeouts added in
# 15982e4 did not help, because the manager they are waiting for never exists.
#
# Nothing here needs to be watched. Use `sim gui` to attach a viewer to a running
# simulation when there is something worth looking at, and the GUI for recording the
# deliverable video.
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
./tools/sim restart --fast --headless 2>&1 | tail -3
echo "=== waiting for the camera to actually stream"
ok=0
for i in $(seq 1 12); do
  sleep 8
  n=$(docker exec erc_sim /entrypoint.sh bash -c 'source /opt/erc_ws/install/setup.bash && timeout 10 ros2 topic hz /head_front_camera/head_front_camera/color/image_raw 2>&1 | grep -c "average rate"')
  echo "  colour frames flowing: $n"
  if [ "$n" -gt 0 ]; then ok=1; break; fi
done
if [ "$ok" -eq 0 ]; then echo "the simulator came up blind; not launching"; exit 1; fi

timeout 800 docker exec erc_sim /entrypoint.sh bash -c \
  'source /opt/erc_ws/install/setup.bash && ros2 launch avaa_solution solution.launch.py shelf_column_number:=3 book_colour:=red' \
  > /tmp/run_raw.log 2>&1 || true
# Keep it. /tmp is cleared out from under this often enough that two separate
# investigations lost the log they were reading halfway through, and a run costs
# thirteen minutes to reproduce.
mkdir -p ~/erc/runs
cp /tmp/run_raw.log ~/erc/runs/"run-$(date +%Y%m%d-%H%M%S).log"
ls -t ~/erc/runs/*.log | tail -n +21 | xargs -r rm -f
echo "=== done, kept at ~/erc/runs/"
grep -E 'avaa_grasp|avaa_mission|avaa_approach' /tmp/run_raw.log | grep -v throttle | tail -30
exit 0
