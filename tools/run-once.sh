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

./tools/sim restart 2>&1 | tail -3
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
