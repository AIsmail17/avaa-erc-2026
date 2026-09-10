#!/usr/bin/env bash
# Start the arena on a real Linux desktop, with the Gazebo window on this machine's screen.
#
#     tools/linux-sim.sh            simulator with its GUI on this screen
#     tools/linux-sim.sh viewer     attach a second viewer to a running simulator
#     tools/linux-sim.sh stop       stop both
#
#     tools/run-once.sh 3 red       restart it through here and run the solution once,
#                                   marker 3 and the red book, in that window
#
# tools/sim does the same job under WSL and most of it is WSL: Mesa driver juggling,
# PowerShell, a window WSLg never presents. None of that applies here -- /dev/dri goes
# straight into the container and Mesa picks the Intel driver on its own.
#
# It works from an SSH session as well as from a terminal on the desktop, which is the
# point of the display detection below. Over SSH there is no DISPLAY and no X cookie, so
# it finds the logged-in desktop's X server itself: the socket in /tmp/.X11-unix and the
# auth file GDM hands Xorg. On the laptop that is display :1, not :0 -- GDM keeps :0 for
# its own greeter -- and guessing :0 gives "cannot open display" with nothing to say why.
set -u
cd "$(dirname "$0")/.."

CONTAINER=erc_sim
MODE="${1:-all}"

find_display() {
    if [ -n "${DISPLAY:-}" ]; then
        echo "$DISPLAY"
        return
    fi
    # The X server the desktop session is using, from its own command line.
    local auth
    auth=$(ps -eo args | awk '/[X]org .*-auth/ { for (i = 1; i <= NF; i++) if ($i == "-auth") print $(i + 1) }' | head -1)
    [ -n "$auth" ] && export XAUTHORITY="$auth"
    local socket
    socket=$(ls /tmp/.X11-unix/ 2>/dev/null | sed -n 's/^X\([0-9]*\)$/\1/p' | sort -n | tail -1)
    echo ":${socket:-0}"
}

DISP=$(find_display)
export DISPLAY="$DISP"

stop_all() {
    docker exec "$CONTAINER" bash -c \
        'pkill -f "[r]os2 launch erc_bringup" 2>/dev/null; pkill -f "[g]z sim" 2>/dev/null; true'
    # Then everything else the launch started, by name, once it has had time to go --
    # what tools/sim's stop_orphans does under WSL, and this script was not doing.
    #
    # Stopping only the launch and Gazebo left the rest running. On 2026-09-10 a restart
    # on the laptop came up beside the previous simulator's three bridges, two gripper
    # command clamps from simulators before that, and its grasp aid, which had last
    # attached a book to the gripper. The new server aborted in DART's constraint solver
    # within fifteen seconds of starting (a contact matrix entry at -3.3e11), leaving
    # run-once waiting on a camera nothing fed. That the old aid caused it is likely but
    # not shown; a restart that keeps any of it is wrong either way.
    sleep 5
    docker exec "$CONTAINER" bash -c '
        ps -eo pid=,args= \
          | grep -E "ros2 launch erc_bringup|gz sim|erc_bringup/lib/erc_bringup/|ros_gz_bridge/parameter_bridge|robot_state_publisher/robot_state_publisher" \
          | grep -v -E "grep|bash -c" | awk "{print \$1}" | xargs -r kill -9 2>/dev/null; true'
    sleep 1
}

ensure_container() {
    if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
        echo "=== starting the container"
        ./docker/up.sh | sed 's/^/  /'
    fi
    # The image does not carry a built workspace; see tools/setup-linux-box.sh.
    if ! docker exec "$CONTAINER" test -f /opt/erc_ws/install/setup.bash; then
        echo "=== building the workspace (once)"
        docker exec "$CONTAINER" /entrypoint.sh bash -c \
            'cd /opt/erc_ws && colcon build --symlink-install' 2>&1 | tail -1 | sed 's/^/  /'
    fi
    # CycloneDDS refuses to create a domain below this; see docker/up.sh.
    docker exec "$CONTAINER" sysctl -q -w net.core.rmem_max=33554432 >/dev/null 2>&1 || true
}

start_server() {
    echo "=== simulator with its GUI on $DISPLAY"
    # With the GUI, not headless, on a laptop with an NVIDIA card as well as an Intel one.
    #
    # Headless, the launch unsets DISPLAY to force Ogre onto surfaceless EGL, and in this
    # container Mesa is the only EGL there is. It enumerates the NVIDIA card too, which
    # runs the proprietary driver Mesa cannot use, and on 2026-09-10 the server failed
    # every time -- eglInitialize failed for the DRM device, then OpenGL 3.3 is not
    # supported -- and nothing spawned. The standalone viewer rendered fine, through GLX
    # on the X display the desktop runs on the Intel GPU. So the server renders that way
    # too: controllers up in 10 s, all 21 models spawned, the head camera at 21 Hz.
    xhost +local: >/dev/null 2>&1 || true
    # With the simulation grasp aid: without it the gripper cannot hold a book, and
    # watching a run that can only push books over shows nothing worth seeing. See
    # tools/run-once.sh for how its absence was found. ERC_GRASP_FIX=0 still turns it off,
    # as it does there.
    docker exec -d -e ERC_GRASP_FIX="${ERC_GRASP_FIX:-1}" -e DISPLAY="$DISPLAY" -e XDG_RUNTIME_DIR=/tmp/runtime-root "$CONTAINER" /entrypoint.sh bash -c \
      'mkdir -p /tmp/runtime-root; source /opt/erc_ws/install/setup.bash && exec ros2 launch erc_bringup simulation.launch.py depth_cloud:=false headless:=false > /tmp/sim.log 2>&1'
    local waited=0
    while [ "$waited" -lt 120 ]; do
        sleep 5
        waited=$((waited + 5))
        # The spawner colours its output, and the escape codes sit BETWEEN "activated"
        # and the controller's name -- so the phrase as one string never matches, and this
        # would wait out its whole two minutes on a simulator that was ready in forty
        # seconds. Match the two halves of the line separately.
        if docker exec "$CONTAINER" bash -c \
               'grep "Configured and activated" /tmp/sim.log 2>/dev/null | grep -q arm_left_controller'; then
            echo "  controllers up after ${waited}s"
            return 0
        fi
        if docker exec "$CONTAINER" bash -c 'grep -q "rmw handle is invalid" /tmp/sim.log 2>/dev/null'; then
            echo "  DDS could not start: raise net.core.rmem_max (see docker/up.sh)"
            return 1
        fi
    done
    echo "  still waiting after ${waited}s; carrying on, check /tmp/sim.log in the container"
}

start_viewer() {
    echo "=== viewer on $DISPLAY"
    # Let local clients -- the container is one, on the host network -- use this display.
    xhost +local: >/dev/null 2>&1 || echo "  (xhost not available; the viewer may be refused)"
    docker exec "$CONTAINER" bash -c 'pkill -f "[g]z sim -g" 2>/dev/null; true'
    docker exec -d -e DISPLAY="$DISPLAY" -e XDG_RUNTIME_DIR=/tmp/runtime-root "$CONTAINER" bash -c \
      'mkdir -p /tmp/runtime-root; exec gz sim -g --gui-config /opt/erc_ws/src/avaa_solution/config/gui.config > /tmp/gui.log 2>&1'
    sleep 15
    if docker exec "$CONTAINER" bash -c 'pgrep -f "[g]z sim -g" >/dev/null'; then
        echo "  viewer is up"
    else
        echo "  viewer died:"
        docker exec "$CONTAINER" bash -c 'grep -iE "error|cannot|fail" /tmp/gui.log | tail -6' | sed 's/^/    /'
        return 1
    fi
}

case "$MODE" in
    stop)
        stop_all
        echo "stopped"
        ;;
    viewer)
        ensure_container
        start_viewer
        ;;
    all)
        ensure_container
        stop_all
        start_server
        ;;
    *)
        echo "usage: tools/linux-sim.sh [all|viewer|stop]"
        exit 1
        ;;
esac
