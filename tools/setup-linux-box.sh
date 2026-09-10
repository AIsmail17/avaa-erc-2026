#!/usr/bin/env bash
# Bring a fresh Ubuntu machine up to a running, watchable simulator.
#
#     tools/setup-linux-box.sh [directory]
#
# Written on 2026-09-10 while doing it by hand on a spare laptop, so that the next
# machine takes ten minutes instead of two hours. It is for a real Linux box with a
# desktop session -- which is the setup worth having, because Gazebo's window then goes
# straight to the screen through /dev/dri instead of being translated twice by WSL and
# then not presented at all.
#
# Measured on an i7-11370H with Intel Xe graphics: direct rendering, GL 4.6, and a
# real-time factor of 0.52 WITH the viewer running. The same repo under WSL manages 0.51
# headless and about 0.2 with a viewer.
#
# What it does not do: install the NVIDIA driver or the container toolkit. Mesa on the
# integrated GPU is enough for this simulation, and up.sh detects a working NVIDIA
# runtime on its own if one is there.
set -euo pipefail

WHERE="${1:-$HOME/erc}"
REPO=https://github.com/AIsmail17/avaa-erc-2026.git
BRANCH=avaa

say() { printf '\n=== %s\n' "$1"; }

say "packages"
sudo apt-get update -qq
sudo apt-get install -y docker.io docker-compose-v2 git mesa-utils >/dev/null
docker --version
docker compose version | head -1

say "letting $USER use docker without sudo"
sudo usermod -aG docker "$USER"
echo "  active from the next login. This script uses sudo where it has to."

say "the repository, in $WHERE"
mkdir -p "$WHERE"
if [ -d "$WHERE/erc_sim_2026/.git" ]; then
    git -C "$WHERE/erc_sim_2026" fetch --quiet origin "$BRANCH"
    git -C "$WHERE/erc_sim_2026" checkout --quiet "$BRANCH"
    git -C "$WHERE/erc_sim_2026" pull --quiet
else
    git clone --quiet --branch "$BRANCH" "$REPO" "$WHERE/erc_sim_2026"
fi
git -C "$WHERE/erc_sim_2026" log --oneline -1 | sed 's/^/  /'

say "the image"
if sudo docker image inspect erc-2026:humble-harmonic >/dev/null 2>&1; then
    echo "  already here"
else
    echo "  not here. Either build it, which takes a while and needs the network:"
    echo "      cd $WHERE/erc_sim_2026 && ./docker/up.sh --build"
    echo "  or copy it from a machine that already has it, which is faster:"
    echo "      docker save erc-2026:humble-harmonic | gzip -1 \\"
    echo "        | ssh $USER@$(hostname -I 2>/dev/null | awk '{print $1}') 'sudo docker load'"
    echo "  then run this script again."
    exit 1
fi

say "the container"
cd "$WHERE/erc_sim_2026"
DISPLAY="${DISPLAY:-:0}" ./docker/up.sh | sed 's/^/  /'

say "building the workspace"
# The image does NOT carry a built workspace. It is built inside the container and lives
# in the container's writable layer, which docker save does not include -- so a machine
# that received the image by transfer has the sources and no install space, and every
# launch fails with "no such file or directory: /opt/erc_ws/install/setup.bash".
sudo docker exec erc_sim /entrypoint.sh bash -c 'cd /opt/erc_ws && colcon build --symlink-install' \
    2>&1 | tail -2 | sed 's/^/  /'

say "checking that the container can draw"
sudo docker exec -e DISPLAY="${DISPLAY:-:0}" erc_sim bash -lc \
    'glxinfo -B 2>/dev/null | grep -E "direct rendering|Device|Accelerated|Max core"' \
    | sed 's/^/  /' || echo "  glxinfo not available in the image; install mesa-utils inside it"

say "done"
cat <<'NOTES'
  Start the arena and a viewer:
      docker exec -d erc_sim /entrypoint.sh bash -c \
        'source /opt/erc_ws/install/setup.bash && ros2 launch erc_bringup simulation.launch.py depth_cloud:=false headless:=true'
      docker exec -d -e DISPLAY=:0 erc_sim bash -c 'gz sim -g --gui-config /opt/erc_ws/src/avaa_solution/config/gui.config'

  If the launch dies with "rcl node's rmw handle is invalid", the socket receive buffer
  cap is too low for CycloneDDS. up.sh raises it; if it could not, do it by hand:
      sudo sysctl -w net.core.rmem_max=33554432
NOTES
