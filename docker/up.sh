#!/bin/bash
xhost +local:docker 2>/dev/null
cd "$(dirname "$0")"
docker compose down 2>/dev/null
docker rm -f erc_sim 2>/dev/null
COMPOSE="-f docker-compose.yml"

if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1 \
   && { docker info 2>/dev/null | grep -qi nvidia || command -v nvidia-ctk >/dev/null 2>&1; }; then
    echo "[up] NVIDIA GPU + container toolkit detected — enabling GPU passthrough"
    COMPOSE="$COMPOSE -f docker-compose.gpu.yml"
else
    echo "[up] No usable NVIDIA runtime — falling back to software/integrated rendering"
fi

ARGS=()
BUILD=0
for arg in "$@"; do
    if [ "$arg" = "--build" ]; then
        BUILD=1
    else
        ARGS+=("$arg")
    fi
done

if [ "$BUILD" -eq 1 ]; then
    echo "[up] Pulling latest base image"
    docker pull osrf/ros:humble-desktop
    echo "[up] Rebuilding image from scratch"
    docker compose $COMPOSE build --no-cache
fi

docker compose $COMPOSE up -d "${ARGS[@]}"

# Raise the socket receive buffer cap, or nothing will start.
#
# config/cyclonedds.xml asks for a 32 MB receive buffer, because one camera frame is
# 691 KB and the Linux default cap is 208 KB -- a third of a frame. CycloneDDS treats
# that minimum as a hard requirement: if the cap is lower it does not warn and carry on,
# it fails to create the domain at all, and every node then dies with
#
#     rmw_create_node: failed to create domain, error Error
#     rcl node's rmw handle is invalid, at ./src/rcl/node.c:415
#
# which says nothing about buffers and reads like a broken DDS install. Two hours went
# into that on a fresh machine. tools/sim has raised this since the day it was found;
# up.sh never did, so anyone starting the container any other way hit it.
#
# The container is privileged and on the host network, so this can be set from inside it
# and applies to the host. Failure is not fatal here -- say so and let the user see what
# breaks -- because a machine may have it set already or forbid it.
WANT=33554432
HAVE=$(docker exec erc_sim sysctl -n net.core.rmem_max 2>/dev/null || echo 0)
if [ "${HAVE:-0}" -lt "$WANT" ]; then
    if docker exec erc_sim sysctl -q -w net.core.rmem_max=$WANT >/dev/null 2>&1; then
        echo "[up] Raised net.core.rmem_max from $HAVE to $WANT (a camera frame is 691 KB)"
    else
        echo "[up] WARNING: could not raise net.core.rmem_max (it is $HAVE)."
        echo "[up]          CycloneDDS wants 32 MB and will refuse to create a domain."
        echo "[up]          On the host:  sudo sysctl -w net.core.rmem_max=$WANT"
    fi
fi

echo "Container running. Attach with: ./docker/attach.sh"
echo "Rebuild image:        ./docker/up.sh --build"
echo "Force GPU manually:   docker compose -f docker-compose.yml -f docker-compose.gpu.yml up -d"