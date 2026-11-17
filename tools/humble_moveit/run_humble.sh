#!/bin/bash
# Run the Humble MoveIt/RViz container on the laptop.
# Requires Docker access (docker group) and a running X server.
set -euo pipefail

WS="${1:-$HOME/jcheng/humble_ws}"
IMAGE="${IMAGE:-armpi-humble-moveit}"
REPO="${REPO:-$HOME/jcheng/jcheng-manipulation}"

if ! command -v docker >/dev/null 2>&1; then
    echo "docker not found; run tools/humble_moveit/install_docker.sh first" >&2
    exit 1
fi
if ! docker info >/dev/null 2>&1; then
    echo "Cannot access the Docker daemon." >&2
    echo "Run 'newgrp docker' or 'sg docker -c \"$0\"' in a terminal." >&2
    exit 1
fi
if [ ! -d "$WS" ]; then
    echo "Workspace $WS does not exist; run setup_humble_ws.sh first" >&2
    exit 1
fi

xhost +local: >/dev/null

exec docker run -it --rm --name armpi-humble \
    --net=host --ipc=host \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e QT_X11_NO_MITSHM=1 \
    -e ROS_DOMAIN_ID=0 \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI=file:///workspace/cyclonedds-laptop.xml \
    -e ROS2CLI_NO_DAEMON=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$WS":/workspace \
    -v "$REPO/tools/humble_moveit/cyclonedds-laptop.xml":/workspace/cyclonedds-laptop.xml:ro \
    -v "$REPO/ros2_ws/src":"$REPO/ros2_ws/src":ro \
    -w /workspace \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp \
    "$IMAGE"
