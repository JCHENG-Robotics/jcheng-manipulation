#!/bin/bash
# Create a Humble workspace that symlinks the MoveIt packages from this repo.
set -euo pipefail

REPO="${1:-$HOME/jcheng/jcheng-manipulation}"
WS="${2:-$HOME/jcheng/humble_ws}"

mkdir -p "$WS/src"
ln -sfn "$REPO/ros2_ws/src/armpi_ultra_description" \
    "$WS/src/armpi_ultra_description"
ln -sfn "$REPO/ros2_ws/src/armpi_ultra_moveit_config" \
    "$WS/src/armpi_ultra_moveit_config"

echo "Humble workspace prepared: $WS"
echo "Sources:"
ls -l "$WS/src"
