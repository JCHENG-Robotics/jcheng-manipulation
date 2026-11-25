#!/bin/bash
# Create a Humble workspace that symlinks this repo's ROS packages.
#
# The list matches the package set .github/workflows/ros-ci.yml builds in
# Humble, so the laptop container, CI and the robot resolve the same packages.
set -euo pipefail

REPO="${1:-$HOME/jcheng/jcheng-manipulation}"
WS="${2:-$HOME/jcheng/humble_ws}"

PACKAGES=(
    arm_interfaces
    armpi_analysis
    armpi_fiducial_localizer
    armpi_grasp_planner
    armpi_safety
    armpi_ultra_description
    armpi_ultra_moveit_config
)

mkdir -p "$WS/src"
for package in "${PACKAGES[@]}"; do
    ln -sfn "$REPO/ros2_ws/src/$package" "$WS/src/$package"
done

echo "Humble workspace prepared: $WS"
echo "Sources:"
ls -l "$WS/src"
