#!/bin/bash
# Launch a headless MoveIt move_group for S17 plan-only acceptance.
# Trajectory execution is disabled; no controller manager is started.
set -e
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
source /home/ubuntu/jcheng-moveit/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo $$ > /home/ubuntu/move_group.pid
exec ros2 launch armpi_ultra_moveit_config move_group.launch.py \
  allow_trajectory_execution:=false \
  > /home/ubuntu/move_group.log 2>&1
