#!/bin/bash
# ArmPi off-board RealSense D435i ROS camera driver.
# Single owner of /dev/video4: publishes /ext_cam/color/image_raw, camera_info
# and the base_link -> ext_cam_optical static TF. The studio gateway follows
# this topic instead of opening the device.
set -euo pipefail

CONTAINER="${ARMPI_EXT_CAM_CONTAINER:-ArmPiUltra}"
WAIT_SECONDS="${ARMPI_EXT_CAM_CONTAINER_WAIT_S:-60}"

deadline=$((SECONDS + WAIT_SECONDS))
until docker inspect -f '{{.State.Running}}' "${CONTAINER}" 2>/dev/null | grep -qx true; do
  if (( SECONDS >= deadline )); then
    echo "ERROR: container ${CONTAINER} is not running after ${WAIT_SECONDS}s" >&2
    exit 1
  fi
  sleep 2
done

exec docker exec -u ubuntu "${CONTAINER}" /bin/bash -lc \
  "source /opt/ros/humble/setup.bash; \
   cd /home/ubuntu/shared/armpi_ext_cam; \
   exec python3 ext_cam_node.py \
     --extrinsic front_camera_extrinsic.json \
     --intrinsics uvc_color_intrinsics_refined.json"
