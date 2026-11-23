#!/usr/bin/env bash
# Start the pose/motion gateway (8766) inside the vendor container.
#
# Installed as /usr/local/lib/armpi-gateway/armpi-gateway-pose-run.sh and
# started explicitly with `systemctl start armpi-gateway-pose`. It is
# deliberately not enabled at boot: the motion gateway requires an operator.
set -euo pipefail

CONTAINER="${ARMPI_GATEWAY_POSE_CONTAINER:-ArmPiUltra}"
BIND="${ARMPI_GATEWAY_POSE_BIND:?ARMPI_GATEWAY_POSE_BIND is not set}"
PORT="${ARMPI_GATEWAY_POSE_PORT:-8766}"
DEPLOY_DIR="${ARMPI_GATEWAY_POSE_DEPLOY_DIR:-/home/ubuntu/shared/armpi_gateway_pose}"
TOKEN_FILE="${ARMPI_GATEWAY_POSE_TOKEN_FILE:-/home/ubuntu/shared/armpi_gateway_deploy/token}"
VENDOR_SETUP="${ARMPI_GATEWAY_POSE_VENDOR_SETUP:-/home/ubuntu/ros2_ws/install/setup.zsh}"
MOTION_CONFIG="${ARMPI_GATEWAY_POSE_MOTION_CONFIG:-${DEPLOY_DIR}/takeover_limits.json}"
POINTCLOUD_TOPIC="${ARMPI_GATEWAY_POSE_POINTCLOUD_TOPIC:-/depth_cam/depth_registered/points}"
CAMERA_TOPIC="${ARMPI_GATEWAY_POSE_CAMERA_TOPIC:-/armpi_pose_camera_off}"
CONTAINER_USER="${ARMPI_GATEWAY_POSE_CONTAINER_USER:-ubuntu}"
WAIT_SECONDS="${ARMPI_GATEWAY_POSE_WAIT_S:-60}"

echo "waiting for container ${CONTAINER} (up to ${WAIT_SECONDS}s)"
deadline=$((SECONDS + WAIT_SECONDS))
until docker inspect -f '{{.State.Running}}' "${CONTAINER}" 2>/dev/null | grep -qx true; do
  if (( SECONDS >= deadline )); then
    echo "ERROR: container ${CONTAINER} is not running after ${WAIT_SECONDS}s" >&2
    exit 1
  fi
  sleep 2
done

quoted_bind=$(printf '%q' "${BIND}")
quoted_port=$(printf '%q' "${PORT}")
quoted_setup=$(printf '%q' "${VENDOR_SETUP}")
quoted_dir=$(printf '%q' "${DEPLOY_DIR}")
quoted_token=$(printf '%q' "${TOKEN_FILE}")
quoted_config=$(printf '%q' "${MOTION_CONFIG}")
quoted_cloud=$(printf '%q' "${POINTCLOUD_TOPIC}")
quoted_camera=$(printf '%q' "${CAMERA_TOPIC}")

exec docker exec -u "${CONTAINER_USER}" \
  "${CONTAINER}" /bin/zsh -lc \
  "source ~/.zshrc >/dev/null 2>&1; \
   source /opt/ros/humble/setup.zsh; \
   source ${quoted_setup}; \
   cd ${quoted_dir}; \
   exec python3 -m armpi_debug_gateway.main \
     --host ${quoted_bind} --port ${quoted_port} \
     --token-file ${quoted_token} \
     --enable-motion --motion-config ${quoted_config} \
     --pointcloud-topic ${quoted_cloud} \
     --no-aux-camera --camera-topic ${quoted_camera}"
