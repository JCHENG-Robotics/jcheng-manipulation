#!/usr/bin/env bash
# Start the read-only gateway inside the vendor container, in the foreground.
#
# Installed on the robot host as /usr/local/lib/armpi-gateway/armpi-gateway-run.sh
# and started by armpi-gateway.service (systemd). It waits for the vendor
# container after a reboot, then execs `docker exec` so systemd tracks the
# gateway process. The token is read from a file inside the container; it is
# never placed on the docker command line.
set -euo pipefail

CONTAINER="${ARMPI_GATEWAY_CONTAINER:-ArmPiUltra}"
BIND="${ARMPI_GATEWAY_BIND:?ARMPI_GATEWAY_BIND is not set (see /etc/armpi-gateway/gateway.env)}"
PORT="${ARMPI_GATEWAY_PORT:-8765}"
DEPLOY_DIR="${ARMPI_GATEWAY_DEPLOY_DIR:-/home/ubuntu/shared/armpi_gateway_deploy}"
TOKEN_FILE="${ARMPI_GATEWAY_TOKEN_FILE:-${DEPLOY_DIR}/token}"
# Empty or unset follows the legacy UVC device; a topic makes the gateway
# subscribe to the robot-side ext_cam_node instead (single UVC owner).
AUX_CAMERA_TOPIC="${ARMPI_GATEWAY_AUX_CAMERA_TOPIC:-/ext_cam/color/image_raw}"
VENDOR_SETUP="${ARMPI_GATEWAY_VENDOR_SETUP:-/home/ubuntu/ros2_ws/install/setup.bash}"
CONTAINER_USER="${ARMPI_GATEWAY_CONTAINER_USER:-ubuntu}"
WAIT_SECONDS="${ARMPI_GATEWAY_CONTAINER_WAIT_S:-60}"

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
quoted_aux=$(printf '%q' "${AUX_CAMERA_TOPIC}")

exec docker exec -u "${CONTAINER_USER}" \
  "${CONTAINER}" /bin/bash -lc \
  "source /opt/ros/humble/setup.bash; \
   source ${quoted_setup}; \
   cd ${quoted_dir}; \
   exec python3 -m armpi_debug_gateway.main \
     --host ${quoted_bind} --port ${quoted_port} --token-file ${quoted_token} \
     --aux-camera-topic ${quoted_aux}"
