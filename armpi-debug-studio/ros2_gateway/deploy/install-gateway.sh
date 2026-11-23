#!/usr/bin/env bash
# Install the read-only gateway as a systemd service on the robot host.
#
# Run as root on the Pi, with this deploy/ directory copied to the robot:
#   sudo bash install-gateway.sh
#
# The script copies the Python package into the shared host directory that the
# vendor container mounts, creates a token file if one does not exist, installs
# the unit, and waits for /health. It does not modify the vendor container or
# the ROS stack.
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "ERROR: run as root (sudo bash install-gateway.sh)" >&2
  exit 1
fi

SELF_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SRC_DIR=$(cd "${SELF_DIR}/.." && pwd)
HOST_DEPLOY_DIR="${ARMPI_GATEWAY_HOST_DEPLOY_DIR:-/home/pi/docker/tmp/armpi_gateway_deploy}"
INSTALL_DIR=/usr/local/lib/armpi-gateway
ENV_DIR=/etc/armpi-gateway
UNIT_PATH=/etc/systemd/system/armpi-gateway.service
TOKEN_FILE="${HOST_DEPLOY_DIR}/token"
# The host wrapper only needs `docker exec`; it does not need root. `pi` is in
# the docker group on the robot. Override these for a different robot account.
SERVICE_USER="${ARMPI_GATEWAY_SERVICE_USER:-pi}"
SERVICE_GROUP="${ARMPI_GATEWAY_SERVICE_GROUP:-docker}"

if ! id -u "${SERVICE_USER}" >/dev/null 2>&1; then
  echo "ERROR: service user ${SERVICE_USER} does not exist" >&2
  exit 1
fi
if ! getent group "${SERVICE_GROUP}" >/dev/null 2>&1; then
  echo "ERROR: service group ${SERVICE_GROUP} does not exist" >&2
  exit 1
fi
RUNUSER_BIN=$(command -v runuser || true)
if [[ -z "${RUNUSER_BIN}" ]]; then
  echo "ERROR: runuser is required to check the service account" >&2
  exit 1
fi
if ! timeout 10 "${RUNUSER_BIN}" -u "${SERVICE_USER}" -- docker info >/dev/null 2>&1; then
  echo "ERROR: ${SERVICE_USER} cannot access the Docker daemon (docker group)" >&2
  exit 1
fi

install -d -m 755 "${HOST_DEPLOY_DIR}" "${INSTALL_DIR}" "${ENV_DIR}"

# Package. The token file lives beside it and is preserved.
rm -rf "${HOST_DEPLOY_DIR}/armpi_debug_gateway"
cp -a "${SRC_DIR}/armpi_debug_gateway" "${HOST_DEPLOY_DIR}/armpi_debug_gateway"
find "${HOST_DEPLOY_DIR}/armpi_debug_gateway" -name __pycache__ -type d -prune -exec rm -rf {} +

if [[ ! -s "${TOKEN_FILE}" ]]; then
  umask 077
  openssl rand -hex 24 > "${TOKEN_FILE}"
  echo "created token ${TOKEN_FILE}"
fi
chmod 600 "${TOKEN_FILE}"
# Container user `ubuntu` is uid 1000 in the vendor image; keep it readable
# across the shared bind mount.
chown 1000:1000 "${TOKEN_FILE}" 2>/dev/null || true

install -m 755 "${SELF_DIR}/armpi-gateway-run.sh" "${INSTALL_DIR}/armpi-gateway-run.sh"
UNIT_TMP=$(mktemp)
sed \
  -e "s|@ARMPI_SERVICE_USER@|${SERVICE_USER}|g" \
  -e "s|@ARMPI_SERVICE_GROUP@|${SERVICE_GROUP}|g" \
  "${SELF_DIR}/armpi-gateway.service" > "${UNIT_TMP}"
install -m 644 "${UNIT_TMP}" "${UNIT_PATH}"
rm -f "${UNIT_TMP}"
install -m 644 "${SELF_DIR}/README.md" "${INSTALL_DIR}/README.md"
if [[ ! -f "${ENV_DIR}/gateway.env" ]]; then
  install -m 640 -g "${SERVICE_GROUP}" \
    "${SELF_DIR}/gateway.env.example" "${ENV_DIR}/gateway.env"
  echo "created ${ENV_DIR}/gateway.env (review ARMPI_GATEWAY_BIND before starting)"
else
  # The unit now runs as ${SERVICE_USER}:${SERVICE_GROUP}; keep the env file
  # readable by that group after an upgrade from the root service.
  chgrp "${SERVICE_GROUP}" "${ENV_DIR}/gateway.env"
  chmod 640 "${ENV_DIR}/gateway.env"
fi

# shellcheck disable=SC1091
set -a
source "${ENV_DIR}/gateway.env"
set +a
BIND="${ARMPI_GATEWAY_BIND:?ARMPI_GATEWAY_BIND missing from ${ENV_DIR}/gateway.env}"
PORT="${ARMPI_GATEWAY_PORT:-8765}"
HEALTH_HOST="${BIND}"
if [[ "${BIND}" == "0.0.0.0" || "${BIND}" == "::" ]]; then
  HEALTH_HOST=127.0.0.1
fi

systemctl daemon-reload
systemctl enable armpi-gateway.service
systemctl restart armpi-gateway.service

TOKEN=$(cat "${TOKEN_FILE}")
deadline=$((SECONDS + 60))
until curl -fsS -H "X-Armpi-Token: ${TOKEN}" "http://${HEALTH_HOST}:${PORT}/health" >/dev/null 2>&1; do
  if (( SECONDS >= deadline )); then
    echo "ERROR: gateway did not become healthy in 60s" >&2
    systemctl status armpi-gateway.service --no-pager >&2 || true
    journalctl -u armpi-gateway.service -n 40 --no-pager >&2 || true
    exit 1
  fi
  sleep 2
done

echo "armpi-gateway is healthy on http://${HEALTH_HOST}:${PORT}"
echo "token file: ${TOKEN_FILE} (not printed)"
echo "set the backend ARMPI_GATEWAY_TOKEN from that file"
