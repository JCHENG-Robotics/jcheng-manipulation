#!/usr/bin/env bash
# Rotate the gateway shared token and restart the service.
#
# Run as root on the robot:
#   sudo bash rotate-token.sh
#
# The new token is written to the shared token file and never printed. Update
# the backend's ARMPI_GATEWAY_TOKEN afterwards, then restart the backend.
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "ERROR: run as root (sudo bash rotate-token.sh)" >&2
  exit 1
fi

ENV_FILE=/etc/armpi-gateway/gateway.env
if [[ ! -f "${ENV_FILE}" ]]; then
  echo "ERROR: ${ENV_FILE} not found; run install-gateway.sh first" >&2
  exit 1
fi

# shellcheck disable=SC1090
set -a
source "${ENV_FILE}"
set +a
HOST_DEPLOY_DIR="${ARMPI_GATEWAY_HOST_DEPLOY_DIR:-/home/pi/docker/tmp/armpi_gateway_deploy}"
TOKEN_FILE="${HOST_DEPLOY_DIR}/token"
BIND="${ARMPI_GATEWAY_BIND:?ARMPI_GATEWAY_BIND missing}"
PORT="${ARMPI_GATEWAY_PORT:-8765}"
HEALTH_HOST="${BIND}"
if [[ "${BIND}" == "0.0.0.0" || "${BIND}" == "::" ]]; then
  HEALTH_HOST=127.0.0.1
fi

umask 077
openssl rand -hex 24 > "${TOKEN_FILE}"
chmod 600 "${TOKEN_FILE}"
chown 1000:1000 "${TOKEN_FILE}" 2>/dev/null || true

systemctl restart armpi-gateway.service

TOKEN=$(cat "${TOKEN_FILE}")
deadline=$((SECONDS + 60))
until curl -fsS -H "X-Armpi-Token: ${TOKEN}" "http://${HEALTH_HOST}:${PORT}/health" >/dev/null 2>&1; do
  if (( SECONDS >= deadline )); then
    echo "ERROR: gateway did not become healthy after token rotation" >&2
    journalctl -u armpi-gateway.service -n 40 --no-pager >&2 || true
    exit 1
  fi
  sleep 2
done

echo "token rotated; update the backend ARMPI_GATEWAY_TOKEN from ${TOKEN_FILE}"
