#!/usr/bin/env bash
# Start the backend in mock mode for the Playwright suite.
#
# Serves the built frontend from frontend/dist, so run `npm run build` first
# (the `pretest:e2e` npm script does this).
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
PORT="${E2E_PORT:-8766}"
DB_PATH="/tmp/armpi_studio_e2e_${PORT}.db"

if [[ ! -f "${ROOT}/frontend/dist/index.html" ]]; then
  echo "ERROR: frontend/dist is missing; run 'npm run build' first" >&2
  exit 1
fi

rm -f "${DB_PATH}" "${DB_PATH}-wal" "${DB_PATH}-shm"

export ARMPI_MODE=mock
export DB_PATH
export BIND_HOST=127.0.0.1
export PORT

# Use `python -m` instead of the venv's console script: this checkout was moved
# and the script shebang still points at the old absolute path.
exec "${ROOT}/backend/.venv/bin/python" -m uvicorn app.main:create_app \
  --factory \
  --app-dir "${ROOT}/backend" \
  --host 127.0.0.1 \
  --port "${PORT}" \
  --log-level warning
