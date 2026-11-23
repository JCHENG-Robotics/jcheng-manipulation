#!/usr/bin/env bash
# Run every check that does not need the robot. This is the local counterpart
# of the python-analysis and studio CI workflows.
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"

echo "== config consistency =="
python3 tools/verify_config_consistency.py

echo "== pure-Python suites =="
PYTHONPATH=packages/jcheng_core:ros2_ws/src/armpi_analysis python3 -m pytest -q \
  ros2_ws/src/armpi_analysis/test/test_error_budget.py \
  ros2_ws/src/armpi_analysis/test/test_paired_trials.py \
  ros2_ws/src/armpi_analysis/test/test_host_benchmark.py \
  tools/phase2/tests tools/phase3/tests tools/phase4/tests tools/tests

echo "== studio backend =="
cd "$ROOT/armpi-debug-studio/backend"
if [[ ! -x .venv/bin/python ]]; then
  python3 -m venv .venv
  .venv/bin/pip install --upgrade pip
  .venv/bin/pip install -e '.[dev]'
  .venv/bin/pip install -e ../../packages/jcheng_core --no-deps
fi
.venv/bin/python -m pytest -q

echo "== studio gateway =="
cd "$ROOT/armpi-debug-studio/ros2_gateway"
../backend/.venv/bin/python -m pytest -q

echo "== frontend build + mock Playwright =="
cd "$ROOT/armpi-debug-studio/frontend"
if [[ ! -d node_modules ]]; then
  npm ci
fi
npm run build
npm run test:e2e
E2E_POSE=1 npm run test:e2e

echo "OFFLINE CHECKS PASSED"
