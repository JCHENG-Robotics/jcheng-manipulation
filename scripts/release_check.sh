#!/usr/bin/env bash
# Release gate for the offline baseline. Runs only checks that do not need the
# robot; the hardware evidence is recorded under evidence/.
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"

VERSION=$(tr -d '[:space:]' < VERSION)
echo "== release gate: $VERSION =="

if [[ "${RELEASE_ALLOW_DIRTY:-0}" != "1" ]] && [[ -n "$(git status --porcelain)" ]]; then
  echo "ERROR: the tree is dirty; commit or set RELEASE_ALLOW_DIRTY=1" >&2
  git status --short
  exit 1
fi

echo "== config consistency =="
python3 tools/verify_config_consistency.py

echo "== vendor baseline manifest =="
python3 tools/vendor_baseline_manifest.py --check

echo "== evidence manifest =="
python3 tools/check_evidence_manifest.py

echo "== analysis + safety + planner + fiducial =="
PYTHONPATH=packages/jcheng_core:ros2_ws/src/armpi_analysis:ros2_ws/src/armpi_safety:ros2_ws/src/armpi_grasp_planner:ros2_ws/src/armpi_fiducial_localizer \
  python3 -m pytest -q \
  ros2_ws/src/armpi_analysis/test/test_error_budget.py \
  ros2_ws/src/armpi_analysis/test/test_paired_trials.py \
  ros2_ws/src/armpi_analysis/test/test_host_benchmark.py \
  ros2_ws/src/armpi_safety/test/test_policy.py \
  ros2_ws/src/armpi_grasp_planner/test/test_joint_limits.py \
  ros2_ws/src/armpi_grasp_planner/test/test_candidates.py \
  ros2_ws/src/armpi_fiducial_localizer/test/test_detector.py \
  ros2_ws/src/armpi_fiducial_localizer/test/test_frames.py

echo "== offline tools =="
PYTHONPATH=packages/jcheng_core:ros2_ws/src/armpi_analysis:tools/phase2:tools/phase3:tools/phase4 \
  python3 -m pytest -q \
  tools/tests tools/phase2/tests tools/phase3/tests tools/phase4/tests

echo "== studio backend =="
cd "$ROOT/armpi-debug-studio/backend"
if [[ ! -x .venv/bin/python ]]; then
  echo "ERROR: backend .venv is missing; see README" >&2
  exit 1
fi
if ! .venv/bin/python -c 'import jcheng_stats' >/dev/null 2>&1; then
  .venv/bin/python -m pip install -e ../../packages/jcheng_core --no-deps
fi
.venv/bin/python -m pytest -q

echo "== studio gateway =="
cd "$ROOT/armpi-debug-studio/ros2_gateway"
../backend/.venv/bin/python -m pytest -q

echo "RELEASE CHECK PASSED: $VERSION"
