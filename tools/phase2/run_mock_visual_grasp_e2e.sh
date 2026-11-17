#!/usr/bin/env bash
# Mock-only visual-grasp E2E: synthetic AprilTag -> localizer -> bridge ->
# grasp search -> safety veto. Nothing is executed on hardware.
#
# Usage: tools/phase2/run_mock_visual_grasp_e2e.sh [output-directory]
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT_DIR="${1:-${REPO}/evidence/phase2/session3_20261117}"
mkdir -p "${OUT_DIR}"

if [ -z "${ROS_DISTRO:-}" ]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
fi
set +u
# shellcheck disable=SC1091
source "${REPO}/ros2_ws/install/setup.bash"
set -u

# Keep the mock chain away from the real robot and any other MoveIt instance.
export ROS_DOMAIN_ID="${MOCK_ROS_DOMAIN_ID:-77}"
export ROS_LOCALHOST_ONLY=1

if ros2 service list 2>/dev/null | grep -q '^/compute_fk$'; then
  echo "ERROR: /compute_fk already exists; stop the other MoveIt instance" >&2
  exit 3
fi

LOG="${OUT_DIR}/mock_visual_grasp_launch.txt"
BRIDGE_LOG="${OUT_DIR}/mock_visual_grasp_bridge.txt"
BRIDGE_JSON="${OUT_DIR}/mock_visual_grasp_bridge.json"

setsid ros2 launch armpi_grasp_planner mock_visual_grasp.launch.py \
  use_rviz:=false > "${LOG}" 2>&1 &
LAUNCH_PID=$!
cleanup() {
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT -"${LAUNCH_PID}" 2>/dev/null || true
    for _ in $(seq 1 10); do
      kill -0 "${LAUNCH_PID}" 2>/dev/null || break
      sleep 0.5
    done
    kill -TERM -"${LAUNCH_PID}" 2>/dev/null || true
    for _ in $(seq 1 10); do
      kill -0 "${LAUNCH_PID}" 2>/dev/null || break
      sleep 0.5
    done
    kill -KILL -"${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

READY=0
for _ in $(seq 1 60); do
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "ERROR: launch exited during startup; see ${LOG}" >&2
    exit 4
  fi
  if ros2 service list 2>/dev/null | grep -q '^/armpi_safety/validate_trajectory$' \
    && ros2 service list 2>/dev/null | grep -q '^/compute_fk$' \
    && ros2 topic list 2>/dev/null | grep -q '^/target_pose$'; then
    READY=1
    break
  fi
  sleep 1
done
if [ "${READY}" -ne 1 ]; then
  echo "ERROR: E2E interfaces did not become ready; see ${LOG}" >&2
  exit 5
fi

ros2 node list > "${OUT_DIR}/mock_visual_grasp_nodes.txt" 2>&1
ros2 service list > "${OUT_DIR}/mock_visual_grasp_services.txt" 2>&1

timeout 120 ros2 run armpi_grasp_planner grasp_target_bridge \
  --safety-service /armpi_safety/validate_trajectory \
  --include-trajectory \
  --json-out "${BRIDGE_JSON}" \
  --search-json-out "${OUT_DIR}/mock_visual_grasp_search.json" \
  > "${BRIDGE_LOG}" 2>&1
BRIDGE_STATUS=$?
echo "bridge exit=${BRIDGE_STATUS}" >> "${BRIDGE_LOG}"

if [ "${BRIDGE_STATUS}" -ne 0 ]; then
  echo "ERROR: bridge failed with exit ${BRIDGE_STATUS}; see ${BRIDGE_LOG}" >&2
  exit 6
fi

python3 -I - "${BRIDGE_JSON}" <<'PY'
import json
import sys

with open(sys.argv[1], 'r', encoding='utf-8') as handle:
    report = json.load(handle)
search = report.get('search_summary') or {}
counts = search.get('counts') or {}
best = search.get('best') or {}
safety = best.get('safety') or {}
print('captured={}'.format(report.get('success')))
print('plan_ok={}'.format(counts.get('plan_ok')))
print('safety_ok={}'.format(counts.get('safety_ok')))
print('best_status={}'.format(best.get('status')))
print('safety_code={}'.format(safety.get('code')))
if not report.get('success'):
    raise SystemExit(1)
if not safety.get('approved'):
    raise SystemExit(1)
PY
SUMMARY_STATUS=$?
if [ "${SUMMARY_STATUS}" -ne 0 ]; then
  echo "ERROR: E2E result is not a safe plan; see ${BRIDGE_JSON}" >&2
  exit 7
fi

timeout 60 python3 "${REPO}/tools/phase2/safety_service_check.py" \
  --input "${BRIDGE_JSON}" \
  > "${OUT_DIR}/safety_service_check.txt" 2>&1
SAFETY_STATUS=$?
echo "safety service check exit=${SAFETY_STATUS}" \
  >> "${OUT_DIR}/safety_service_check.txt"
if [ "${SAFETY_STATUS}" -ne 0 ]; then
  echo "ERROR: safety service negative controls failed" >&2
  exit 8
fi

echo "E2E PASS" | tee "${OUT_DIR}/mock_visual_grasp_result.txt"
