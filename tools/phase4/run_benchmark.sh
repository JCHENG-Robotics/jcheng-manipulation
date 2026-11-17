#!/usr/bin/env bash
# Read-only host and ROS/DDS benchmark collection.
#
# It measures the laptop and the ArmPi host/container, then a ping RTT. No
# command is sent to a controller and no motion is possible. The robot may be
# reached over SSH; the container ROS measurement runs inside the vendor
# container with its own CycloneDDS environment.
#
# Usage:
#   tools/phase4/run_benchmark.sh [output-directory]
#
# Environment overrides:
#   ROBOT_HOST=10.42.0.218
#   SSH_KEY=~/.ssh/id_ed25519_armpi
#   CONTAINER=ArmPiUltra
#   DURATION=10
#   TOPICS=/joint_states,/controller_manager/joint_states
#   LAPTOP_ROS_SETUP=/opt/ros/jazzy/setup.bash
#   SKIP_ROBOT=1        # laptop-only run
#   SKIP_LAPTOP_ROS=1   # do not run the laptop ROS subscriber
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT_DIR="${1:-${REPO}/evidence/phase3/session4_20261117}"
ROBOT_HOST="${ROBOT_HOST:-10.42.0.218}"
SSH_KEY="${SSH_KEY:-${HOME}/.ssh/id_ed25519_armpi}"
CONTAINER="${CONTAINER:-ArmPiUltra}"
DURATION="${DURATION:-10}"
TOPICS="${TOPICS:-/joint_states,/controller_manager/joint_states,/ros_robot_controller/battery}"
LAPTOP_ROS_SETUP="${LAPTOP_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
BENCH="${REPO}/ros2_ws/src/armpi_analysis/armpi_analysis/host_benchmark.py"
SSH_OPTS=(
  -i "${SSH_KEY}"
  -o BatchMode=yes
  -o ConnectTimeout=5
  -o StrictHostKeyChecking=no
)

mkdir -p "${OUT_DIR}"
if [ ! -f "${BENCH}" ]; then
  echo "ERROR: benchmark script not found: ${BENCH}" >&2
  exit 2
fi

echo "BENCHMARK output=${OUT_DIR} robot=${ROBOT_HOST} topics=${TOPICS}"

echo "STEP laptop_host"
python3 "${BENCH}" --mode host --sample-s 1.0 \
  --json-out "${OUT_DIR}/laptop_host.json" \
  > "${OUT_DIR}/laptop_host.txt" 2>&1 || {
  echo "ERROR: laptop host benchmark failed" >&2
  exit 3
}

echo "STEP ping"
ping -c 10 -i 0.2 -W 1 "${ROBOT_HOST}" \
  > "${OUT_DIR}/ping_robot.txt" 2>&1 || true

if [ "${SKIP_ROBOT:-0}" != "1" ]; then
  if ssh "${SSH_OPTS[@]}" "pi@${ROBOT_HOST}" 'echo ok' \
      > /dev/null 2>&1; then
    echo "STEP robot_host"
    ssh "${SSH_OPTS[@]}" "pi@${ROBOT_HOST}" \
      'python3 - --mode host --sample-s 1.0 \
        --json-out /tmp/armpi_host_benchmark.json' \
      < "${BENCH}" > "${OUT_DIR}/robot_host.txt" 2>&1 || true
    ssh "${SSH_OPTS[@]}" "pi@${ROBOT_HOST}" \
      'cat /tmp/armpi_host_benchmark.json' \
      > "${OUT_DIR}/robot_host.json" 2>/dev/null || true
    ssh "${SSH_OPTS[@]}" "pi@${ROBOT_HOST}" \
      'rm -f /tmp/armpi_host_benchmark.json' > /dev/null 2>&1 || true

    if ssh "${SSH_OPTS[@]}" "pi@${ROBOT_HOST}" \
        "sudo -n docker ps --format '{{.Names}}'" 2>/dev/null \
        | grep -qx "${CONTAINER}"; then
      echo "STEP robot_ros"
      ssh "${SSH_OPTS[@]}" "pi@${ROBOT_HOST}" \
        "sudo -n docker exec -i -u ubuntu -w /home/ubuntu ${CONTAINER} \
          /bin/zsh -c 'source ~/.zshrc >/dev/null 2>&1; \
          exec python3 - --mode ros --topics ${TOPICS} \
          --duration-s ${DURATION} \
          --json-out /tmp/armpi_ros_benchmark.json'" \
        < "${BENCH}" > "${OUT_DIR}/robot_ros.txt" 2>&1 || true
      ssh "${SSH_OPTS[@]}" "pi@${ROBOT_HOST}" \
        "sudo -n docker exec ${CONTAINER} \
          cat /tmp/armpi_ros_benchmark.json" \
        > "${OUT_DIR}/robot_ros.json" 2>/dev/null || true
      ssh "${SSH_OPTS[@]}" "pi@${ROBOT_HOST}" \
        "sudo -n docker exec ${CONTAINER} \
          rm -f /tmp/armpi_ros_benchmark.json" \
        > /dev/null 2>&1 || true
    else
      echo "WARN container ${CONTAINER} is not running; skipping robot_ros"
    fi
  else
    echo "WARN robot ${ROBOT_HOST} is unreachable; skipping robot steps"
  fi
fi

if [ "${SKIP_LAPTOP_ROS:-0}" != "1" ] && [ -f "${LAPTOP_ROS_SETUP}" ]; then
  echo "STEP laptop_ros"
  # shellcheck disable=SC1090
  bash -c "source '${LAPTOP_ROS_SETUP}' >/dev/null 2>&1; \
    export ROS_DOMAIN_ID='${ROS_DOMAIN_ID:-0}'; \
    python3 '${BENCH}' --mode ros --topics '${TOPICS}' \
      --duration-s '${DURATION}' \
      --json-out '${OUT_DIR}/laptop_ros.json'" \
    > "${OUT_DIR}/laptop_ros.txt" 2>&1 || true
fi

echo "DONE ${OUT_DIR}"
ls -1 "${OUT_DIR}"
