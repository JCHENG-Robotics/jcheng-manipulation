#!/usr/bin/env bash
# Start the ArmPi Debug Studio backend with the safety checks this project
# learned the hard way (see docs/hardware/ros_environment.md and
# armpi-debug-studio/ros2_gateway/deploy/README.md).
#
# Hardware (read-only) mode, the default:
#   tools/studio/start_studio.sh
#
# Mock mode (no robot, no ROS):
#   tools/studio/start_studio.sh --mock
#
# Pose takeover (G7) against the pose gateway on 8766. Requires the pose
# gateway instance and the fixed ARMPI_TAKEOVER_ACK phrase:
#   ARMPI_TAKEOVER_ACK=I_AM_AT_THE_ROBOT_WITH_A_CLEAR_WORKSPACE \
#     tools/studio/start_studio.sh --takeover
#
# The script does not move the arm by itself. It starts the camera launch only
# when the camera has no publisher, because that launch is manual and dies
# silently after a container restart or a SIGKILL.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BACKEND_DIR="$REPO_ROOT/armpi-debug-studio/backend"
PORT="${PORT:-8000}"
HOST="${HOST:-127.0.0.1}"
ROBOT_HOST="${ROBOT_HOST:-10.42.0.218}"   # direct Ethernet (the only inbound path since 2026-11-18)
GATEWAY_PORT="${GATEWAY_PORT:-}"
CAMERA_GATEWAY_PORT="${CAMERA_GATEWAY_PORT:-}"
SSH_KEY="${SSH_KEY:-$HOME/.ssh/id_ed25519_armpi}"
SSH_USER="${SSH_USER:-pi}"
CONTAINER="${CONTAINER:-ArmPiUltra}"
TOKEN_FILE="${TOKEN_FILE:-/home/pi/docker/tmp/armpi_gateway_deploy/token}"
DB_PATH="${DB_PATH:-/tmp/armpi_studio_hardware_$(date +%Y%m%d).db}"
LOG="${LOG:-/tmp/studio_studio_${PORT}.log}"
MODE="hardware"
START_CAMERA=1
TAKEOVER=0

usage() {
    sed -n '2,16p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    cat <<'EOF'

Options:
  --mock            run the mock adapter (no robot, no ROS)
  --takeover        run ARMPI_MOTION_SCOPE=pose against the 8766 gateway;
                    requires the fixed ARMPI_TAKEOVER_ACK phrase
  --no-camera       do not start vision.launch.py when the camera is down
  -h, --help        show this help

Environment:
  PORT (8000), HOST (127.0.0.1), ROBOT_HOST (10.42.0.218, direct Ethernet)
  GATEWAY_PORT (8765, or 8766 with --takeover), SSH_KEY, SSH_USER (pi),
  CAMERA_GATEWAY_PORT (8765 with --takeover; camera-only gateway),
  CONTAINER (ArmPiUltra), ARMPI_GATEWAY_TOKEN,
  ARMPI_TAKEOVER_ACK (required by --takeover), DB_PATH, LOG
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mock) MODE="mock"; shift ;;
        --takeover) TAKEOVER=1; shift ;;
        --no-camera) START_CAMERA=0; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ "$TAKEOVER" == "1" ]]; then
    GATEWAY_PORT="${GATEWAY_PORT:-8766}"
    # The pose gateway is motion-only; the persistent read-only 8765 instance
    # owns both cameras (the UVC aux device is exclusive). Route camera
    # endpoints to it instead of disabling the Dashboard.
    CAMERA_GATEWAY_PORT="${CAMERA_GATEWAY_PORT:-8765}"
    if [[ "$MODE" != "hardware" ]]; then
        echo "--takeover is a hardware mode option" >&2
        exit 2
    fi
    if [[ "${ARMPI_TAKEOVER_ACK:-}" != "I_AM_AT_THE_ROBOT_WITH_A_CLEAR_WORKSPACE" ]]; then
        echo "FAIL: --takeover needs ARMPI_TAKEOVER_ACK=I_AM_AT_THE_ROBOT_WITH_A_CLEAR_WORKSPACE" >&2
        exit 2
    fi
else
    GATEWAY_PORT="${GATEWAY_PORT:-8765}"
fi

step() { printf '\n== %s\n' "$*"; }
fail() { printf 'FAIL: %s\n' "$*" >&2; exit 1; }

# 1. Port ---------------------------------------------------------------
step "port $PORT"
if ss -ltnp 2>/dev/null | grep -q ":$PORT "; then
    holders="$(ss -ltnp 2>/dev/null | grep ":$PORT " | grep -o 'pid=[0-9]*' | cut -d= -f2 | sort -u | tr '\n' ' ')"
    printf 'port %s is already in use by pid(s): %s\n' "$PORT" "$holders"
    for pid in $holders; do
        if [[ -r "/proc/$pid/cmdline" ]] && tr '\0' ' ' < "/proc/$pid/cmdline" | grep -q 'app.main:create_app'; then
            printf '  killing stale studio backend pid %s\n' "$pid"
            kill "$pid" 2>/dev/null || true
        else
            fail "pid $pid is not a studio backend; refusing to kill it"
        fi
    done
    for _ in $(seq 1 15); do
        ss -ltnp 2>/dev/null | grep -q ":$PORT " || break
        sleep 1
    done
    ss -ltnp 2>/dev/null | grep -q ":$PORT " && fail "port $PORT still busy"
fi
echo "free"

# 2. Token and gateway --------------------------------------------------
if [[ "$MODE" == "mock" ]]; then
    step "mode mock (no gateway, no robot access)"
    export ARMPI_MODE=mock
    [[ "$DB_PATH" == *hardware* ]] && DB_PATH="${DB_PATH//hardware/mock}"
    export DB_PATH
else
    step "gateway token"
    if [[ -n "${ARMPI_GATEWAY_TOKEN:-}" ]]; then
        TOKEN="$ARMPI_GATEWAY_TOKEN"
        echo "using ARMPI_GATEWAY_TOKEN from the environment"
    else
        TOKEN="$(timeout 25 ssh -o BatchMode=yes -o ConnectTimeout=8 \
            -i "$SSH_KEY" "$SSH_USER@$ROBOT_HOST" \
            "sudo cat '$TOKEN_FILE' 2>/dev/null" || true)"
        [[ -n "$TOKEN" ]] || fail "cannot read $TOKEN_FILE on $ROBOT_HOST (check SSH reachability and the token path)"
        echo "read ${#TOKEN}-char token from $ROBOT_HOST:$TOKEN_FILE"
    fi

    step "gateway http://$ROBOT_HOST:$GATEWAY_PORT"
    health=""
    for _ in $(seq 1 10); do
        health="$(curl -s -m 4 --noproxy '*' -H "X-Armpi-Token: $TOKEN" \
            "http://$ROBOT_HOST:$GATEWAY_PORT/health" || true)"
        [[ -n "$health" ]] && break
        sleep 1
    done
    [[ -n "$health" ]] || fail "gateway unreachable; check: sudo systemctl status armpi-gateway on the robot"
    echo "$health" | grep -q '"status": *"ok"' \
        || fail "gateway rejected the token: $health"
    echo "$health" | grep -q '"ros_ready": *true' || fail "gateway up but ROS is not ready"
    echo "gateway ok"

    # 3. Camera ---------------------------------------------------------
    # In takeover mode the motion gateway (8766) is camera-free; the
    # persistent read-only gateway (8765) serves both cameras.
    CAMERA_GATEWAY_PORT="${CAMERA_GATEWAY_PORT:-$GATEWAY_PORT}"
    step "camera gateway http://$ROBOT_HOST:$CAMERA_GATEWAY_PORT"
    camera_health=""
    for _ in $(seq 1 10); do
        camera_health="$(curl -s -m 4 --noproxy '*' -H "X-Armpi-Token: $TOKEN" \
            "http://$ROBOT_HOST:$CAMERA_GATEWAY_PORT/health" || true)"
        [[ -n "$camera_health" ]] && break
        sleep 1
    done
    if [[ -z "$camera_health" ]]; then
        echo "WARN: camera gateway unreachable; camera panels will be unavailable"
    elif ! echo "$camera_health" | grep -q '"status": *"ok"'; then
        echo "WARN: camera gateway health is not ok: $camera_health"
    fi
    if echo "$camera_health" | grep -q '"camera_ready": *true'; then
        echo "camera already publishing"
    elif [[ "$START_CAMERA" == "1" ]]; then
        step "camera: no publisher, starting vision.launch.py"
        # The aurora driver is only registered by the isolated install prefix
        # (the third_party_ws overlay's local_setup symlinks dangle since its
        # build/ tree was deleted). Without it the launch dies with
        # "package 'deptrum-ros-driver-aurora930' not found" while
        # `docker exec -d` still returns 0. See docs/hardware/ros_environment.md.
        AURORA_PREFIX=/home/ubuntu/jcheng_aurora_install/deptrum-ros-driver-aurora930
        timeout 30 ssh -o BatchMode=yes -o ConnectTimeout=8 \
            -i "$SSH_KEY" "$SSH_USER@$ROBOT_HOST" \
            "sudo docker exec -d -u ubuntu -w /home/ubuntu $CONTAINER /bin/zsh -c \
             'source ~/.zshrc >/dev/null 2>&1; \
              export AMENT_PREFIX_PATH=$AURORA_PREFIX:\$AMENT_PREFIX_PATH; \
              export LD_LIBRARY_PATH=$AURORA_PREFIX/lib:\$LD_LIBRARY_PATH; \
              ros2 pkg prefix deptrum-ros-driver-aurora930 >/dev/null 2>&1 \
                || { echo aurora-driver-not-resolvable > /tmp/vision.log; exit 3; }; \
              exec ros2 launch bringup vision.launch.py > /tmp/vision.log 2>&1'" \
            || fail "could not start vision.launch.py"
        echo "launch started; waiting for frames (log: /tmp/vision.log on the robot)"
        ready=0
        for _ in $(seq 1 20); do
            status="$(curl -s -m 4 --noproxy '*' -H "X-Armpi-Token: $TOKEN" \
                "http://$ROBOT_HOST:$CAMERA_GATEWAY_PORT/camera/status" || true)"
            if echo "$status" | grep -q '"available": *true'; then ready=1; break; fi
            sleep 2
        done
        if [[ "$ready" == "1" ]]; then
            echo "camera ready"
        else
            echo "WARN: camera still has no frames; battery and joints will still work"
            echo "      check: docker exec $CONTAINER tail -40 /tmp/vision.log"
        fi
    else
        echo "WARN: camera not publishing and --no-camera was passed"
    fi

    export ARMPI_MODE=hardware
    export ARMPI_GATEWAY_URL="http://$ROBOT_HOST:$GATEWAY_PORT"
    export ARMPI_GATEWAY_TOKEN="$TOKEN"
    export ARMPI_CAMERA_GATEWAY_URL="http://$ROBOT_HOST:$CAMERA_GATEWAY_PORT"
    export ARMPI_CAMERA_GATEWAY_TOKEN="$TOKEN"
    export DB_PATH="$DB_PATH"
    if [[ "$TAKEOVER" == "1" ]]; then
        export ARMPI_HARDWARE_MOTION=1
        export ARMPI_MOTION_SCOPE=pose
        export ARMPI_TAKEOVER_ACK
    fi
fi

# 4. Backend ------------------------------------------------------------
step "backend on http://$HOST:$PORT"
[[ -x "$BACKEND_DIR/.venv/bin/python" ]] \
    || fail "missing $BACKEND_DIR/.venv; create it with: python3 -m venv .venv && .venv/bin/pip install -e '.[dev]'"
cd "$BACKEND_DIR"
nohup .venv/bin/python -m uvicorn app.main:create_app --factory \
    --host "$HOST" --port "$PORT" > "$LOG" 2>&1 &
BACKEND_PID=$!
echo "pid $BACKEND_PID, log $LOG"

up=0
for _ in $(seq 1 25); do
    if curl -sf -m 3 "http://$HOST:$PORT/api/health" >/dev/null; then up=1; break; fi
    kill -0 "$BACKEND_PID" 2>/dev/null || { tail -20 "$LOG" >&2; fail "backend exited during startup"; }
    sleep 1
done
[[ "$up" == "1" ]] || fail "backend did not answer /api/health within 25 s"

# 5. Report -------------------------------------------------------------
step "status"
curl -s "http://$HOST:$PORT/api/health"; echo
peripheral_status() {
    curl -s "http://$HOST:$PORT/api/peripherals" -o "/tmp/studio_peripherals_$PORT.json"
    python3 - "/tmp/studio_peripherals_$PORT.json" <<'PY'
import json, sys

with open(sys.argv[1]) as handle:
    per = json.load(handle)
for name in ("battery", "camera", "aux_camera"):
    data = per.get(name, {})
    if not data.get("available"):
        print("  %-8s UNAVAILABLE" % name)
    elif name == "battery":
        print("  %-8s %.3f V  %s%%  truth=%s" % (
            name, data["voltage_v"], data["percent_estimate"], data["truth"]))
    else:
        print("  %-8s %sx%s %s  %s fps  truth=%s" % (
            name, data["width"], data["height"], data["encoding"],
            data["fps"], data["truth"]))
PY
}
peripheral_status
cat <<EOF

Open http://$HOST:$PORT
Stop with: kill $BACKEND_PID
EOF
