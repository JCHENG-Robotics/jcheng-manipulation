#!/bin/bash
# Start the ArmPi camera launch inside the vendor container once the core graph
# is publishing. Called by armpi-vision.service at boot.
#
# The vendor core is started by start_node.service (bringup.launch.py). Starting
# vision.launch.py before the core publishes leaves the camera node unable to
# initialise, so wait for the core launcher process first. The check used to run
# `ros2 topic list` through a full `.zshrc` in the container; under load that
# takes tens of seconds per call and the unit hit TimeoutStartSec before the
# launch ever ran (2026-11-24), so it is a plain `ps` now.
#
# 2026-11-24: the third_party_ws overlay has been unusable since its build/ tree
# was deleted -- every per-package local_setup.* under install/ is a dangling
# symlink, so sourcing it silently adds nothing and `ros2 launch` cannot find
# `deptrum-ros-driver-aurora930`. The driver is instead installed in the
# isolated prefix /home/ubuntu/jcheng_aurora_install. Prepend that prefix and
# verify the launch actually produced a driver node; the old version returned 0
# from `docker exec -d` even when the launch died within a second.
#
# See docs/hardware/ros_environment.md (persistent camera launch).

CONTAINER="${CONTAINER:-ArmPiUltra}"
AURORA_PREFIX=/home/ubuntu/jcheng_aurora_install/deptrum-ros-driver-aurora930
TOKEN_FILE="${TOKEN_FILE:-/home/pi/docker/tmp/armpi_gateway_deploy/token}"
GATEWAY_URL="${GATEWAY_URL:-http://10.42.0.218:8765}"

# 1. Wait for the core launcher process (cheap: no ROS CLI, no shell rc files).
core_ready() {
    timeout 10 docker exec "$CONTAINER" /bin/sh -c \
        'ps -eo args | grep -q "[b]ringup.launch.py"' >/dev/null 2>&1
}
for _ in $(seq 1 120); do
    core_ready && break
    sleep 1
done

# 2. Launch. `exec` keeps the launch owned by the container; a `nohup ... &`
#    inside docker exec gets reaped and leaves a defunct aurora930_node.
#    If a previous launch died but left its driver alive, the new node cannot
#    open the device; only an *orphaned* node is killed (a live launch, e.g. the
#    perception stack's, is left alone).
if ! timeout 10 docker exec "$CONTAINER" /bin/sh -c \
    'ps -eo args | grep -q "[v]ision.launch.py"' >/dev/null 2>&1; then
    timeout 15 docker exec "$CONTAINER" pkill -f 'aurora930_node --ros-args' \
        >/dev/null 2>&1 || true
fi
timeout 20 docker exec -d -u ubuntu -w /home/ubuntu "$CONTAINER" /bin/zsh -c "
    source ~/.zshrc >/dev/null 2>&1
    export AMENT_PREFIX_PATH=$AURORA_PREFIX:\$AMENT_PREFIX_PATH
    export LD_LIBRARY_PATH=$AURORA_PREFIX/lib:\$LD_LIBRARY_PATH
    if ! ros2 pkg prefix deptrum-ros-driver-aurora930 >/dev/null 2>&1; then
        echo \"aurora driver not resolvable; AMENT_PREFIX_PATH=\$AMENT_PREFIX_PATH\" > /tmp/vision.log
        exit 3
    fi
    exec ros2 launch bringup vision.launch.py > /tmp/vision.log 2>&1
" || { echo "docker exec -d failed" >&2; exit 1; }

# 3. The launch must really have produced a driver node (a zombie shows up as
#    '[aurora930_node] <defunct>' without --ros-args, so match the full line).
node_running() {
    timeout 10 docker exec "$CONTAINER" /bin/sh -c \
        'ps -eo args | grep -q "[a]urora930_node --ros-args"' >/dev/null 2>&1
}
for _ in $(seq 1 40); do
    node_running && break
    sleep 1
done
if ! node_running; then
    echo "vision launch did not start aurora930_node" >&2
    timeout 10 docker exec -u ubuntu "$CONTAINER" tail -n 5 /tmp/vision.log 2>&1 || true
    exit 1
fi

# 4. A running driver is enough for the unit to be healthy; the gateway check is
#    informational because the gateway may still be starting at boot.
if [ -r "$TOKEN_FILE" ]; then
    token=$(cat "$TOKEN_FILE")
    for _ in $(seq 1 20); do
        if curl -s -m 3 --noproxy '*' -H "X-Armpi-Token: $token" \
            "$GATEWAY_URL/camera/status" | grep -q '"available": *true'; then
            exit 0
        fi
        sleep 1
    done
    echo "aurora930_node runs but the gateway has no frame yet" >&2
fi
exit 0
