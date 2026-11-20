#!/bin/bash
# ArmPi D435i depth driver: realsense2_camera (librealsense) inside the vendor
# container, depth only. Colour stays with armpi-ext-cam.service, which is
# ordered after this unit because the depth nodes must be opened first - if the
# colour node claims the UVC device first, librealsense fails VIDIOC_S_FMT with
# EBUSY.
#
# Boot ordering: start_node.sh runs ~/.stop_ros.sh first, which `kill -9`s every
# process whose command line contains "ros". systemd marks start_node.service as
# started long before that kill lands (~15 s later), so this script waits for
# the vendor launch process itself before touching anything.
#
# Cold-start wake-up: a freshly powered D435i can take a while before its
# depth/IR unit answers, and a depth stream opened too early stays at
# `Frames didn't arrived within 5 seconds`. Streaming the IR node for a few
# frames first brings the stereo unit up, so the wake-up is retried until it
# succeeds (bounded), and only then is the node started. The IR stream is not
# kept open afterwards: depth + IR + colour together drop the depth rate to
# ~3 Hz with stalls, while depth + colour runs at 4-8 Hz.
set -euo pipefail

CONTAINER="${ARMPI_RS_DEPTH_CONTAINER:-ArmPiUltra}"
WAIT_SECONDS="${ARMPI_RS_DEPTH_CONTAINER_WAIT_S:-60}"
VENDOR_TIMEOUT="${ARMPI_RS_DEPTH_VENDOR_S:-180}"
WAKE_TIMEOUT="${ARMPI_RS_DEPTH_WAKE_S:-90}"

deadline=$((SECONDS + WAIT_SECONDS))
until docker inspect -f '{{.State.Running}}' "${CONTAINER}" 2>/dev/null | grep -qx true; do
  if (( SECONDS >= deadline )); then
    echo "ERROR: container ${CONTAINER} is not running after ${WAIT_SECONDS}s" >&2
    exit 1
  fi
  sleep 2
done

# Wait for the vendor bringup (the kill in ~/.stop_ros.sh runs before it).
start=$SECONDS
until docker exec -u root "${CONTAINER}" pgrep -f "bringup.launch.py" >/dev/null 2>&1; do
  if (( SECONDS - start >= VENDOR_TIMEOUT )); then
    echo "WARNING: vendor bringup not seen within ${VENDOR_TIMEOUT}s; continuing" >&2
    break
  fi
  sleep 3
done

wake_ir() {
  docker exec -u root "${CONTAINER}" bash -lc '
    set -u
    ir=""
    for node in /sys/class/video4linux/video*; do
      name=$(basename "$node")
      [ "$(cat "$node/name" 2>/dev/null)" = "Intel(R) RealSense(TM) Depth Ca" ] || continue
      if v4l2-ctl -d "/dev/$name" --list-formats 2>/dev/null | grep -q GREY; then
        ir="/dev/$name"
        break
      fi
    done
    [ -n "$ir" ] || { echo "IR node not found"; exit 2; }
    timeout 8 v4l2-ctl -d "$ir" --stream-mmap --stream-count=3 >/dev/null 2>&1 || exit 1
    echo "woke the depth unit through ${ir}"
    exit 0
  '
}

start=$SECONDS
attempt=0
wake_ok=0
while (( SECONDS - start < WAKE_TIMEOUT )); do
  attempt=$((attempt + 1))
  if out=$(wake_ir 2>&1); then
    echo "depth wake-up ok on attempt ${attempt}: ${out##*$'\n'}"
    wake_ok=1
    break
  fi
  echo "depth wake-up attempt ${attempt} failed: ${out##*$'\n'}"
  sleep 3
done
if (( ! wake_ok )); then
  echo "WARNING: depth wake-up did not succeed within ${WAKE_TIMEOUT}s; starting the node anyway" >&2
fi

exec docker exec -u ubuntu "${CONTAINER}" /bin/bash -lc \
  "source /opt/ros/humble/setup.bash; \
   source /home/ubuntu/rs_ws/install/setup.bash; \
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
   export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml; \
   exec ros2 launch realsense2_camera rs_launch.py \
     depth_module.depth_profile:=640x480x15 \
     enable_color:=false"
