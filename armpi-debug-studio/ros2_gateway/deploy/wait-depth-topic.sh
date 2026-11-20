#!/bin/bash
# Best effort: wait until the D435i depth node has opened its sensors before
# ext_cam_node claims /dev/video4. Ordering is not enough - systemd starts the
# depth unit before the colour unit, but the node needs a few seconds to reach
# STREAMON, and vid4l2 S_FMT fails with EBUSY in between.
# Never blocks the colour path indefinitely: it is a wait, not a hard requirement.
set -u

CONTAINER="${ARMPI_EXT_CAM_CONTAINER:-ArmPiUltra}"
LIMIT="${ARMPI_WAIT_DEPTH_S:-40}"

start=$SECONDS
while (( SECONDS - start < LIMIT )); do
  if docker exec -u ubuntu "${CONTAINER}" /bin/bash -lc \
      'source /opt/ros/humble/setup.bash; \
       source /home/ubuntu/rs_ws/install/setup.bash; \
       export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
       export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml; \
       timeout 3 ros2 topic list 2>/dev/null | grep -qx /camera/camera/depth/image_rect_raw' \
      >/dev/null 2>&1; then
    echo "depth topic present after $((SECONDS - start))s"
    exit 0
  fi
  sleep 2
done
echo "WARNING: depth topic not seen within ${LIMIT}s; starting colour anyway" >&2
exit 0
