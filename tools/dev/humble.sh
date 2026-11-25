#!/usr/bin/env bash
# Run repository tools and colcon builds inside the laptop Humble container.
#
# The container uses the same image and workspace as tools/humble_moveit/
# run_humble.sh, so analysis tools and tests see the same ROS 2 Humble
# packages as the robot and the CI workflow:
#
#   tools/dev/humble.sh build                      # rebuild the whole workspace
#   tools/dev/humble.sh build armpi_analysis       # one package
#   tools/dev/humble.sh test                       # colcon test, CI package set
#   tools/dev/humble.sh run python3 tools/phase3/random_site_session.py --help
#   tools/dev/humble.sh check                      # imports + tool --help pass
#   tools/dev/humble.sh shell                      # interactive shell
#
# The workspace defaults to ~/jcheng/humble_ws. The repository is mounted
# read-write at its host path because the tools write evidence bundles; the
# container runs as the host user, not root. DDS uses the wired CycloneDDS
# config, so tools that subscribe to robot topics work over the direct
# Ethernet link.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="${WS:-$HOME/jcheng/humble_ws}"
IMAGE="${IMAGE:-armpi-humble-moveit:latest}"
DDS_CONFIG="$REPO_ROOT/tools/humble_moveit/cyclonedds-laptop.xml"
SHARED="$REPO_ROOT/packages/jcheng_core"

usage() {
    awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"
}

require() {
    [[ -d "$WS" ]] || { echo "FAIL: workspace not found: $WS" >&2; exit 1; }
    [[ -f "$DDS_CONFIG" ]] || { echo "FAIL: missing DDS config: $DDS_CONFIG" >&2; exit 1; }
    command -v docker >/dev/null 2>&1 || { echo "FAIL: docker not found" >&2; exit 1; }
    docker info >/dev/null 2>&1 || {
        echo "FAIL: cannot access the Docker daemon (add the user to the docker group)" >&2
        exit 1
    }
}

# The repository is mounted at the same absolute path inside the container,
# so repository-relative paths and the DDS config URI stay valid.
container() {
    local workdir="${WORKDIR:-/workspace}"
    docker run --rm --net=host --ipc=host \
        --user "$(id -u):$(id -g)" \
        -e HOME=/tmp \
        -e REPO_ROOT="$REPO_ROOT" \
        -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        -e CYCLONEDDS_URI="file://$DDS_CONFIG" \
        -e ROS2CLI_NO_DAEMON=1 \
        -e PYTHONPATH="$SHARED${PYTHONPATH:+:$PYTHONPATH}" \
        -v "$WS":/workspace \
        -v "$REPO_ROOT":"$REPO_ROOT" \
        -w "$workdir" \
        "$@"
}

quote_all() {
    # shellcheck disable=SC2059  # the format is a constant
    printf ' %q' "$@"
}

cmd_build() {
    local inner='source /opt/ros/humble/setup.bash && colcon build --symlink-install'
    if (($# > 0)); then
        inner+=" --packages-select$(quote_all "$@")"
    fi
    # shellcheck disable=SC2086  # words are already quoted by quote_all
    container "$IMAGE" bash -lc "$inner"
}

cmd_test() {
    local inner='source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && colcon test --event-handlers console_cohesion+'
    if (($# > 0)); then
        inner+=" --packages-select$(quote_all "$@")"
    fi
    inner+=' && colcon test-result --verbose'
    container "$IMAGE" bash -lc "$inner"
}

cmd_run() {
    (($# > 0)) || { echo "usage: $0 run <command...>" >&2; exit 2; }
    local inner='source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash 2>/dev/null || true; exec'
    inner+="$(quote_all "$@")"
    WORKDIR="$REPO_ROOT" container "$IMAGE" bash -lc "$inner"
}

cmd_check() {
    local inner
    inner=$(cat <<'INNER'
set -e
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
python3 - <<'PY'
import cv2
import armpi_analysis
import armpi_fiducial_localizer.detector
import armpi_fiducial_localizer.node
import armpi_grasp_planner.reachability
print('imports OK: cv2={} armpi_analysis={} fiducial={} reachability={}'.format(
    cv2.__version__,
    armpi_analysis.__file__,
    armpi_fiducial_localizer.__file__,
    armpi_grasp_planner.reachability.__file__))
PY
for tool in \
    tools/phase2/low_config_error_report.py \
    tools/phase2/low_config_model_fit.py \
    tools/phase2/low_config_correction.py \
    tools/phase2/grasp_joint_fit.py \
    tools/phase2/front_camera_extrinsic.py \
    tools/phase2/chessboard_hand_eye.py \
    tools/phase2/calibration_pose_selector.py \
    tools/phase2/rgbd_tag_check.py \
    tools/phase2/rgbd_board_check.py \
    tools/phase2/depth_plane_fit.py \
    tools/phase2/object_segment_rgbd.py \
    tools/phase3/random_site_session.py \
    tools/phase3/visual_displacement_run.py \
    tools/phase3/fast_motion_e2e.py \
    tools/phase3/execution_fixture_run.py; do
    python3 "$tool" --help >/dev/null
    echo "help OK: $tool"
done
INNER
    )
    WORKDIR="$REPO_ROOT" container "$IMAGE" bash -lc "$inner"
}

cmd_shell() {
    container -it "$IMAGE" bash -lc \
        'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash 2>/dev/null || true; echo "Humble workspace ready at /workspace (repo mounted at its host path)"; exec bash -i'
}

main() {
    local command="${1:-}"
    case "$command" in
        build) shift; require; cmd_build "$@" ;;
        test) shift; require; cmd_test "$@" ;;
        run) shift; require; cmd_run "$@" ;;
        check) shift; require; cmd_check ;;
        shell) shift; require; cmd_shell ;;
        -h|--help|help) usage ;;
        *) usage >&2; exit 2 ;;
    esac
}

main "$@"
