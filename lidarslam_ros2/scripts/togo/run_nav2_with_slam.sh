#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

set -u

WAIT_FOR_SLAM_TF=${WAIT_FOR_SLAM_TF:-true}
TF_WAIT_TIMEOUT_SEC=${TF_WAIT_TIMEOUT_SEC:-60}
NAV2_RVIZ=${NAV2_RVIZ:-true}
NAV2_DEBUG_MAP=${NAV2_DEBUG_MAP:-false}
NAV2_USE_SLAM_MAP=${NAV2_USE_SLAM_MAP:-true}
NAV2_REQUEST_INITIAL_MAP_SAVE=${NAV2_REQUEST_INITIAL_MAP_SAVE:-true}

wait_for_tf() {
  local target_frame="$1"
  local source_frame="$2"
  local start_time
  start_time=$(date +%s)

  echo "Waiting for TF ${target_frame} -> ${source_frame}..."
  while true; do
    local output
    output=$(timeout 2 ros2 run tf2_ros tf2_echo "$target_frame" "$source_frame" 2>&1 || true)
    printf '%s\n' "$output" >/tmp/nav2_tf_wait.log
    if printf '%s\n' "$output" | grep -q "At time"; then
      echo "TF ${target_frame} -> ${source_frame} is available."
      return 0
    fi

    local now
    now=$(date +%s)
    if (( now - start_time >= TF_WAIT_TIMEOUT_SEC )); then
      echo "Timed out waiting for TF ${target_frame} -> ${source_frame} after ${TF_WAIT_TIMEOUT_SEC}s." >&2
      echo "Last tf2_echo output:" >&2
      tail -n 20 /tmp/nav2_tf_wait.log >&2 || true
      echo "Visible TF-related topics:" >&2
      ros2 topic list 2>/dev/null | grep -E '^/(tf|tf_static)$|/rko_lio/odometry|/modified_path|/modified_map' >&2 || true
      echo "Recent /tf sample:" >&2
      timeout 2 ros2 topic echo /tf --once >&2 || true
      return 1
    fi

    sleep 1
  done
}

if [[ "$WAIT_FOR_SLAM_TF" == "true" ]]; then
  wait_for_tf odom base_link
  wait_for_tf map base_link
fi

ros2 launch togo_navigation rover_nav2.launch.py \
  use_sim_time:=true \
  autostart:=true \
  rviz:="$NAV2_RVIZ" \
  debug_map:="$NAV2_DEBUG_MAP" \
  use_slam_map:="$NAV2_USE_SLAM_MAP" \
  request_initial_map_save:="$NAV2_REQUEST_INITIAL_MAP_SAVE"
