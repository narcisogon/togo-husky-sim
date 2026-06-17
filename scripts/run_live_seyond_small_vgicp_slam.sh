#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
set -u

LAUNCH_FILE=/ws/src/lidarslam_ros2/lidarslam/launch/seyond_small_vgicp_slam.launch.py
SLAM_PARAM_FILE=${SLAM_PARAM_FILE:-/ws/src/lidarslam_ros2/lidarslam/param/seyond_small_vgicp_slam.yaml}
REGISTRATION_METHOD=${REGISTRATION_METHOD:-NDT}

if [[ ! -f "$LAUNCH_FILE" ]]; then
  echo "Missing $LAUNCH_FILE" >&2
  echo "Make sure ./lidarslam_ros2 is mounted into the Docker container." >&2
  exit 1
fi

if [[ ! -f "$SLAM_PARAM_FILE" ]]; then
  echo "Missing $SLAM_PARAM_FILE" >&2
  echo "Edit or create the small_vgicp frontend/backend YAML before launching." >&2
  exit 1
fi

echo "Launching Seyond scanmatcher SLAM with REGISTRATION_METHOD=${REGISTRATION_METHOD}."
if [[ "$REGISTRATION_METHOD" == "SMALL_VGICP" || "$REGISTRATION_METHOD" == "SMALL_GICP" ]]; then
  if ! ros2 pkg executables scanmatcher | grep -q small_gicp_odom_node; then
    echo "WARNING: installed scanmatcher does not appear to include small_gicp support." >&2
    echo "If scan_matcher exits with 'invalid registration method', install/build small_gicp and rebuild scanmatcher." >&2
    echo "Temporary fallback: REGISTRATION_METHOD=GICP bash /scripts/run_live_seyond_small_vgicp_slam.sh" >&2
  fi
fi

ros2 launch "$LAUNCH_FILE" \
  slam_param_file:="$SLAM_PARAM_FILE" \
  use_sim_time:=true \
  registration_method:="$REGISTRATION_METHOD" \
  map_save_period:=60 \
  rviz:=true
