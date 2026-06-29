#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
set -u

LAUNCH_FILE=/ws/src/lidarslam_ros2/lidarslam/launch/seyond_dlio_slam.launch.py
COMBINED_PARAM_FILE=${COMBINED_PARAM_FILE:-/ws/src/lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml}
SLAM_PARAM_FILE=${SLAM_PARAM_FILE:-$COMBINED_PARAM_FILE}
DLIO_PARAM_FILE=${DLIO_PARAM_FILE:-$COMBINED_PARAM_FILE}
ENABLE_MAP_SAVE_PULSE=${ENABLE_MAP_SAVE_PULSE:-false}
SLAM_RVIZ=${SLAM_RVIZ:-true}
TIMED_CLOUD_SCAN_PERIOD=${TIMED_CLOUD_SCAN_PERIOD:-0.0666666667}
TIMED_CLOUD_REVERSE_COLUMNS=${TIMED_CLOUD_REVERSE_COLUMNS:-false}
DLIO_DESKEW=${DLIO_DESKEW:-true}

if [[ ! -f "$LAUNCH_FILE" ]]; then
  echo "Missing $LAUNCH_FILE" >&2
  echo "Make sure ./lidarslam_ros2 is mounted into the Docker container." >&2
  exit 1
fi

if [[ ! -f "$SLAM_PARAM_FILE" ]]; then
  echo "Missing $SLAM_PARAM_FILE" >&2
  exit 1
fi

if [[ ! -f "$DLIO_PARAM_FILE" ]]; then
  echo "Missing $DLIO_PARAM_FILE" >&2
  exit 1
fi

ros2 launch "$LAUNCH_FILE" \
  slam_param_file:="$SLAM_PARAM_FILE" \
  dlio_param_file:="$DLIO_PARAM_FILE" \
  use_sim_time:=true \
  map_save_period:=60 \
  enable_map_save_pulse:="$ENABLE_MAP_SAVE_PULSE" \
  timed_cloud_scan_period:="$TIMED_CLOUD_SCAN_PERIOD" \
  timed_cloud_reverse_columns:="$TIMED_CLOUD_REVERSE_COLUMNS" \
  dlio_deskew:="$DLIO_DESKEW" \
  rviz:="$SLAM_RVIZ"
