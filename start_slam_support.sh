#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

pids=()
cleanup() {
  echo
  echo "Stopping SLAM support nodes..."
  for pid in "${pids[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

start_node() {
  local label="$1"
  shift
  echo "Starting ${label}: $*"
  "$@" &
  pids+=("$!")
  sleep 0.5
}

# The custom Clearpath extras launch starts the Seyond Gazebo->ROS bridges.
# This helper only makes TF easy for tools launched outside the Clearpath
# namespace or inside Docker.
start_node "TF relay" \
  ros2 run topic_tools relay /a300_0000/tf /tf --ros-args --log-level error

start_node "Seyond mount static TF" \
  ros2 run tf2_ros static_transform_publisher 0.30 0.0 0.42 0.0 0.0 0.0 base_link seyond_robin_w_link

start_node "Seyond LiDAR static TF" \
  ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 seyond_robin_w_link seyond_robin_w_lidar_frame

start_node "Seyond IMU static TF" \
  ros2 run tf2_ros static_transform_publisher 0.15 -0.05 0.08 0.0 0.0 0.0 seyond_robin_w_link seyond_robin_w_imu_frame

echo
echo "Seyond SLAM support is running. Keep this terminal open."
echo "Checks you can run in another terminal:"
echo "  timeout 6 ros2 topic hz /a300_0000/sensors/seyond_robin_w/points"
echo "  timeout 6 ros2 topic hz /a300_0000/sensors/seyond_robin_w/imu"
echo "  timeout 5 ros2 run tf2_ros tf2_echo base_link seyond_robin_w_lidar_frame"
echo
wait
