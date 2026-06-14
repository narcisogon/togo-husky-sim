#!/usr/bin/env bash`nset -eo pipefail`n`nsource /opt/ros/jazzy/setup.bash`nset -u

OUT_NAME="${1:-seyond_slam_run}"
mkdir -p /bags

ros2 bag record \
  /clock \
  /a300_0000/sensors/seyond_robin_w/scan/points \
  /a300_0000/sensors/seyond_robin_w/imu \
  /a300_0000/platform/odom \
  /a300_0000/tf \
  /a300_0000/tf_static \
  /tf \
  /tf_static \
  /rko_lio/odometry \
  /rko_lio/frame \
  /rko_lio/local_map \
  /modified_map \
  /modified_path \
  /modified_map_array \
  -o "/bags/${OUT_NAME}"