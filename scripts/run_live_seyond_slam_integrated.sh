#!/usr/bin/env bash
set -eo pipefail

exec bash /ws/src/lidarslam_ros2/scripts/togo/run_live_seyond_slam_integrated.sh "$@"
