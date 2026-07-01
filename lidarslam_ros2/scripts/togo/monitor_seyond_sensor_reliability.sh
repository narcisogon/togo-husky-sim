#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
if [[ -f /ws/install/setup.bash ]]; then
  source /ws/install/setup.bash
fi
set -u

DURATION="${1:-6}"
RAW_CLOUD="${RAW_CLOUD:-/a300_0000/sensors/seyond_robin_w/scan/points}"
TIMED_CLOUD="${TIMED_CLOUD:-/a300_0000/sensors/seyond_robin_w/scan/points_timed}"
IMU_TOPIC="${IMU_TOPIC:-/a300_0000/sensors/seyond_robin_w/imu}"
DIAG_TOPIC="${DIAG_TOPIC:-/dlio/frontend_diagnostics}"
VERBOSE="${VERBOSE:-false}"

echo "=== Seyond Sensor Reliability (${DURATION}s windows) ==="
echo "raw_cloud:   ${RAW_CLOUD}"
echo "timed_cloud: ${TIMED_CLOUD}"
echo "imu:         ${IMU_TOPIC}"
echo "diag:        ${DIAG_TOPIC}"
echo

echo "=== Topic availability ==="
ros2 topic list | grep -E "seyond_robin_w|dlio|modified_map|clock" || true
echo

echo "=== QoS / endpoints ==="
if [[ "${VERBOSE}" == "true" ]]; then
  ros2 topic info "${RAW_CLOUD}" -v || true
  echo
  ros2 topic info "${TIMED_CLOUD}" -v || true
  echo
  ros2 topic info "${IMU_TOPIC}" -v || true
else
  ros2 topic info "${RAW_CLOUD}" || true
  ros2 topic info "${TIMED_CLOUD}" || true
  ros2 topic info "${IMU_TOPIC}" || true
  echo "Set VERBOSE=true for full endpoint QoS."
fi
echo

echo "=== Rates ==="
timeout "${DURATION}" ros2 topic hz /clock || true
timeout "${DURATION}" ros2 topic hz "${RAW_CLOUD}" || true
timeout "${DURATION}" ros2 topic hz "${TIMED_CLOUD}" || true
timeout "${DURATION}" ros2 topic hz "${IMU_TOPIC}" || true
echo

echo "=== Cloud geometry and fields ==="
echo "raw height/width:"
ros2 topic echo "${RAW_CLOUD}" --once --field height || true
ros2 topic echo "${RAW_CLOUD}" --once --field width || true
echo "raw fields:"
ros2 topic echo "${RAW_CLOUD}" --once --field fields || true
echo
echo "timed height/width:"
ros2 topic echo "${TIMED_CLOUD}" --once --field height || true
ros2 topic echo "${TIMED_CLOUD}" --once --field width || true
echo "timed fields:"
ros2 topic echo "${TIMED_CLOUD}" --once --field fields || true
echo

echo "=== Header/frame sanity ==="
ros2 topic echo "${RAW_CLOUD}" --once --field header || true
ros2 topic echo "${TIMED_CLOUD}" --once --field header || true
ros2 topic echo "${IMU_TOPIC}" --once --field header || true
echo

echo "=== IMU sample sanity ==="
ros2 topic echo "${IMU_TOPIC}" --once --field angular_velocity || true
ros2 topic echo "${IMU_TOPIC}" --once --field linear_acceleration || true
echo

echo "=== Static extrinsics ==="
timeout 4 ros2 run tf2_ros tf2_echo base_link seyond_robin_w_lidar_frame || true
timeout 4 ros2 run tf2_ros tf2_echo base_link seyond_robin_w_imu_frame || true
timeout 4 ros2 run tf2_ros tf2_echo seyond_robin_w_lidar_frame seyond_robin_w_imu_frame || true
echo

echo "=== DLIO timing/gate snapshot ==="
echo "Fields of interest:"
echo "  [16]=deskew_status [17]=deskew_size [20]=rejected [21]=bad_streak"
echo "  [31]=angular_rate [33]=imu_age_ms [40]=imu_covers_start [41]=imu_covers_end"
echo "  [42]=latest_imu_minus_lidar_ms [43]=deskew_time_buckets [45]=scan_duration_ms"
echo "  [57]=registration_ms [63]=total_callback_ms [65]=gicp_align_ms [68]=recovery_ms"
if [[ -f /ws/src/lidarslam_ros2/scripts/togo/dlio_diag_once.py ]]; then
  timeout "${DURATION}" python3 /ws/src/lidarslam_ros2/scripts/togo/dlio_diag_once.py "${DIAG_TOPIC}" || true
else
  ros2 topic echo "${DIAG_TOPIC}" --once || true
fi
