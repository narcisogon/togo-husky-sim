#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
RELAY_SIM_TF_TO_ROS_TF="${RELAY_SIM_TF_TO_ROS_TF:-false}"
BRIDGE_SEYOND_TOPICS="${BRIDGE_SEYOND_TOPICS:-false}"

pids=()
cleanup() {
  echo
  echo "Stopping Seyond support nodes..."
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

topic_has_publishers() {
  local topic="$1"
  ros2 topic info "$topic" 2>/dev/null | grep -Eq 'Publisher count: [1-9][0-9]*'
}

should_bridge_seyond_topics() {
  case "${BRIDGE_SEYOND_TOPICS}" in
    true)
      return 0
      ;;
    false)
      return 1
      ;;
    auto)
      if topic_has_publishers /a300_0000/sensors/seyond_robin_w/scan/points ||
         topic_has_publishers /a300_0000/sensors/seyond_robin_w/imu; then
        echo "Seyond ROS sensor topics already have publishers; skipping duplicate sensor bridges."
        echo "Set BRIDGE_SEYOND_TOPICS=true to force the legacy bridges."
        return 1
      fi
      return 0
      ;;
    *)
      echo "Unknown BRIDGE_SEYOND_TOPICS=${BRIDGE_SEYOND_TOPICS}; use auto, true, or false." >&2
      return 1
      ;;
  esac
}

if should_bridge_seyond_topics; then
  start_node "Seyond IMU bridge" \
    ros2 run ros_gz_bridge parameter_bridge \
    '/a300_0000/sensors/seyond_robin_w/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'

  start_node "Seyond LiDAR point bridge" \
    ros2 run ros_gz_bridge parameter_bridge \
    '/a300_0000/sensors/seyond_robin_w/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'

  start_node "Seyond LiDAR scan bridge" \
    ros2 run ros_gz_bridge parameter_bridge \
    '/a300_0000/sensors/seyond_robin_w/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
fi


start_node "Gazebo dynamic pose bridge" \
  ros2 run ros_gz_bridge parameter_bridge \
  '/world/enhanced_lunar_test/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' \
  --ros-args \
  -r /world/enhanced_lunar_test/dynamic_pose/info:=/gazebo/dynamic_pose
if [[ "${RELAY_SIM_TF_TO_ROS_TF}" == "true" ]]; then
  start_node "TF relay" \
    ros2 run topic_tools relay /a300_0000/tf /tf --ros-args --log-level error
else
  echo "Skipping sim TF relay (/a300_0000/tf -> /tf); SLAM owns odom->base_link."
fi

start_node "Seyond mount static TF" \
  ros2 run tf2_ros static_transform_publisher 0.30 0.0 0.42 0.0 0.0 0.0 base_link seyond_robin_w_link

start_node "Seyond LiDAR static TF" \
  ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 seyond_robin_w_link seyond_robin_w_lidar_frame

start_node "Seyond IMU static TF" \
  ros2 run tf2_ros static_transform_publisher 0.15 -0.05 0.08 0.0 0.0 0.0 seyond_robin_w_link seyond_robin_w_imu_frame

echo
echo "Seyond support is running. Keep this terminal open."
echo "Checks you can run in another terminal:"
echo "  timeout 6 ros2 topic hz /a300_0000/sensors/seyond_robin_w/scan/points"
echo "  timeout 6 ros2 topic hz /a300_0000/sensors/seyond_robin_w/imu"
echo "  timeout 5 ros2 run tf2_ros tf2_echo base_link seyond_robin_w_lidar_frame"
echo "Set BRIDGE_SEYOND_TOPICS=true only if the sim is not already bridging LiDAR/IMU."
echo
wait
