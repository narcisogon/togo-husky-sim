# TOGO SLAM/Nav Scripts

These are the rover-specific helper scripts used by the integrated SLAM and Nav2 workflow.

Primary entrypoints:

- `run_live_seyond_slam_integrated.sh`: starts RKO-LIO frontend, graph_based_slam backend, helper path publishers, and optional RViz
- `run_live_seyond_dlio_slam.sh`: starts the vendored DLIO frontend, Seyond timed-cloud adapter, graph_based_slam backend, helper path publishers, and optional RViz
- `run_nav2_with_slam.sh`: waits for SLAM TF, starts the SLAM-to-OGM mapper, Nav2, and optional RViz

Useful tools:

- `monitor_rko_diagnostics.sh`: prints frontend runtime, registration diagnostics, and odometry rates
- `save_seyond_map_clean.sh`: calls `/map_save`
- `record_seyond_slam_bag.sh`: records useful SLAM topics
- `add_intensity_to_cloud.py`: adds a fixed intensity field to a PointCloud2
- `odom_to_path.py`: publishes a Path from odometry
- `gazebo_pose_to_aligned_path.py`: publishes a reference path from Gazebo pose messages
- `pose_to_odom.py`: helper for the older scanmatcher/small-vgicp launch path

DLIO setup notes:

- The DLIO source is vendored at `lidarslam_ros2/direct_lidar_inertial_odometry` so it is available from this repository without a separate clone.
- Build it with `colcon build --packages-select direct_lidar_inertial_odometry --symlink-install`.
- The Seyond sim cloud should go through `seyond_cloud_time_adapter` so DLIO receives per-point `time` data on `/a300_0000/sensors/seyond_robin_w/scan/points_timed`.
- The rover-specific DLIO parameters live in `lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml` and `lidarslam_ros2/direct_lidar_inertial_odometry/cfg/seyond_robin_w_dlio.yaml`.

The root `/scripts` copies are kept only for Docker command compatibility. New edits should happen here.
