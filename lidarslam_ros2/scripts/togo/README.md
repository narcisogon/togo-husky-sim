# TOGO SLAM/Nav Scripts

These are the rover-specific helper scripts used by the integrated SLAM and Nav2 workflow.

Primary entrypoints:

- `run_live_seyond_slam_integrated.sh`: starts RKO-LIO frontend, graph_based_slam backend, helper path publishers, and optional RViz
- `run_nav2_with_slam.sh`: waits for SLAM TF, starts the SLAM-to-OGM mapper, Nav2, and optional RViz

Useful tools:

- `monitor_rko_diagnostics.sh`: prints frontend runtime, registration diagnostics, and odometry rates
- `save_seyond_map_clean.sh`: calls `/map_save`
- `record_seyond_slam_bag.sh`: records useful SLAM topics
- `add_intensity_to_cloud.py`: adds a fixed intensity field to a PointCloud2
- `odom_to_path.py`: publishes a Path from odometry
- `gazebo_pose_to_aligned_path.py`: publishes a reference path from Gazebo pose messages
- `pose_to_odom.py`: helper for the older scanmatcher/small-vgicp launch path

The root `/scripts` copies are kept only for Docker command compatibility. New edits should happen here.

