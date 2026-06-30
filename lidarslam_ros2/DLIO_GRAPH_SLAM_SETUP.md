# Seyond DLIO + Graph SLAM Setup

This is the current rover SLAM stack used for the Seyond Robin W simulation. The frontend is DLIO, the backend is `graph_based_slam`, and the backend publishes corrected maps for RViz/Nav2 while the frontend keeps a smooth local odometry frame.

## What Runs

The stack is split into four pieces:

- **Simulator / robot support**: publishes the rover, `/clock`, TF, IMU, and raw Seyond cloud.
- **Cloud time adapter**: adds synthetic per-point time to the simulated organized cloud.
- **DLIO frontend**: publishes local odometry and deskewed/keyframe clouds.
- **Graph SLAM backend**: consumes frontend odom/cloud, publishes corrected map products.

Main launch script:

```bash
bash /ws/src/lidarslam_ros2/scripts/togo/run_live_seyond_dlio_slam.sh
```

Main config:

```text
/ws/src/lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml
```

Main launch file:

```text
/ws/src/lidarslam_ros2/lidarslam/launch/seyond_dlio_slam.launch.py
```

## Build

Inside the SLAM Docker container:

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

For faster rebuilds after editing DLIO or graph SLAM:

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select direct_lidar_inertial_odometry graph_based_slam togo_navigation --symlink-install
source install/setup.bash
```

If only `seyond_dlio_graph.yaml` changed, no rebuild is needed.

## Run Order

Start the sim/support side first, then start SLAM.

From the host WSL Husky workspace:

```bash
cd /mnt/c/Users/Username/OneDrive/Desktop/husky
source /opt/ros/jazzy/setup.bash
source source_togo_custom.sh
bash start_seyond_support.sh
```

Start your Clearpath/Gazebo simulation separately if it is not already running.

Then in the SLAM Docker container:

```bash
cd /ws
source install/setup.bash
bash /ws/src/lidarslam_ros2/scripts/togo/run_live_seyond_dlio_slam.sh
```

Important: keep the rover still during DLIO startup. DLIO calibrates the IMU for about 3 seconds. If the rover moves during calibration, odometry can become unstable or crash.

## Main Topics

Frontend:

```text
/dlio/odometry
/dlio/path
/dlio/path_simple
/dlio/deskewed
/dlio/keyframe_cloud
/dlio/frontend_diagnostics
```

Backend:

```text
/map
/map_array
/modified_map
/modified_map_timed
/modified_map_array
/modified_path
/loop_diagnostics
```

Reference path:

```text
/reference/path
```

TF:

```text
map -> odom
odom -> base_link
```

DLIO owns the local `odom -> base_link` motion. `graph_based_slam` owns the corrected global `map -> odom` transform.

## What The Backend Should And Should Not Do

The frontend should match against its own local DLIO submap in `odom`. Loop closures should not continuously rewrite the frontend local map.

Loop closure should affect:

```text
map -> odom
/modified_map
/modified_path
Nav2/global planning
```

Loop closure should not directly disturb:

```text
/dlio/odometry
DLIO's local scan-to-map submap
```

This keeps local odometry smooth while still letting the backend correct global drift.

## Config File

Use:

```text
lidarslam/param/seyond_dlio_graph.yaml
```

This one file configures:

- `dlio_odom_node`
- `dlio_map_node`
- `graph_based_slam`
- `reference_path_publisher`

Frontend sections to tune:

```yaml
pointcloud/deskew: true
odom/preprocessing/voxelFilter/res: 0.25
odom/keyframe/threshD: 0.5
odom/keyframe/threshR: 20.0
odom/submap/keyframe/knn: 10
odom/gicp/kCorrespondences: 16
odom/gicp/maxCorrespondenceDistance: 1.0
odom/gicp/maxIterations: 48
```

Gate/IMU fallback section:

```yaml
odom/gicp/rejectBadCorrections: true
odom/gicp/spinProtection/enabled: true
odom/gicp/timingProtection/enabled: true
odom/gicp/timingProtection/dropStaleScans: false
odom/gicp/freezeOnBadCorrection: true
odom/gicp/spinProtection/useImuPriorOnReject: true
```

Backend loop closure section:

```yaml
registration_method: NDT
threshold_loop_closure_score: 15.0
scan_context_threshold: 0.45
use_distance_loop_candidates: false
loop_max_translation_delta: 15.0
loop_max_rotation_delta_deg: 45.0
loop_edge_info_weight: 200.0
```

If bad loop closures deform `/modified_map`, make these stricter before trusting the backend correction:

```yaml
threshold_loop_closure_score: 1.5
scan_context_threshold: 0.30
max_loop_candidate_count: 1
loop_max_translation_delta: 5.0
loop_max_rotation_delta_deg: 20.0
loop_edge_info_weight: 50.0
```

Watch loop-closure decisions while driving:

```bash
ros2 topic echo /loop_diagnostics
```

If `ros2 topic echo` shortens the JSON with `...`, use:

```bash
ros2 topic echo /loop_diagnostics --full-length
```

Each message is a JSON string. The most useful fields are:

- `event`: scan-context candidate, candidate result, no valid candidate, or loop edge result
- `scan_context_query_ms`: descriptor lookup time
- `registration_ms`: NDT/GICP verification time for that candidate
- `loop_search_ms`: total backend loop-search time for that query
- `fitness`, `translation_delta_m`, `rotation_delta_deg`: why a loop passed or failed
- `reject_reason`: `fitness_threshold`, `translation_cap`, `rotation_cap`, or `registration_not_converged`

For the DLIO live config, `use_distance_loop_candidates` is disabled. This keeps
loop closure descriptor-driven: Scan Context can propose places, but NDT/GICP
still has to verify them. The old distance fallback can accept a wrong loop when
frontend odom has drifted close to an old submap, so keep it off unless you are
doing a controlled comparison.

## Per-Point Timestamps

DLIO works best when each point in a LiDAR scan has a relative firing time. This lets deskew use IMU motion during the scan.

The current Gazebo Seyond cloud has an organized shape and fields like:

```text
x, y, z, intensity, ring
height: 32
width: 256
```

Simulation usually does not provide true per-point firing time. The launch therefore starts:

```text
togo_navigation/seyond_cloud_time_adapter
```

It subscribes to:

```text
/a300_0000/sensors/seyond_robin_w/scan/points
```

and publishes:

```text
/a300_0000/sensors/seyond_robin_w/scan/points_timed
```

with a synthetic per-column `t` field.

The adapter assumes a 15 Hz scan by default:

```bash
TIMED_CLOUD_SCAN_PERIOD=0.0666666667
```

If scan direction is backwards, test:

```bash
TIMED_CLOUD_REVERSE_COLUMNS=true \
bash /ws/src/lidarslam_ros2/scripts/togo/run_live_seyond_dlio_slam.sh
```

If deskew seems to make turning worse, isolate with:

```bash
DLIO_DESKEW=false \
bash /ws/src/lidarslam_ros2/scripts/togo/run_live_seyond_dlio_slam.sh
```

## Setting This Up On Another Sim

You need these inputs:

```text
PointCloud2 LiDAR cloud
sensor_msgs/Imu
/clock if using sim time
TF frames for base_link, lidar, and imu
```

Update the launch remaps in:

```text
lidarslam/launch/seyond_dlio_slam.launch.py
```

Current hardcoded input topics:

```text
/a300_0000/sensors/seyond_robin_w/scan/points
/a300_0000/sensors/seyond_robin_w/imu
```

Update the config frames/extrinsics in:

```text
lidarslam/param/seyond_dlio_graph.yaml
```

Important fields:

```yaml
frames/odom: odom
frames/baselink: base_link
frames/lidar: seyond_robin_w_lidar_frame
frames/imu: seyond_robin_w_imu_frame
extrinsics/baselink2imu/t: [0.45, -0.05, 0.50]
extrinsics/baselink2lidar/t: [0.30, 0.0, 0.42]
```

For another sim, check the cloud fields:

```bash
ros2 topic echo /your/cloud/topic --once --field height
ros2 topic echo /your/cloud/topic --once --field width
ros2 topic echo /your/cloud/topic --once --field fields
```

If the cloud already has per-point time, use the real field and avoid synthetic timing if possible.

Common time field names:

```text
t
time
timestamp
```

If the sim has no per-point time:

- If the cloud is organized, the synthetic column-time adapter is acceptable for testing.
- If the cloud is unorganized, synthetic timing is only a rough approximation.
- If you need accurate high-speed turning, add true per-point time in the simulator/plugin or disable deskew for comparison.

For real Seyond hardware, prefer the driver's real per-point timestamp/ring output instead of synthetic timing.

## Startup Checks

Before SLAM:

```bash
ros2 topic hz /a300_0000/sensors/seyond_robin_w/scan/points
ros2 topic hz /a300_0000/sensors/seyond_robin_w/imu
ros2 topic echo /a300_0000/sensors/seyond_robin_w/scan/points --once --field fields
```

After SLAM starts:

```bash
ros2 topic hz /dlio/odometry
ros2 topic hz /dlio/deskewed
ros2 topic hz /modified_map
ros2 topic hz /modified_map_timed
ros2 topic echo /dlio/frontend_diagnostics
ros2 topic echo /loop_diagnostics
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

Check map fields:

```bash
ros2 topic echo /modified_map_timed --once --field fields
```

RViz displays:

- `/dlio/deskewed`: current frontend cloud
- `/modified_map`: corrected backend map
- `/modified_map_timed`: corrected backend map with `time` and `submap_index`
- `/dlio/path_simple`: frontend path
- `/modified_path`: backend corrected path
- `/reference/path`: Gazebo truth path aligned to initial frontend pose

For `/modified_map_timed`, use PointCloud2 color transformer `Field` and choose:

```text
time
submap_index
```

## Navigation

After SLAM is running and publishing `map -> odom`, start Nav2:

```bash
bash /ws/src/lidarslam_ros2/scripts/togo/run_nav2_with_slam.sh
```

Navigation uses the backend/global map products. The frontend odometry remains the high-rate local motion source.

## Troubleshooting

DLIO crashes near startup:

- Keep the rover still during IMU calibration.
- Try disabling calibration once:

```bash
DLIO_PARAM_FILE=/ws/src/lidarslam_ros2/direct_lidar_inertial_odometry/cfg/seyond_robin_w_dlio_no_imu_calib.yaml \
bash /ws/src/lidarslam_ros2/scripts/togo/run_live_seyond_dlio_slam.sh
```

No odometry:

```bash
ros2 topic hz /a300_0000/sensors/seyond_robin_w/scan/points_timed
ros2 topic hz /a300_0000/sensors/seyond_robin_w/imu
ros2 topic echo /dlio/frontend_diagnostics
```

Bad turning/spiral map:

- Confirm `odom/keyframe/threshR` is not too large.
- Confirm rotation-only keyframes are built into DLIO.
- Compare `DLIO_DESKEW=true` vs `DLIO_DESKEW=false`.
- Test `TIMED_CLOUD_REVERSE_COLUMNS=true`.
- Check extrinsics with TF:

```bash
ros2 run tf2_ros tf2_echo base_link seyond_robin_w_lidar_frame
ros2 run tf2_ros tf2_echo base_link seyond_robin_w_imu_frame
ros2 run tf2_ros tf2_echo seyond_robin_w_lidar_frame seyond_robin_w_imu_frame
```

Bad loop closures:

- Tighten backend thresholds.
- Lower `loop_edge_info_weight`.
- Reduce `loop_max_translation_delta` and `loop_max_rotation_delta_deg`.
- Temporarily disable loop closure while tuning the frontend.

Map appears but Nav2 cannot plan:

```bash
ros2 topic echo /map --once --field info --qos-reliability reliable --qos-durability transient_local
ros2 topic echo /global_costmap/costmap --once --field info --qos-reliability reliable --qos-durability transient_local
ros2 run tf2_ros tf2_echo map base_link
```

## Quick Commands

Build:

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run SLAM:

```bash
bash /ws/src/lidarslam_ros2/scripts/togo/run_live_seyond_dlio_slam.sh
```

Run SLAM without RViz:

```bash
SLAM_RVIZ=false \
bash /ws/src/lidarslam_ros2/scripts/togo/run_live_seyond_dlio_slam.sh
```

Run with deskew disabled:

```bash
DLIO_DESKEW=false \
bash /ws/src/lidarslam_ros2/scripts/togo/run_live_seyond_dlio_slam.sh
```

Run Nav2:

```bash
bash /ws/src/lidarslam_ros2/scripts/togo/run_nav2_with_slam.sh
```
