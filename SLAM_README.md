# SLAM System Summary

This workspace runs an integrated LiDAR-inertial SLAM stack for the simulated Husky/A300 rover with a Seyond Robin W-style 3D LiDAR and IMU. The goal is to test a navigation-ready SLAM architecture for lunar-like terrain: a live frontend for local motion, a backend for global consistency, and a high-rate pose bridge for control.

## Main Files

```text
Main launch:
  lidarslam_ros2/lidarslam/launch/seyond_live_slam.launch.py

Main SLAM parameters:
  lidarslam_ros2/lidarslam/param/seyond_live_slam.yaml

Main run script:
  scripts/run_live_seyond_slam_integrated.sh

Sensor bridge / TF support:
  start_seyond_support.sh
```

Most tuning should happen in:

```text
lidarslam_ros2/lidarslam/param/seyond_live_slam.yaml
```

That file contains both the frontend and backend parameter sections:

```yaml
rko_lio_online_node:
  ros__parameters:
    # Frontend LiDAR-inertial odometry

graph_based_slam:
  ros__parameters:
    # Backend pose graph, loop closure, and map output
```

## Overall Data Flow

```text
Gazebo Seyond LiDAR + IMU
  -> ROS bridge / static TF support
  -> RKO-LIO frontend
  -> /rko_lio/odometry + /rko_lio/frame
  -> XYZI adapter
  -> graph_based_slam backend
  -> map->odom correction + /modified_path + /modified_map
```

The frontend estimates local motion. The backend corrects long-term drift. Navigation should use the corrected TF chain:

```text
map -> odom -> base_link
```

## Frontend: RKO-LIO

RKO-LIO is the live local odometry source. It uses the simulated Seyond LiDAR point cloud and IMU.

Inputs:

```text
/a300_0000/sensors/seyond_robin_w/scan/points
/a300_0000/sensors/seyond_robin_w/imu
/tf
/tf_static
/clock
```

Outputs:

```text
/rko_lio/odometry
/rko_lio/frame
/rko_lio/local_map
/rko_lio/path
/rko_lio/registration_diagnostics
/rko_lio/runtime_diagnostics
```

The frontend currently uses point-to-plane ICP. It does not use the backend `registration_method` parameter. NDT/GICP settings belong to `graph_based_slam`, not RKO-LIO.

Current frontend behavior:

```text
LiDAR owns local position.
IMU helps stabilize orientation.
Point-to-plane residuals improve local surface alignment.
Vertical spike filtering clamps sudden bad Z jumps.
Runtime diagnostics expose processing time and dropped frames.
Registration diagnostics expose overlap, residuals, and Hessian conditioning.
```

Important frontend parameters:

```yaml
registration_error_model: point_to_plane
enable_imu_pose_prior: true
imu_pose_prior_translation_weight: 0.0
imu_pose_prior_rotation_weight: 0.5
enable_stationary_hold: false
enable_degeneracy_damping: false
enable_vertical_spike_filter: true
max_vertical_update_m: 0.08
gravity_magnitude: 9.8107
```

The current sim uses Earth gravity, so `gravity_magnitude` is set to `9.8107`. For a true lunar-gravity simulation this would need to change to about `1.625`.

## IMU Pose Prior

The IMU pose prior is a lightweight FAST-LIO-inspired improvement inside RKO-LIO. It does not replace the scan matcher.

The idea is:

```text
RKO predicts a pose from IMU propagation.
ICP still aligns the LiDAR scan to the local map.
A soft prior pulls the registration toward the IMU-predicted orientation.
Translation weight is currently zero, so LiDAR still owns position.
Rotation weight is nonzero, so IMU helps reduce attitude/yaw jitter.
```

Current settings:

```yaml
enable_imu_pose_prior: true
imu_pose_prior_translation_weight: 0.0
imu_pose_prior_rotation_weight: 0.5
```

This was chosen because the frontend behaved better when IMU influenced rotation but did not force translation.

## Vertical Spike Filter

The regular frontend path sometimes had brief upward Z jumps. This is usually caused by weak vertical observability in flat or repetitive terrain, not necessarily by the IMU prediction node.

The frontend now has an optional Z update clamp:

```yaml
enable_vertical_spike_filter: true
max_vertical_update_m: 0.08
```

This only limits sudden scan-to-scan Z jumps. It does not force the rover to stay flat, and real elevation change can still accumulate over multiple scans.

## IMU Prediction Node

The IMU prediction node is separate from the main frontend odometry. It does not replace `/rko_lio/odometry`.

It publishes:

```text
/rko_lio/odometry_imu_predict
/rko_lio/path_imu_predict
```

Purpose:

```text
RKO-LIO may publish at roughly 5-15 Hz.
The IMU publishes faster, often around 100 Hz.
The prediction node fills the short gap between RKO updates.
When new RKO odometry arrives, prediction resets to that corrected anchor.
```

The prediction is intentionally short-horizon. It should not free-run for seconds.

Current conservative settings:

```text
max_prediction_horizon_sec: 0.12
max_position_extrapolation_m: 0.08
use_acceleration: false
velocity_smoothing_alpha: 0.9
path_min_distance_m: 0.03
```

Mental model:

```text
RKO odometry gives the trusted anchor.
IMU gyro helps bridge between anchors.
Linear acceleration is currently disabled to avoid drift.
The output is useful as a high-rate control/navigation feed.
```

Run with IMU prediction enabled:

```bash
ENABLE_IMU_PREDICTION=true bash /scripts/run_live_seyond_slam_integrated.sh
```

Check it:

```bash
ros2 topic hz /rko_lio/odometry_imu_predict
ros2 topic echo /rko_lio/odometry_imu_predict --field pose.pose.position
```

## Frontend Stability Filter

There is also a separate frontend stability filter node:

```text
/rko_lio/odometry_stable
/rko_lio/path_stable
/rko_lio/stability_status
```

It uses odometry plus registration diagnostics to gate or smooth unstable frontend motion. It is currently experimental and disabled by default because earlier tests showed it could continue moving after the rover stopped.

Default:

```bash
ENABLE_FRONTEND_STABILITY_FILTER=false
```

For now, the main reliable pose sources are:

```text
/rko_lio/odometry
/rko_lio/odometry_imu_predict
TF map->base_link
```

## XYZI Adapter

The backend expects a point cloud with an `intensity` field. RKO-LIO's `/rko_lio/frame` may not include intensity, so this adapter is launched:

```text
scripts/add_intensity_to_cloud.py
```

It converts:

```text
/rko_lio/frame
  -> /rko_lio/frame_xyzi
```

This does not improve the data. It only makes the cloud format compatible with the backend.

## Backend: graph_based_slam

The backend consumes frontend odometry and registered clouds:

```text
odom_input  -> /rko_lio/odometry
cloud_input -> /rko_lio/frame_xyzi
```

It builds submaps, adds graph constraints, searches for loop closures, and publishes/saves map products.

Backend outputs:

```text
/modified_path
/modified_map
/modified_map_array
/map_save
```

The backend currently uses NDT:

```yaml
registration_method: "NDT"
ndt_resolution: 5.0
```

Important distinction:

```text
Frontend RKO-LIO:
  point-to-plane ICP + IMU

Backend graph_based_slam:
  NDT for submap/loop alignment
```

## Backend Global Correction

The backend now publishes the global correction transform:

```text
map -> odom
```

RKO-LIO publishes the local continuous transform:

```text
odom -> base_link
```

Together they produce:

```text
map -> base_link
```

This is the corrected navigation pose.

The point is to avoid making the frontend jump. RKO-LIO can remain a smooth local odometry source, while the backend adjusts `map -> odom` when graph optimization or loop closure changes the global estimate.

Use this for navigation:

```text
TF map -> base_link
```

Use this to inspect raw frontend behavior:

```text
/rko_lio/odometry
/rko_lio/path
```

Use this to inspect backend correction:

```text
/modified_path
/modified_map
```

## Map Publishing

The integrated launch periodically calls:

```text
/map_save
```

This refreshes backend map outputs while driving. The period is controlled by:

```bash
map_save_period
```

The run script currently defaults to a slower save period for live use.

Manual save:

```bash
ros2 service call /map_save std_srvs/srv/Empty
```

Expected backend map outputs:

```text
/modified_map
/modified_path
```

Saved files go under:

```text
/ws/src/lidarslam_ros2/output/husky_seyond_graph
```

## Which Pose To Use

Different topics have different purposes:

```text
/rko_lio/odometry
  Raw local frontend pose. Good for debugging scan matching.

/rko_lio/odometry_imu_predict
  High-rate short-horizon pose bridge. Useful for control latency.

TF map->base_link
  Backend-corrected navigation pose. Best target for global navigation.

/modified_path
  Backend optimized path. Good for SLAM inspection.

/modified_map
  Backend map. Future source for occupancy/elevation mapping.
```

Recommended navigation mental model:

```text
Use RKO-LIO for local motion.
Use IMU prediction to fill small gaps for high-rate control.
Use graph_based_slam map->odom for global correction.
Use TF map->base_link as the corrected navigation pose.
```

## Known Weak Points

Current limitations:

```text
Flat terrain weakens scan matching.
Repeated rocks/craters can create local minima.
Narrow FOV gives fewer geometric constraints than a 360 degree LiDAR.
Z, roll, and pitch can be weakly constrained on planar terrain.
Deskew is disabled until point timing is trusted.
The IMU sim is Earth gravity right now.
Ground truth is not fully solved yet.
Backend loop closure depends heavily on terrain distinctiveness.
The IMU prediction node is not a full error-state Kalman filter.
```

## Ideas To Discuss Next

Good ideas to tie into this stack:

```text
Proper /ground_truth/odom and /ground_truth/path from Gazebo model pose.
Frontend observability scoring from Hessian, overlap, and residuals.
Adaptive graph weights based on frontend registration quality.
Full error-state iterated EKF instead of the lightweight IMU prior.
Better local map search, eventually ikd-tree style.
Terrain-aware vertical/ground constraints instead of hard flat-world assumptions.
Loop closure confidence using multiple descriptors plus geometric verification.
Backend-corrected occupancy grid or elevation map for navigation.
RTK/GNSS-style factor for Earth hardware testing, if available.
```

## Quick Test Commands

Check frontend rate:

```bash
ros2 topic hz /rko_lio/odometry
```

Check high-rate prediction:

```bash
ros2 topic hz /rko_lio/odometry_imu_predict
```

Check backend path/map:

```bash
ros2 topic hz /modified_path
ros2 topic hz /modified_map
```

Check corrected navigation transform:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

Check raw local transform:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

Check frontend diagnostics:

```bash
ros2 topic echo /rko_lio/runtime_diagnostics
ros2 topic echo /rko_lio/registration_diagnostics
```

## Short Explanation

The stack works like this:

```text
RKO-LIO estimates the rover's local motion from LiDAR and IMU.
The IMU prediction node fills tiny timing gaps to create a higher-rate pose stream.
The backend consumes RKO odometry and registered clouds to build a pose graph.
When the backend improves the global estimate, it publishes map->odom.
Navigation should use map->base_link, which combines backend correction with frontend local motion.
```

The frontend is for fast local tracking. The backend is for global consistency. The IMU prediction topic is a short-horizon bridge for smoother, higher-rate control.
