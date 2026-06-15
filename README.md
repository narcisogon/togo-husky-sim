# TOGO / Husky Lunar SLAM Simulation Runbook

This workspace is a WSL + Docker + ROS 2 Jazzy testbed for bringing up a Clearpath Husky/A300-style rover with a Seyond Robin W-style 3D LiDAR and co-located IMU, then testing LiDAR-inertial odometry and graph-based SLAM in Gazebo lunar-like terrain.

The goal is not just to make a robot appear in simulation. The goal is to build a repeatable workflow that helps answer internship-relevant questions for TOGO/Husky work:

- Can we simulate a Husky/TOGO rover with realistic enough LiDAR and IMU data?
- Can we run a LiDAR-inertial SLAM frontend live from Gazebo?
- Can we feed that frontend into a graph backend for submaps, loop closure, map export, and later navigation?
- What is reliable now, what is experimental, and what should be improved before using this as a serious testbed?

## Current Status

Working:

- Clearpath Gazebo simulation launches from WSL.
- Custom Seyond Robin W-style GPU LiDAR and IMU are mounted on the rover.
- LiDAR and IMU are bridged from Gazebo to ROS 2.
- Dockerized `lidarslam_ros2` runs ROS 2 Jazzy with RViz.
- RKO-LIO frontend publishes odometry, current LiDAR frame, local map, and frontend path.
- Graph backend receives RKO-LIO odometry plus an XYZI point cloud adapter.
- Backend can build submaps, search for loops, and save map outputs.
- Enhanced lunar terrain worlds can be generated from local height images with added craters, rocks, roughness, and texture.
- GPU OpenGL acceleration works inside the Docker container on WSL when `/dev/dxg` and WSL GPU libraries are mounted.

Experimental / not solved yet:

- `/reference/path` is not a trusted ground-truth path yet. The Gazebo dynamic pose bridge did not preserve usable entity names consistently, so the true path can glitch or align incorrectly.
- Backend loop closure quality is terrain-dependent. Flat or self-similar lunar terrain can make scan matching and loop detection weak.
- The simulated LiDAR is a Robin W-style Gazebo `gpu_lidar`, not the real Seyond driver or a full vendor-accurate simulator.
- Navigation is not yet the main focus. The current stack is mapping and localization bringup first.

## Repository Layout

Important files and folders:

```text
husky/
  robot.yaml
  robot.urdf.xacro
  source_togo_custom.sh
  start_seyond_support.sh
  wasd_teleop.py

  custom_ws/src/togo_custom/
    togo_description/urdf/togo_description.urdf.xacro
    togo_bringup/launch/togo_bringup.launch.py
    togo_bringup/config/seyond_bridge.yaml

  lidarslam_ros2/
    Thirdparty/rko_lio/
    graph_based_slam/
    lidarslam/
      launch/seyond_live_slam.launch.py
      param/lidarslam_mid360_rko_graph.yaml

  scripts/
    run_live_seyond_slam_integrated.sh
    run_rko_lio_seyond.sh
    run_graph_slam_seyond_xyzi.sh
    add_intensity_to_cloud.py
    odom_to_path.py
    gazebo_pose_to_aligned_path.py
    record_seyond_slam_bag.sh
    save_seyond_map_clean.sh
    check_seyond_sim.sh
    make_enhanced_lunar_world.py

  lunar_enhanced_nac_04_small/
    enhanced_lunar.world.sdf
    enhanced_lunar_terrain.obj
    enhanced_lunar_height.png

  bags/
  slam_output/
  docker-compose.lidarslam.yml
  Dockerfile.lidarslam
```

Notes:

- `lidarslam_ros2/` is currently ignored by `.gitignore`, because it is an external cloned repo. If you change files inside it and want those changes tracked in this repo, either force-add them intentionally or copy important local launch/config files into a tracked folder.
- `bags/`, `slam_output/`, `build/`, `install/`, and `log/` should stay out of Git.
- The root `README.md`, `scripts/`, `custom_ws/`, terrain generator scripts, `robot.yaml`, and Docker files are the main things worth preserving.

## System Architecture

High-level data flow:

```text
Gazebo Harmonic / Clearpath sim
  |
  |  GPU LiDAR, IMU, wheel odom, TF, clock
  v
ros_gz_bridge + TF/static TF support
  |
  |  /a300_0000/sensors/seyond_robin_w/scan/points
  |  /a300_0000/sensors/seyond_robin_w/imu
  |  /clock
  |  /tf, /tf_static
  v
RKO-LIO frontend
  |
  |  /rko_lio/odometry
  |  /rko_lio/frame
  |  /rko_lio/local_map
  v
XYZI adapter
  |
  |  /rko_lio/frame_xyzi
  v
graph_based_slam backend
  |
  |  /modified_path
  |  /modified_map
  |  /modified_map_array
  |  map_save outputs
  v
RViz + saved map files
```

The frontend and backend are connected in the integrated launch. The frontend estimates motion continuously. The backend consumes frontend odometry and LiDAR submaps to build a pose graph and search for loop closures.

## Robot and Sensor Model

The simulated rover uses the Clearpath configuration style. The namespace is `/a300_0000`.

Current custom sensor package:

```text
Seyond Robin W-style simulated sensor
  parent frame: base_link
  mount xyz:   0.30 0.0 0.42
  lidar frame: seyond_robin_w_lidar_frame
  imu frame:   seyond_robin_w_imu_frame
```

Simulated public Robin W-style field of view target:

```text
Horizontal FOV: 120 deg, -60 deg to +60 deg
Vertical FOV:    70 deg, -35 deg to +35 deg
```

Current Gazebo sensor settings in `custom_ws/src/togo_custom/togo_description/urdf/togo_description.urdf.xacro`:

```text
LiDAR type:     gpu_lidar
LiDAR rate:     15 Hz
LiDAR samples:  256 horizontal x 32 vertical
LiDAR range:    0.2 m to 80.0 m
LiDAR topic:    /a300_0000/sensors/seyond_robin_w/scan
Point topic:    /a300_0000/sensors/seyond_robin_w/scan/points

IMU type:       imu
IMU rate:       100 Hz
IMU topic:      /a300_0000/sensors/seyond_robin_w/imu
```

Important distinction:

- The real Seyond ROS driver is for physical hardware.
- This simulation uses Gazebo's `gpu_lidar` sensor to create a Robin W-style point cloud.
- For early SLAM testing, that is enough to validate topic flow, timing, TF, frontend behavior, backend behavior, map saving, and navigation concepts.
- For hardware realism, we still need sensor noise validation, timestamp validation, extrinsic calibration, and real driver testing.
## SLAM Repo Overview

This workspace uses `rsasaki0109/lidarslam_ros2` as the SLAM base.

Major parts:

```text
Thirdparty/rko_lio/       LiDAR-inertial odometry frontend
graph_based_slam/         pose graph backend, submaps, loop detection, map saving
lidarslam/                launch and configuration glue
lidarslam_msgs/           custom messages used by backend map outputs
Thirdparty/ndt_omp_ros2/  fast NDT registration support
```

### Frontend: RKO-LIO

RKO-LIO is the live odometry frontend. It estimates the rover trajectory from LiDAR plus IMU.

Inputs:

```text
/a300_0000/sensors/seyond_robin_w/scan/points  sensor_msgs/msg/PointCloud2
/a300_0000/sensors/seyond_robin_w/imu          sensor_msgs/msg/Imu
/tf, /tf_static                                sensor extrinsics
/clock                                         simulation time
```

Outputs:

```text
/rko_lio/odometry   nav_msgs/msg/Odometry
/rko_lio/frame      sensor_msgs/msg/PointCloud2, current registered scan
/rko_lio/local_map  sensor_msgs/msg/PointCloud2, local map
/rko_lio/path       nav_msgs/msg/Path, helper output from odom_to_path.py
```

What it gives us:

- A continuous local odometry estimate.
- A live registered LiDAR view.
- A local map useful for checking whether registration is alive.
- The frontend trajectory that the backend can optimize later.

What it does not solve by itself:

- It does not provide global loop closure by itself.
- It can drift over time.
- It can struggle when the terrain is flat, repetitive, or lacks geometric structure.
- If TF, time, or IMU topics are wrong, it may initialize once and then appear frozen.

Current RKO-LIO settings in the integrated launch:

```text
voxel_size:                   0.20
double_downsample:            false
max_correspondance_distance:  4.0
max_scan_delta_sec:           10.0
min_range:                    0.2
max_range:                    80.0
deskew:                       false
publish_deskewed_scan:        true
publish_local_map:            true
use_sim_time:                 true
```

`deskew` is currently false because the simulated point cloud timing is not yet trusted enough. Deskewing should be revisited once per-point timestamps or equivalent scan timing are understood.

### XYZI Adapter

The backend expects a point cloud with an `intensity` field. RKO-LIO's `/rko_lio/frame` did not always include intensity, so graph SLAM printed:

```text
Failed to find match for field 'intensity'.
```

The helper script fixes that:

```text
/scripts/add_intensity_to_cloud.py
```

It subscribes to `/rko_lio/frame` and publishes `/rko_lio/frame_xyzi`.

This is a compatibility adapter. It does not improve the scan data; it just makes the cloud format acceptable to the backend.

### Backend: graph_based_slam

The backend consumes frontend odometry and point clouds, builds submaps, adds constraints, searches for loop closures, and saves map outputs.

Inputs in the integrated launch:

```text
odom_input  -> /rko_lio/odometry
cloud_input -> /rko_lio/frame_xyzi
```

Outputs:

```text
/modified_path       optimized or backend path
/modified_map        optimized saved map cloud
/modified_map_array  map/submap array message when valid
/map_save            service used to save/publish map products
```

Important behavior:

- It does not necessarily publish a pretty global map every frame.
- It creates submaps based on motion thresholds.
- It periodically searches for loop closures.
- It often publishes or refreshes the global map when `/map_save` is called.
- In the integrated launch, a map-save pulse calls `/map_save` every `map_save_period` seconds so `/modified_map` and `/modified_path` refresh while you drive.

Expected backend logs:

```text
Direct odom+cloud input mode enabled
First odom received: (...)
First cloud received, ... bytes
searching Loop, num_submaps: 2
searching Loop, num_submaps: 3
...
```

Seeing `searching Loop` means the backend is alive and building submaps. It does not guarantee a loop closure was accepted. Loop closure requires enough distinct geometry and a good candidate match.

## Current Graph SLAM Parameters

The main parameter file is:

```text
lidarslam_ros2/lidarslam/param/lidarslam_mid360_rko_graph.yaml
```

Current important values:

```text
registration_method: NDT
ndt_resolution: 5.0
voxel_leaf_size: 0.2
loop_detection_period: 1000
threshold_loop_closure_score: 15.0
distance_loop_closure: 100.0
range_of_searching_loop_closure: 50.0
search_submap_num: 5
submap_distance_threshold: 1.5
num_adjacent_pose_cnstraints: 5
use_scan_context: false
use_bev_descriptor: false
use_solid_descriptor: false
use_triangle_descriptor: false
use_dynamic_object_filter: false
use_save_map_in_loop: true
```

The integrated launch overrides some backend parameters:

```text
use_odom_input: true
global_frame_id: map
submap_distance_threshold: 0.8
debug_flag: true
map_save_dir: /ws/src/lidarslam_ros2/output/husky_seyond_graph
```

### NDT vs GICP

The backend can use NDT or GICP depending on config.

NDT is often useful when point clouds have enough 3D structure and local distributions are stable. It can be a good candidate for outdoor terrain, but lunar terrain can be difficult because:

- Some areas are too flat.
- Some areas are repetitive.
- Rocks and craters may be sparse.
- A narrow-FOV LiDAR sees only a slice of the scene.
- Registration can become underconstrained in yaw or lateral motion.

GICP can sometimes behave better on detailed local geometry but may be noisier or more expensive depending on cloud density. It is worth A/B testing NDT and GICP on the same bag.

## How To Run The Current Stack

Use separate terminals. Keep each long-running terminal open.

### Terminal 1: launch Gazebo sim

```bash
cd /mnt/c/Users/Username/OneDrive/Desktop/husky
source /opt/ros/jazzy/setup.bash
source source_togo_custom.sh
```

Optional: install the current enhanced lunar world as Clearpath's `warehouse` world slot:

```bash
cp /mnt/c/Users/Username/OneDrive/Desktop/husky/lunar_enhanced_nac_04_small/enhanced_lunar.world.sdf \
  $(ros2 pkg prefix clearpath_gz)/share/clearpath_gz/worlds/warehouse.sdf
```

Launch:

```bash
ros2 launch clearpath_gz simulation.launch.py \
  setup_path:=/mnt/c/Users/Username/OneDrive/Desktop/husky \
  world:=warehouse \
  z:=4.0 \
  rviz:=false
```

If the rover falls through the terrain, increase `z` to `6.0`. If the sim is very slow, use a smaller world, lower mesh resolution, lower LiDAR sample count, or disable RViz/Gazebo GUI extras.

### Terminal 2: bridge Seyond sensors and TF

```bash
cd /mnt/c/Users/Username/OneDrive/Desktop/husky
bash start_seyond_support.sh
```

This starts:

- Gazebo to ROS bridge for Seyond IMU.
- Gazebo to ROS bridge for Seyond LiDAR point cloud.
- Gazebo to ROS bridge for Seyond 2D scan.
- Static TFs from `base_link` to the Seyond LiDAR and IMU frames.
- TF relay from `/a300_0000/tf` to `/tf`.
- Experimental Gazebo dynamic pose bridge for reference path work.

The reference path part is experimental. Do not treat `/reference/path` as truth yet.

### Terminal 3: launch SLAM Docker

```bash
cd /mnt/c/Users/Username/OneDrive/Desktop/husky
ROS_DOMAIN_ID=0 docker compose -f docker-compose.lidarslam.yml run --rm lidarslam
```

Inside the Docker shell:

```bash
bash /scripts/run_live_seyond_slam_integrated.sh
```

That launch starts:

- RKO-LIO frontend.
- `/rko_lio/frame` to `/rko_lio/frame_xyzi` adapter.
- graph_based_slam backend.
- frontend path publisher `/rko_lio/path`.
- experimental reference path publisher `/reference/path`.
- static `map -> odom` TF.
- periodic `/map_save` calls.
- RViz.

### Terminal 4: drive the rover

```bash
cd /mnt/c/Users/Username/OneDrive/Desktop/husky
source /opt/ros/jazzy/setup.bash
python3 wasd_teleop.py
```

Typical controls:

```text
W/S: forward/back
A/D: turn left/right
Space or stop key: stop
```

## RViz: What You Should Look At

Set RViz fixed frame depending on what you are inspecting:

```text
odom  for frontend RKO-LIO outputs
map   for backend graph outputs
```

Useful displays:

```text
PointCloud2: /a300_0000/sensors/seyond_robin_w/scan/points
PointCloud2: /rko_lio/frame
PointCloud2: /rko_lio/local_map
PointCloud2: /rko_lio/frame_xyzi
Path:        /rko_lio/path
Path:        /modified_path
PointCloud2: /modified_map
TF
RobotModel, if available
```

Interpretation:

- `/a300_0000/sensors/seyond_robin_w/scan/points` is raw simulated LiDAR.
- `/rko_lio/frame` is the frontend's current scan after processing.
- `/rko_lio/local_map` is the frontend local map.
- `/rko_lio/path` is the frontend odometry path.
- `/modified_path` is the backend graph path after backend processing.
- `/modified_map` is the backend global map output, usually refreshed by `/map_save`.

If the map looks still but the camera moves, check the fixed frame and topic. A local map in a moving frame can appear stable even while the rover moves. A global map should be viewed in `map` or `odom` depending on the publisher frame.

## Validation Commands

Run from WSL unless stated otherwise.

Check sensor topics:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list -t | grep seyond
```

Expected:

```text
/a300_0000/sensors/seyond_robin_w/imu [sensor_msgs/msg/Imu]
/a300_0000/sensors/seyond_robin_w/scan [sensor_msgs/msg/LaserScan]
/a300_0000/sensors/seyond_robin_w/scan/points [sensor_msgs/msg/PointCloud2]
```

Check rates:

```bash
timeout 6 ros2 topic hz /a300_0000/sensors/seyond_robin_w/scan/points
timeout 6 ros2 topic hz /a300_0000/sensors/seyond_robin_w/imu
```

Check frames:

```bash
timeout 5 ros2 run tf2_ros tf2_echo base_link seyond_robin_w_lidar_frame
timeout 5 ros2 run tf2_ros tf2_echo base_link seyond_robin_w_imu_frame
```

Check frontend:

```bash
timeout 8 ros2 topic hz /rko_lio/odometry
timeout 8 ros2 topic hz /rko_lio/frame
timeout 8 ros2 topic hz /rko_lio/local_map
```

Check backend:

```bash
timeout 10 ros2 topic hz /modified_path
timeout 10 ros2 topic hz /modified_map
ros2 service list | grep map_save
```

Run the helper:

```bash
cd /mnt/c/Users/Username/OneDrive/Desktop/husky
bash scripts/check_seyond_sim.sh
```

## GPU Checks In Docker

Inside the Docker container:

```bash
glxinfo -B
```

Good output for WSL GPU acceleration looks like:

```text
Device: D3D12 (NVIDIA GeForce RTX 2080)
Accelerated: yes
OpenGL renderer string: D3D12 (NVIDIA GeForce RTX 2080)
```

Bad output looks like:

```text
Device: llvmpipe
Accelerated: no
OpenGL renderer string: llvmpipe
```

If Docker falls back to `llvmpipe`, confirm `docker-compose.lidarslam.yml` includes:

```yaml
devices:
  - /dev/dxg:/dev/dxg
volumes:
  - /usr/lib/wsl:/usr/lib/wsl:ro
  - /mnt/wslg:/mnt/wslg:rw
environment:
  LD_LIBRARY_PATH: /usr/lib/wsl/lib:${LD_LIBRARY_PATH}
  LIBGL_ALWAYS_SOFTWARE: "0"
  MESA_D3D12_DEFAULT_ADAPTER_NAME: NVIDIA
  GALLIUM_DRIVER: d3d12
  MESA_LOADER_DRIVER_OVERRIDE: d3d12
```

On WSL there may be no `/dev/dri`. Use `/dev/dxg`.

## Recording Bags

Recording is the best way to debug SLAM because you can replay the exact same data after changing parameters.

Inside Docker, with sim and SLAM running:

```bash
bash /scripts/record_seyond_slam_bag.sh seyond_lunar_test_01
```

This records to `/bags/seyond_lunar_test_01`, which maps to the host path:

```text
/mnt/c/Users/Username/OneDrive/Desktop/husky/bags/seyond_lunar_test_01
```

Inspect a bag from WSL:

```bash
ros2 bag info /mnt/c/Users/Username/OneDrive/Desktop/husky/bags/seyond_lunar_test_01
```

Important recorded topics:

```text
/clock
/a300_0000/sensors/seyond_robin_w/scan/points
/a300_0000/sensors/seyond_robin_w/imu
/a300_0000/platform/odom
/a300_0000/tf
/a300_0000/tf_static
/tf
/tf_static
/rko_lio/odometry
/rko_lio/path
/rko_lio/frame
/rko_lio/frame_xyzi
/rko_lio/local_map
/modified_map
/modified_path
/modified_map_array
/reference/path
```

Play slower:

```bash
ros2 bag play bags/seyond_lunar_test_01 --clock --rate 0.5
```

Loop playback:

```bash
ros2 bag play bags/seyond_lunar_test_01 --clock --loop
```

If replaying into SLAM, be careful not to also have the live sim publishing the same topics.

## Saving Maps

The integrated launch periodically calls `/map_save`.

You can manually trigger it from inside Docker:

```bash
bash /scripts/save_seyond_map_clean.sh
```

Expected output locations:

```text
/ws/src/lidarslam_ros2/output/husky_seyond_graph/
```

Mounted on host as:

```text
/mnt/c/Users/Username/OneDrive/Desktop/husky/slam_output/husky_seyond_graph/
```

Look for files such as:

```text
map.pcd
pose_graph.g2o
pointcloud_map_metadata.yaml
map_projector_info.yaml
```

The exact file set depends on the backend save path and whether enough valid map data exists.

## Lunar Terrain Workflow

The most reliable path so far is not importing a huge lunar mesh directly. Huge or complex meshes slowed Gazebo heavily and sometimes caused rendering issues. The better workflow is:

1. Start from a local height image or DEM crop.
2. Generate a bounded terrain mesh.
3. Add artificial but plausible detail: craters, small rocks, roughness, darker material.
4. Keep the world small enough for Gazebo sensors to run fast.

Current generator:

```text
scripts/make_enhanced_lunar_world.py
```

Current preferred world:

```text
lunar_enhanced_nac_04_small/enhanced_lunar.world.sdf
```

Why smaller is better:

- Gazebo real-time factor drops hard when the terrain mesh is large.
- GPU LiDAR rendering gets more expensive with more geometry.
- RViz and Gazebo both compete for graphics resources.
- For SLAM testing, a 50 m to 150 m local course is usually more useful than a giant slow map.

Good terrain for LiDAR SLAM should include:

- Uneven slopes.
- Distinct rocks.
- Crater rims.
- Ridges or berms.
- Enough vertical variation to constrain scan matching.
- A flat spawn area so the rover does not fall through or tip immediately.

Bad terrain for LiDAR SLAM:

- Perfectly flat planes.
- Repeating noise without stable structure.
- Very sparse features.
- Huge meshes that run at 3 percent real-time factor.

## Why The Frontend Can Drift

RKO-LIO drift on lunar terrain is not surprising. Common causes:

```text
Flat terrain: weak geometric constraints.
Repeated rocks/craters: scan matching can choose wrong local minima.
Narrow FOV: fewer constraints than a spinning 360 LiDAR.
Low LiDAR rate: fewer updates during turns.
No deskew: moving scans can be warped.
Poor simulated sensor timing: IMU/LiDAR sync can be imperfect.
Simplified IMU noise: not hardware-realistic.
Extrinsic assumptions: LiDAR and IMU are currently co-located and simple.
```

Ways to improve frontend reliability:

- Add richer terrain geometry, especially asymmetric rocks and crater rims.
- Tune `voxel_size`; smaller preserves detail but costs CPU.
- Tune `max_correspondance_distance`; too large can accept bad matches, too small can lose tracking.
- Increase simulated LiDAR rate or samples only if real-time factor remains healthy.
- Enable deskew only after validating scan timestamps.
- Add realistic IMU noise and bias behavior.
- Validate TF extrinsics carefully.
- Test the same route with bags so parameter changes are comparable.

## Why The Backend May Not Show Loop Closure

The backend can be alive without accepting loop closures.

Signs it is alive:

```text
First odom received
First cloud received
searching Loop, num_submaps:N
```

Signs a map is being saved:

```text
/map_save service exists
/modified_map publishes after service call
/modified_path publishes after service call
```

Reasons loop closures may not happen:

- The rover has not revisited a previous area.
- The revisit angle is too different for the current descriptor/registration settings.
- Terrain is too self-similar.
- The loop closure threshold is too strict.
- NDT rejects the candidate after descriptor search.
- The backend is waiting for enough submaps.
- Global map publication is tied to `/map_save`, not every incoming scan.

## Parameters Worth Tuning First

Frontend RKO-LIO:

```text
voxel_size
max_correspondance_distance
max_scan_delta_sec
min_range
max_range
deskew
publish_local_map
```

Backend graph SLAM:

```text
registration_method
ndt_resolution
voxel_leaf_size
submap_distance_threshold
num_adjacent_pose_cnstraints
adjacent_edge_info_weight
loop_edge_info_weight
threshold_loop_closure_score
range_of_searching_loop_closure
search_submap_num
use_scan_context
use_bev_descriptor
use_solid_descriptor
use_triangle_descriptor
```

Good first experiments:

1. Try `registration_method: NDT` vs `GICP` on the same bag.
2. Try smaller `submap_distance_threshold` on short lunar tests.
3. Enable one descriptor at a time, not all together.
4. Lower loop thresholds gradually, then inspect false positives.
5. Compare `/rko_lio/path` and `/modified_path` after a loop-shaped route.
6. Save maps every 10 seconds during testing, then less often later.

## Improvement Roadmap For Our Goals

### 1. Fix Ground Truth Properly

Current `/reference/path` is experimental. The dynamic pose bridge did not reliably preserve the robot entity name, so the reference path can be wrong.

Best fix:

- Write a small Gazebo system plugin or ROS/GZ node that publishes the model pose of exactly `a300-0000` or the real robot entity.
- Publish as `nav_msgs/msg/Odometry` and `nav_msgs/msg/Path` on stable topics:

```text
/ground_truth/odom
/ground_truth/path
```

Do not infer truth from blank TF child names or from platform odometry if the goal is evaluation.

Why this matters:

- Without ground truth, it is hard to know whether SLAM improved or just looked better.
- Frontend and backend comparisons need absolute trajectory error or at least consistent relative error.

### 2. Add Adaptive Noise / Adaptive Weighting

This is a strong improvement idea for lunar terrain.

Right now many weights are static:

```text
adjacent_edge_info_weight
loop_edge_info_weight
loop_edge_robust_kernel_delta
```

A better backend should scale confidence based on measurement quality:

```text
Point count after filtering
NDT/GICP fitness score
Estimated degeneracy of scan geometry
Terrain roughness / feature richness
IMU excitation and saturation
LiDAR range distribution
Submap overlap
GNSS/RTK covariance when available
```

Example policy:

```text
If scan is flat and low-feature:
  lower scan-matching confidence
  increase reliance on IMU/wheel/GNSS factors

If scan has strong 3D structure and good NDT fitness:
  increase adjacent edge confidence

If loop candidate score is marginal:
  keep robust kernel strong
  require descriptor agreement or geometric verification
```

This is especially relevant for lunar-like terrain where observability changes constantly.

### 3. Add GNSS / RTK Factors

Your mentor mentioned enabling RTK corrections for onboard GPS antennas. For Earth outdoor testing, RTK can be a huge stabilizer.

For the real Husky/TOGO outdoor testbed:

- Use `robot_localization` to fuse wheel odom, IMU, and GNSS for a baseline estimate.
- Feed GNSS covariance into the graph backend if supported.
- Treat RTK fix quality differently from float or no-fix.
- Use GNSS as a global prior, not a replacement for local LiDAR odometry.

For lunar simulation, GNSS is not physically lunar-realistic, but a simulated global pose sensor can act as ground truth or as an analog for external localization.

### 4. Improve Loop Closure For Lunar Terrain

Default descriptors are off. For lunar terrain, test these carefully:

```text
use_scan_context
use_bev_descriptor
use_solid_descriptor
use_triangle_descriptor
```

Suggested order:

1. Baseline with all descriptors off.
2. Enable Scan Context if the sensor coverage is wide enough.
3. Try BEV descriptor if terrain has height variation.
4. Try triangle descriptor with `edge_3d` if rocks/ridges create stable keypoints.
5. Add geometric verification and strict robust kernels to reject false loops.

Lunar terrain is self-similar, so false positive loop closures are dangerous. A bad loop closure can make the map worse than no loop closure.

### 5. Improve Sensor Realism

Current simulation is stable but simplified.

Potential improvements:

- More realistic LiDAR noise as a function of range and incidence angle.
- Dropout on dark or steep surfaces.
- Motion distortion with deskew enabled.
- IMU bias random walk.
- Slight extrinsic offsets between LiDAR and IMU.
- Lower update rates to match real hardware.
- Real Seyond driver testing with recorded hardware bags when available.

### 6. Build A Repeatable Evaluation Harness

Instead of tuning live by eye, create test routes:

```text
short straight route
square loop
figure-eight route
slope traversal
rock field traversal
crater loop
```

For each route record:

```text
raw LiDAR
raw IMU
clock
TF
platform odom
ground truth, once fixed
frontend outputs
backend outputs
```

Then compute:

```text
frontend drift
backend drift
loop closure count
false loop count
map consistency
runtime / real-time factor
CPU/GPU load
```

### 7. Navigation Layer Later

Once localization and mapping are stable, add navigation.

Likely path:

```text
Point cloud / local map
  -> obstacle representation
  -> costmap or elevation map
  -> planner
  -> controller
```

Options:

- Nav2 with voxel or obstacle layers for near-term testing.
- Elevation mapping or grid_map for rough terrain.
- Traversability analysis for slopes, rocks, and craters.
- Local planner tuned for skid-steer/differential drive.

Do not make Nav2 the first debugging target if SLAM is still unstable. Bad localization makes navigation look bad even when the planner is fine.

## Alternative SLAM Approaches

This repo is a good starting point, but not the only option.

### KISS-ICP

Pros:

- Simple, strong LiDAR odometry baseline.
- Good for quickly testing point cloud geometry.
- Less complex than full LIO.

Cons:

- No IMU by default in the basic approach.
- Still drifts without loop closure.

Use it as a baseline to ask: is the LiDAR geometry alone enough?

### FAST-LIO / FAST-LIO2

Pros:

- Popular LiDAR-inertial odometry family.
- Fast and strong for many platforms.

Cons:

- Integration and licensing need review.
- Backend loop closure still needs separate handling.

Useful if RKO-LIO remains fragile with the simulated sensor.

### LIO-SAM

Pros:

- Full factor-graph style LiDAR-inertial SLAM with loop closure and GPS factor support.
- Conceptually aligned with outdoor robot plus GNSS goals.

Cons:

- Licensing and ROS 2 Jazzy compatibility need careful checking.
- More setup complexity.

Useful as a reference architecture even if not adopted directly.

### Cartographer / slam_toolbox

Pros:

- Mature mapping tools.
- `slam_toolbox` is great for 2D LiDAR.

Cons:

- Less ideal for full 3D lunar terrain with 3D LiDAR.
- May not use the 3D structure we care about.

Useful if the task becomes mostly 2D navigation and occupancy mapping.

### RTAB-Map / Visual-LiDAR Options

Pros:

- Can use cameras plus LiDAR.
- Useful when optical relative navigation becomes part of the project.

Cons:

- More sensor dependencies.
- Lunar lighting and texture assumptions can be hard.

Useful later if event cameras or optical relative navigation become central.

## Recommended Best Setup Right Now

For your current internship-style goal, the best practical setup is:

```text
Gazebo Clearpath Husky/A300 sim
  + stable Robin W-style gpu_lidar
  + stable co-located IMU
  + enhanced small lunar terrain
  + RKO-LIO frontend
  + graph_based_slam backend
  + bag-first evaluation workflow
```

Why this is the right near-term setup:

- It is close to the real Husky/TOGO hardware direction.
- It keeps LiDAR and IMU in the loop.
- It gives you a live frontend and backend to inspect.
- It gives you room to add GNSS/RTK later.
- It does not overcommit to navigation before localization is understood.
- It lets you build a portfolio-quality capstone workflow: simulate, drive, record, map, evaluate, tune, repeat.

## Common Failure Modes

### RKO-LIO initializes once and then nothing moves

Check:

```bash
timeout 6 ros2 topic hz /a300_0000/sensors/seyond_robin_w/scan/points
timeout 6 ros2 topic hz /a300_0000/sensors/seyond_robin_w/imu
timeout 5 ros2 run tf2_ros tf2_echo base_link seyond_robin_w_lidar_frame
ros2 param get /rko_lio_online_node use_sim_time
```

Likely causes:

- Missing IMU bridge.
- Missing TF.
- `use_sim_time` false in Docker node.
- Wrong LiDAR topic, using `/points` instead of `/scan/points`.
- Sim paused or real-time factor extremely low.

### RViz says message filter queue is full

Likely causes:

- Fixed frame mismatch.
- Missing transform between message frame and RViz fixed frame.
- Messages timestamped with sim time but node not using sim time.
- Large point clouds overloading RViz.

Try:

```text
Set fixed frame to odom for frontend.
Set fixed frame to map for backend.
Increase queue size on PointCloud2 display.
Disable extra point cloud displays.
```

### Backend logs `searching Loop` but no visible map

This can be normal. Try:

```bash
ros2 service call /map_save std_srvs/srv/Empty
timeout 10 ros2 topic hz /modified_map
timeout 10 ros2 topic hz /modified_path
```

Also confirm graph inputs:

```bash
timeout 8 ros2 topic hz /rko_lio/odometry
timeout 8 ros2 topic hz /rko_lio/frame_xyzi
```

### `Failed to find match for field 'intensity'`

Use `/rko_lio/frame_xyzi`, not `/rko_lio/frame`, as backend cloud input.

### Docker RViz uses CPU rendering

Inside Docker:

```bash
glxinfo -B
```

If it says `llvmpipe`, fix Docker GPU mounts/env. On WSL use `/dev/dxg`, not `/dev/dri`.

### Sim is too slow

Reduce one or more:

```text
terrain mesh resolution
terrain physical size
LiDAR horizontal samples
LiDAR vertical samples
LiDAR update rate
number of RViz point cloud displays
Gazebo GUI rendering load
```

## Git / Sharing Notes

This repo has a local commit checkpoint, but pushing may require GitHub auth setup in WSL.

Current intended remote:

```text
https://github.com/narcisogon/togo-husky-sim.git
```

If push auth fails, use one of:

```bash
gh auth login
```

or switch remote to SSH after setting up an SSH key:

```bash
git remote set-url origin git@github.com:narcisogon/togo-husky-sim.git
git push -u origin main
```

Do not commit large generated data:

```text
bags/
slam_output/
*.tif
*.tiff
build/
install/
log/
```

Do commit:

```text
README.md
scripts/
custom_ws/
robot.yaml
Dockerfile.lidarslam
docker-compose.lidarslam.yml
small generated terrain worlds if they are not too large
```

## Suggested Next Milestones

1. Stabilize the current live workflow.
   - Sim, bridge, Docker SLAM, drive, save map.

2. Record three repeatable bags.
   - Straight path.
   - Square loop.
   - Crater/rock loop.

3. Fix real ground truth.
   - Publish `/ground_truth/odom` and `/ground_truth/path` from Gazebo model pose.

4. Compare frontend vs backend.
   - `/rko_lio/path` vs `/modified_path` vs `/ground_truth/path`.

5. Tune backend on bags, not live only.
   - NDT vs GICP.
   - Descriptor options.
   - Loop thresholds.

6. Add adaptive confidence.
   - Scale graph weights using scan quality, registration score, and terrain degeneracy.

7. Add GNSS/RTK path once hardware or simulated global pose is available.
   - Use it as a factor/prior, not as a replacement for LiDAR odometry.

8. Start navigation after localization is understandable.
   - Local obstacle map first.
   - Then Nav2 or rough-terrain traversability.

## Mental Model

Think of the stack like this:

```text
Gazebo creates the world and fake sensors.
Bridges make those fake sensors look like ROS topics.
RKO-LIO estimates where the rover is right now.
graph_based_slam tries to make that trajectory globally consistent.
RViz shows whether the data flow and transforms make sense.
Bags let you replay the same truth when tuning.
Ground truth, once fixed, tells you whether anything actually improved.
```

That is the core workflow. When something breaks, debug in this order: sim, bridge, topics, TF, frontend, adapter, backend, visualization.
