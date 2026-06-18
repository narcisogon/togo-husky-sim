# Navigation Architecture On Top Of SLAM

This document describes the planned real-time navigation layer for the simulated Husky/A300 lunar-style rover. The current SLAM stack is already working, so the navigation layer should build on top of it rather than replacing the frontend.

The goal is to use the existing LiDAR-inertial SLAM pose and map products for Nav2 planning, local obstacle avoidance, loop-closure-safe navigation, and eventually lunar terrain traversability.

## Existing SLAM Base

Current platform:

```text
Rover:    simulated Husky/A300
Sensors:  Seyond Robin W-style 3D LiDAR + IMU
Frontend: RKO-LIO
Backend:  graph_based_slam
ROS:      ROS 2 Jazzy
```

Main SLAM files:

```text
Launch:
  lidarslam_ros2/lidarslam/launch/seyond_live_slam.launch.py

Parameters:
  lidarslam_ros2/lidarslam/param/seyond_live_slam.yaml

Run script:
  scripts/run_live_seyond_slam_integrated.sh
```

Current SLAM data flow:

```text
Gazebo Seyond LiDAR + IMU
  -> ROS bridge / static TF support
  -> RKO-LIO frontend
  -> /rko_lio/odometry + /rko_lio/frame
  -> XYZI adapter
  -> graph_based_slam backend
  -> map->odom correction + /modified_path + /modified_map
```

Frontend outputs:

```text
/rko_lio/odometry
/rko_lio/frame
/rko_lio/local_map
/rko_lio/path
/rko_lio/registration_diagnostics
/rko_lio/runtime_diagnostics
```

Backend outputs:

```text
map -> odom
/modified_path
/modified_map
/modified_map_array
/map_save
```

## Frame Design

RKO-LIO publishes smooth local odometry:

```text
odom -> base_link
```

The backend publishes global correction:

```text
map -> odom
```

Together, navigation should use:

```text
map -> odom -> base_link
```

The corrected navigation pose is:

```text
map -> base_link
```

Important rule:

```text
Do not make the frontend jump when loop closure happens.
Let graph_based_slam correct global drift through map->odom.
Keep RKO-LIO publishing smooth odom->base_link.
```

## Target Navigation Stack

Use upstream Nav2 Jazzy unless there is a clear, well-maintained, Jazzy-compatible reason not to.

Current package scaffold:

```text
lidarslam_ros2/togo_navigation/
  launch/rover_nav2.launch.py
  config/nav2_slam_params.yaml
  src/slam_to_occupancy_grid.cpp
  scripts/debug_map_publisher.py
  rviz/rover_nav_debug.rviz

scripts/run_nav2_with_slam.sh
```

The SLAM Dockerfile now installs:

```text
ros-jazzy-navigation2
ros-jazzy-nav2-bringup
```

High-level architecture:

```text
RKO-LIO frontend
  -> smooth local odometry
  -> local registered point clouds

graph_based_slam backend
  -> global map->odom correction
  -> optimized path and map products

Navigation mapper
  -> converts SLAM map and/or live LiDAR to /map
  -> creates occupancy/traversability grids

Nav2
  -> global planning in map frame
  -> local obstacle avoidance in odom frame
  -> recovery, replanning, backup, wait, costmap clearing

Loop closure monitor
  -> detects sudden map->odom corrections
  -> pauses/clears/replans safely
```

## Nav2 Frame Configuration

Global costmap:

```yaml
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  rolling_window: false
```

Local costmap:

```yaml
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  rolling_window: true
```

Why:

```text
Global costmap in map:
  follows globally corrected SLAM and supports long-range planning.

Local costmap in odom:
  remains smooth during loop closures and protects immediate obstacle avoidance.
```

## Initial Nav2 Choices

Global planner:

```text
Start: Smac Hybrid-A*
Later: Smac State Lattice if rover motion constraints need better modeling
Avoid: plain NavFn-style planning as the final solution if rover kinematics matter
```

Controller:

```text
Start: Regulated Pure Pursuit
Later: MPPI for more advanced local control and obstacle avoidance
```

Behavior tree:

```text
Enable replanning.
Enable costmap clearing.
Enable backup behavior.
Enable wait behavior.
Add loop-closure correction behavior later.
```

## Costmap Design

Use two costmaps.

### Local Costmap

Frame:

```text
odom
```

Type:

```text
rolling window
```

Purpose:

```text
real-time nearby obstacle avoidance
```

Inputs:

```text
live LiDAR
filtered point cloud
optional voxel/STVL layer
optional local traversability layer
```

Recommended layers:

```text
Obstacle layer or Voxel/STVL layer
Inflation layer
Denoise layer if available
Optional custom traversability layer
```

### Global Costmap

Frame:

```text
map
```

Type:

```text
global/static or semi-static
```

Purpose:

```text
long-range planning using corrected SLAM map
```

Inputs:

```text
backend-corrected OGM
/modified_map converted to OccupancyGrid
optional global traversability map
optional keepout/speed zones
```

Recommended layers:

```text
Static layer from SLAM-generated occupancy grid
Inflation layer
Optional keepout/speed filter
Custom terrain/traversability cost layer
```

## Occupancy Grid Mapper

The first navigation mapper should be simple and Nav2-compatible.

Inputs:

```text
/modified_map
/rko_lio/frame_xyzi
TF map->odom->base_link
```

Outputs:

```text
/map  nav_msgs/msg/OccupancyGrid
```

Basic logic:

```text
Transform points into the target frame.
Apply height filtering.
Voxel downsample or grid bin points.
Mark cells occupied if obstacle points exceed a threshold.
Mark observed cells free if no obstacle is present.
Leave unobserved cells unknown.
Publish nav_msgs/OccupancyGrid.
```

Configurable parameters:

```text
resolution
map_width_m
map_height_m
min_obstacle_height_m
max_obstacle_height_m
min_points_per_cell
unknown_cost
occupied_threshold
free_threshold
max_cell_age_sec
```

## Traversability Mapper

Stock Nav2 is not slope-aware by default. Lunar navigation needs a terrain layer that converts geometry risk into costmap values.

Possible inputs:

```text
/rko_lio/frame_xyzi for live local terrain
/modified_map for backend-corrected global terrain
TF map->odom->base_link
/rko_lio/registration_diagnostics
/rko_lio/runtime_diagnostics
```

Possible outputs:

```text
/traversability_grid  nav_msgs/msg/OccupancyGrid
/terrain_costmap      nav_msgs/msg/OccupancyGrid
grid_map_msgs/GridMap if multi-layer grid support is added
custom Nav2 costmap layer later
```

Terrain features per cell:

```text
height mean
height variance
max height
min height
height range
slope
roughness
step height
obstacle height
negative obstacle risk
unknown-space risk
confidence score
```

Cost formula concept:

```text
terrain_cost =
  weighted_slope_cost
  + weighted_roughness_cost
  + weighted_step_height_cost
  + obstacle_cost
  + unknown_risk_cost
  + confidence_penalty
```

Then clamp to Nav2-compatible cost values.

Suggested thresholds:

```text
max_safe_slope_deg
max_traversable_slope_deg
max_step_height_m
max_roughness_m
max_obstacle_height_m
unknown_cost
lethal_cost_threshold
min_points_per_cell
max_cell_age_sec
```

Interpretation:

```text
flat safe terrain       -> low cost
mild slope              -> medium-low cost
rough terrain           -> medium/high cost
steep slope             -> high or lethal cost
large rock              -> lethal obstacle
crater edge / drop-off  -> lethal or near-lethal cost
unknown region          -> mission-configurable high/lethal cost
```

## Outlier Handling

Outliers should be handled at multiple levels.

### SLAM Frontend Outliers

Use:

```text
/rko_lio/registration_diagnostics
/rko_lio/runtime_diagnostics
```

Reject or down-weight frames when:

```text
overlap is too low
residual is too high
Hessian conditioning is poor
sudden Z jump occurs
unrealistic velocity occurs
unrealistic yaw rate occurs
processing time is too high
dropped frames are detected
```

Actions:

```text
Do not insert bad frames into OGM/traversability maps.
Do not use bad frames for keyframe generation.
Slow navigation if frontend confidence is poor.
Trigger cautious mode if diagnostics stay poor.
```

### Backend Loop Closure Outliers

Loop closure should not be accepted from a descriptor alone.

Require:

```text
descriptor candidate
geometric verification
ICP/NDT/GICP fitness check
overlap check
correction size check
yaw correction check
vertical correction check
consistency with recent trajectory
robust kernel or switchable constraint if available
```

Reject if:

```text
fitness is too high
overlap is too low
translation correction is too large
yaw correction is too large
vertical correction is suspicious
terrain is repetitive and confidence is weak
candidate disagrees with odometry history
```

### Point Cloud / Terrain Mapping Outliers

Before costmap generation:

```text
voxel downsample
remove isolated points
remove impossible height spikes
filter points outside rover-relevant height band
use radius/statistical outlier removal
reject cells with too few points
track cell age
decay stale obstacles
preserve high-confidence persistent obstacles
classify unknown separately from free
```

### Nav2 Costmap Outliers

Use Nav2 tools for:

```text
denoise layer
obstacle clearing
costmap clearing recovery
inflation
keepout zones
speed zones
replanning
backup behavior
wait behavior
```

## Loop Closure Handling

Loop closure can shift `map->odom`. Local odometry should remain smooth, but the global plan may become stale.

Add a `loop_closure_monitor` node.

Monitor:

```text
map -> odom
```

Detect sudden correction:

```text
translation jump > configurable threshold, e.g. 0.25 m
yaw jump > configurable threshold, e.g. 5 deg
vertical jump > configurable threshold if relevant
```

When a major correction is detected:

```text
Pause or slow the rover.
Cancel or pause current Nav2 goal if necessary.
Clear global costmap.
Optionally clear local costmap for large corrections.
Read current corrected map->base_link pose.
Replan from corrected pose.
Resume once TF and costmaps are consistent.
```

Do not clear everything for tiny corrections. Small `map->odom` updates should be allowed.

Pseudo logic:

```text
previous_tf = map_to_odom

while running:
  current_tf = lookup map_to_odom
  delta = inverse(previous_tf) * current_tf

  if delta.translation_norm > threshold or delta.yaw > threshold:
      publish /slam_correction_event
      pause navigation
      clear global costmap
      optionally clear local costmap
      request replan
      resume navigation

  previous_tf = current_tf
```

## Navigation Modes

Create mode logic later:

```text
Normal:
  frontend confidence good
  terrain confidence good
  normal speed

Cautious:
  weak SLAM observability
  rough terrain
  unknown map nearby
  reduced speed
  increased inflation
  prefer known safe cells

Stop / Wait / Recover:
  frontend diagnostics bad
  loop closure correction large
  costmap inconsistent
  no safe path
  stop and replan
```

Inputs:

```text
SLAM diagnostics
traversability confidence
costmap state
Nav2 planner/controller status
```

Outputs:

```text
/navigation_mode
speed limit commands or Nav2 speed zone integration
recovery behavior triggers
```

## Implementation Phases

### Phase 1: Validate TF And Pose Sources

Tasks:

```text
Confirm map->odom is published by graph_based_slam.
Confirm odom->base_link is published by RKO-LIO.
Confirm tf2_echo map base_link works.
Confirm /rko_lio/odometry is smooth.
Confirm /modified_path changes after backend correction.
Confirm Nav2 can use map, odom, and base_link frames.
```

Acceptance tests:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
```

Expected:

```text
No TF tree breaks.
map->odom changes when backend corrects drift.
odom->base_link remains smooth.
```

### Phase 2: Bring Up Basic Nav2

Tasks:

```text
Add Nav2 Jazzy launch.
Configure robot_base_frame as base_link.
Use map for global costmap.
Use odom for local costmap.
Use corrected TF map->base_link for navigation.
Start with a simple static map or basic occupancy grid.
Use Smac Hybrid-A* and Regulated Pure Pursuit.
```

Acceptance tests:

```text
Nav2 launches.
Nav2 accepts a goal.
Global plan is generated.
Local controller follows the path in simulation.
Local obstacle avoidance works with live sensor data.
No major TF extrapolation errors.
```

### Phase 3: Build Basic OGM From SLAM Map

Tasks:

```text
Create mapper node. Initial C++ node exists as togo_navigation/slam_to_occupancy_grid.
Subscribe to /modified_map.
Transform points into map frame for global map.
Build nav_msgs/OccupancyGrid.
Publish /map with transient-local QoS.
Feed /map into Nav2 global costmap static layer.
```

Launch switches:

```bash
NAV2_USE_SLAM_MAP=true NAV2_DEBUG_MAP=false bash /scripts/run_nav2_with_slam.sh
NAV2_USE_SLAM_MAP=false NAV2_DEBUG_MAP=true bash /scripts/run_nav2_with_slam.sh
```

Acceptance tests:

```text
/map publishes correctly.
RViz displays occupancy grid aligned with SLAM map.
Nav2 global costmap uses /map.
Global planner avoids occupied cells.
Map remains aligned with map->base_link.
```

### Phase 4: Add Live Local Obstacle Avoidance

Tasks:

```text
Feed filtered live point cloud into local costmap.
Use Voxel Layer or STVL if available and stable in Jazzy.
Configure obstacle marking and clearing.
Tune obstacle height limits for rover.
Tune inflation radius for rover footprint.
```

Acceptance tests:

```text
Rocks/obstacles appear in local costmap.
Obstacles clear after they leave sensor view.
Rover avoids new obstacles not present in global map.
No excessive ghost obstacles.
Local costmap remains stable during small SLAM corrections.
```

### Phase 5: Add Loop-Closure Event Handling

Tasks:

```text
Write loop_closure_monitor node.
Monitor map->odom over time.
Detect sudden correction.
Publish /slam_correction_event.
Integrate with Nav2 behavior tree or lifecycle service calls.
Clear/reload/replan after large corrections.
```

Acceptance tests:

```text
Artificial map->odom jump triggers event.
Nav2 replans after correction.
Robot does not continue following stale path.
Local avoidance remains stable after loop closure.
Global plan updates to corrected pose.
```

### Phase 6: Add Traversability / Slope Mapping

Tasks:

```text
Build elevation grid from point cloud.
Compute slope, roughness, height range, and step height.
Compute confidence from point density and observation history.
Convert terrain features into cost values.
Publish traversability costmap.
```

Acceptance tests:

```text
Flat terrain is low cost.
Steep slopes are high cost.
Large rocks become lethal.
Crater/drop edges become high or lethal.
Unknown cells follow configured safety behavior.
Global planner chooses safer terrain even if longer.
```

### Phase 7: Add Cautious Navigation Modes

Tasks:

```text
Use SLAM diagnostics and terrain confidence to select mode.
Reduce speed when observability is weak.
Increase inflation or prefer known cells in caution mode.
Stop/recover when confidence is bad or no safe path exists.
```

Acceptance tests:

```text
Bad diagnostics reduce speed or pause.
Recovery behavior triggers when path is blocked.
Rover backs up if obstacle blocks the front.
Rover does not drive into unknown/high-risk terrain unless configured.
```

### Phase 8: Testing And Metrics

Repeatable simulation tests:

```text
Flat featureless terrain
Repeated rocks/craters
Large obstacle in path
Crater/drop-off
Loop closure correction during navigation
Ghost obstacle/noisy LiDAR
```

Metrics:

```text
path success rate
collision count
near-collision count
average planning time
controller loop rate
costmap update rate
map->odom correction magnitude
number of replans
number of recoveries
false obstacle persistence time
terrain classification accuracy when truth exists
```

## Proposed Package Structure

Future package:

```text
nav_stack/
  launch/
    rover_nav2.launch.py
    rover_navigation_integrated.launch.py

  config/
    nav2_params.yaml
    costmap_params.yaml
    planner_params.yaml
    controller_params.yaml
    behavior_tree.xml

  src/
    loop_closure_monitor.cpp
    slam_to_occupancy_grid.cpp
    traversability_mapper.cpp
    terrain_costmap_layer.cpp

  include/
    nav_stack/
      loop_closure_monitor.hpp
      slam_to_occupancy_grid.hpp
      traversability_mapper.hpp
      terrain_costmap_layer.hpp

  scripts/
    run_nav2_with_slam.sh
    test_loop_closure_event.sh
    save_nav_debug_bag.sh

  rviz/
    rover_nav_debug.rviz

  README.md
```

## Important Topics

Existing SLAM topics:

```text
/rko_lio/odometry
/rko_lio/frame
/rko_lio/frame_xyzi
/rko_lio/local_map
/rko_lio/path
/rko_lio/registration_diagnostics
/rko_lio/runtime_diagnostics
/modified_map
/modified_path
/tf
/tf_static
```

New navigation topics:

```text
/map
/traversability_grid
/terrain_costmap
/slam_correction_event
/navigation_mode
/terrain_diagnostics
/costmap_debug_points
```

Important services/actions:

```text
Nav2 NavigateToPose action
Clear global costmap service
Clear local costmap service
/map_save
optional pause/resume navigation behavior
```

## Design Principles

```text
Keep the SLAM frontend smooth.
Let backend correction happen through map->odom.
Use map->base_link for corrected global navigation.
Use local costmap in odom frame.
Use global costmap in map frame.
Treat loop closure as a replan event.
Do not trust raw point cloud as terrain truth without filtering.
Do not trust every SLAM frame equally.
Do not rely on stock Nav2 for slope awareness.
Add traversability as a custom layer.
Prefer upstream Nav2 Jazzy over unmaintained forks.
Make every threshold configurable.
Test with rosbag replay and repeatable Gazebo scenarios.
```

## Final Target Architecture

```text
Gazebo sensors
  -> RKO-LIO frontend
  -> smooth odom->base_link

RKO-LIO frame/cloud
  -> XYZI adapter
  -> graph_based_slam backend
  -> corrected map->odom
  -> /modified_map and /modified_path

/modified_map + live LiDAR
  -> OGM / elevation / traversability mapper
  -> /map and /traversability_grid

Nav2
  -> global planner in map frame
  -> local controller/costmap in odom frame
  -> obstacle avoidance
  -> recovery behaviors
  -> backup/replan/costmap clearing

Loop closure monitor
  -> detects map->odom jumps
  -> clears/reloads/replans
  -> prevents stale path following
```

The final system should allow the rover to drive using corrected SLAM pose, avoid local obstacles in real time, replan after loop closures, and eventually prefer safer lunar terrain based on slope, roughness, rocks, craters, and unknown-space risk.
