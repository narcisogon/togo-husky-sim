# TOGO SLAM and Navigation Overview

This workspace uses `lidarslam_ros2` as the home for the rover SLAM and navigation stack. The current stack is split into four layers:

1. RKO-LIO frontend odometry
2. graph_based_slam backend correction and map building
3. SLAM-to-occupancy-grid conversion for Nav2
4. Nav2 planning and control

The main idea is:

```text
LiDAR + IMU
  -> RKO-LIO frontend
  -> /rko_lio/odometry and /rko_lio/frame_xyzi
  -> graph_based_slam backend
  -> map->odom correction, /modified_map, /modified_path
  -> slam_to_occupancy_grid
  -> /map
  -> Nav2 global costmap, planner, controller
```

## What Runs What

Start SLAM:

```bash
bash /ws/src/lidarslam_ros2/scripts/togo/run_live_seyond_slam_integrated.sh
```

Start navigation after SLAM has transforms:

```bash
bash /ws/src/lidarslam_ros2/scripts/togo/run_nav2_with_slam.sh
```

The old Docker shortcuts still work:

```bash
bash /scripts/run_live_seyond_slam_integrated.sh
bash /scripts/run_nav2_with_slam.sh
```

Those `/scripts` files are now compatibility wrappers. The editable source lives in `lidarslam_ros2/scripts/togo`.

## Frontend

The frontend is RKO-LIO. It estimates the high-rate local rover motion from LiDAR and IMU.

Important outputs:

- `/rko_lio/odometry`: the main local odometry estimate
- `/rko_lio/frame`: frontend local cloud
- `/rko_lio/frame_xyzi`: same cloud with intensity added for backend compatibility
- `/rko_lio/runtime_diagnostics`: frontend runtime timing and buffer status
- `/rko_lio/registration_diagnostics`: registration health, correspondence counts, overlap/error metrics
- `/rko_lio/odometry_imu_predict`: optional high-rate short-horizon IMU prediction

Main files:

- `Thirdparty/rko_lio/rko_lio/ros/node.cpp`: ROS node, parameters, publishers, subscribers
- `Thirdparty/rko_lio/rko_lio/core/lio.cpp`: frontend registration and state update behavior
- `Thirdparty/rko_lio/rko_lio/core/lio.hpp`: frontend state/config declarations
- `lidarslam/param/seyond_live_slam.yaml`: frontend runtime parameters
- `lidarslam/launch/seyond_live_slam.launch.py`: launches RKO-LIO and helper nodes

Modify these when tuning frontend stability:

- `enable_imu_pose_prior`
- `imu_pose_prior_translation_weight`
- `imu_pose_prior_rotation_weight`
- `enable_stationary_hold`
- `enable_degeneracy_damping`
- point-to-plane or correspondence settings in RKO-LIO
- IMU prediction settings if you only need a smoother high-rate topic for control

## Backend

The backend is `graph_based_slam`. It receives frontend odometry and frontend clouds, stores keyframes/submaps, performs graph optimization, and publishes the global correction.

Important outputs:

- `map -> odom`: backend correction transform
- `/modified_map`: corrected backend point cloud map
- `/modified_path`: corrected backend path
- `/map_save`: service that forces backend map/projector/map output update

Main files:

- `graph_based_slam/src/graph_based_slam_component.cpp`: graph backend node, map updates, TF publishing, map save behavior
- `graph_based_slam/include/graph_based_slam/graph_based_slam_component.hpp`: backend declarations
- `lidarslam/param/seyond_live_slam.yaml`: backend parameters in the same YAML
- `lidarslam/launch/seyond_live_slam.launch.py`: remaps backend input to RKO-LIO topics

The backend does not replace the frontend as the high-rate pose source. Instead:

- RKO-LIO publishes fast local motion as `odom -> base_link`.
- graph_based_slam publishes slower global correction as `map -> odom`.
- Nav2 asks TF for `map -> base_link`, which combines both.

This gives Nav2 a pose that moves at frontend rate while still receiving backend loop-closure corrections.

For navigation, the backend can also refresh `/modified_map` periodically with `modified_map_publish_period_sec`. The TOGO default is `1.0`, so the global OGM can keep receiving backend-corrected map updates without waiting for a manual `/map_save` call. Loop closures still correct the backend graph; the next `/modified_map` refresh carries that correction into `/map`.

The published `/modified_map` is voxelized with `modified_map_leaf_size` before it is sent to Nav2. This reduces overlapping points as the backend map grows while keeping the backend's internal submaps unchanged for optimization and loop closure. The TOGO default is `0.15 m`, which is close to the current OGM resolution and should keep hazard geometry without flooding Nav2 with duplicate points.

## Occupancy Grid

Nav2 needs an occupancy grid. The backend map is a point cloud, so `togo_navigation` converts the backend cloud into `/map`.

Important outputs:

- `/map`: `nav_msgs/OccupancyGrid` used by Nav2 static layer
- `/local_hazard_map`: rolling local hazard grid built from live `/rko_lio/frame_xyzi`
- `/map_debug_points`: point cloud visualization of occupied cells in `/map`
- `/local_hazard_debug_points`: point cloud visualization of occupied cells in `/local_hazard_map`
- `/global_costmap/debug_points`: point cloud visualization of occupied cells in Nav2 global costmap

Main files:

- `togo_navigation/src/slam_to_occupancy_grid.cpp`: converts `/modified_map` to `/map`
- `togo_navigation/src/local_hazard_grid.cpp`: converts recent live frontend clouds to `/local_hazard_map`
- `togo_navigation/src/occupancy_grid_to_points.cpp`: converts occupancy grids to point clouds for RViz debugging
- `togo_navigation/launch/rover_nav2.launch.py`: starts the mapper and debug point publishers

Current mapper source:

```text
/modified_map -> slam_to_occupancy_grid -> /map
/rko_lio/frame_xyzi -> local_hazard_grid -> /local_hazard_map
```

That means the global OGM is built from the backend-corrected map, while local hazards are built from recent live frontend clouds. The global map stays corrected by loop closures; the local map reacts quickly and naturally clears old hazards as its short time window rolls forward.

For current TOGO navigation, aggressive slope/step terrain hazards are intentionally kept out of the global `/map`. The global map is used as corrected structure, and `/local_hazard_map` handles immediate slope/step risk. This prevents a loop-closure shift in the global backend map from creating large inflated hazard blobs near the rover.

Nav2 consumes local hazards through `/local_hazard_debug_points` as an obstacle layer, not as a static local map. That avoids repeated local costmap resize events from the rolling `/local_hazard_map` origin.

Useful mapper parameters:

- `resolution`: grid cell size
- `width_m` and `height_m`: total grid size
- `center_map_on_robot`: keeps the grid centered around the rover
- `min_obstacle_height`: ignores ground/low points
- `max_obstacle_height`: ignores high points
- `clear_robot_radius_m`: clears cells around the robot body
- `obstacle_dilation_cells`: expands occupied cells before Nav2 inflation
- `enable_gradient_costs`: publishes soft costs around obstacle cells instead of only hard occupied/free values
- `gradient_radius_m`: radius around obstacle cores that receives nonzero cost
- `gradient_min_cost`: lowest nonzero soft cost at the outside of the gradient
- `gradient_power`: controls how quickly cost falls away from obstacles
- `enable_terrain_hazards`: marks terrain cells as no-go hazards from slope/step checks
- `terrain_slope_hazard_deg`: no-go slope threshold; current TOGO default is `22.0`
- `terrain_step_hazard_m`: no-go local height spread threshold inside one grid cell
- `terrain_min_points_per_cell`: minimum terrain points before a cell can be judged
- `terrain_neighbor_radius_cells`: neighbor radius used to estimate slope from elevation differences

When gradient costs are enabled, hard obstacle cores still publish as `100`. Nearby cells publish values between `gradient_min_cost` and `100`, so Nav2 can prefer wider/cleaner paths instead of treating every nonzero cell as equally blocked. The Nav2 static layer uses `trinary_costmap: false` so these costs are preserved.

When terrain hazards are enabled, the mapper separately keeps low/ground terrain points, estimates each cell's mean elevation, and marks cells as no-go if the local slope exceeds `terrain_slope_hazard_deg` or if the within-cell height spread exceeds `terrain_step_hazard_m`. These hazard cells become hard occupied cells, then the normal gradient cost spreading runs around them.

## Navigation

Nav2 uses:

- `/map` for the global costmap
- `map -> odom -> base_link` for the robot pose
- `/rko_lio/odometry` for controller odometry feedback
- `/a300_0000/platform/cmd_vel` for rover commands

Main files:

- `togo_navigation/config/nav2_slam_params.yaml`: planner, controller, costmap, smoother, velocity limits
- `togo_navigation/launch/rover_nav2.launch.py`: minimal Nav2 bringup
- `togo_navigation/rviz/rover_nav_debug.rviz`: RViz config for navigation
- `scripts/togo/run_nav2_with_slam.sh`: waits for TF and launches Nav2

For now, the global planner is `SmacPlanner2D`. It is simpler and more forgiving while the OGM is still being developed. A rover-shaped planner such as `SmacPlannerHybrid` can come back once the map and footprint behavior are solid.

## What To Edit

Frontend path is jittery or slow:

- `Thirdparty/rko_lio/rko_lio/core/lio.cpp`
- `Thirdparty/rko_lio/rko_lio/ros/node.cpp`
- `lidarslam/param/seyond_live_slam.yaml`

Backend map/path is not corrected or not publishing:

- `graph_based_slam/src/graph_based_slam_component.cpp`
- `lidarslam/param/seyond_live_slam.yaml`
- check `/modified_map`, `/modified_path`, and `map -> odom`

Nav2 sees no map or bad map:

- `togo_navigation/src/slam_to_occupancy_grid.cpp`
- `togo_navigation/config/nav2_slam_params.yaml`
- check `/map`, `/map_debug_points`, `/global_costmap/costmap`

RViz map display looks blank or broken:

- Add a PointCloud2 display for `/map_debug_points`
- Add a PointCloud2 display for `/global_costmap/debug_points`
- Add a PointCloud2 display for `/modified_map`

## Quick Checks

SLAM frontend:

```bash
ros2 topic hz /rko_lio/odometry
ros2 topic echo /rko_lio/registration_diagnostics
ros2 run tf2_ros tf2_echo odom base_link
```

Backend correction:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /modified_path --once
ros2 topic echo /modified_map --once
```

Navigation map:

```bash
ros2 topic echo /map --once --field info --qos-reliability reliable --qos-durability transient_local
ros2 topic echo /global_costmap/costmap --once --field info --qos-reliability reliable --qos-durability transient_local
ros2 topic echo /plan --once
```

## Current Design Choices

- Keep RKO-LIO as the main high-rate frontend pose.
- Keep graph_based_slam as the global backend correction source.
- Use backend-corrected `/modified_map` to publish `/map`.
- Use debug point clouds for RViz map visualization because the RViz Map display can fail with a GLSL texture error in this WSL setup.
- Keep Nav2 bringup minimal: planner, controller, smoother, behavior server, BT navigator, waypoint follower, velocity smoother.
