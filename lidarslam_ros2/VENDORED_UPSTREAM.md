# Vendored upstream information

This copy of lidarslam_ros2 is intentionally vendored into togo-husky-sim so local TOGO/Husky SLAM modifications can be checkpointed and pushed with the project.

## Upstream repositories

- lidarslam_ros2: https://github.com/rsasaki0109/lidarslam_ros2.git
  - vendored from commit: 4402e5af2033e1bf5d0bd9a31405a888b4cb22da
  - upstream branch at clone time: develop
- rko_lio: https://github.com/rsasaki0109/rko_lio.git
  - vendored from commit: 85d62f4176e7ea33761489af50889e71efe05353
- ndt_omp_ros2: https://github.com/rsasaki0109/ndt_omp_ros2
  - vendored from commit: 497411279593eb261a3e3d04cdcbb4717af33ca3

## Local changes at vendoring time

- Added/modified Seyond live SLAM launch integration.
- Modified RKO-LIO launch support for sim-time and custom topics.
- Modified SLAM parameters for the Husky/Seyond/Gazebo workflow.

The nested Git metadata was removed so the parent repository tracks these as normal files rather than as submodules or embedded Git repositories.