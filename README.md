# Husky / TOGO Simulation Setup

This folder contains a starter Clearpath Husky simulation setup for learning the TOGO architecture.

Current folder from Windows:

```text
C:\Users\Username\OneDrive\Desktop\husky
```

Current folder from WSL:

```bash
/mnt/c/Users/Username/OneDrive/Desktop/husky
```

## What This Is

This is not the real TOGO robot configuration yet. It is a starter Husky/A300 simulation using Clearpath's ROS 2 Jazzy simulator.

The goal is to get a basic Husky-style rover running in Gazebo/GZ so we can understand:

```text
robot.yaml -> generated robot description -> Gazebo sim -> ROS topics -> teleop/Nav2
```

## Installed Packages

The Clearpath simulator was installed in WSL with:

```bash
sudo apt-get install ros-jazzy-clearpath-simulator
```

This installed:

```text
ros-jazzy-clearpath-simulator
ros-jazzy-clearpath-gz
ros-jazzy-clearpath-generator-gz
```

## Files In This Folder

```text
robot.yaml        Clearpath robot configuration for the simulated Husky/A300
wasd_teleop.py    Simple WASD teleop node that publishes TwistStamped commands
README.md         This setup guide
```

## robot.yaml

The current `robot.yaml` describes a basic Clearpath A300-style robot with:

```text
namespace: /a300_0000
platform controller: ps4
IMU: phidgets_spatial
2D LiDAR: hokuyo_ust
```

The namespace matters. Most robot topics will be under:

```text
/a300_0000/...
```

The drive command topic is expected to be:

```text
/a300_0000/cmd_vel
```

## Source ROS

In every new WSL terminal where you use ROS commands, run:

```bash
source /opt/ros/jazzy/setup.bash
```

## Launch The Husky Sim

From WSL:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch clearpath_gz simulation.launch.py \
  setup_path:=/mnt/c/Users/Username/OneDrive/Desktop/husky \
  world:=warehouse \
  rviz:=true
```

We use `world:=warehouse` because Clearpath's launch file only accepts these world names:

```text
construction
office
orchard
pipeline
solar_farm
warehouse
```

## Lunar Terrain Workaround

We created a custom `lunar.sdf`, but Clearpath's launch file rejected `world:=lunar` because the allowed world names are hardcoded.

The workaround is to replace the installed `warehouse.sdf` with the lunar SDF and still launch using:

```bash
world:=warehouse
```

Find the installed Clearpath worlds directory:

```bash
source /opt/ros/jazzy/setup.bash
WORLD_DIR=$(ros2 pkg prefix clearpath_gz)/share/clearpath_gz/worlds
echo $WORLD_DIR
ls $WORLD_DIR
```

Back up the original warehouse world:

```bash
cp "$WORLD_DIR/warehouse.sdf" "$WORLD_DIR/warehouse.sdf.bak"
```

Copy the lunar world over warehouse:

```bash
cp "$WORLD_DIR/lunar.sdf" "$WORLD_DIR/warehouse.sdf"
```

Restore the original warehouse later:

```bash
cp "$WORLD_DIR/warehouse.sdf.bak" "$WORLD_DIR/warehouse.sdf"
```

Longer term, the better fix is to create a custom launch file or patch the Clearpath launch file so `lunar` is an allowed world.

## Inspect The Running Sim

Open a second WSL terminal while the sim is running:

```bash
source /opt/ros/jazzy/setup.bash
ros2 node list
ros2 topic list
ros2 topic list | grep a300
ros2 topic list | grep cmd_vel
```

Useful checks:

```bash
ros2 topic info /a300_0000/cmd_vel
ros2 topic echo /a300_0000/odom
ros2 topic echo /tf
```

## Drive With WASD

Run the WASD teleop script in a second WSL terminal:

```bash
cd /mnt/c/Users/Username/OneDrive/Desktop/husky
source /opt/ros/jazzy/setup.bash
python3 wasd_teleop.py
```

Controls:

```text
w / s      forward / backward
a / d      rotate left / rotate right
space      stop
+ / -      increase / decrease linear speed
] / [      increase / decrease angular speed
q          quit
```

By default, the script publishes `geometry_msgs/msg/TwistStamped` to:

```text
/a300_0000/cmd_vel
```

If the command topic is different, find it:

```bash
ros2 topic list | grep cmd_vel
```

Then run:

```bash
python3 wasd_teleop.py --topic /your_cmd_vel_topic
```

## Manual Drive Command

If you want to test without the WASD script:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub /a300_0000/cmd_vel geometry_msgs/msg/TwistStamped \
'{"twist":{"linear":{"x":0.2}}}' -r 10
```

Rotate:

```bash
ros2 topic pub /a300_0000/cmd_vel geometry_msgs/msg/TwistStamped \
'{"twist":{"angular":{"z":0.35}}}' -r 10
```

Stop the command with `Ctrl+C`.

## PS4 Controller Notes

The `robot.yaml` currently says:

```yaml
platform:
  controller: ps4
```

Clearpath supports PS4-style joystick teleop, but WSL does not automatically see a Bluetooth PS4 controller as `/dev/input/js0`.

For joystick testing in WSL, use USB passthrough with `usbipd-win`:

```powershell
usbipd list
usbipd bind --busid YOUR-BUSID
usbipd attach --wsl --busid YOUR-BUSID
```

Then check in WSL:

```bash
ls /dev/input
```

You want to see:

```text
js0
```

Until that exists, use the WASD teleop script.

## How This Connects To TOGO

This starter sim is useful because TOGO is expected to be a Clearpath Husky-based outdoor GN&C rover.

The real TOGO setup will likely need:

```text
TOGO-specific robot.yaml
actual Husky model/version, probably A200 or A300
actual sensor list and mount frames
LiDAR config
IMU config
GPS/RTK config
camera/event camera config
radar config, if used
Nav2 params
localization config, likely robot_localization for GPS + IMU + wheel odom
```

The important architecture is:

```text
robot.yaml
  -> robot description and sensor frames
  -> Gazebo/GZ simulation
  -> /tf, /odom, /scan, /imu
  -> /cmd_vel
  -> Nav2 later
```

## Next Steps

1. Confirm the sim launches reliably.
2. Confirm `/a300_0000/cmd_vel` moves the robot.
3. Confirm `/a300_0000/odom`, `/tf`, and LiDAR topics exist.
4. Add or launch Nav2.
5. Replace the generic A300 config with the real TOGO config when available.
6. Improve lunar terrain using a real heightmap/DEM instead of the simple SDF workaround.

## LiDAR SLAM Docker Setup

The SLAM stack is built in Docker using ROS 2 Jazzy. The Docker image builds the `lidarslam_ros2` workspace and provides these packages:

```text
lidarslam
rko_lio
graph_based_slam
ndt_omp_ros2
lidarslam_msgs
```

Important files:

```text
Dockerfile.lidarslam              Builds the Jazzy SLAM image
docker-compose.lidarslam.yml      Runs the SLAM container with host networking
docker/lidarslam_entrypoint.sh    Sources ROS and the built SLAM workspace
lidarslam_ros2/                   Third-party SLAM repo, not committed if using .gitignore
bags/                             Optional rosbag input/output folder
slam_output/                      Optional map / SLAM output folder
```

### Clone The SLAM Repo

If `lidarslam_ros2/` is not present, clone it from the Husky folder:

```bash
cd /mnt/c/Users/Username/OneDrive/Desktop/husky
git clone --recursive https://github.com/rsasaki0109/lidarslam_ros2.git
```

### Build The Docker Image

```bash
cd /mnt/c/Users/Username/OneDrive/Desktop/husky
docker compose -f docker-compose.lidarslam.yml build
```

This creates the image:

```text
togo-lidarslam:jazzy
```

### Enter The SLAM Container

```bash
cd /mnt/c/Users/Username/OneDrive/Desktop/husky
docker compose -f docker-compose.lidarslam.yml run --rm lidarslam
```

Inside the container, verify the packages:

```bash
ros2 pkg list | grep -E "lidarslam|rko|ndt|graph"
```

Expected packages include:

```text
graph_based_slam
lidarslam
lidarslam_msgs
ndt_omp_ros2
rko_lio
```

### ROS Network Settings

The compose file uses host networking and defaults to:

```text
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

If your sim or robot uses a different domain, override it when starting the container:

```bash
ROS_DOMAIN_ID=7 docker compose -f docker-compose.lidarslam.yml run --rm lidarslam
```

Inside Docker, confirm it can see the sim topics:

```bash
ros2 topic list | grep -E "lidar3d|points|imu|scan"
```

### Run Live RKO-LIO

The online RKO-LIO launch consumes a live `PointCloud2` and `Imu` stream:

```bash
ros2 launch rko_lio odometry.launch.py \
  mode:=online \
  lidar_topic:=/a300_0000/sensors/lidar3d_0/points \
  imu_topic:=/a300_0000/sensors/imu_0/data \
  base_frame:=base_link \
  odom_frame:=odom \
  imu_frame:=imu_0_link \
  lidar_frame:=lidar3d_0_sensor_link \
  deskew:=false \
  publish_deskewed_scan:=true \
  publish_local_map:=true \
  rviz:=true
```

Note: the Clearpath sim currently publishes the 3D LiDAR correctly, but the simulated IMU is still being sorted out. For raw LiDAR testing, verify:

```bash
ros2 topic hz /a300_0000/sensors/lidar3d_0/points
ros2 topic echo /a300_0000/sensors/lidar3d_0/points --once | head -40
```

### Record A Bag From Live Sim

Recording a bag is separate from SLAM. RKO-LIO does not automatically create bags.

```bash
cd /mnt/c/Users/Username/OneDrive/Desktop/husky
mkdir -p bags
source /opt/ros/jazzy/setup.bash

ros2 bag record \
  /a300_0000/sensors/lidar3d_0/points \
  /a300_0000/sensors/lidar3d_0/scan \
  /a300_0000/sensors/lidar2d_0/scan \
  /a300_0000/platform/odom \
  /tf \
  /tf_static \
  /clock \
  -o bags/husky_lidar_test
```

Drive for 30-60 seconds, press `Ctrl+C`, then inspect:

```bash
ros2 bag info bags/husky_lidar_test
```

### Transfer To Another Computer

On another machine, install WSL 2 and Docker, clone/copy this repo, clone `lidarslam_ros2/` if it is not included, then run:

```bash
cd /mnt/c/Users/Username/OneDrive/Desktop/husky
docker compose -f docker-compose.lidarslam.yml build
docker compose -f docker-compose.lidarslam.yml run --rm lidarslam
```

If using live robot/sim topics, make sure `ROS_DOMAIN_ID`, middleware, and networking match the robot/sim.

## Live SLAM Support Helper

For RKO-LIO with the Clearpath sim, keep this helper running in a WSL terminal before launching SLAM:

```bash
cd /mnt/c/Users/Username/OneDrive/Desktop/husky
bash start_slam_support.sh
```

It starts:

```text
Gazebo IMU -> ROS Imu bridge
/a300_0000/tf -> /tf relay
base_link -> imu_0_link static TF
base_link -> lidar3d_0_sensor_link static TF
```

The WASD teleop script publishes to the Clearpath platform command topic:

```bash
python3 wasd_teleop.py
```

Default command topic:

```text
/a300_0000/platform/cmd_vel
```

## Seyond-Only Simulation Profile

The active sim profile is intentionally simplified to:

```text
Clearpath A300/Husky rover
Seyond Robin W-style GPU LiDAR, 120 deg x 70 deg FOV
Co-located simulated IMU
```

Active sensor topics:

```text
/a300_0000/sensors/seyond_robin_w/scan/points
/a300_0000/sensors/seyond_robin_w/imu
/a300_0000/sensors/seyond_robin_w/scan
```

Active sensor frames:

```text
seyond_robin_w_link
seyond_robin_w_lidar_frame
seyond_robin_w_imu_frame
```

After editing `robot.yaml` or the custom URDF/launch files, regenerate Clearpath files:

```bash
cd /mnt/c/Users/Username/OneDrive/Desktop/husky
source /opt/ros/jazzy/setup.bash
ros2 run clearpath_generator_common generate_description -s /mnt/c/Users/Username/OneDrive/Desktop/husky
ros2 run clearpath_generator_gz generate_launch -s /mnt/c/Users/Username/OneDrive/Desktop/husky
ros2 run clearpath_generator_gz generate_param -s /mnt/c/Users/Username/OneDrive/Desktop/husky
```

Run order:

```bash
# Terminal 1: sim
cd /mnt/c/Users/Username/OneDrive/Desktop/husky
source /opt/ros/jazzy/setup.bash
source source_togo_custom.sh
ros2 launch clearpath_gz simulation.launch.py setup_path:=/mnt/c/Users/Username/OneDrive/Desktop/husky world:=construction rviz:=true

# Terminal 2: TF support for Docker/RKO
cd /mnt/c/Users/Username/OneDrive/Desktop/husky
bash start_slam_support.sh

# Terminal 3: SLAM Docker
cd /mnt/c/Users/Username/OneDrive/Desktop/husky
ROS_DOMAIN_ID=0 docker compose -f docker-compose.lidarslam.yml run --rm lidarslam

# Inside Docker: RKO-LIO
bash /scripts/run_rko_lio_seyond.sh

# Optional, another Docker terminal: graph SLAM
bash /scripts/run_graph_slam_seyond.sh
```

Quick validation:

```bash
timeout 6 ros2 topic hz /a300_0000/sensors/seyond_robin_w/scan/points
timeout 6 ros2 topic hz /a300_0000/sensors/seyond_robin_w/imu
timeout 5 ros2 run tf2_ros tf2_echo base_link seyond_robin_w_lidar_frame
```
