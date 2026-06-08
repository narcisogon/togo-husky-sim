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
