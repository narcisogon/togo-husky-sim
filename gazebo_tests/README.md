# Gazebo Rendered Sensor Docker Tests

These worlds isolate rendered sensors from the Husky/Clearpath stack.

## GPU LiDAR Only

Inside the sim Docker container:

```bash
source /opt/ros/jazzy/setup.bash
gz sim -r -v 4 /husky/gazebo_tests/seyond_robin_w_gpu_lidar.sdf
```

In another shell in the same container:

```bash
gz topic -l | grep seyond
gz topic -e -t /seyond_robin_w/scan/points -n 1
```

Bridge to ROS:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /seyond_robin_w/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked
```

Then:

```bash
ros2 topic hz /seyond_robin_w/scan/points
```

The simulated Seyond Robin W field of view is approximated as 120 deg horizontal x 70 deg vertical:

```text
horizontal: -1.0472 to +1.0472 rad
vertical:   -0.6109 to +0.6109 rad
```

## Camera Only

```bash
gz sim -r -v 4 /husky/gazebo_tests/camera_render_test.sdf
```

Check Gazebo topics:

```bash
gz topic -l | grep camera
gz topic -e -t /camera_0/image -n 1
```

## GPU vs Software Render Test

GPU path:

```bash
glxinfo -B | grep -E "renderer|Accelerated|Device"
gz sim -r -v 4 /husky/gazebo_tests/seyond_robin_w_gpu_lidar.sdf
```

Software path:

```bash
LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe gz sim -r -v 4 /husky/gazebo_tests/seyond_robin_w_gpu_lidar.sdf
```

If software works but GPU fails, the issue is in the Gazebo/Ogre2/driver/rendered sensor path, not the sensor SDF shape.
