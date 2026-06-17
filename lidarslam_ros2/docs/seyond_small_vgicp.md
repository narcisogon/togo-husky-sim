# Seyond SMALL_VGICP Frontend

`SMALL_VGICP` is implemented inside `scanmatcher_node` only when the
`scanmatcher` package is built with `small_gicp` available to CMake.

If launch prints:

```text
invalid registration method: SMALL_VGICP
```

then `find_package(small_gicp QUIET)` failed when `scanmatcher` was built, so
the `HAS_SMALL_GICP` compile branch was not included.

Check the installed build:

```bash
ros2 pkg executables scanmatcher
```

If `small_gicp_odom_node` is missing from that list, install or build
`small_gicp`, then rebuild:

```bash
cd /ws
colcon build --packages-select scanmatcher
source /ws/install/setup.bash
```

The live test launcher defaults to `SMALL_VGICP`:

```bash
bash /scripts/run_live_seyond_small_vgicp_slam.sh
```

While `small_gicp` support is unavailable, use the same launch with a built-in
fallback method:

```bash
REGISTRATION_METHOD=GICP bash /scripts/run_live_seyond_small_vgicp_slam.sh
REGISTRATION_METHOD=NDT bash /scripts/run_live_seyond_small_vgicp_slam.sh
```
