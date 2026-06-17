# EllipseLIO test workspace

Standalone test of [EllipseLIO](https://github.com/v4rl-ucy/ellipselio) — "Adaptive
LiDAR Inertial Odometry with an Ellipsoid Representation" — kept fully separate from
`~/fastlio_ws`. Nothing here touches the existing Fast-LIO2 / MOLA setup.

## Status
- Cloned, dependencies installed (rosdep), **builds clean** on ROS 2 Humble.
- One local patch: `CMakeLists.txt` guarded `USE_SCOPED_HEADER_INSTALL_DIR`
  (a Jazzy-only `ament_auto_package` option) behind `if(ROS_JAZZY)`, so it builds
  on Humble. Upstream targets Humble + Jazzy but the option breaks Humble.

## Build
```bash
cd ~/ellipselio_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ellipselio --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Run (live)
EllipseLIO subscribes to the same topics the robot already publishes — `/livox/lidar`
(PointCloud2) and `/livox/imu` (Imu). It does NOT need its own driver, just the
existing TCP bridges running.

1. Bring up the Livox bridges (any of your existing scripts that publish the /livox
   topics works), e.g. in one terminal:
   ```bash
   bash ~/hexplorer/scripts/start_fastlio.sh
   ```
2. In another terminal:
   ```bash
   bash ~/ellipselio_ws/start_ellipselio.sh          # no RViz
   bash ~/ellipselio_ws/start_ellipselio.sh --rviz   # with RViz
   ```

## Config
`config/mid360.yaml` (ships upstream) already matches this robot:
- lidar topic `/livox/lidar`, type LIVOX, 10 Hz, 40 lines
- imu topic `/livox/imu`, 200 Hz
- IMU→LiDAR extrinsics for the Mid360

## Output topics
| Topic | Type | Notes |
|-------|------|-------|
| `/ellipselio_odom` | nav_msgs/Odometry | pose estimate |
| `/cloud_map` | sensor_msgs/PointCloud2 | accumulated map |
| `/cloud_scan` | sensor_msgs/PointCloud2 | registered scan |
| `/visualization_marker` | MarkerArray | ellipsoids |
| `/analytics` | ellipselio/EllipseLioAnalytics | timing/odom freq |

Output frame: `odom_ellipselio` → `imu_prop_ellipselio`.

## Known caveat (live data)
EllipseLIO's LIVOX point format expects per-point fields
`x,y,z,intensity,tag,line,timestamp`. The robot's `livox_tcp_receiver.py` bridge
publishes only `x,y,z,intensity` (16-byte points). So:
- XYZ + intensity populate fine — the node runs.
- Per-point `timestamp` is zero → **intra-scan motion deskewing is disabled**.
- Fine for slow walking (~0.15 m/s); expect degraded accuracy during fast turns.

To get full deskewing later, the Jetson-side Livox publisher would need to emit the
per-point timestamp (and tag/line) fields.
