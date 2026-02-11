# ROS2 Topics & MOLA SLAM Reference

## MOLA SLAM & Odometry

### What is MOLA?

MOLA (Modular Object Localization and Mapping Architecture) provides **LiDAR-only odometry** for this robot. It uses **ICP (Iterative Closest Point)** scan matching via the `lidar3d-katana` pipeline with `Matcher_Points_DistanceThreshold`. No IMU is used because the Livox Mid360's built-in IMU has significant drift.

### The Pipeline

```
Livox Mid360 LiDAR (on Jetson)
        |
        v
livox_lidar_node + livox_tcp_bridge.py (persistent systemd service on Jetson)
        |
        v  TCP:9998
livox_tcp_receiver.py (Mini PC)
        |
        v
  /livox/lidar  (raw PointCloud2, ~15k pts, ~10 Hz)
        |
        v
  filterpass.py (removes noise: intensity + angle filter)
        |
        v
  /livox/lidar_filtered  (~5k pts, ~10 Hz, QoS: Best Effort)
        |
        v
  MOLA LiDAR Odometry (mola-cli, lidar3d-katana pipeline)
    - Decimates to ~5000 points
    - Runs ICP scan matching (50 iterations)
    - Adaptive threshold (0.3-0.8m matching distance)
    - Robust outlier rejection (Geman-McClure kernel)
    - Keyframe management (every 0.15m or 3 deg)
    - Map density control (5 pts/voxel, 5cm min spacing)
        |
        +------> /lidar_odometry/pose (Odometry)
        +------> /tf (map->odom->base_link)
        +------> /lidar_odometry/localmap_points (accumulated map)
```

State estimator is disabled (`use_state_estimator:=False`) to avoid GTSAM crashes. Pose is published directly by `StateEstimationSimple` on `/lidar_odometry/pose`.

### SLAM vs Odometry

MOLA-LO on this robot is technically **odometry**, not full SLAM:
- It estimates relative motion between scans (odometry)
- It builds a local map from keyframes
- It does **NOT** do loop closure (recognizing previously visited places)
- Over long distances, drift will accumulate

### Quick Start

```bash
bash ~/hexplorer/scripts/start_mola_slam.sh              # Default
bash ~/hexplorer/scripts/start_mola_slam.sh --gui         # With MOLA GUI
bash ~/hexplorer/scripts/start_mola_slam.sh --no-rviz     # No RViz
```

---

## Complete ROS2 Topic Reference

### Sensor Bridge Topics

| Topic | Type | Source | Rate | Description |
|-------|------|--------|------|-------------|
| `/livox/lidar` | PointCloud2 | livox_tcp_receiver.py | ~10 Hz | Raw LiDAR from Jetson (frame: `livox_frame`, ~15k pts) |
| `/livox/lidar_filtered` | PointCloud2 | filterpass.py | ~10 Hz | Intensity+angle filtered LiDAR (~5k pts) |
| `/camera/color/image_raw` | Image | depth_bridge_receiver.py | ~6 Hz | 640x480 BGR8 color (only in --no-track mode) |
| `/camera/depth/image_raw` | Image | depth_bridge_receiver.py | ~6 Hz | 640x480 16UC1 depth in mm (only in --no-track mode) |
| `/camera/points` | PointCloud2 | depth_bridge_receiver.py | ~2-3 Hz | XYZRGB pointcloud (~180k pts, only in --no-track mode) |

### MOLA Odometry Topics

| Topic | Type | Source | Rate | Description |
|-------|------|--------|------|-------------|
| `/lidar_odometry/pose` | Odometry | MOLA (StateEstimationSimple) | ~20-30 Hz | **Primary pose output** - position+orientation |
| `/lidar_odometry/localmap_points` | PointCloud2 | MOLA mapping | ~1-2 Hz | Accumulated keyframe map for visualization |
| `/tf` | TFMessage | MOLA + static publishers | ~20-30 Hz | Transform tree: `map->odom->base_link->livox_frame` |

### Object Tracking Topics

| Topic | Type | Source | Rate | Description |
|-------|------|--------|------|-------------|
| `/object_detection` | String | detection_receiver.py | ~20-30 Hz | JSON: `{detected, center_x, center_y, distance_mm, confidence, label}` |
| `/object_position` | Point | detection_receiver.py | ~20-30 Hz | x=pixel_x, y=pixel_y, z=distance_mm |
| `/object_tracking/marker` | Marker | tracking_rviz_visualizer.py | ~10 Hz | 3D sphere at detected object location |
| `/object_tracking/text` | Marker | tracking_rviz_visualizer.py | ~10 Hz | Distance label text |
| `/object_tracking/markers` | MarkerArray | tracking_rviz_visualizer.py | ~10 Hz | Bounding box visualization |
| `/object_tracking/image` | Image | tracking_rviz_visualizer.py | ~10 Hz | Camera image with detection overlay |
| `/object_tracking/state_text` | Marker | tracking_rviz_visualizer.py | ~1 Hz | Current robot state display |
| `/smart_follower/state` | String | smart_follower.py | ~20 Hz | JSON state info (FOLLOWING, SEARCH, BLOCKED, etc.) |

### Robot Control Topics

| Topic | Type | Source | Rate | Description |
|-------|------|--------|------|-------------|
| `/robot_cmd` | RobotCommand | follower/nav scripts | 20 Hz | State machine commands (0-4) |
| `/vel_cmd` | Twist | follower/nav scripts | 20 Hz | Walking velocity (linear.x, angular.z) |
| `/robot_state` | RobotState | robot firmware | ~200 Hz | Joint positions, velocities, torques, body pose |
| `/joy` | Joy | gamepad driver | ~50 Hz | Gamepad input |

---

## TF Tree

```
map
  odom                              (MOLA puts motion estimate here)
    base_link
      livox_frame                   (0.2m above base_link)
      camera_depth_optical_frame    (rotated: qx=-0.5, qy=0.5, qz=-0.5, qw=0.5)
```

---

## Camera Sharing Limitation

The RealSense can only be used by **one process at a time**:

| Mode | Camera User | Available Topics | Not Available |
|------|-------------|------------------|---------------|
| Tracking (`start_sensor_demo.sh`) | jetson_object_tracker.py | tracking topics, LiDAR | depth, pointcloud |
| No-track (`start_sensor_demo.sh --no-track`) | realsense_depth_tcp_publisher.py | depth, pointcloud, LiDAR | tracking topics |

LiDAR is **always available** regardless of mode.

---

## Network Architecture

```
Mini PC (192.168.1.10)              Jetson Orin Nano (192.168.1.20)
========================            ================================

depth_bridge_receiver.py  <--TCP:9999--  realsense_depth_tcp_publisher.py
livox_tcp_receiver.py     <--TCP:9998--  livox_tcp_bridge.py
detection_receiver.py     <--TCP:9997--  jetson_object_tracker.py
(image stream)            <--TCP:9996--  (from tracker)
```

TCP bridges are used because ROS2 DDS (both FastDDS and CycloneDDS) fails for large messages and custom message types across machines.

---

## Filterpass Node

**File:** `~/MOLA-SLAM/mola_ws/install/mola_bringup/lib/mola_bringup/filterpass.py`

Preprocesses raw LiDAR to improve ICP matching:
- **Intensity filter:** Removes points with intensity outside 1.0-255.0 (spurious returns)
- **Angle filter:** Keeps points within a configurable sector (default 360 deg)
- Subscribes to `/livox/lidar`, publishes `/livox/lidar_filtered`
- Uses BEST_EFFORT QoS to match MOLA expectations

Typical reduction: ~15,000 raw points -> ~5,000 filtered points.

---

## MOLA ICP Tuning Summary

**Config file:**
```
~/MOLA-SLAM/mola_ws/install/mola_lidar_odometry/share/mola_lidar_odometry/pipelines/lidar3d-katana.yaml
```

| Parameter | Current Value | Effect |
|-----------|---------------|--------|
| `min_icp_goodness` | 0.92 | Min quality to accept a match (0.80=permissive, 0.95=strict) |
| `maximum_sigma` | 0.8 | Max matching distance in meters (lower=less drift) |
| `maxIterations` | 50 | ICP iterations per scan (30=fast, 80=accurate) |
| `robustKernelParam` | 4.0 | Outlier rejection (3.0=strict, 6.0=permissive) |
| `min_translation_between_keyframes` | 0.15m | How often to add keyframes |
| `min_rotation_between_keyframes` | 3.0 deg | Rotation threshold for new keyframe |
| `max_points_per_voxel` | 5 | Limits density in revisited areas |
| `min_distance_between_points` | 0.05m | 5cm minimum point spacing |

Full tuning guide: `~/hexplorer/docs/MOLA_SLAM_TUNING.md`

---

## Map Operations

```bash
# Save map during operation
ros2 service call /map_save mola_msgs/srv/MapSave \
  "map_path: '/home/robot/mola_maps/my_map'"

# Auto-saved on shutdown:
#   final_map.simplemap
#   estimated_trajectory.tum

# Localize in existing map
ros2 launch mola_bringup mola_localize_launch.py
```
