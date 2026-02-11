# Hexplorer Robot Software

Organized software for the Dobot Hexplorer hexapod robot with RealSense D435 camera, Livox Mid360 LiDAR, and object tracking capabilities.

## Folder Structure

```
hexplorer/
├── sensors/          # Sensor publishers and viewers
├── tracking/         # Object detection and following
├── navigation/       # Autonomous navigation and dances
├── bridges/          # TCP bridges for cross-machine communication
├── config/           # RViz configurations
├── docs/             # Setup logs and documentation
└── scripts/          # Launch scripts
```

## Quick Start

### Full Sensor Demo with Object Tracking
```bash
bash ~/hexplorer/scripts/start_sensor_demo.sh
```

### Sensors Only (no tracking, full depth available)
```bash
bash ~/hexplorer/scripts/start_sensor_demo.sh --no-track
```

### Object Tracking with RViz
```bash
bash ~/hexplorer/scripts/start_object_tracking.sh --rviz
```

### Robot Follows Yellow Object
```bash
bash ~/hexplorer/scripts/start_object_tracking.sh
```

### Smart Follower (obstacle avoidance + SLAM-based search)
```bash
bash ~/hexplorer/scripts/start_object_tracking.sh --smart
```

### MOLA LiDAR Odometry (SLAM)
```bash
bash ~/hexplorer/scripts/start_mola_slam.sh
```

### Obstacle Avoidance Navigation
```bash
bash ~/hexplorer/scripts/start_obstacle_avoidance.sh
```

### Macarena Dance
```bash
bash ~/hexplorer/scripts/start_macarena.sh
```

## Components

### Sensors (`sensors/`)
| File | Description |
|------|-------------|
| `realsense_depth_tcp_publisher.py` | RealSense depth + color publisher with TCP |
| `camera_viewer.py` | Simple camera viewer |

### Tracking (`tracking/`)
| File | Description |
|------|-------------|
| `jetson_object_tracker.py` | Color-based object detection (runs on Jetson) |
| `detection_receiver.py` | Receives detections via TCP, publishes to ROS2 |
| `object_follower.py` | Robot control to follow detected object |
| `smart_follower.py` | Smart follower with obstacle avoidance + SLAM search |
| `tracking_rviz_visualizer.py` | RViz markers and overlay visualization |
| `tracking_visualizer.py` | Terminal-based tracking display |
| `follow_white_box.py` | Legacy yellow object follower |

### Navigation (`navigation/`)
| File | Description |
|------|-------------|
| `obstacle_avoidance.py` | Autonomous navigation avoiding obstacles |
| `human_follower.py` | Follow a human using depth camera |
| `frontier_explorer.py` | SLAM frontier detection |
| `macarena_dance.py` | Macarena dance routine |

### Bridges (`bridges/`)
| File | Description |
|------|-------------|
| `depth_bridge_receiver.py` | Receives depth/color/pointcloud via TCP |
| `livox_tcp_bridge.py` | Sends LiDAR data via TCP (runs on Jetson) |
| `livox_tcp_receiver.py` | Receives LiDAR data via TCP |
| `imu_tcp_bridge.py` | Sends IMU data via TCP (runs on Jetson) |
| `imu_tcp_receiver.py` | Receives IMU data via TCP |

### Configuration (`config/`)
| File | Description |
|------|-------------|
| `sensor_visualization.rviz` | Full sensor demo RViz config |
| `tracking_visualization.rviz` | Tracking-only RViz config |
| `mola_slam.rviz` | MOLA SLAM RViz config |

### Scripts (`scripts/`)
| File | Description |
|------|-------------|
| `common.sh` | Shared helpers: Jetson connectivity, process management |
| `jetson_services.sh` | Persistent LiDAR services (auto-starts on Jetson boot) |
| `start_sensor_demo.sh` | Full sensor demo (camera + LiDAR + tracking) |
| `start_object_tracking.sh` | Object tracking with optional following |
| `start_obstacle_avoidance.sh` | Autonomous obstacle avoidance |
| `start_mola_slam.sh` | MOLA LiDAR odometry/SLAM |
| `start_macarena.sh` | Macarena dance routine |

## Network Architecture

```
Mini PC (192.168.1.10)          Jetson Orin Nano (192.168.1.20)
====================            ============================

depth_bridge_receiver.py  <--TCP:9999--  realsense_depth_tcp_publisher.py
livox_tcp_receiver.py     <--TCP:9998--  livox_tcp_bridge.py (persistent)
detection_receiver.py     <--TCP:9997--  jetson_object_tracker.py
                          <--TCP:9996--  (image stream)
```

Jetson LiDAR services (driver + TCP bridge) run as a systemd service that auto-starts at boot. Camera processes are started/stopped per-script.

## ROS2 Topics

### Sensor Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | Image | Color camera |
| `/camera/depth/image_raw` | Image | Depth camera |
| `/camera/points` | PointCloud2 | Camera pointcloud |
| `/livox/lidar` | PointCloud2 | LiDAR pointcloud |

### MOLA Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/livox/lidar_filtered` | PointCloud2 | Filtered LiDAR (input to MOLA) |
| `/lidar_odometry/pose` | Odometry | Robot pose estimate |
| `/lidar_odometry/localmap_points` | PointCloud2 | Accumulated map |

### Tracking Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/object_detection` | String | JSON detection data |
| `/object_position` | Point | Object position (x,y,z) |
| `/object_tracking/marker` | Marker | 3D sphere at object |
| `/object_tracking/text` | Marker | Distance label |
| `/object_tracking/image` | Image | Camera with overlay |

### Robot Control Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/robot_cmd` | RobotCommand | State control |
| `/vel_cmd` | Twist | Velocity control |
| `/robot_state` | RobotState | Robot feedback |

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `TARGET_COLOR` | yellow | Color to track (yellow/red/green/blue) |
| `TARGET_DISTANCE` | 800 | Target follow distance (mm) |
| `STOP_DISTANCE` | 0.6 | Obstacle stop distance (m) |
| `FORWARD_SPEED` | 0.5 | Walking speed (m/s) |

## Documentation

See `docs/` folder:
- `ROS2_TOPICS_AND_SLAM.md` - All ROS2 topics and MOLA SLAM explained
- `HEXPLORER_CONTROL.md` - Robot control reference (states, topics, messages)
- `MOLA_SLAM_SETUP_LOG.md` - MOLA SLAM architecture and usage
- `MOLA_SLAM_TUNING.md` - MOLA ICP parameter tuning guide
- `CAMERA_SETUP_LOG.md` - RealSense camera known issues
- `DEPTH_BRIDGE_SETUP_LOG.md` - TCP depth bridge setup
- `LIDAR_SETUP_LOG.md` - Livox LiDAR setup
- `WIFI_BOOT_SETUP_LOG.md` - WiFi configuration
- `XRDP_SETUP_LOG.md` - Remote desktop setup
