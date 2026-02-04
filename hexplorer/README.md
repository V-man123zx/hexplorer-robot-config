# Hexplorer Robot Software

Organized software for the Dobot Hexplorer hexapod robot with RealSense D435 camera, Livox Mid360 LiDAR, and object tracking capabilities.

## Folder Structure

```
hexplorer/
├── sensors/          # Sensor publishers and viewers
├── tracking/         # Object detection and following
├── navigation/       # Autonomous navigation (obstacle avoidance)
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

### Object Tracking Only (with RViz)
```bash
bash ~/hexplorer/scripts/start_object_tracking.sh --rviz
```

### Robot Follows Yellow Object
```bash
bash ~/hexplorer/scripts/start_object_tracking.sh
```

### Obstacle Avoidance Navigation
```bash
bash ~/hexplorer/scripts/start_obstacle_avoidance.sh
```

## Components

### Sensors (`sensors/`)
| File | Description |
|------|-------------|
| `realsense_depth_tcp_publisher.py` | RealSense depth + color publisher with TCP |
| `realsense_depth_publisher.py` | RealSense local publisher |
| `camera_viewer.py` | Simple camera viewer |

### Tracking (`tracking/`)
| File | Description |
|------|-------------|
| `jetson_object_tracker.py` | Color-based object detection (runs on Jetson) |
| `detection_receiver.py` | Receives detections via TCP, publishes to ROS2 |
| `object_follower.py` | Robot control to follow detected object |
| `tracking_rviz_visualizer.py` | RViz markers and overlay visualization |
| `tracking_visualizer.py` | Terminal-based tracking display |
| `follow_white_box.py` | Legacy yellow object follower |

### Navigation (`navigation/`)
| File | Description |
|------|-------------|
| `obstacle_avoidance.py` | Autonomous navigation avoiding obstacles |
| `human_follower.py` | Follow a human using depth camera |

### Bridges (`bridges/`)
| File | Description |
|------|-------------|
| `depth_bridge_receiver.py` | Receives depth/color/pointcloud via TCP |
| `livox_tcp_bridge.py` | Sends LiDAR data via TCP (runs on Jetson) |
| `livox_tcp_receiver.py` | Receives LiDAR data via TCP |

### Configuration (`config/`)
| File | Description |
|------|-------------|
| `sensor_visualization.rviz` | Full sensor demo RViz config |
| `tracking_visualization.rviz` | Tracking-only RViz config |

## Network Architecture

```
Mini PC (192.168.1.10)          Jetson Orin Nano (192.168.1.20)
====================            ============================

depth_bridge_receiver.py  <--TCP:9999--  realsense_depth_tcp_publisher.py
livox_tcp_receiver.py     <--TCP:9998--  livox_tcp_bridge.py
detection_receiver.py     <--TCP:9997--  jetson_object_tracker.py
                          <--TCP:9996--  (image stream)
```

## ROS2 Topics

### Sensor Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | Image | Color camera |
| `/camera/depth/image_raw` | Image | Depth camera |
| `/camera/points` | PointCloud2 | Camera pointcloud |
| `/livox/pointcloud` | PointCloud2 | LiDAR pointcloud |

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

See `docs/` folder for detailed setup logs:
- `HEXPLORER_CONTROL.md` - Robot control reference
- `CAMERA_SETUP_LOG.md` - RealSense setup
- `LIDAR_SETUP_LOG.md` - Livox LiDAR setup
- `DEPTH_BRIDGE_SETUP_LOG.md` - TCP bridge setup
- `WIFI_BOOT_SETUP_LOG.md` - WiFi configuration
- `XRDP_SETUP_LOG.md` - Remote desktop setup
