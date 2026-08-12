# Hexplorer Robot Software

Command reference for the Dobot Hexplorer hexapod with RealSense D435 camera and Livox
Mid360 LiDAR. Hardware layout, network design and setup are in the [top-level
README](../README.md).

## Folder Structure

```
hexplorer/
├── sensors/          # Sensor publishers and viewers
├── tracking/         # Object detection and following
├── navigation/       # Autonomous navigation and dances
├── bridges/          # TCP bridges and the odometry frame relay
├── voice/            # ElevenLabs voice control
├── ellipselio/       # EllipseLIO odometry evaluation
├── config/           # RViz configurations
├── docs/             # Setup logs and documentation
└── scripts/          # Launch scripts
```

Copy `.env.example` to `.env` and set `JETSON_PASS` before running anything — every script
sources it through `scripts/common.sh`.

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

### Robot Follows Detected Object
```bash
# YOLO — follow a person (default)
bash ~/hexplorer/scripts/start_object_tracking.sh

# YOLO — follow a specific COCO class
TARGET=bottle bash ~/hexplorer/scripts/start_object_tracking.sh

# YOLO-World — follow any object by text description
DETECT_MODE=yolo-world TARGET="yellow ball" bash ~/hexplorer/scripts/start_object_tracking.sh

# Color mode — follow by color (legacy)
DETECT_MODE=color TARGET=red bash ~/hexplorer/scripts/start_object_tracking.sh
```

### Smart Follower (obstacle avoidance + SLAM-based search)
```bash
bash ~/hexplorer/scripts/start_object_tracking.sh --smart
```

### Object Search (scan-navigate cycle to find objects)
Search logic was tested in Feb 2026 and has not been re-run since the switch to Fast-LIO2.
```bash
# Search for person (default)
bash ~/hexplorer/scripts/start_object_search.sh

# Search for specific object
TARGET=bottle bash ~/hexplorer/scripts/start_object_search.sh

# YOLO-World open vocabulary
DETECT_MODE=yolo-world TARGET="red toolbox" bash ~/hexplorer/scripts/start_object_search.sh

# With RViz visualization
bash ~/hexplorer/scripts/start_object_search.sh --rviz
```

### Fast-LIO2 LiDAR-Inertial Odometry
```bash
bash ~/hexplorer/scripts/start_fastlio.sh
bash ~/hexplorer/scripts/start_fastlio.sh --rviz

# MOLA (legacy, LiDAR-only — superseded, see docs/FASTLIO_SETUP_LOG.md)
bash ~/hexplorer/scripts/start_mola_slam_legacy.sh
```

### Obstacle Avoidance Navigation
```bash
bash ~/hexplorer/scripts/start_obstacle_avoidance.sh
```

### Voice Control
```bash
bash ~/hexplorer/scripts/start_voice_demo.sh
bash ~/hexplorer/scripts/start_voice_demo.sh --debug   # no robot commands
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
| `jetson_object_tracker.py` | Object detection: YOLO, YOLO-World, or color (runs on Jetson) |
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
| `object_searcher.py` | Scan-navigate search for target objects |
| `human_follower.py` | Follow a human using depth camera |
| `macarena_dance.py` | Macarena dance routine |

### Voice (`voice/`)
| File | Description |
|------|-------------|
| `voice_demo.py` | Wake word, ElevenLabs agent session, behaviour dispatch |

### Bridges (`bridges/`)
| File | Description |
|------|-------------|
| `depth_bridge_receiver.py` | Receives depth/color/pointcloud via TCP |
| `livox_tcp_bridge.py` | Sends LiDAR data via TCP (runs on Jetson) |
| `livox_tcp_receiver.py` | Receives LiDAR data via TCP |
| `imu_tcp_bridge.py` | Sends IMU data via TCP (runs on Jetson) |
| `imu_tcp_receiver.py` | Receives IMU data via TCP |
| `odom_relay.py` | Remaps Fast-LIO2 frames to `odom`/`base_link` and broadcasts TF |

### Configuration (`config/`)
| File | Description |
|------|-------------|
| `sensor_visualization.rviz` | Full sensor demo RViz config |
| `tracking_visualization.rviz` | Tracking-only RViz config |
| `search_visualization.rviz` | Object search RViz config |
| `fastlio.rviz` | Fast-LIO2 odometry RViz config |
| `mola_slam.rviz` | MOLA RViz config (legacy) |

### Scripts (`scripts/`)
| File | Description |
|------|-------------|
| `common.sh` | Shared helpers: `.env` loading, Jetson connectivity, process management |
| `jetson_services.sh` | Persistent LiDAR + IMU services (auto-start on Jetson boot) |
| `start_sensor_demo.sh` | Full sensor demo (camera + LiDAR + tracking) |
| `start_object_tracking.sh` | Object tracking with optional following |
| `start_obstacle_avoidance.sh` | Autonomous obstacle avoidance |
| `start_object_search.sh` | Object search (odometry + tracking infra) |
| `start_fastlio.sh` | Fast-LIO2 LiDAR-inertial odometry |
| `start_mola_slam_legacy.sh` | MOLA LiDAR odometry (legacy) |
| `start_voice_demo.sh` | Voice-controlled demo |
| `start_macarena.sh` | Macarena dance routine |
| `tracking_menu.sh`, `yolo_world_menu.sh` | Interactive menus over the tracking modes |

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
| `/livox/imu` | Imu | LiDAR IMU (~200 Hz) |

### Odometry Topics (Fast-LIO2)
| Topic | Type | Description |
|-------|------|-------------|
| `/lidar_odometry/pose` | Odometry | Robot pose estimate — subscribe to this |
| `/Odometry` | Odometry | Raw Fast-LIO2 output before frame remap |
| `/cloud_registered` | PointCloud2 | Registered scan |
| `/Laser_map` | PointCloud2 | Accumulated map |

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

## Detection Modes

| Mode | Model | FPS (TensorRT) | Use Case |
|------|-------|-----------------|----------|
| `yolo` (default) | YOLOv8n | ~63 | Fast tracking of 80 COCO classes (person, bottle, chair, etc.) |
| `yolo-world` | YOLOv8s-worldv2 | ~62 (PyTorch) | Detect any object by text description ("yellow ball", "red toolbox") |
| `color` | HSV thresholds | ~30 | Simple color tracking (yellow, red, green, blue) |

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `DETECT_MODE` | yolo | Detection mode: `yolo`, `yolo-world`, or `color` |
| `TARGET` | person | What to detect (COCO class, text description, or color name) |
| `TARGET_DISTANCE` | 800 | Target follow distance (mm) |
| `STOP_DISTANCE` | 0.6 | Obstacle stop distance (m) |
| `FORWARD_SPEED` | 0.5 | Walking speed (m/s) |

## Documentation

See `docs/` folder:
- `HEXPLORER_CONTROL.md` - Robot control reference (states, topics, messages)
- `ROS2_TOPICS_AND_SLAM.md` - All ROS2 topics and the odometry pipeline
- `FASTLIO_SETUP_LOG.md` - Fast-LIO2 architecture and usage (current odometry)
- `CAMERA_SETUP_LOG.md` - RealSense camera known issues
- `DEPTH_BRIDGE_SETUP_LOG.md` - TCP depth bridge setup
- `LIDAR_SETUP_LOG.md` - Livox LiDAR setup
- `OBJECT_SEARCH_SETUP_LOG.md` - Object search design and test notes
- `WIFI_BOOT_SETUP_LOG.md` - WiFi configuration
- `XRDP_SETUP_LOG.md` - Remote desktop setup
- `MOLA_SLAM_SETUP_LOG.md`, `MOLA_SLAM_TUNING.md` - MOLA odometry (legacy)

The EllipseLIO evaluation lives separately in [`ellipselio/`](ellipselio/).
