# Depth Bridge Setup

## Overview

TCP bridge for streaming RealSense D435 depth, pointcloud, and color data from Jetson to Mini PC. Bypasses DDS large message transfer issues.

## Architecture

```
Jetson (192.168.1.20)                    Mini PC (192.168.1.10)
realsense_depth_tcp_publisher.py  ─TCP:9999─►  depth_bridge_receiver.py
  - Reads RealSense D435                         - Republishes to ROS2
  - Sends color/depth/pointcloud                  - Local topics
```

## ROS2 Topics (on Mini PC)

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | Image | 640x480 BGR8 color |
| `/camera/depth/image_raw` | Image | 640x480 16UC1 depth (mm) |
| `/camera/points` | PointCloud2 | XYZRGB pointcloud |

## Quick Start

Use the unified launch script:
```bash
bash ~/hexplorer/scripts/start_sensor_demo.sh
```

## Key Files

| File | Machine | Purpose |
|------|---------|---------|
| `~/hexplorer/sensors/realsense_depth_tcp_publisher.py` | Jetson | Camera + TCP server |
| `~/hexplorer/bridges/depth_bridge_receiver.py` | Mini PC | TCP client + ROS2 republisher |

## TF Setup

Camera frame needs static transform:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map camera_depth_optical_frame
```

## Performance

| Stream | Rate |
|--------|------|
| Color | ~6 Hz |
| Depth | ~6 Hz |
| Pointcloud | ~2-3 Hz |
| Points per cloud | ~180,000 |

## Troubleshooting

| Problem | Solution |
|---------|----------|
| No data in RViz | Check Fixed Frame is `map`, verify TF publisher running |
| Connection refused | Ensure TCP publisher started on Jetson first |
| "RGB modules inconsistency" | Use librealsense 2.55: `export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH` |
| Port in use | `fuser -k 9999/tcp` |

## Build Notes

pyrealsense2 v2.55 built from source on Jetson (pip package doesn't exist for aarch64, and v2.56.4 has RGB bug).
- Source: `/home/robot/librealsense-2.55/build`
- Installed: `/usr/local/lib/python3.10/dist-packages/pyrealsense2*.so`
