# Depth Bridge Setup Log - 2026-01-27

## Overview

This document describes the TCP bridge system for streaming RealSense D435 depth, pointcloud, and color data from the Jetson Orin Nano to the Intel Mini PC.

**Problem Solved:** CycloneDDS cross-machine topic discovery fails for large messages (depth images, pointclouds). The TCP bridge bypasses DDS entirely for reliable data transfer.

---

## Architecture

```
┌─────────────────────────┐         TCP (port 9999)        ┌─────────────────────────┐
│   Jetson Orin Nano      │ ──────────────────────────────▶│    Intel Mini PC        │
│   192.168.1.20          │                                │    192.168.1.10         │
│                         │                                │                         │
│ realsense_depth_tcp_    │                                │ depth_bridge_           │
│ publisher.py            │                                │ receiver.py             │
│                         │                                │                         │
│ - Reads RealSense D435  │                                │ - Receives TCP data     │
│ - Publishes to local    │                                │ - Republishes to ROS2   │
│   ROS2 topics           │                                │   topics                │
│ - Sends via TCP         │                                │                         │
└─────────────────────────┘                                └─────────────────────────┘
```

---

## Scripts Created

### 1. `/home/robot/realsense_depth_tcp_publisher.py` (Jetson)

**Purpose:** Combined RealSense camera publisher + TCP server. Reads from RealSense D435, publishes to local ROS2 topics, and streams data via TCP to Mini PC.

**Location:** Runs on Jetson Orin Nano (192.168.1.20)

**ROS2 Topics Published (local to Jetson):**
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/camera/camera/color/image_raw` | sensor_msgs/Image | 640x480 BGR8 color image |
| `/camera/camera/depth/image_rect_raw` | sensor_msgs/Image | 640x480 16UC1 depth (mm) |
| `/camera/camera/points` | sensor_msgs/PointCloud2 | XYZRGB pointcloud |
| `/camera/camera/color/camera_info` | sensor_msgs/CameraInfo | Color camera intrinsics |
| `/camera/camera/depth/camera_info` | sensor_msgs/CameraInfo | Depth camera intrinsics |

**TCP Protocol:**
- Port: 9999
- Message Type 1: Depth image (16-bit)
- Message Type 2: Pointcloud (XYZRGB)
- Message Type 3: Color image (BGR8)

**Start Command:**
```bash
# On Jetson
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/usr/local/lib/python3.10/dist-packages:$PYTHONPATH
python3 /home/robot/realsense_depth_tcp_publisher.py
```

---

### 2. `/home/robot/depth_bridge_receiver.py` (Mini PC)

**Purpose:** TCP client that connects to Jetson, receives depth/pointcloud/color data, and republishes to ROS2 topics on Mini PC.

**Location:** Runs on Intel Mini PC (192.168.1.10)

**ROS2 Topics Published:**
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/camera/color/image_raw` | sensor_msgs/Image | 640x480 BGR8 color image |
| `/camera/depth/image_raw` | sensor_msgs/Image | 640x480 16UC1 depth (mm) |
| `/camera/points` | sensor_msgs/PointCloud2 | XYZRGB pointcloud |

**Start Command:**
```bash
# On Mini PC
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 /home/robot/depth_bridge_receiver.py
```

---

### 3. `/home/robot/start_depth_tcp.sh` (Jetson)

**Purpose:** Helper script to start the combined publisher on Jetson.

```bash
#!/bin/bash
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/usr/local/lib/python3.10/dist-packages:$PYTHONPATH
echo "Starting RealSense Depth + TCP publisher..."
echo "TCP server will listen on port 9999"
python3 /home/robot/realsense_depth_tcp_publisher.py
```

---

### 4. `/home/robot/realsense_depth_publisher.py` (Jetson - older version)

**Purpose:** Original RealSense publisher (without TCP). Publishes only to local ROS2 topics.

**Note:** Superseded by `realsense_depth_tcp_publisher.py` which includes TCP streaming.

---

### 5. `/home/robot/depth_bridge_sender.py` (Jetson - older version)

**Purpose:** Standalone TCP sender that subscribes to ROS2 topics and forwards via TCP.

**Note:** Had issues with CycloneDDS config mismatch. Superseded by combined publisher.

---

## TCP Protocol Specification

### Message Format

All messages start with a 1-byte type identifier:

**Type 1 - Depth Image:**
```
| Type (1B) | Width (4B) | Height (4B) | Stamp_sec (4B) | Stamp_nsec (4B) | Data_len (4B) | Data |
|    0x01   |   uint32   |   uint32    |    uint32      |     uint32      |    uint32     | bytes|
```
- Encoding: 16UC1 (16-bit unsigned, 1 channel)
- Step: width * 2

**Type 2 - Pointcloud:**
```
| Type (1B) | Width (4B) | Height (4B) | Point_step (4B) | Row_step (4B) | Stamp_sec (4B) | Stamp_nsec (4B) | Data_len (4B) | Data |
|    0x02   |   uint32   |   uint32    |     uint32      |    uint32     |    uint32      |     uint32      |    uint32     | bytes|
```
- Point format: XYZRGB (16 bytes per point)
- Fields: x(float32), y(float32), z(float32), rgb(uint32)

**Type 3 - Color Image:**
```
| Type (1B) | Width (4B) | Height (4B) | Stamp_sec (4B) | Stamp_nsec (4B) | Data_len (4B) | Data |
|    0x03   |   uint32   |   uint32    |    uint32      |     uint32      |    uint32     | bytes|
```
- Encoding: BGR8 (8-bit, 3 channels)
- Step: width * 3

---

## RViz2 Configuration

### To view pointcloud:
1. Set **Fixed Frame** to `map`
2. Add → PointCloud2
3. Set **Topic** to `/camera/points`
4. Set **Size** to `0.01`

### To view color image:
1. Add → Image
2. Set **Topic** to `/camera/color/image_raw`

### TF Setup:
A static transform publisher is needed:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map camera_depth_optical_frame
```

---

## Quick Start

### Start everything:

**Terminal 1 - Jetson (SSH):**
```bash
sshpass -p "123" ssh robot@192.168.1.20
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/usr/local/lib/python3.10/dist-packages:$PYTHONPATH
python3 /home/robot/realsense_depth_tcp_publisher.py
```

**Terminal 2 - Mini PC (Receiver):**
```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 /home/robot/depth_bridge_receiver.py
```

**Terminal 3 - Mini PC (TF):**
```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map camera_depth_optical_frame
```

**Terminal 4 - Mini PC (RViz):**
```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
rviz2
```

---

## Troubleshooting

### No data in RViz
- Check Fixed Frame is set to `map`
- Ensure topic is selected (not empty)
- Verify TF publisher is running

### Connection refused
- Ensure sender is running on Jetson first
- Check port 9999 is not in use: `fuser -k 9999/tcp`

### "RGB modules inconsistency" error
- Use librealsense 2.55, not 2.56.4
- Set: `export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH`

---

## Frame IDs

| Frame ID | Used By |
|----------|---------|
| `camera_color_optical_frame` | Color images |
| `camera_depth_optical_frame` | Depth images, Pointcloud |
| `map` | RViz fixed frame |

---

## Performance

- Color: ~6 Hz
- Depth: ~6 Hz
- Pointcloud: ~2-3 Hz (every 3rd frame)
- Points per cloud: ~180,000

---

## Files Location Summary

| File | Machine | Purpose |
|------|---------|---------|
| `/home/robot/realsense_depth_tcp_publisher.py` | Jetson | Main publisher + TCP server |
| `/home/robot/depth_bridge_receiver.py` | Mini PC | TCP client + ROS2 republisher |
| `/home/robot/start_depth_tcp.sh` | Jetson | Start script |
| `/tmp/depth_tcp.log` | Jetson | Publisher log |
| `/tmp/bridge_receiver.log` | Mini PC | Receiver log |

---

## Build Notes

pyrealsense2 v2.55 was built from source on Jetson because:
1. pip package doesn't exist for aarch64
2. librealsense 2.56.4 has "RGB modules inconsistency" bug

Build location: `/home/robot/librealsense-2.55/build`
Installed to: `/usr/local/lib/python3.10/dist-packages/pyrealsense2.cpython-310-aarch64-linux-gnu.so`
