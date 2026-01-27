# Livox Mid360 LiDAR Setup Log - 2026-01-27

## Overview

This document describes the setup for the Livox Mid360 LiDAR and alignment with the RealSense D435 camera pointcloud.

---

## Hardware

- **LiDAR Model:** Livox Mid360
- **Serial Number:** 47MCN8F0031553
- **IP Address:** 192.168.1.153 (changed from default 192.168.1.190)
- **Connection:** Ethernet to Jetson Orin Nano

---

## Architecture

```
Jetson (192.168.1.20)                    Mini PC (192.168.1.10)
┌─────────────────────┐                  ┌─────────────────────┐
│ livox_lidar_node    │                  │ livox_tcp_receiver  │
│ (ROS2 driver)       │                  │ (republishes to     │
│         │           │                  │  /livox/pointcloud) │
│         ▼           │                  │         ▲           │
│ livox_tcp_bridge.py │ ──TCP:9998────▶  │         │           │
│ (sends via TCP)     │                  │         │           │
└─────────────────────┘                  └─────────────────────┘
```

**Why TCP Bridge?**
- ROS2 DDS (both FastDDS and CycloneDDS) has cross-machine discovery issues for custom message types
- The Livox driver publishes `custom_msg/msg/LivoxPointcloud` which RViz cannot display directly
- TCP bridge converts to standard `sensor_msgs/msg/PointCloud2` and bypasses DDS issues

---

## Configuration Files

### LiDAR IP Configuration
**File:** `/home/robot/robot_controller_release/ros2_packages/livox_lidar_node/share/livox_lidar_node/config/lidar_parameters.json`

```json
{
  "MID360": {
    "lidar_net_info" : {
      "cmd_data_port"  : 56100,
      "push_msg_port"  : 56200,
      "point_data_port": 56300,
      "imu_data_port"  : 56400,
      "log_data_port"  : 56500
    },
    "host_net_info" : [
      {
        "host_ip"        : "192.168.1.20",
        "lidar_ip"       : ["192.168.1.153"],
        "cmd_data_port"  : 56101,
        "push_msg_port"  : 56201,
        "point_data_port": 56301,
        "imu_data_port"  : 56401,
        "log_data_port"  : 56501
      }
    ]
  }
}
```

**Note:** Original IP was 192.168.1.190, changed to 192.168.1.153 to match actual LiDAR IP.

---

## Scripts

### 1. `/home/robot/livox_tcp_bridge.py` (Jetson)

**Purpose:** Subscribes to Livox custom pointcloud topic, sends data via TCP to Mini PC.

**Subscribes to:** `/livox_Lidar_node/sn153/xyz/pointcloud` (custom_msg/msg/LivoxPointcloud)

**TCP Port:** 9998

**Protocol:**
```
Header (16 bytes):
  - stamp_sec (4 bytes, uint32, network order)
  - stamp_nsec (4 bytes, uint32, network order)
  - num_points (4 bytes, uint32, network order)
  - data_len (4 bytes, uint32, network order)

Data:
  - For each point: x(float32), y(float32), z(float32), intensity(float32)
  - Total: num_points * 16 bytes
```

### 2. `/home/robot/livox_tcp_receiver.py` (Mini PC)

**Purpose:** Receives LiDAR data via TCP, publishes as standard PointCloud2.

**Publishes to:** `/livox/pointcloud` (sensor_msgs/msg/PointCloud2)

**Frame ID:** `livox_frame`

**PointCloud2 Fields:**
- x (FLOAT32, offset 0)
- y (FLOAT32, offset 4)
- z (FLOAT32, offset 8)
- intensity (FLOAT32, offset 12)

**Point step:** 16 bytes

---

## Start Commands

### On Jetson (192.168.1.20):

**Terminal 1 - Livox Driver:**
```bash
source /opt/ros/humble/setup.bash
source /home/robot/robot_controller_release/ros2_packages/setup.bash
ros2 launch livox_lidar_node start_node.launch.py
```

**Terminal 2 - TCP Bridge:**
```bash
source /opt/ros/humble/setup.bash
source /home/robot/robot_controller_release/ros2_packages/setup.bash
python3 /home/robot/livox_tcp_bridge.py
```

### On Mini PC (192.168.1.10):

**Terminal 1 - TCP Receiver:**
```bash
source /opt/ros/humble/setup.bash
python3 /home/robot/livox_tcp_receiver.py
```

**Terminal 2 - TF Publishers:**
```bash
source /opt/ros/humble/setup.bash

# LiDAR frame (no rotation)
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id livox_frame

# Camera frame (with rotation to align with LiDAR)
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 --frame-id map --child-frame-id camera_depth_optical_frame
```

**Terminal 3 - RViz:**
```bash
source /opt/ros/humble/setup.bash
rviz2
```

---

## TF Frame Alignment

### Problem
The RealSense camera uses optical frame convention (Z forward, Y down, X right) while the LiDAR uses robot convention (X forward, Y left, Z up). The pointclouds appeared misaligned in RViz.

### Solution
Applied rotation transform to `camera_depth_optical_frame`:

**Quaternion:** `qx=-0.5, qy=0.5, qz=-0.5, qw=0.5`

This represents:
1. 90° CCW rotation around Y axis (to align viewing direction)
2. 90° CW rotation around the viewing axis (roll correction)

### TF Tree
```
map
├── livox_frame (identity transform)
│   └── LiDAR pointcloud (/livox/pointcloud)
│
└── camera_depth_optical_frame (rotated)
    └── Camera pointcloud (/camera/points)
```

---

## ROS2 Topics

### On Jetson (local):
| Topic | Type | Description |
|-------|------|-------------|
| `/livox_Lidar_node/sn153/xyz/pointcloud` | custom_msg/msg/LivoxPointcloud | Raw LiDAR data |
| `/livox_Lidar_node/sn153/imu/raw_data` | sensor_msgs/msg/Imu | IMU data |

### On Mini PC (via TCP bridge):
| Topic | Type | Description |
|-------|------|-------------|
| `/livox/pointcloud` | sensor_msgs/msg/PointCloud2 | LiDAR pointcloud (converted) |
| `/camera/points` | sensor_msgs/msg/PointCloud2 | Camera pointcloud |
| `/camera/color/image_raw` | sensor_msgs/msg/Image | Camera color image |
| `/camera/depth/image_raw` | sensor_msgs/msg/Image | Camera depth image |

---

## RViz Configuration

### Display Settings:
1. **Fixed Frame:** `map`

2. **LiDAR PointCloud2:**
   - Topic: `/livox/pointcloud`
   - Size: 0.02
   - Style: Points
   - Color Transformer: Intensity or FlatColor

3. **Camera PointCloud2:**
   - Topic: `/camera/points`
   - Size: 0.01
   - Style: Points
   - Color Transformer: RGB8

4. **Camera Image:**
   - Topic: `/camera/color/image_raw`

---

## Performance

- **LiDAR Rate:** ~10 Hz
- **Points per frame:** ~14,000-15,000
- **TCP Bridge Port:** 9998
- **Camera Bridge Port:** 9999

---

## Troubleshooting

### LiDAR not detected
1. Check IP: `ping 192.168.1.153` from Jetson
2. Verify config file has correct IP (192.168.1.153, not 192.168.1.190)
3. Check LiDAR is powered (heatsink warm, motor spinning)

### TCP connection refused
1. Ensure Livox driver is running on Jetson first
2. Check TCP bridge is running: `ps aux | grep livox_tcp_bridge`
3. Verify port 9998 is not blocked

### Pointclouds misaligned
1. Verify TF publishers are running: `ros2 run tf2_ros tf2_echo map livox_frame`
2. Check camera TF has correct quaternion: `qx=-0.5, qy=0.5, qz=-0.5, qw=0.5`

### RViz shows 0 messages
1. Ensure RViz and publishers use same DDS (recommend: don't set RMW_IMPLEMENTATION)
2. Check topic is publishing: `ros2 topic hz /livox/pointcloud`
3. Restart RViz after starting publishers

---

## Files Summary

| File | Location | Purpose |
|------|----------|---------|
| `livox_tcp_bridge.py` | Jetson `/home/robot/` | TCP sender for LiDAR |
| `livox_tcp_receiver.py` | Mini PC `/home/robot/` | TCP receiver, publishes PointCloud2 |
| `lidar_parameters.json` | Jetson (ros2_packages) | LiDAR IP configuration |
| `LIDAR_SETUP_LOG.md` | Mini PC `/home/robot/` | This documentation |
