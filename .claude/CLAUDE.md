# Claude Code Memory

## Skills Repository Tracking

### Primary Skills Repository
- **URL**: https://github.com/travisvn/awesome-claude-skills
- **Purpose**: Curated collection of Claude Skills with installation guides
- **Check for updates**: Periodically review this repository for new skills

### Installed Marketplaces

1. **anthropics/skills** (Official Anthropic Skills)
   - Location: `~/.claude/plugins/marketplaces/anthropic-skills`
   - Includes: PDF, DOCX, PPTX, XLSX document skills + creative/development skills
   - Update: `git -C ~/.claude/plugins/marketplaces/anthropic-skills pull`

2. **obra/superpowers-marketplace** (Community Skills)
   - Location: `~/.claude/plugins/marketplaces/superpowers-marketplace`
   - Includes: Core superpowers, Chrome automation, Elements of Style, Episodic Memory
   - Update: `git -C ~/.claude/plugins/marketplaces/superpowers-marketplace pull`

3. **anthropics/claude-plugins-official** (Official Plugins)
   - Location: `~/.claude/plugins/marketplaces/claude-plugins-official`
   - Includes: LSP servers, PR review toolkit, commit commands, etc.

### Available Skills Quick Reference

**Document Processing (anthropic-skills):**
- `pdf` - PDF extraction, creation, merging, forms
- `docx` - Word document creation/editing
- `pptx` - PowerPoint presentations
- `xlsx` - Excel spreadsheets

**Creative/Development (anthropic-skills):**
- `skill-creator` - Create new skills interactively
- `mcp-builder` - Build MCP servers
- `webapp-testing` - Playwright testing
- `frontend-design` - Bold design decisions
- `canvas-design` - Visual art creation
- `algorithmic-art` - Generative art with p5.js

**Superpowers (superpowers-marketplace):**
- `superpowers` - Core skills: TDD, debugging, collaboration
- `superpowers-chrome` - Chrome DevTools automation
- `elements-of-style` - Writing guidance
- `episodic-memory` - Persistent memory across sessions

### Maintenance Commands

```bash
# Update all marketplaces
git -C ~/.claude/plugins/marketplaces/anthropic-skills pull
git -C ~/.claude/plugins/marketplaces/superpowers-marketplace pull
git -C ~/.claude/plugins/marketplaces/claude-plugins-official pull

# Check for new skills in awesome-claude-skills
# Visit: https://github.com/travisvn/awesome-claude-skills
```

### Notes
- Last updated: 2026-01-22
- Review awesome-claude-skills monthly for new skills
- Security: Only install skills from trusted sources

---

## Dobot Hexplorer Robot Control

### CRITICAL: Command Publishing Requirements

**WRONG (does not work):**
```bash
ros2 topic pub -1 /robot_cmd custom_msg/msg/RobotCommand "{target_state: 4}"
```
This sends ONE message and exits. The robot ignores single messages.

**CORRECT (works):**
Must publish commands **continuously at ~20Hz** using Python:
```python
for _ in range(40):  # 2 seconds
    cmd_pub.publish(cmd)
    time.sleep(0.05)
```

### Robot State Machine

| State | Value | Description |
|-------|-------|-------------|
| PASSIVE | 0 | Damping mode - legs compliant |
| STANDDOWN | 1 | Position folding |
| STANDUP | 2 | Legs extend |
| BALANCESTAND | 3 | Force-control standing |
| WALK | 4 | Walking mode |

### Correct Sequence to Walk

```
PASSIVE(0) → STANDDOWN(1) → STANDUP(2) → BALANCESTAND(3) → WALK(4)
```

Each state needs ~2 seconds of continuous publishing before transitioning.

### Network Architecture

| Device | IP Address | Purpose |
|--------|------------|---------|
| Intel Mini PC | 192.168.1.10 | Robot controller, ROS2 master |
| Jetson Orin Nano | 192.168.1.20 | Sensor processing |
| Livox Mid360 LiDAR | 192.168.1.190 | 3D scanning |

Jetson NFS share: `/.update_share_folder/nano/`

### Sensors

- **RealSense D435**: Depth camera, USB connected to Jetson
  - Topic: `/realsense_camera_node/sn.../color/bgr/image_raw`
- **Livox Mid360**: LiDAR, IP 192.168.1.190
  - Topic: `/livox_Lidar_node/sn.../xyz/pointcloud`

### ROS2 Topics

- `/robot_cmd` - State commands (custom_msg/msg/RobotCommand)
- `/robot_state` - Feedback (custom_msg/msg/RobotState)
- `/vel_cmd` - Velocity (geometry_msgs/msg/Twist)
- `/joy` - Joystick (sensor_msgs/msg/Joy)

### Working Walk Script

```bash
source /home/robot/robot_controller_release/ros2_packages/setup.bash
python3 /home/robot/robot_controller_release/walk_forward.py 0.5  # forward
python3 /home/robot/robot_controller_release/walk_forward.py -0.5 # backward
```

### Python Template for Robot Control

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msg.msg import RobotCommand
from geometry_msgs.msg import Twist
import time

def control_robot():
    rclpy.init()
    node = Node('robot_controller')
    cmd_pub = node.create_publisher(RobotCommand, '/robot_cmd', 10)
    vel_pub = node.create_publisher(Twist, '/vel_cmd', 10)
    time.sleep(0.3)

    cmd = RobotCommand()
    vel = Twist()

    # Stand up sequence (states 1,2,3)
    for state in [1, 2, 3]:
        cmd.target_state = state
        for _ in range(40):  # 2 sec each
            cmd_pub.publish(cmd)
            time.sleep(0.05)

    # Walk (state 4) with velocity
    cmd.target_state = 4
    vel.linear.x = 0.15   # forward (negative = backward)
    vel.angular.z = 0.0   # turn (negative = right)
    for _ in range(70):   # adjust for distance
        cmd_pub.publish(cmd)
        vel_pub.publish(vel)
        time.sleep(0.05)

    # Damping mode (state 0)
    cmd.target_state = 0
    for _ in range(20):
        cmd_pub.publish(cmd)
        time.sleep(0.05)

    node.destroy_node()
    rclpy.shutdown()
```

### Gamepad Controls (Thor G30s)

- LT + A: Toggle position mode / standing
- START: Enter walk mode
- Left Stick: Forward/backward/strafe
- Right Stick: Turn
- RT + DpadUp: Boxing pose (gamepad-only, not via ROS2)

### Full Documentation

See: `/home/robot/robot_controller_release/HEXPLORER_CONTROL.md`

---

## RealSense Camera Setup (2026-01-27)

### WORKING SOLUTION: Official Intel ROS2 Package

The custom `realsense_camera_node` from INFFNI Robotics has bugs. Use the official Intel package instead.

#### Camera Info
- **Model:** Intel RealSense D435
- **Serial:** 406122070499
- **Firmware:** 5.12.7.150

#### Start Camera on Jetson
```bash
sshpass -p "123" ssh robot@192.168.1.20
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH  # Use working librealsense 2.55
ros2 launch realsense2_camera rs_launch.py
```

Or use the helper script:
```bash
bash /home/robot/start_official_realsense.sh
```

#### Official Package Topics
- `/camera/camera/color/image_raw` - 640x480 RGB8 @ 30fps
- `/camera/camera/depth/image_rect_raw` - 848x480 Z16 @ 30fps
- `/camera/camera/color/camera_info` - Camera intrinsics
- `/camera/camera/depth/camera_info` - Depth intrinsics

#### View Camera on Jetson (X11)
```bash
ssh -X robot@192.168.1.20
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
python3 /home/robot/official_realsense_viewer.py
```

#### Key Learnings

1. **Librealsense Version Conflict**
   - ROS package installed librealsense 2.56.4 which has "RGB modules inconsistency" bug
   - Local `/usr/local/lib/librealsense2.so.2.55` works correctly
   - **Fix:** `export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH` before launching

2. **ROS2 Cross-Machine Large Message Issue**
   - FastDDS discovery works but large message (image) transfer fails between Mini PC and Jetson
   - **Workaround:** Run viewer directly on Jetson with X11 forwarding

3. **Official Package vs Custom Node**
   - Custom `realsense_camera_node`: Topics created but no data published
   - Official `realsense2_camera`: Works correctly, standard topic names

### SSH to Jetson
```bash
sshpass -p "123" ssh robot@192.168.1.20
```

### Legacy Custom Node (DOES NOT WORK)
The old custom node can still be launched but doesn't publish data:
```bash
ros2 launch realsense_camera_node start_node.launch.py
# Topics: /realsense_camera_node/sn406122070499/color/bgr/image_raw (no data)
```

### Detailed Log
See: `/home/robot/robot_controller_release/CAMERA_SETUP_LOG.md`

---

## CycloneDDS Setup for Cross-Machine Image Streaming (2026-01-27)

### Problem Solved
FastDDS (ROS2 default) fails for large image messages between Mini PC and Jetson due to UDP fragmentation issues.

### Solution: CycloneDDS + V4L2 Camera Publisher

CycloneDDS is installed on both machines and configured for reliable image transfer.

#### Configuration
Both machines have in `~/.bashrc`:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Socket buffers set to 8MB on both machines:
```bash
net.core.rmem_max=8388608
net.core.wmem_max=8388608
```

#### RealSense Librealsense Bug
- `ros-humble-realsense2-camera` requires librealsense 2.56.4
- librealsense 2.56.4 has "RGB modules inconsistency" bug with D435 camera
- **Workaround:** Use V4L2/OpenCV-based publisher instead of ROS realsense package

#### Start Camera (on Jetson)
```bash
sshpass -p "123" ssh robot@192.168.1.20
bash /home/robot/start_camera_cyclone.sh
```

Or manually:
```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 /home/robot/realsense_v4l2_publisher.py
```

#### View Camera (on Mini PC)
```bash
bash /home/robot/view_camera.sh
```

Or manually:
```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 /home/robot/camera_viewer.py
```

#### Topics
- `/camera/camera/color/image_raw` - 640x480 BGR8 images

#### Key Files
- `/home/robot/realsense_v4l2_publisher.py` - Camera publisher (on Jetson)
- `/home/robot/camera_viewer.py` - Image viewer (on Mini PC)
- `/home/robot/start_camera_cyclone.sh` - Start script (on Jetson)
- `/home/robot/view_camera.sh` - Viewer script (on Mini PC)

#### Verify CycloneDDS is Working
```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic hz /camera/camera/color/image_raw
```

Should show ~4-30 Hz depending on network/processing load.

---

## RealSense Depth/PointCloud with pyrealsense2 (2026-01-27)

### Overview
Built pyrealsense2 v2.55 Python bindings from source to access depth and pointcloud data. The apt package (v2.56.4) has "RGB modules inconsistency" bug.

### What Was Built
- librealsense 2.55 source downloaded and built with Python bindings
- pyrealsense2 module installed to `/usr/local/lib/python3.10/dist-packages/`
- Custom ROS2 publisher for color, depth, and pointcloud

### Topics Published
| Topic | Type | Description |
|-------|------|-------------|
| `/camera/camera/color/image_raw` | Image | 640x480 BGR8 color |
| `/camera/camera/depth/image_rect_raw` | Image | 640x480 Z16 depth (mm) |
| `/camera/camera/points` | PointCloud2 | 3D colored pointcloud |
| `/camera/camera/color/camera_info` | CameraInfo | Color camera intrinsics |
| `/camera/camera/depth/camera_info` | CameraInfo | Depth camera intrinsics |

### Start Depth Publisher (on Jetson)
```bash
sshpass -p "123" ssh robot@192.168.1.20
bash /home/robot/start_realsense_depth.sh
```

Or manually:
```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
python3 /home/robot/realsense_depth_publisher.py
```

### Verify pyrealsense2 Works
```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
python3 -c "import pyrealsense2 as rs; print('pyrealsense2 version:', rs.__version__)"
# Should print: pyrealsense2 version: 2.55.1
```

### Cross-Machine Topic Discovery Issue
- Color images stream successfully to Mini PC via CycloneDDS
- Depth/pointcloud topics exist on Jetson but aren't discovered on Mini PC
- **Workaround:** View depth on Jetson directly via X11 forwarding

### View Depth on Jetson (X11)
```bash
ssh -X robot@192.168.1.20
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /camera/camera/depth/image_rect_raw --no-arr --once
```

### Key Files (on Jetson)
- `/home/robot/realsense_depth_publisher.py` - Main depth publisher
- `/home/robot/start_realsense_depth.sh` - Start script
- `/home/robot/librealsense-2.55/` - Source code used for build
- `/usr/local/lib/python3.10/dist-packages/pyrealsense2*.so` - Python bindings

### Build Notes
- librealsense 2.55 source: https://github.com/IntelRealSense/librealsense/tree/v2.55.1
- Dependencies (pybind11, nlohmann/json) were downloaded on Mini PC and transferred to Jetson (no internet on Jetson)
- CMake files were patched to skip downloads when dependencies exist locally
- Build time: ~20 minutes on Jetson Orin Nano with `-j4`

---

## TCP Bridge for Depth/PointCloud/Color (2026-01-27)

### Problem Solved
CycloneDDS cross-machine topic discovery fails for depth/pointcloud topics. TCP bridge bypasses DDS for reliable streaming.

### Architecture
```
Jetson (192.168.1.20)                    Mini PC (192.168.1.10)
realsense_depth_tcp_publisher.py  --->   depth_bridge_receiver.py
        TCP port 9999
```

### Start Commands

**On Jetson:**
```bash
sshpass -p "123" ssh robot@192.168.1.20
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/usr/local/lib/python3.10/dist-packages:$PYTHONPATH
python3 /home/robot/realsense_depth_tcp_publisher.py
```

**On Mini PC:**
```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 /home/robot/depth_bridge_receiver.py
```

**TF Publisher (Mini PC):**
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map camera_depth_optical_frame
```

**RViz (Mini PC):**
```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
rviz2
```

### ROS2 Topics on Mini PC (from bridge receiver)

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | sensor_msgs/Image | 640x480 BGR8 color |
| `/camera/depth/image_raw` | sensor_msgs/Image | 640x480 16UC1 depth (mm) |
| `/camera/points` | sensor_msgs/PointCloud2 | XYZRGB pointcloud |

### RViz Setup
1. Set **Fixed Frame** to `map`
2. Add PointCloud2, set Topic to `/camera/points`, Size to `0.01`
3. Add Image, set Topic to `/camera/color/image_raw`

### Key Files

| File | Location | Purpose |
|------|----------|---------|
| `realsense_depth_tcp_publisher.py` | Jetson | Camera + TCP server |
| `depth_bridge_receiver.py` | Mini PC | TCP client + ROS2 republisher |
| `start_depth_tcp.sh` | Jetson | Start script |
| `DEPTH_BRIDGE_SETUP_LOG.md` | Mini PC | Full documentation |

### Performance
- Color/Depth: ~6 Hz
- Pointcloud: ~2-3 Hz
- Points per cloud: ~180,000

---

## Livox Mid360 LiDAR Setup (2026-01-27)

### Hardware
- **Model:** Livox Mid360
- **Serial:** 47MCN8F0031553
- **IP Address:** 192.168.1.153 (NOT default 192.168.1.190)
- **Connection:** Ethernet to Jetson

### Architecture
Uses TCP bridge (port 9998) to bypass DDS cross-machine issues with custom message types.

```
Jetson: livox_lidar_node → livox_tcp_bridge.py --TCP:9998--> Mini PC: livox_tcp_receiver.py → /livox/pointcloud
```

### Start Commands

**Jetson - Terminal 1 (Livox Driver):**
```bash
source /opt/ros/humble/setup.bash
source /home/robot/robot_controller_release/ros2_packages/setup.bash
ros2 launch livox_lidar_node start_node.launch.py
```

**Jetson - Terminal 2 (TCP Bridge):**
```bash
source /opt/ros/humble/setup.bash
source /home/robot/robot_controller_release/ros2_packages/setup.bash
python3 /home/robot/livox_tcp_bridge.py
```

**Mini PC - Terminal 1 (Receiver):**
```bash
source /opt/ros/humble/setup.bash
python3 /home/robot/livox_tcp_receiver.py
```

**Mini PC - Terminal 2 (TF for alignment):**
```bash
source /opt/ros/humble/setup.bash
# LiDAR frame
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id livox_frame
# Camera frame (rotated to align with LiDAR)
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 --frame-id map --child-frame-id camera_depth_optical_frame
```

### Camera-LiDAR Alignment Transform

The camera optical frame needs rotation to align with LiDAR frame:
- **Quaternion:** `qx=-0.5, qy=0.5, qz=-0.5, qw=0.5`
- This applies: 90° CCW around Y axis + 90° CW roll

### ROS2 Topics on Mini PC

| Topic | Type | Description |
|-------|------|-------------|
| `/livox/pointcloud` | PointCloud2 | LiDAR data (~10 Hz, ~15k points) |
| `/camera/points` | PointCloud2 | Camera depth (~3 Hz) |
| `/camera/color/image_raw` | Image | Camera color |

### Key Files

| File | Location | Purpose |
|------|----------|---------|
| `livox_tcp_bridge.py` | Jetson | TCP sender |
| `livox_tcp_receiver.py` | Mini PC | TCP receiver |
| `lidar_parameters.json` | Jetson ros2_packages | LiDAR IP config |
| `LIDAR_SETUP_LOG.md` | Mini PC | Full documentation |

### Config File Location
`/home/robot/robot_controller_release/ros2_packages/livox_lidar_node/share/livox_lidar_node/config/lidar_parameters.json`

**Important:** LiDAR IP must be set to `192.168.1.153` (not default 192.168.1.190)

---

## XRDP Remote Desktop Setup (2026-01-28)

### Connection Methods

#### Option 1: WiFi (RECOMMENDED for robot operation)
| Setting | Value |
|---------|-------|
| **IP Address** | 192.168.0.91 |
| **Protocol** | RDP (port 3389) |
| **Username** | robot |

**Safe for robot operation** - uses separate WiFi network, does not interfere with motor control.

#### Option 2: Wired via Jetson (NOT RECOMMENDED)
| Setting | Value |
|---------|-------|
| **IP Address** | 192.168.1.20 (Jetson) |
| **Protocol** | RDP via SSH tunnel or direct |

**WARNING: CAUSES ROBOT CONTROL FAILURES!**
- RDP traffic competes with real-time UDP motor commands on same network
- Causes UDP packet loss/timeouts → watchdog triggers → safety damping mode
- Symptoms: Robot stands up then immediately goes limp with `COMM ERROR motor id 0` in logs

### Network Interfaces

| Interface | IP Address | Purpose |
|-----------|------------|---------|
| enp2s0 (Ethernet) | 192.168.1.10 | Robot control network (KEEP CLEAR) |
| wlo1 (WiFi) | 192.168.0.91 | Remote desktop access (USE THIS) |

### CRITICAL: Network Separation

The robot motor controller requires **dedicated bandwidth** on 192.168.1.x network:
- Motor commands: High-frequency UDP (~200Hz)
- Latency-sensitive: Timeouts trigger safety shutdown
- **DO NOT** route RDP, large file transfers, or streaming through wired network during robot operation

### Safe Remote Access During Robot Operation
1. Use WiFi RDP: `192.168.0.91`
2. Or use SSH (low bandwidth): `ssh robot@192.168.1.10`
3. Avoid wired RDP through Jetson

### Service Commands
```bash
# Check status
systemctl status xrdp xrdp-sesman

# Restart if needed
sudo systemctl restart xrdp xrdp-sesman
```

### Detailed Log
See: `/home/robot/robot_controller_release/XRDP_SETUP_LOG.md`

---

## WiFi Boot Configuration (2026-01-28)

### Overview
WiFi configured for "Client mode primary, AP secondary" - connects to external networks while also running access point.

### Boot Sequence

| Order | Service | Action |
|-------|---------|--------|
| 1 | `wifi-enable.service` (systemd) | Enables WiFi radio |
| 2 | NetworkManager | Auto-connects to saved WiFi |
| 3 | `start_ap.sh` (GNOME autostart) | Adds AP mode as secondary |

### WiFi Interface (wlo1) IPs

| Mode | IP Address | Network | Purpose |
|------|------------|---------|---------|
| Client | 192.168.0.91 | GennFlex | RDP access, internet |
| Access Point | 192.168.12.1 | YJ-MiniHexV2-152 | Direct robot connection |

### Access Point Details
- **SSID:** YJ-MiniHexV2-152
- **Password:** 1234abcd
- **Channel:** 11

### Key Files

| File | Purpose |
|------|---------|
| `/etc/systemd/system/wifi-enable.service` | Enables WiFi at boot |
| `/home/robot/.config/autostart/start_ap.sh.desktop` | GNOME autostart entry |
| `/home/robot/.config/autostart/scripts/start_ap.sh` | AP startup script |
| `/home/robot/.config/autostart/scripts/hostapd.conf` | hostapd configuration |

### Verification
```bash
# Check both IPs present
ip addr show wlo1 | grep inet

# Check client connection
nmcli connection show --active

# Check AP services
ps aux | grep -E "(hostapd|dhcpd)" | grep -v grep
```

### Detailed Log
See: `/home/robot/robot_controller_release/WIFI_BOOT_SETUP_LOG.md`

---

## One-Command Sensor Demo (2026-01-28)

### Overview
Single script that launches all sensors (RealSense + Livox LiDAR) with TCP bridges, TF transforms, and RViz visualization.

### Quick Start
```bash
bash /home/robot/start_sensor_demo.sh
```

Press `Ctrl+C` to stop all processes (includes automatic cleanup on Jetson).

### What It Starts

| Step | Component | Location |
|------|-----------|----------|
| 1 | RealSense depth TCP publisher | Jetson |
| 2 | Livox LiDAR driver | Jetson |
| 3 | Livox TCP bridge | Jetson |
| 4 | Depth bridge receiver | Mini PC |
| 5 | Livox TCP receiver | Mini PC |
| 6 | TF publishers (lidar + camera) | Mini PC |
| 7 | RViz with sensor config | Mini PC |

### Available Topics After Launch

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | Image | 640x480 BGR8 color |
| `/camera/depth/image_raw` | Image | 640x480 16UC1 depth (mm) |
| `/camera/points` | PointCloud2 | Camera XYZRGB pointcloud |
| `/livox/pointcloud` | PointCloud2 | LiDAR pointcloud |

### Key Files

| File | Purpose |
|------|---------|
| `/home/robot/start_sensor_demo.sh` | Main launch script |
| `/home/robot/robot_controller_release/sensor_visualization.rviz` | RViz config |

### Cleanup
The script automatically:
- Kills local background processes on exit
- SSHs to Jetson and kills remote processes (`realsense_depth_tcp_publisher.py`, `livox_tcp_bridge.py`, `livox_lidar_node`)

---

## Obstacle Avoidance Navigation (2026-02-04)

### Overview
Autonomous obstacle avoidance using RealSense depth camera and Livox LiDAR. Robot walks forward, slows down near obstacles, and turns to avoid them.

### Quick Start
```bash
bash /home/robot/start_obstacle_avoidance.sh
```

Press `Ctrl+C` to stop - robot will sit down safely.

### Configurable Parameters

Parameters can be set via environment variables:

```bash
STOP_DISTANCE=0.8 FORWARD_SPEED=0.3 bash /home/robot/start_obstacle_avoidance.sh
```

Or pass directly to Python script:

```bash
source /home/robot/robot_controller_release/ros2_packages/setup.bash
python3 /home/robot/obstacle_avoidance.py --stop-distance 0.8 --forward-speed 0.3
```

| Parameter | Flag | Default | Description |
|-----------|------|---------|-------------|
| Stop distance | `--stop-distance` | 1.2m | Distance to stop and turn |
| Slow distance | `--slow-distance` | 1.8m | Distance to slow down |
| Forward speed | `--forward-speed` | 0.5 m/s | Normal walking speed |
| Slow speed | `--slow-speed` | 0.24 m/s | Speed near obstacles |
| Turn speed | `--turn-speed` | 0.1 rad/s | Rotation speed |

### View All Options
```bash
python3 /home/robot/obstacle_avoidance.py --help
```

### Behavior
1. **Stand up** - transitions through states 1→2→3
2. **Walk forward** - at `forward-speed` when path is clear
3. **Slow down** - at `slow-speed` when obstacle within `slow-distance`
4. **Stop and turn** - when obstacle within `stop-distance`
5. **Reverse and retry** - if stuck for >5 seconds
6. **Sit down safely** - on Ctrl+C (states 3→1→0)

### Key Files

| File | Purpose |
|------|---------|
| `~/hexplorer/scripts/start_obstacle_avoidance.sh` | Launch script (starts sensors + navigation) |
| `~/hexplorer/navigation/obstacle_avoidance.py` | Main navigation node |

---

## Object Tracking System (2026-02-04)

### Overview
Multi-component object tracking system with color-based detection on Jetson, TCP streaming to Mini PC, and robot following capability.

### Architecture
```
Jetson (192.168.1.20)                    Mini PC (192.168.1.10)
=====================                    =======================

jetson_object_tracker.py                 detection_receiver.py
  - RealSense color detection              - TCP client (port 9997)
  - TCP server (port 9997)                 - Publishes /object_detection
  - Image stream (port 9996)                      |
         |                                        v
         +-------- TCP (57 bytes/msg) -----> object_follower.py
         +-------- TCP (images) -----------> tracking_rviz_visualizer.py
```

### Quick Start

```bash
# Full sensor demo with tracking (RViz)
bash ~/hexplorer/scripts/start_sensor_demo.sh

# Tracking only with RViz visualization
bash ~/hexplorer/scripts/start_object_tracking.sh --rviz

# Robot follows yellow object
bash ~/hexplorer/scripts/start_object_tracking.sh

# SMART MODE: Robot follows with obstacle avoidance + active search
bash ~/hexplorer/scripts/start_object_tracking.sh --smart

# Track different color
TARGET_COLOR=red bash ~/hexplorer/scripts/start_object_tracking.sh --rviz
```

### Supported Colors

| Color | HSV Range |
|-------|-----------|
| yellow | H: 20-40, S: 80+, V: 80+ |
| red | H: 0-10 or 170-180, S: 100+, V: 100+ |
| green | H: 35-85, S: 80+, V: 80+ |
| blue | H: 100-130, S: 80+, V: 80+ |

### ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/object_detection` | String | JSON detection data |
| `/object_position` | Point | x=pixel_x, y=pixel_y, z=distance_mm |
| `/object_tracking/marker` | Marker | 3D sphere at detected object |
| `/object_tracking/text` | Marker | Distance label |
| `/object_tracking/image` | Image | Camera with detection overlay |

### Detection Message Format (TCP)

57-byte binary message:
| Field | Type | Description |
|-------|------|-------------|
| detected | uint8 | Object found (0/1) |
| center_x | uint16 | Center X in pixels |
| center_y | uint16 | Center Y in pixels |
| bbox_x/y/w/h | uint16 | Bounding box |
| distance_mm | uint32 | Depth at center |
| confidence | float32 | Detection confidence |
| timestamp | uint32 | Unix timestamp |
| label | char[32] | Object class name |

### Key Files

| File | Purpose |
|------|---------|
| `~/hexplorer/tracking/jetson_object_tracker.py` | Runs on Jetson, color detection |
| `~/hexplorer/tracking/detection_receiver.py` | TCP client, ROS2 publisher |
| `~/hexplorer/tracking/object_follower.py` | Robot control to follow object |
| `~/hexplorer/tracking/tracking_rviz_visualizer.py` | RViz markers and overlay |
| `~/hexplorer/scripts/start_object_tracking.sh` | Launch script |

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `TARGET_COLOR` | yellow | Color to track |
| `TARGET_DISTANCE` | 800 | Follow distance (mm) |
| `MAX_SPEED` | 0.3 | Max forward speed (m/s) |
| `TURN_SPEED` | 0.15 | Turn speed (rad/s) |

### IMPORTANT: Camera Sharing Limitation

The RealSense camera can only be used by ONE process at a time:
- **Tracking mode**: `jetson_object_tracker.py` uses camera → no depth/pointcloud available
- **No-tracking mode**: `realsense_depth_tcp_publisher.py` uses camera → full depth + pointcloud

```bash
# With tracking (color + tracking overlay, NO depth)
bash ~/hexplorer/scripts/start_sensor_demo.sh

# Without tracking (full depth + pointcloud)
bash ~/hexplorer/scripts/start_sensor_demo.sh --no-track
```

---

## Smart Object Follower (2026-02-04)

### Overview
Enhanced object following with LiDAR-based obstacle avoidance and active search patterns. Combines tracking from `object_follower.py` with LiDAR sensing (always available during tracking).

### Quick Start
```bash
bash ~/hexplorer/scripts/start_object_tracking.sh --smart
```

### State Machine

| State | Description |
|-------|-------------|
| INIT | Stand up sequence |
| IDLE | Wait for target detection |
| FOLLOWING | Follow target, monitor obstacles |
| EVADE | Steer around obstacle to reach target |
| BLOCKED | Stop - obstacle blocks path to target |
| SEARCH | Active search when target lost |

### Decision Logic

| Target Visible | Obstacle Ahead | Target Direction | Action |
|----------------|----------------|------------------|--------|
| Yes | No | Any | FOLLOW normally |
| Yes | Yes | Same as obstacle | BLOCKED - stop, wait |
| Yes | Yes | Opposite side | EVADE - steer around |
| No | Any | N/A | SEARCH pattern |

### Active Search Pattern

| Phase | Time | Behavior |
|-------|------|----------|
| 1 | 0-3s | Turn toward last seen + walk forward |
| 2 | 3-8s | Zigzag - walk forward, alternate ±45° |
| 3 | 8-15s | Expanding spiral |
| 4 | >15s | Timeout - return to IDLE |

### Parameters

```bash
# Environment variables
TARGET_COLOR=yellow OBSTACLE_STOP=0.8 SEARCH_TIMEOUT=15 \
  bash ~/hexplorer/scripts/start_object_tracking.sh --smart
```

| Parameter | Flag | Default | Description |
|-----------|------|---------|-------------|
| Target distance | `--target-distance` | 800mm | Distance to maintain |
| Max speed | `--max-speed` | 0.3 m/s | Forward speed |
| Turn speed | `--turn-speed` | 0.15 rad/s | Angular velocity |
| Obstacle stop | `--obstacle-stop` | 0.8m | Stop if closer |
| Obstacle slow | `--obstacle-slow` | 1.2m | Slow if closer |
| Search timeout | `--search-timeout` | 15s | Give up after |
| Search speed | `--search-speed` | 0.1 m/s | Speed while searching |

### 360° Obstacle Avoidance

The smart follower uses **360-degree LiDAR detection** to prevent collisions in any direction:

```
                FRONT (0°)
               lidar_front
                    ↑
                    |
      LEFT (+90°)   |   RIGHT (-90°)
     lidar_left ←---+---→ lidar_right
                    |
                    ↓
                BACK (±180°)
               lidar_back
```

| Distance | Action |
|----------|--------|
| < 0.5m (EMERGENCY) | **STOP** + turn to safety |
| < 0.8m (STOP) | Stop motion in that direction |
| < 1.2m (SLOW) | Reduce speed proportionally |

Safety checks apply to:
- Forward motion → checks front
- Backward motion → checks back
- Strafe left → checks left
- Strafe right → checks right
- Very close in ANY direction → total stop

### Key Insight: Sensor Availability

| Sensor | During Tracking | Notes |
|--------|-----------------|-------|
| LiDAR (`/livox/pointcloud`) | ✅ Always | Separate Ethernet |
| Depth camera | ❌ Not available | Shared with tracker |
| Color camera | ✅ Via tracker | Detection + images |

LiDAR is sufficient for obstacle avoidance and preferred for reliability.

### Key Files

| File | Purpose |
|------|---------|
| `~/hexplorer/tracking/smart_follower.py` | Main smart follower node |
| `~/hexplorer/scripts/start_object_tracking.sh` | Launch with `--smart` flag |

---

## MOLA LiDAR Odometry System (2026-02-10) - RECOMMENDED

### Overview

**MOLA is the recommended SLAM/odometry system for this robot.**

LiDAR-only odometry using MOLA (Modular Object Localization and Mapping Architecture). Uses GICP (Generalized ICP) algorithms for scan matching. No IMU required.

**Why MOLA (not Fast-LIO):**
- LiDAR-only - avoids IMU drift problems
- The Livox Mid360 IMU has significant drift issues
- More reliable odometry for this robot
- Tuned ICP parameters for indoor environments

### Architecture

```
Jetson (192.168.1.20)                    Mini PC (192.168.1.10)
========================                  ========================

livox_lidar_node                         MOLA-SLAM
  └─/livox/lidar ───TCP:9998───►         FilterPass
      (livox_tcp_bridge.py)                └─/livox/lidar_filtered
                                                    ↓
                                              MOLA Mapping
                                                    ↓
                                         /tf (map→odom→base_link)
                                         /localmap (PointCloud2)
                                         /Odometry
```

### Quick Start

```bash
bash ~/start_mola_slam.sh
```

Options:
- `--no-gui` - Disable MOLA GUI
- `--no-rviz` - Disable RViz

Press `Ctrl+C` to stop all processes.

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/livox/lidar` | PointCloud2 | Raw LiDAR from TCP bridge |
| `/livox/lidar_filtered` | PointCloud2 | Filtered (input to MOLA) |
| `/state_estimator/pose` | Odometry | Robot pose estimate (USE THIS) |
| `/lidar_odometry/localmap_points` | PointCloud2 | Accumulated map |
| `/tf` | TF | map→odom→base_link transforms |

### TF Tree

```
map
 └── odom
      └── base_link
           └── livox_frame
```

### Key Files

| File | Location | Purpose |
|------|----------|---------|
| `start_mola_slam.sh` | ~/hexplorer/scripts/ | Main launch script |
| `livox_tcp_receiver.py` | ~/hexplorer/bridges/ | LiDAR TCP receiver |
| `mola_slam_launch.py` | MOLA-SLAM workspace | MOLA launch file |
| `mola_slam.rviz` | ~/hexplorer/config/ | RViz config |

### Workspace

**Location:** `/home/robot/MOLA-SLAM/mola_ws/`

**Source environment:**
```bash
source /opt/ros/humble/setup.bash
source ~/MOLA-SLAM/mola_ws/install/setup.bash
```

**Key packages:**
- `mola_bringup` - Launch files and utilities
- `mola_lidar_odometry` - Main LO algorithm
- `mp2p_icp` - ICP implementation
- `mrpt_ros_bridge` - MRPT/ROS2 bridge

### Map Operations

**Save map during operation:**
```bash
ros2 service call /map_save mola_msgs/srv/MapSave \
  "map_path: '/home/robot/mola_maps/my_map'"
```

**Files created:**
- `my_map.simplemap` - Keyframe data
- `my_map.mm` - Metric map
- `my_map.tum` - Trajectory

**Auto-save on shutdown:**
- `final_map.simplemap` in current directory
- `estimated_trajectory.tum` in current directory

### Localization Mode

To localize in an existing map:
```bash
ros2 launch mola_bringup mola_localize_launch.py
```
Then provide initial pose via the trajectory GUI.

### ICP Tuning Parameters

**Config file:** `~/MOLA-SLAM/mola_ws/install/mola_lidar_odometry/share/mola_lidar_odometry/pipelines/lidar3d-gicp-katana.yaml`

| Parameter | Value | Description |
|-----------|-------|-------------|
| `min_icp_goodness` | 0.92 | Min quality to accept match (0.85-0.95) |
| `maximum_sigma` | 0.8 | Max matching distance in meters |
| `maxIterations` | 50 | ICP iterations (30-80) |
| `robustKernelParam` | 4.0 | Outlier rejection (lower=stricter) |

**Edit and restart workflow:**
```bash
# Edit config
nano ~/MOLA-SLAM/mola_ws/install/mola_lidar_odometry/share/mola_lidar_odometry/pipelines/lidar3d-gicp-katana.yaml

# Restart MOLA
pkill -9 -f mola-cli
bash ~/hexplorer/scripts/start_mola_slam.sh
```

**Full tuning guide:** `~/hexplorer/docs/MOLA_SLAM_TUNING.md`

### Troubleshooting

**No points in /livox/lidar_filtered:**
- Check FilterPass node is running
- Verify /livox/lidar has data: `ros2 topic hz /livox/lidar`

**MOLA not starting:**
- Ensure workspace is sourced: `source ~/MOLA-SLAM/mola_ws/install/setup.bash`

**High CPU usage:**
- Reduce visualization: `--no-gui` or `--no-rviz`

**Map jumps/displacement:**
- Increase `min_icp_goodness` to 0.95
- Decrease `maximum_sigma` to 0.5
- Move robot slower

---

## Fast-LIO (LEGACY - DO NOT USE)

**Fast-LIO is NOT recommended for this robot.**

The Livox Mid360 IMU has significant drift issues that cause poor SLAM performance with Fast-LIO. Use MOLA LiDAR Odometry instead.

The script `~/hexplorer/scripts/start_fastlio.sh` exists but is marked as legacy and will show a warning if run.

---

## Hexplorer Software Organization (2026-02-04)

### Folder Structure

All custom software is organized in `~/hexplorer/`:

```
~/hexplorer/
├── sensors/          # Sensor publishers and viewers
│   ├── realsense_depth_tcp_publisher.py
│   ├── realsense_depth_publisher.py
│   └── camera_viewer.py
├── tracking/         # Object detection and following
│   ├── jetson_object_tracker.py
│   ├── detection_receiver.py
│   ├── object_follower.py
│   ├── smart_follower.py        # With obstacle avoidance + SLAM search
│   ├── tracking_rviz_visualizer.py
│   ├── tracking_visualizer.py
│   └── follow_white_box.py
├── navigation/       # Autonomous navigation
│   ├── obstacle_avoidance.py
│   ├── frontier_explorer.py     # SLAM frontier detection
│   └── human_follower.py
├── slam/             # SLAM system components
│   ├── odometry_publisher.py    # RobotState → /odom
│   └── config/
│       ├── slam_params.yaml     # slam_toolbox config
│       └── pc_to_scan.yaml      # pointcloud_to_laserscan config
├── bridges/          # TCP bridges for cross-machine comm
│   ├── depth_bridge_receiver.py
│   ├── livox_tcp_bridge.py
│   └── livox_tcp_receiver.py
├── config/           # RViz configurations
│   ├── sensor_visualization.rviz
│   └── tracking_visualization.rviz
├── docs/             # Setup logs and documentation
│   ├── HEXPLORER_CONTROL.md
│   ├── CAMERA_SETUP_LOG.md
│   ├── LIDAR_SETUP_LOG.md
│   └── ...
├── scripts/          # Launch scripts
│   ├── start_sensor_demo.sh
│   ├── start_object_tracking.sh
│   ├── start_obstacle_avoidance.sh
│   └── start_mola_slam.sh       # MOLA LiDAR Odometry (RECOMMENDED)
└── README.md         # Full documentation
```

### Convenience Symlinks

Symlinks in home directory point to hexplorer scripts:
```bash
~/start_sensor_demo.sh -> ~/hexplorer/scripts/start_sensor_demo.sh
~/start_object_tracking.sh -> ~/hexplorer/scripts/start_object_tracking.sh
~/start_obstacle_avoidance.sh -> ~/hexplorer/scripts/start_obstacle_avoidance.sh
~/start_mola_slam.sh -> ~/hexplorer/scripts/start_mola_slam.sh
```

### Quick Reference

```bash
# Full sensor demo with tracking
bash ~/hexplorer/scripts/start_sensor_demo.sh

# Sensors only (no tracking)
bash ~/hexplorer/scripts/start_sensor_demo.sh --no-track

# Object tracking with RViz
bash ~/hexplorer/scripts/start_object_tracking.sh --rviz

# Robot follows object
bash ~/hexplorer/scripts/start_object_tracking.sh

# Smart follower (obstacle avoidance + SLAM-based search)
bash ~/hexplorer/scripts/start_object_tracking.sh --smart

# MOLA LiDAR Odometry (RECOMMENDED for SLAM)
bash ~/hexplorer/scripts/start_mola_slam.sh

# Obstacle avoidance
bash ~/hexplorer/scripts/start_obstacle_avoidance.sh
```
