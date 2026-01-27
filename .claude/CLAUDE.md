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

### Connection Details

| Setting | Value |
|---------|-------|
| **WiFi IP Address** | 172.16.151.110 |
| **Protocol** | RDP (port 3389) |
| **Username** | robot |

### How to Connect from Windows

1. Open **Remote Desktop Connection** (mstsc.exe)
2. Enter `172.16.151.110`
3. Click Connect
4. Login with username `robot` and password

### Network Interfaces

| Interface | IP Address | Purpose |
|-----------|------------|---------|
| enp2s0 (Ethernet) | 192.168.1.10 | Robot control network |
| wlo1 (WiFi) | 172.16.151.110 | Remote desktop access |

### Project Impact

RDP over WiFi does **not** interfere with robot operations:
- Robot traffic uses **wired ethernet** (192.168.1.x)
- RDP uses **WiFi** (172.16.151.x)
- Separate network paths

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
| Client | 172.16.151.110 | GennFlex | RDP access, internet |
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
