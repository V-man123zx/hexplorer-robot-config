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
| Livox Mid360 LiDAR | 192.168.1.153 | 3D scanning |

Jetson NFS share: `/.update_share_folder/nano/`

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

See: `/home/robot/hexplorer/docs/HEXPLORER_CONTROL.md`

---

## Sensor System (TCP Bridges)

### Overview

All sensors use TCP bridges to bypass DDS cross-machine issues. Launch everything with:
```bash
bash ~/hexplorer/scripts/start_sensor_demo.sh
```

### RealSense D435 Camera
- **Serial:** 406122070499, **Firmware:** 5.12.7.150
- Uses pyrealsense2 v2.55 (built from source; v2.56.4 has "RGB modules inconsistency" bug)
- **Fix:** `export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH` before launching
- Custom `realsense_camera_node` from INFFNI Robotics does NOT work (topics created but no data)
- TCP bridge on port 9999: `realsense_depth_tcp_publisher.py` (Jetson) → `depth_bridge_receiver.py` (Mini PC)

### Livox Mid360 LiDAR
- **IP:** 192.168.1.153 (NOT default 192.168.1.190)
- **Config:** `/home/robot/robot_controller_release/ros2_packages/livox_lidar_node/share/livox_lidar_node/config/lidar_parameters.json`
- LiDAR TCP bridge on port 9998: `livox_tcp_bridge.py` (Jetson) → `livox_tcp_receiver.py` (Mini PC)
- IMU TCP bridge on port 9995: `imu_tcp_bridge.py` (Jetson) → `imu_tcp_receiver.py` (Mini PC)

### Camera-LiDAR TF Alignment
- Camera quaternion: `qx=-0.5, qy=0.5, qz=-0.5, qw=0.5` (rotates optical frame to robot frame)

### Topics on Mini PC

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | Image | 640x480 BGR8 color |
| `/camera/depth/image_raw` | Image | 640x480 16UC1 depth (mm) |
| `/camera/points` | PointCloud2 | XYZRGB pointcloud |
| `/livox/lidar` | PointCloud2 | LiDAR (~10 Hz, ~15k points) |
| `/livox/imu` | Imu | IMU (~200 Hz, accel + gyro) |

### SSH to Jetson
```bash
sshpass -p "123" ssh robot@192.168.1.20
```

---

## XRDP Remote Desktop Setup (2026-01-28)

### Connection: WiFi ONLY (safe for robot operation)

| Setting | Value |
|---------|-------|
| **WiFi IP** | varies (check `ip addr show wlo1`) |
| **Protocol** | RDP (port 3389) |
| **Username** | robot |

**WARNING:** Do NOT use wired RDP through Jetson (192.168.1.x) during robot operation - RDP traffic causes UDP motor command loss → safety damping mode.

### Network Interfaces

| Interface | IP Address | Purpose |
|-----------|------------|---------|
| enp2s0 (Ethernet) | 192.168.1.10 | Robot control network (KEEP CLEAR) |
| wlo1 (WiFi) | varies | Remote desktop access (USE THIS) |

### Service Commands
```bash
systemctl status xrdp xrdp-sesman
sudo systemctl restart xrdp xrdp-sesman
```

---

## WiFi Boot Configuration (2026-01-28)

WiFi: "Client mode primary, AP secondary". Boot order: `wifi-enable.service` → NetworkManager → `start_ap.sh`.

| Mode | IP Address | Network | Purpose |
|------|------------|---------|---------|
| Client | varies | GennFlex | RDP access, internet |
| Access Point | 192.168.12.1 | YJ-MiniHexV2-152 | Direct robot connection |

AP password: `1234abcd`, channel 11.

---

## One-Command Sensor Demo (2026-02-10)

### Quick Start
```bash
bash ~/hexplorer/scripts/start_sensor_demo.sh           # With tracking
bash ~/hexplorer/scripts/start_sensor_demo.sh --slam     # With MOLA odometry
bash ~/hexplorer/scripts/start_sensor_demo.sh --no-track # Full depth, no tracking
```

Press `Ctrl+C` to stop (auto-cleans Jetson processes).

### Camera Sharing Limitation
RealSense can only be used by ONE process:
- **Tracking mode**: `jetson_object_tracker.py` uses camera → no depth/pointcloud
- **No-tracking mode**: `realsense_depth_tcp_publisher.py` uses camera → full depth + pointcloud

---

## Obstacle Avoidance Navigation (2026-02-04)

```bash
bash ~/hexplorer/scripts/start_obstacle_avoidance.sh
```

Parameters: `STOP_DISTANCE=0.8 FORWARD_SPEED=0.3 bash ~/hexplorer/scripts/start_obstacle_avoidance.sh`

Behavior: Stand up → Walk forward → Slow near obstacles → Stop and turn → Reverse if stuck → Sit down on Ctrl+C.

---

## Object Tracking System (2026-02-12)

### Detection Modes

| Mode | Model | FPS (TensorRT) | Use Case |
|------|-------|-----------------|----------|
| `yolo` (default) | YOLOv8n | ~63 | Fast tracking of 80 COCO classes |
| `yolo-world` | YOLOv8s-worldv2 | ~62 (PyTorch) | Detect any object by text description |
| `color` | HSV thresholds | ~30 | Simple color tracking |

### Quick Start

```bash
# YOLO — follow a person (default)
bash ~/hexplorer/scripts/start_object_tracking.sh

# YOLO — follow a specific object
TARGET=bottle bash ~/hexplorer/scripts/start_object_tracking.sh

# YOLO-World — detect by text description
DETECT_MODE=yolo-world TARGET="yellow ball" bash ~/hexplorer/scripts/start_object_tracking.sh

# Color mode (legacy)
DETECT_MODE=color TARGET=red bash ~/hexplorer/scripts/start_object_tracking.sh

# With RViz visualization
bash ~/hexplorer/scripts/start_object_tracking.sh --rviz

# Smart follower with obstacle avoidance
bash ~/hexplorer/scripts/start_object_tracking.sh --smart
```

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `DETECT_MODE` | yolo | Detection mode: `yolo`, `yolo-world`, or `color` |
| `TARGET` | person | What to detect (COCO class, text description, or color) |
| `TARGET_DISTANCE` | 800 | Follow distance (mm) |
| `MAX_SPEED` | 0.3 | Max forward speed (m/s) |
| `TURN_SPEED` | 0.8 | Turn speed (rad/s) |

### Key Files

| File | Purpose |
|------|---------|
| `~/hexplorer/tracking/jetson_object_tracker.py` | Runs on Jetson: YOLO, YOLO-World, or color detection |
| `~/hexplorer/tracking/detection_receiver.py` | TCP client, ROS2 publisher |
| `~/hexplorer/tracking/object_follower.py` | Robot control to follow object |
| `~/hexplorer/tracking/smart_follower.py` | Smart follower with MOLA + obstacle avoidance |
| `~/hexplorer/scripts/start_object_tracking.sh` | Launch script |

---

## Smart Object Follower (2026-02-10)

```bash
bash ~/hexplorer/scripts/start_object_tracking.sh --smart
```

### State Machine
INIT → IDLE → FOLLOWING / EVADE / BLOCKED / SEARCH

### 360° LiDAR Obstacle Avoidance
| Distance | Action |
|----------|--------|
| < 0.5m (EMERGENCY) | STOP + turn to safety |
| < 0.8m (STOP) | Stop motion in that direction |
| < 1.2m (SLOW) | Reduce speed proportionally |

### Search Pattern
Uses MOLA odometry to track visited areas on 0.5m grid. Navigates toward unvisited regions. No timeout - searches until target found.

### Sensor Availability During Tracking
- LiDAR: Always available
- Fast-LIO2 odometry: Always available (LiDAR+IMU fused)
- Depth camera: NOT available (shared with tracker)

---

## Fast-LIO2 Odometry System (2026-03-24) - RECOMMENDED

**Fast-LIO2 is the recommended odometry system.** LiDAR+IMU fused via EKF. Replaces MOLA (LiDAR-only, had scan matching glitches from hexapod gait oscillation).

### Quick Start
```bash
bash ~/hexplorer/scripts/start_fastlio.sh                 # Default (no RViz)
bash ~/hexplorer/scripts/start_fastlio.sh --rviz          # With RViz
```

### Workspace
```bash
source /opt/ros/humble/setup.bash
source ~/fastlio_ws/install/setup.bash
```

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/Odometry` | Odometry | Raw Fast-LIO2 output (camera_init→body) |
| `/lidar_odometry/pose` | Odometry | Remapped pose (odom→base_link) — USE THIS |
| `/cloud_registered` | PointCloud2 | Registered scan |
| `/Laser_map` | PointCloud2 | Accumulated map |
| `/path` | Path | Trajectory |
| `/tf` | TF | odom→base_link (dynamic, from odom_relay) |

### Config
`~/fastlio_ws/src/FAST_LIO/config/hexplorer_mid360.yaml`

### Key Architecture
- IMU TCP bridge runs on Jetson (port 9995), started by `jetson_services.sh`
- `odom_relay.py` remaps Fast-LIO2 frames and broadcasts TF
- No filterpass needed (Fast-LIO2 has built-in voxel filtering)

### Legacy MOLA
Still available at `~/hexplorer/scripts/start_mola_slam_legacy.sh` as fallback.

---

## Hexplorer Software Organization (2026-02-04)

All custom software in `~/hexplorer/`. See `~/hexplorer/README.md` for full component listing.

### Convenience Symlinks

```bash
~/start_sensor_demo.sh -> ~/hexplorer/scripts/start_sensor_demo.sh
~/start_object_tracking.sh -> ~/hexplorer/scripts/start_object_tracking.sh
~/start_obstacle_avoidance.sh -> ~/hexplorer/scripts/start_obstacle_avoidance.sh
~/start_slam.sh -> ~/hexplorer/scripts/start_fastlio.sh
~/start_fastlio.sh -> ~/hexplorer/scripts/start_fastlio.sh
```

### Quick Reference

```bash
bash ~/hexplorer/scripts/start_sensor_demo.sh             # Full sensor demo
bash ~/hexplorer/scripts/start_sensor_demo.sh --no-track   # No tracking
bash ~/hexplorer/scripts/start_object_tracking.sh --rviz   # Tracking + RViz
bash ~/hexplorer/scripts/start_object_tracking.sh          # Robot follows object
bash ~/hexplorer/scripts/start_object_tracking.sh --smart  # Smart follower
bash ~/hexplorer/scripts/start_object_search.sh            # Object search (UNTESTED)
bash ~/hexplorer/scripts/start_fastlio.sh                  # Fast-LIO2 odometry
bash ~/hexplorer/scripts/start_obstacle_avoidance.sh       # Obstacle avoidance
```

---

## Object Search System (2026-02-19) — Updated 2026-03-24

Standalone scan-navigate search program. Uses Fast-LIO2 for odometry (replaced MOLA). Built on proven patterns from `obstacle_avoidance.py` and `object_follower.py`.

### Quick Start
```bash
bash ~/hexplorer/scripts/start_object_search.sh                                    # Search for person
TARGET=bottle bash ~/hexplorer/scripts/start_object_search.sh                      # Search for bottle
DETECT_MODE=yolo-world TARGET="red toolbox" bash ~/hexplorer/scripts/start_object_search.sh
bash ~/hexplorer/scripts/start_object_search.sh --rviz                             # With RViz
bash ~/hexplorer/scripts/start_object_search.sh --no-approach                      # Confirm without approaching
```

### State Machine
```
STANDUP -> SCANNING (360 rotate) -> NAVIGATING (toward unvisited) -> SCANNING -> ...
               |                          |
             FOUND                      FOUND
               |
           APPROACH -> CONFIRMED -> SHUTDOWN
```

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `DETECT_MODE` | yolo | Detection mode: yolo, yolo-world, color |
| `TARGET` | person | What to search for |
| `SEARCH_SPEED` | 0.15 | Navigation speed (m/s) |
| `SCAN_SPEED` | 0.15 | Scan rotation speed (rad/s) |
| `NAVIGATE_DISTANCE` | 2.0 | Meters between scans |
| `STOP_DISTANCE` | 0.8 | Obstacle stop distance (m) |
| `CONFIRM_DISTANCE` | 1500 | Approach distance (mm) |

### Key Files

| File | Purpose |
|------|---------|
| `~/hexplorer/navigation/object_searcher.py` | Standalone search node (all code in one file) |
| `~/hexplorer/scripts/start_object_search.sh` | Launch script (MOLA + tracking infra) |
| `~/hexplorer/config/search_visualization.rviz` | RViz config |

### RViz Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/object_searcher/visited_grid` | OccupancyGrid | Visited=white, unvisited=grey |
| `/object_searcher/goal_marker` | Marker (Arrow) | Navigate direction |
| `/object_searcher/path_marker` | Marker (LINE_STRIP) | Search path traveled |
| `/object_searcher/scan_marker` | Marker (CYLINDER) | Current scan location |
| `/object_searcher/state` | String (JSON) | State for monitoring |

### Testing Status
Core search logic tested on 2026-02-25. Updated 2026-03-24 to use Fast-LIO2 (LiDAR+IMU) instead of MOLA. **Needs re-testing** with Fast-LIO2 odometry.
