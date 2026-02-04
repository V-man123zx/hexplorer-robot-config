# RealSense Camera Setup Log

## Date: 2026-01-27

## Summary
Attempted to set up live video feed from RealSense D435 camera on Jetson to Mini PC.

## Network Architecture
| Device | IP | Role |
|--------|-----|------|
| Intel Mini PC | 192.168.1.10 | Robot controller, visualization |
| Jetson Orin Nano | 192.168.1.20 | Camera/sensor processing |
| Livox Mid360 LiDAR | 192.168.1.190 | 3D scanning |

## Issues Discovered

### 1. Image Message Bug (FIXED)
**Problem:** The realsense_camera_node publishes images with incorrect `step` field.
- Published: `step: 921600` (total image size)
- Should be: `step: 1920` (width × 3 channels)

**Impact:** rqt_image_view shows gray gradient instead of actual image.

**Solution:** Created custom viewer (`simple_viewer.py`) that manually decodes BGR8 data ignoring the step field:
```python
data = np.frombuffer(msg.data, dtype=np.uint8)
img = data[:msg.height*msg.width*3].reshape((msg.height, msg.width, 3))
```

### 2. ROS2 DDS Cross-Machine Data Transfer (PARTIALLY RESOLVED)
**Problem:** ROS2 topic discovery works between Mini PC and Jetson, but actual data transfer fails for large messages (images).
- `ros2 topic list` shows camera topics from Jetson
- `ros2 topic info` shows Publisher count: 1
- `ros2 topic hz` times out (no data received)
- Small messages (like robot_state) transfer fine

**Root Cause:** FastDDS default configuration doesn't reliably transfer large UDP messages across network.

**Attempted Solutions:**
1. FastDDS peer discovery XML config - helped with discovery but not data
2. QoS adjustments (BEST_EFFORT vs RELIABLE) - no improvement

**Workaround:** NFS-based frame relay:
- Jetson runs `camera_relay.py` to save frames to shared filesystem
- Mini PC runs viewer to display frames from shared path

### 3. Camera Hardware Issue (BLOCKING)
**Problem:** Camera node crashes with `"failed to set power state"` error.

**Error Message:**
```
terminate called after throwing an instance of 'rs2::error'
  what():  failed to set power state
```

**Cause:** RealSense USB/power issue on Jetson. Could be:
- USB power insufficient
- Camera needs physical reset
- Multiple processes trying to access camera

**Solution:** May need to:
1. Physically reconnect the camera USB
2. Power cycle the Jetson
3. Kill any lingering camera processes

## Files Created

### On Mini PC (`/home/robot/robot_controller_release/`):
| File | Purpose |
|------|---------|
| `camera_viewer.py` | Original viewer (doesn't open window) |
| `camera_viewer_fixed.py` | Fixed BGR8 decoder |
| `camera_viewer_network.py` | Network-optimized with BEST_EFFORT QoS |
| `simple_viewer.py` | Working viewer with threading |
| `sensor_status.py` | Check sensor connectivity and topics |
| `start_sensors.sh` | Start LiDAR node |
| `setup_ros2_network.sh` | Configure FastDDS for network |
| `fastdds_config.xml` | FastDDS peer discovery config |
| `sensor_visualization.rviz` | RViz config for sensors |
| `view_nfs_camera.py` | View frames from NFS share |
| `fetch_camera.sh` | SCP-based frame fetcher |
| `view_remote_camera.py` | View SCP-fetched frames |

### On Jetson (via NFS at `/.update_share_folder/nano/robot_controller_release/`):
| File | Purpose |
|------|---------|
| `camera_relay.py` | Save camera frames to filesystem |
| `start_camera.sh` | Complete camera startup script |
| `setup_camera.sh` | Simplified ROS2 setup |
| `fastdds_config.xml` | FastDDS config for Jetson |

## Configuration Changes

### Camera Point Cloud Enabled
File: `/.update_share_folder/nano/robot_controller_release/ros2_packages/realsense_camera_node/share/realsense_camera_node/config/realsense_camera_node_parameters_setting.yaml`
```yaml
is_publish_pointcloud: true  # was false
```

## What Works

1. ✅ Camera node starts and detects camera on Jetson
2. ✅ Camera topics are published (when node running)
3. ✅ Topic discovery works across network
4. ✅ Single frame can be captured and displayed
5. ✅ BGR8 decoding fix shows correct image (not gray gradient)
6. ✅ SSH access to Jetson (password: 123)

## What Doesn't Work

1. ❌ Continuous video stream over ROS2 network (DDS large message issue)
2. ❌ Camera node stability (USB power state errors)
3. ❌ NFS bidirectional sync (only Mini PC → Jetson works)

## Critical Bug Found (2026-01-27)

The custom `realsense_camera_node` package has a **publishing bug**:
- Camera hardware works (verified with `rs-enumerate-devices`)
- Node starts and reports "Camera Pipeline Success!"
- ROS2 topics are created
- **BUT no data is ever published to the topics**

### Evidence:
1. `ros2 topic info` shows Publisher count: 1
2. `ros2 topic echo` times out with no data
3. Python subscribers receive 0 frames
4. System monitor topic doesn't trigger publishing

### Root Cause:
The custom node's publishing logic is broken. It's proprietary code from INFFNI Robotics.

### Solutions:
1. **Replace with official realsense-ros package** (recommended)
2. **Contact INFFNI Robotics** for a fix
3. **Use X11 forwarding** to run realsense-viewer directly on Jetson

## Recommended Next Steps

1. **Install official RealSense ROS2 package:**
   ```bash
   sudo apt install ros-humble-realsense2-camera
   ```

2. **Or contact vendor** about the broken custom node

2. **For reliable video streaming:**
   - Option A: Run viewer directly on Jetson with X11 forwarding
   - Option B: Use SCP-based frame relay (`fetch_camera.sh` + `view_remote_camera.py`)
   - Option C: Install CycloneDDS which handles large messages better

3. **Commands to start camera (when hardware is fixed):**
   ```bash
   # On Jetson
   source /opt/ros/humble/setup.bash
   source ~/robot_controller_release/ros2_packages/local_setup.bash
   ros2 launch realsense_camera_node start_node.launch.py

   # On Mini PC (simple viewer)
   source /home/robot/robot_controller_release/ros2_packages/setup.bash
   python3 /home/robot/robot_controller_release/simple_viewer.py
   ```

## SSH Access
```bash
sshpass -p "123" ssh robot@192.168.1.20
```
