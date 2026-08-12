# RealSense Camera Setup

## Camera Info
- **Model:** Intel RealSense D435
- **Serial:** 406122070499
- **Firmware:** 5.12.7.150
- **Connection:** USB to Jetson Orin Nano

## Known Issues

### Custom `realsense_camera_node` (INFFNI Robotics) - DO NOT USE
- Topics are created but **no data is published**
- Image `step` field is incorrect (sends total size instead of row stride)
- This is proprietary code with no fix available

### librealsense Version Bug
- `ros-humble-realsense2-camera` installs librealsense 2.56.4
- librealsense 2.56.4 has "RGB modules inconsistency" bug with D435
- **Fix:** Use librealsense 2.55 built from source on Jetson
- `export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH` before launching

### ROS2 DDS Large Message Transfer
- FastDDS and CycloneDDS both fail for large messages (images) across machines
- **Solution:** TCP bridge system (see DEPTH_BRIDGE_SETUP_LOG.md)

## Working Solution

Use the TCP bridge system documented in `DEPTH_BRIDGE_SETUP_LOG.md`. All sensor startup is handled by:
```bash
bash ~/hexplorer/scripts/start_sensor_demo.sh
```

## SSH to Jetson
```bash
source ~/hexplorer/.env
sshpass -p "$JETSON_PASS" ssh "$JETSON_USER@$JETSON_IP"
```
