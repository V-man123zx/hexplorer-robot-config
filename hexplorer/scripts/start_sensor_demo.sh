#!/bin/bash
# Combined script to launch full RViz sensor demo
# Starts: RealSense camera, Livox LiDAR, Object Tracking, TCP bridges, TF, and RViz
#
# Usage:
#   bash start_sensor_demo.sh              # All sensors + tracking (yellow)
#   bash start_sensor_demo.sh --no-track   # Sensors only, no object tracking
#
# Environment variables:
#   TARGET_COLOR=red    # Color to track (yellow, red, green, blue)

set -e

# Resolve symlinks to get actual script location
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
HEXPLORER_DIR="$(dirname "$SCRIPT_DIR")"
JETSON_IP="192.168.1.20"
JETSON_PASS="123"

# Parse arguments
ENABLE_TRACKING=true
for arg in "$@"; do
    case $arg in
        --no-track)
            ENABLE_TRACKING=false
            shift
            ;;
    esac
done

# Tracking color (default: yellow)
TARGET_COLOR="${TARGET_COLOR:-yellow}"

# Cleanup function
cleanup() {
    echo ""
    echo "Shutting down sensor demo..."

    # Kill local background processes
    kill $PID_DEPTH_RECEIVER $PID_LIVOX_RECEIVER $PID_TF_LIDAR $PID_TF_CAMERA $PID_DETECTION_RECEIVER $PID_TRACKING_VIZ $PID_TF_COLOR 2>/dev/null || true

    # Kill remote processes on Jetson
    sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no robot@$JETSON_IP "pkill -f realsense_depth_tcp_publisher.py; pkill -f livox_tcp_bridge.py; pkill -f livox_lidar_node; pkill -f jetson_object_tracker.py" 2>/dev/null || true

    echo "Cleanup complete."
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "=== Hexplorer Sensor Demo ==="
if [ "$ENABLE_TRACKING" = true ]; then
    echo "  Object Tracking: ENABLED (color: $TARGET_COLOR)"
else
    echo "  Object Tracking: DISABLED"
fi
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash
source /home/robot/robot_controller_release/ros2_packages/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Sync tracking script to Jetson if tracking enabled
if [ "$ENABLE_TRACKING" = true ]; then
    echo "[0/10] Syncing tracking script to Jetson..."
    sshpass -p "$JETSON_PASS" scp -o StrictHostKeyChecking=no \
        "$HEXPLORER_DIR/tracking/jetson_object_tracker.py" \
        robot@$JETSON_IP:/home/robot/jetson_object_tracker.py 2>/dev/null
fi

# Only start depth publisher if tracking is disabled (they share the camera)
if [ "$ENABLE_TRACKING" = false ]; then
    echo "[1/10] Starting RealSense depth publisher on Jetson..."
    sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no robot@$JETSON_IP "
        source /opt/ros/humble/setup.bash
        export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH
        export PYTHONPATH=/usr/local/lib/python3.10/dist-packages:\$PYTHONPATH
        nohup python3 /home/robot/realsense_depth_tcp_publisher.py > /tmp/realsense.log 2>&1 &
    " &
    sleep 2
else
    echo "[1/10] Skipping depth publisher (tracking uses camera)..."
fi

echo "[2/10] Starting Livox LiDAR driver on Jetson..."
sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no robot@$JETSON_IP "
    source /opt/ros/humble/setup.bash
    source /home/robot/robot_controller_release/ros2_packages/setup.bash
    nohup ros2 launch livox_lidar_node start_node.launch.py > /tmp/livox_driver.log 2>&1 &
" &
sleep 3

echo "[3/10] Starting Livox TCP bridge on Jetson..."
sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no robot@$JETSON_IP "
    source /opt/ros/humble/setup.bash
    source /home/robot/robot_controller_release/ros2_packages/setup.bash
    nohup python3 /home/robot/livox_tcp_bridge.py > /tmp/livox_bridge.log 2>&1 &
" &
sleep 2

# Start object tracker on Jetson if tracking enabled
if [ "$ENABLE_TRACKING" = true ]; then
    echo "[4/10] Starting object tracker on Jetson..."
    sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no robot@$JETSON_IP "
        source /opt/ros/humble/setup.bash
        export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH
        export PYTHONPATH=/usr/local/lib/python3.10/dist-packages:\$PYTHONPATH
        nohup python3 /home/robot/jetson_object_tracker.py --mode color --target $TARGET_COLOR --stream-images > /tmp/tracker.log 2>&1 &
    " &
    sleep 3
else
    echo "[4/10] Skipping object tracker (disabled)..."
fi

# Only start depth receiver if tracking is disabled
if [ "$ENABLE_TRACKING" = false ]; then
    echo "[5/10] Starting depth bridge receiver (local)..."
    python3 "$HEXPLORER_DIR/bridges/depth_bridge_receiver.py" &
    PID_DEPTH_RECEIVER=$!
    sleep 1
else
    echo "[5/10] Skipping depth receiver (tracking provides images)..."
    PID_DEPTH_RECEIVER=""
fi

echo "[6/10] Starting Livox TCP receiver (local)..."
python3 "$HEXPLORER_DIR/bridges/livox_tcp_receiver.py" &
PID_LIVOX_RECEIVER=$!
sleep 1

# Start detection receiver if tracking enabled
if [ "$ENABLE_TRACKING" = true ]; then
    echo "[7/10] Starting detection receiver (local)..."
    python3 "$HEXPLORER_DIR/tracking/detection_receiver.py" --with-images &
    PID_DETECTION_RECEIVER=$!
    sleep 1
else
    echo "[7/10] Skipping detection receiver (tracking disabled)..."
    PID_DETECTION_RECEIVER=""
fi

echo "[8/10] Starting TF publishers..."
# LiDAR frame
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id livox_frame &
PID_TF_LIDAR=$!

# Camera depth frame (rotated to align with LiDAR)
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 --frame-id map --child-frame-id camera_depth_optical_frame &
PID_TF_CAMERA=$!

# Camera color frame (for tracking markers)
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 --frame-id map --child-frame-id camera_color_optical_frame &
PID_TF_COLOR=$!
sleep 1

# Start tracking visualizer if tracking enabled
if [ "$ENABLE_TRACKING" = true ]; then
    echo "[9/10] Starting tracking RViz visualizer..."
    python3 "$HEXPLORER_DIR/tracking/tracking_rviz_visualizer.py" &
    PID_TRACKING_VIZ=$!
    sleep 1
else
    echo "[9/10] Skipping tracking visualizer (disabled)..."
    PID_TRACKING_VIZ=""
fi

echo "[10/10] Launching RViz with sensor config..."
echo ""
echo "=== Sensor Demo Running ==="
echo "Topics available:"
echo "  /livox/pointcloud          - LiDAR pointcloud"
if [ "$ENABLE_TRACKING" = true ]; then
echo "  /camera/color/image_raw    - Color image (from tracker)"
echo "  /object_detection          - Object detection JSON"
echo "  /object_tracking/marker    - 3D tracking marker"
echo "  /object_tracking/image     - Tracking overlay image"
echo ""
echo "NOTE: Depth/pointcloud disabled when tracking (camera shared)"
else
echo "  /camera/color/image_raw    - Color image"
echo "  /camera/depth/image_raw    - Depth image"
echo "  /camera/points             - Camera pointcloud"
fi
echo ""
echo "Press Ctrl+C to stop all processes."
echo ""

# Run RViz with saved config
rviz2 -d "$HEXPLORER_DIR/config/sensor_visualization.rviz"

# If RViz exits, cleanup
cleanup
