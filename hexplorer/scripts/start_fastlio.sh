#!/bin/bash
#
# Fast-LIO2 Launch Script for Hexplorer Robot
#
# LiDAR+IMU fused odometry via EKF. Replaces MOLA LiDAR-only odometry.
#
# Usage:
#   bash ~/hexplorer/scripts/start_fastlio.sh [OPTIONS]
#
# Options:
#   --no-rviz   Disable RViz visualization (default: no RViz)
#   --rviz      Enable RViz visualization
#

set -e

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
HEXPLORER_DIR="$(dirname "$SCRIPT_DIR")"
source "$SCRIPT_DIR/common.sh"

# Parse arguments
USE_RVIZ="false"
for arg in "$@"; do
    case $arg in
        --rviz) USE_RVIZ="true" ;;
        --no-rviz) USE_RVIZ="false" ;;
    esac
done

# Kill stale LOCAL processes only (Jetson LiDAR stays running)
echo "Cleaning up stale local processes..."
pkill -9 -f "fastlio_mapping" 2>/dev/null || true
pkill -9 -f "odom_relay" 2>/dev/null || true
pkill -9 -f "livox_tcp_receiver" 2>/dev/null || true
pkill -9 -f "imu_tcp_receiver" 2>/dev/null || true
pkill -9 -f "static_transform_publisher" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
sleep 1

echo "========================================"
echo "  Fast-LIO2 for Hexplorer Robot"
echo "========================================"
echo "  RViz: $USE_RVIZ"
echo "========================================"
echo ""

declare -a PIDS=()

cleanup() {
    echo ""
    echo "Shutting down Fast-LIO2..."
    for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null || true; done
    pkill -9 -f "fastlio_mapping" 2>/dev/null || true
    pkill -9 -f "odom_relay" 2>/dev/null || true
    pkill -9 -f "static_transform_publisher" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    # NOTE: Jetson services left running (persistent)
    echo "Cleanup complete."
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# Source ROS2 and Fast-LIO2
source /opt/ros/humble/setup.bash
source /home/robot/robot_controller_release/ros2_packages/setup.bash
source ~/fastlio_ws/install/setup.bash

# Ensure Jetson LiDAR + IMU services are running
echo "[1/7] Ensuring Jetson LiDAR + IMU services..."
ensure_jetson_lidar

# Start static TFs
echo "[2/7] Starting static TF publishers..."
# camera_init = odom (both are Fast-LIO2's world origin)
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id odom --child-frame-id camera_init &
PIDS+=($!)
# base_link -> livox_frame (LiDAR mount position)
ros2 run tf2_ros static_transform_publisher --x 0.3 --y 0 --z 0.2 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id base_link --child-frame-id livox_frame &
PIDS+=($!)
sleep 0.5

# Start LiDAR TCP receiver
echo "[3/7] Starting LiDAR TCP receiver..."
ensure_local "livox_tcp_receiver" "python3 $HEXPLORER_DIR/bridges/livox_tcp_receiver.py"
sleep 2

# Start IMU TCP receiver
echo "[4/7] Starting IMU TCP receiver..."
ensure_local "imu_tcp_receiver" "python3 $HEXPLORER_DIR/bridges/imu_tcp_receiver.py"
sleep 2

# Verify topics
echo "[5/7] Verifying topics..."
timeout 5 ros2 topic hz /livox/lidar --window 3 2>/dev/null || echo "  Warning: Waiting for /livox/lidar..."
timeout 5 ros2 topic hz /livox/imu --window 3 2>/dev/null || echo "  Warning: Waiting for /livox/imu..."

# Start RViz if enabled
if [ "$USE_RVIZ" = "true" ]; then
    echo "[6/7] Starting RViz..."
    rviz2 -d "$HEXPLORER_DIR/config/fastlio.rviz" 2>/dev/null &
    PIDS+=($!)
    sleep 2
else
    echo "[6/7] Skipping RViz (disabled)..."
fi

# Start odom relay (Fast-LIO2 /Odometry -> /lidar_odometry/pose + TF)
echo "[7/7] Starting Fast-LIO2 + odom relay..."
python3 "$HEXPLORER_DIR/bridges/odom_relay.py" &
PIDS+=($!)
sleep 0.5

echo ""
echo "  Input topics:  /livox/lidar, /livox/imu"
echo "  Output topic:  /lidar_odometry/pose"
echo "  TF:            odom -> base_link"
echo ""

# Launch Fast-LIO2 (foreground — blocks until Ctrl+C)
ros2 launch fast_lio mapping.launch.py \
    config_file:=hexplorer_mid360.yaml \
    rviz:=false

# Wait for user interrupt
wait
