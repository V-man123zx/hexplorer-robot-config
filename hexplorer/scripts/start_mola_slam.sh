#!/bin/bash
#
# MOLA-SLAM Launch Script for Hexplorer Robot
#
# Ensures Jetson LiDAR services are running (doesn't duplicate),
# then starts local MOLA pipeline + RViz.
#
# Usage:
#   bash ~/hexplorer/scripts/start_mola_slam.sh [OPTIONS]
#
# Options:
#   --no-rviz   Disable RViz visualization
#   --gui       Enable MOLA GUI
#
# Config:
#   ~/MOLA-SLAM/mola_ws/install/mola_lidar_odometry/share/
#   mola_lidar_odometry/pipelines/lidar3d-katana.yaml
#

set -e

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
HEXPLORER_DIR="$(dirname "$SCRIPT_DIR")"
source "$SCRIPT_DIR/common.sh"

# Parse arguments
USE_RVIZ="true"
USE_MOLA_GUI="False"
for arg in "$@"; do
    case $arg in
        --no-rviz) USE_RVIZ="false" ;;
        --gui) USE_MOLA_GUI="True" ;;
    esac
done

# Kill stale LOCAL processes only (Jetson LiDAR stays running)
echo "Cleaning up stale local processes..."
pkill -9 -f "mola-cli" 2>/dev/null || true
pkill -9 -f "filterpass" 2>/dev/null || true
pkill -9 -f "livox_tcp_receiver" 2>/dev/null || true
pkill -9 -f "static_transform_publisher" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
sleep 1

echo "========================================"
echo "  MOLA-SLAM for Hexplorer Robot"
echo "========================================"
echo "  RViz:     $USE_RVIZ"
echo "  MOLA GUI: $USE_MOLA_GUI"
echo "========================================"
echo ""

declare -a PIDS=()

cleanup() {
    echo ""
    echo "Shutting down MOLA-SLAM..."
    for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null || true; done
    pkill -9 -f "mola-cli" 2>/dev/null || true
    pkill -9 -f "filterpass" 2>/dev/null || true
    pkill -9 -f "static_transform_publisher" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    # NOTE: Jetson LiDAR services left running (persistent)
    echo "Cleanup complete."
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# Source ROS2 and MOLA
source /opt/ros/humble/setup.bash
source ~/MOLA-SLAM/mola_ws/install/setup.bash

# Ensure Jetson LiDAR services are running
echo "[1/7] Ensuring Jetson LiDAR services..."
ensure_jetson_lidar

# Start static TF publishers
echo "[2/7] Starting static TF publishers..."
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id odom --child-frame-id base_link &
PIDS+=($!)
sleep 0.3
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.2 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id base_link --child-frame-id livox_frame &
PIDS+=($!)
sleep 0.5

# Start LiDAR TCP receiver
echo "[3/7] Starting LiDAR TCP receiver..."
ensure_local "livox_tcp_receiver" "python3 $HEXPLORER_DIR/bridges/livox_tcp_receiver.py"
sleep 3

# Verify LiDAR topic
echo "[4/7] Verifying /livox/lidar topic..."
timeout 5 ros2 topic hz /livox/lidar --window 3 2>/dev/null || echo "  Warning: Waiting for /livox/lidar..."

# Start filterpass node
echo "[5/7] Starting filterpass node..."
ensure_local "filterpass" "python3 ~/MOLA-SLAM/mola_ws/install/mola_bringup/lib/mola_bringup/filterpass.py"
sleep 2

# Start RViz if enabled
if [ "$USE_RVIZ" = "true" ]; then
    echo "[6/7] Starting RViz..."
    rviz2 -d "$HEXPLORER_DIR/config/mola_slam.rviz" 2>/dev/null &
    PIDS+=($!)
    sleep 2
else
    echo "[6/7] Skipping RViz (disabled)..."
fi

# Start MOLA SLAM
echo "[7/7] Starting MOLA LiDAR Odometry..."
echo ""
echo "  Input topic:  /livox/lidar_filtered"
echo "  Output topic: /lidar_odometry/pose"
echo ""

ros2 launch mola_lidar_odometry ros2-lidar-odometry-katana.launch.py \
    lidar_topic_name:=/livox/lidar_filtered \
    ignore_lidar_pose_from_tf:=true \
    use_rviz:=false \
    use_mola_gui:=$USE_MOLA_GUI \
    use_state_estimator:=False \
    mola_lo_pipeline:=../pipelines/lidar3d-katana.yaml

# Wait for user interrupt
wait
