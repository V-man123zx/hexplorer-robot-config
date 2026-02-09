#!/bin/bash
#
# MOLA-SLAM Launch Script for Hexplorer Robot
#
# This script starts all components needed for LiDAR-only SLAM using MOLA:
# 1. Static TF publishers (odom->base_link, base_link->livox_frame)
# 2. LiDAR driver on Jetson (via SSH)
# 3. LiDAR TCP bridge on Jetson (via SSH)
# 4. LiDAR TCP receiver on Mini PC (publishes to /livox/lidar)
# 5. Filterpass node (publishes to /livox/lidar_filtered)
# 6. MOLA LiDAR Odometry
# 7. RViz visualization (optional)
#
# Usage:
#   bash ~/hexplorer/scripts/start_mola_slam.sh [OPTIONS]
#
# Options:
#   --no-rviz   Disable RViz visualization
#   --no-gui    Disable MOLA GUI (default: disabled)
#
# Configuration:
#   Edit ICP parameters in:
#   ~/MOLA-SLAM/mola_ws/install/mola_lidar_odometry/share/mola_lidar_odometry/pipelines/lidar3d-gicp-katana.yaml
#
# Key Parameters:
#   min_icp_goodness: 0.92      - Minimum ICP quality to accept (0.85-0.95)
#   maximum_sigma: 0.8          - Max matching distance in meters (0.5-1.0)
#   maxIterations: 50           - ICP iterations (30-80)
#   robustKernelParam: 4.0      - Outlier rejection strength (3.0-6.0, lower=stricter)
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HEXPLORER_DIR="$(dirname "$SCRIPT_DIR")"
JETSON_IP="192.168.1.20"
JETSON_USER="robot"
JETSON_PASS="123"

# Parse command line arguments
USE_RVIZ="true"
USE_MOLA_GUI="false"
for arg in "$@"; do
    case $arg in
        --no-rviz)
            USE_RVIZ="false"
            shift
            ;;
        --gui)
            USE_MOLA_GUI="true"
            shift
            ;;
    esac
done

echo "========================================"
echo "  MOLA-SLAM for Hexplorer Robot"
echo "========================================"
echo "  RViz:     $USE_RVIZ"
echo "  MOLA GUI: $USE_MOLA_GUI"
echo "========================================"
echo ""
echo "Config file:"
echo "  ~/MOLA-SLAM/mola_ws/install/mola_lidar_odometry/share/"
echo "  mola_lidar_odometry/pipelines/lidar3d-gicp-katana.yaml"
echo ""

# Cleanup function
cleanup() {
    echo ""
    echo "Shutting down MOLA-SLAM..."

    # Kill local processes
    pkill -f "livox_tcp_receiver" 2>/dev/null || true
    pkill -9 -f "mola-cli" 2>/dev/null || true
    pkill -f "filterpass" 2>/dev/null || true
    pkill -f "static_transform_publisher" 2>/dev/null || true
    pkill -f "rviz2" 2>/dev/null || true

    # Kill Jetson processes
    echo "Cleaning up Jetson processes..."
    sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no "$JETSON_USER@$JETSON_IP" \
        "pkill -f livox_tcp_bridge.py 2>/dev/null; pkill -f livox_lidar_node 2>/dev/null" 2>/dev/null || true

    echo "Cleanup complete."
    exit 0
}

trap cleanup SIGINT SIGTERM

# Source ROS2 and MOLA
echo "[1/7] Setting up ROS2 and MOLA environment..."
source /opt/ros/humble/setup.bash
source ~/MOLA-SLAM/mola_ws/install/setup.bash

# Check if sshpass is available
if ! command -v sshpass &> /dev/null; then
    echo "ERROR: sshpass is required but not installed."
    echo "Install with: sudo apt install sshpass"
    exit 1
fi

# Start static TF publishers
echo "[2/7] Starting static TF publishers..."
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id odom --child-frame-id base_link &
sleep 0.5
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.2 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id base_link --child-frame-id livox_frame &
sleep 0.5

# Start LiDAR driver on Jetson
echo "[3/7] Starting Livox LiDAR driver on Jetson..."
sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no "$JETSON_USER@$JETSON_IP" \
    "source /opt/ros/humble/setup.bash && \
     source /home/robot/robot_controller_release/ros2_packages/setup.bash && \
     ros2 launch livox_lidar_node start_node.launch.py" &
sleep 4

# Start LiDAR TCP bridge on Jetson
echo "[4/7] Starting Livox TCP bridge on Jetson..."
sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no "$JETSON_USER@$JETSON_IP" \
    "source /opt/ros/humble/setup.bash && \
     source /home/robot/robot_controller_release/ros2_packages/setup.bash && \
     python3 /home/robot/livox_tcp_bridge.py" &
sleep 2

# Start LiDAR TCP receiver on Mini PC
echo "[5/7] Starting LiDAR TCP receiver..."
python3 ~/hexplorer/bridges/livox_tcp_receiver.py &
sleep 3

# Verify LiDAR topic
echo "Verifying /livox/lidar topic..."
timeout 5 ros2 topic hz /livox/lidar --window 3 2>/dev/null || echo "Warning: Waiting for /livox/lidar..."

# Start filterpass node
echo "[6/7] Starting filterpass node..."
python3 ~/MOLA-SLAM/mola_ws/install/mola_bringup/lib/mola_bringup/filterpass.py &
sleep 2

# Start RViz if enabled
if [ "$USE_RVIZ" = "true" ]; then
    echo "Starting RViz..."
    rviz2 -d "$HEXPLORER_DIR/config/mola_slam.rviz" 2>/dev/null &
    sleep 2
fi

# Start MOLA SLAM
echo "[7/7] Starting MOLA LiDAR Odometry..."
echo ""
echo "  Input topic:  /livox/lidar_filtered"
echo "  Output topic: /state_estimator/pose"
echo ""

export MOLA_LIDAR_TOPIC=/livox/lidar_filtered
export MOLA_USE_FIXED_LIDAR_POSE=true
export MOLA_WITH_GUI=$USE_MOLA_GUI

ros2 launch mola_lidar_odometry ros2-lidar-odometry-katana.launch.py \
    lidar_topic_name:=/livox/lidar_filtered \
    ignore_lidar_pose_from_tf:=true \
    use_rviz:=false \
    use_mola_gui:=$USE_MOLA_GUI

# Wait for user interrupt
wait
