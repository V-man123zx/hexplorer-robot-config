#!/bin/bash
#
# Start obstacle avoidance demo
# This script starts all sensors and the obstacle avoidance navigation
#
# Environment variables:
#   STOP_DISTANCE=0.8    # Distance to stop/turn (m)
#   FORWARD_SPEED=0.3    # Normal forward speed (m/s)
#

set -e

# Resolve symlinks to get actual script location
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
HEXPLORER_DIR="$(dirname "$SCRIPT_DIR")"
JETSON_IP="192.168.1.20"
JETSON_USER="robot"
JETSON_PASS="123"

echo "========================================="
echo "  Hexplorer Obstacle Avoidance Demo"
echo "========================================="
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash
source /home/robot/robot_controller_release/ros2_packages/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Array to track background PIDs
declare -a PIDS=()

cleanup() {
    echo ""
    echo "Cleaning up..."

    # Kill local processes
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done

    # Kill processes on Jetson
    echo "Stopping Jetson processes..."
    sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no "$JETSON_USER@$JETSON_IP" \
        "pkill -f 'realsense_depth_tcp_publisher.py' 2>/dev/null; \
         pkill -f 'livox_tcp_bridge.py' 2>/dev/null; \
         pkill -f 'livox_lidar_node' 2>/dev/null" 2>/dev/null || true

    echo "Cleanup complete"
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# Check Jetson connectivity
echo "[1/6] Checking Jetson connectivity..."
if ! sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 "$JETSON_USER@$JETSON_IP" "echo ok" >/dev/null 2>&1; then
    echo "ERROR: Cannot connect to Jetson at $JETSON_IP"
    exit 1
fi
echo "  Jetson connected"

# Start RealSense depth TCP publisher on Jetson
echo "[2/6] Starting RealSense depth camera on Jetson..."
sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no "$JETSON_USER@$JETSON_IP" \
    "source /opt/ros/humble/setup.bash && \
     export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH && \
     export PYTHONPATH=/usr/local/lib/python3.10/dist-packages:\$PYTHONPATH && \
     python3 /home/robot/realsense_depth_tcp_publisher.py" &
PIDS+=($!)
sleep 3

# Start Livox LiDAR on Jetson
echo "[3/6] Starting Livox LiDAR driver on Jetson..."
sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no "$JETSON_USER@$JETSON_IP" \
    "source /opt/ros/humble/setup.bash && \
     source /home/robot/robot_controller_release/ros2_packages/setup.bash && \
     ros2 launch livox_lidar_node start_node.launch.py" &
PIDS+=($!)
sleep 2

# Start Livox TCP bridge on Jetson
echo "[4/6] Starting Livox TCP bridge on Jetson..."
sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no "$JETSON_USER@$JETSON_IP" \
    "source /opt/ros/humble/setup.bash && \
     source /home/robot/robot_controller_release/ros2_packages/setup.bash && \
     python3 /home/robot/livox_tcp_bridge.py" &
PIDS+=($!)
sleep 2

# Start depth bridge receiver on Mini PC
echo "[5/6] Starting depth bridge receiver on Mini PC..."
python3 "$HEXPLORER_DIR/bridges/depth_bridge_receiver.py" &
PIDS+=($!)
sleep 1

# Start Livox receiver on Mini PC
echo "[5/6] Starting Livox receiver on Mini PC..."
python3 "$HEXPLORER_DIR/bridges/livox_tcp_receiver.py" &
PIDS+=($!)
sleep 2

echo ""
echo "========================================="
echo "  Sensors running - starting navigation"
echo "========================================="
echo ""

# Configurable parameters (can be set as environment variables)
STOP_DISTANCE="${STOP_DISTANCE:-0.6}"
SLOW_DISTANCE="${SLOW_DISTANCE:-1.8}"
FORWARD_SPEED="${FORWARD_SPEED:-0.5}"
SLOW_SPEED="${SLOW_SPEED:-0.24}"
TURN_SPEED="${TURN_SPEED:-0.4}"

echo "Obstacle avoidance parameters:"
echo "  - Stop distance:  ${STOP_DISTANCE}m"
echo "  - Slow distance:  ${SLOW_DISTANCE}m"
echo "  - Forward speed:  ${FORWARD_SPEED} m/s"
echo "  - Slow speed:     ${SLOW_SPEED} m/s"
echo "  - Turn speed:     ${TURN_SPEED} rad/s"
echo ""
echo "To customize, set environment variables before running:"
echo "  STOP_DISTANCE=0.8 FORWARD_SPEED=0.3 bash start_obstacle_avoidance.sh"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Run obstacle avoidance (foreground) with parameters
python3 "$HEXPLORER_DIR/navigation/obstacle_avoidance.py" \
    --stop-distance "$STOP_DISTANCE" \
    --slow-distance "$SLOW_DISTANCE" \
    --forward-speed "$FORWARD_SPEED" \
    --slow-speed "$SLOW_SPEED" \
    --turn-speed "$TURN_SPEED"
