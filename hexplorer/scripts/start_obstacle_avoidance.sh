#!/bin/bash
#
# Start obstacle avoidance demo
# Ensures Jetson services are running (doesn't duplicate), starts local components.
#
# Environment variables:
#   STOP_DISTANCE=0.8    # Distance to stop/turn (m)
#   FORWARD_SPEED=0.3    # Normal forward speed (m/s)

set -e

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
HEXPLORER_DIR="$(dirname "$SCRIPT_DIR")"
source "$SCRIPT_DIR/common.sh"

echo "========================================="
echo "  Hexplorer Obstacle Avoidance Demo"
echo "========================================="
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash
source /home/robot/robot_controller_release/ros2_packages/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

declare -a PIDS=()

cleanup() {
    echo ""
    echo "Cleaning up..."
    for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null || true; done
    # Kill depth publisher on Jetson, leave LiDAR running
    jetson_kill_camera
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

# Ensure Jetson LiDAR services
echo "[2/6] Ensuring Jetson LiDAR services..."
ensure_jetson_lidar

# Ensure depth publisher on Jetson
echo "[3/6] Ensuring depth publisher on Jetson..."
ensure_jetson_depth

# Start local receivers
echo "[4/6] Starting local receivers..."
ensure_local "depth_bridge_receiver" "python3 $HEXPLORER_DIR/bridges/depth_bridge_receiver.py"
sleep 1
ensure_local "livox_tcp_receiver" "python3 $HEXPLORER_DIR/bridges/livox_tcp_receiver.py"
sleep 2

echo ""
echo "========================================="
echo "  Sensors running - starting navigation"
echo "========================================="
echo ""

# Configurable parameters
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
echo "Press Ctrl+C to stop"
echo ""

# Run obstacle avoidance (foreground)
python3 "$HEXPLORER_DIR/navigation/obstacle_avoidance.py" \
    --stop-distance "$STOP_DISTANCE" \
    --slow-distance "$SLOW_DISTANCE" \
    --forward-speed "$FORWARD_SPEED" \
    --slow-speed "$SLOW_SPEED" \
    --turn-speed "$TURN_SPEED"
