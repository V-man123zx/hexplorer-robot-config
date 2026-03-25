#!/bin/bash
#
# Start voice-controlled YOLO-World demo
# Starts all infrastructure (LiDAR, Fast-LIO2, detection receiver),
# then runs voice_demo.py which manages the Jetson tracker and behaviors.
#
# Usage:
#   bash start_voice_demo.sh           # Normal mode
#   bash start_voice_demo.sh --debug   # Debug mode (no robot commands)
#   bash start_voice_demo.sh --rviz    # With RViz visualization
#
# Requires:
#   - ElevenLabs API key + Agent ID in ~/hexplorer/voice/.env
#   - Microphone and speaker connected to Mini PC
#   - Jetson reachable at 192.168.1.20

set -e

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
HEXPLORER_DIR="$(dirname "$SCRIPT_DIR")"
source "$SCRIPT_DIR/common.sh"

# Parse arguments
DEBUG_FLAG=""
RVIZ_MODE=false
for arg in "$@"; do
    case $arg in
        --debug) DEBUG_FLAG="--debug" ;;
        --rviz) RVIZ_MODE=true ;;
    esac
done

echo "========================================="
echo "  Hexplorer Voice-Controlled Demo"
echo "========================================="
if [ -n "$DEBUG_FLAG" ]; then
    echo "  MODE: DEBUG (no robot commands)"
fi
if [ "$RVIZ_MODE" = true ]; then
    echo "  RViz: Enabled"
fi
echo ""

# Source ROS2 + robot + Fast-LIO2 workspaces
source /opt/ros/humble/setup.bash
source /home/robot/robot_controller_release/ros2_packages/setup.bash
source ~/fastlio_ws/install/setup.bash 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Check .env file
if [ ! -f "$HEXPLORER_DIR/voice/.env" ]; then
    echo "ERROR: Missing ~/hexplorer/voice/.env"
    echo "Create it with:"
    echo "  ELEVENLABS_API_KEY=sk_your_key_here"
    echo "  AGENT_ID=your_agent_id_here"
    exit 1
fi

declare -a PIDS=()

cleanup() {
    echo ""
    echo "Cleaning up..."
    for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null || true; done
    pkill -9 -f "fastlio_mapping" 2>/dev/null || true
    pkill -9 -f "odom_relay" 2>/dev/null || true
    # Kill camera processes on Jetson, leave LiDAR running
    jetson_kill_camera
    echo "Cleanup complete"
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# ─── Infrastructure ──────────────────────────────────────────────────────────

echo "[1/8] Checking Jetson connectivity..."
if ! sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 "$JETSON_USER@$JETSON_IP" "echo ok" >/dev/null 2>&1; then
    echo "ERROR: Cannot connect to Jetson at $JETSON_IP"
    exit 1
fi
echo "  Jetson connected"

echo "[2/8] Syncing tracker script to Jetson..."
sshpass -p "$JETSON_PASS" scp -o StrictHostKeyChecking=no \
    "$HEXPLORER_DIR/tracking/jetson_object_tracker.py" \
    "$JETSON_USER@$JETSON_IP:/home/robot/jetson_object_tracker.py" 2>/dev/null

echo "[3/8] Ensuring Jetson LiDAR services..."
ensure_jetson_lidar

echo "[4/8] Starting local LiDAR receiver..."
ensure_local "livox_tcp_receiver" "python3 $HEXPLORER_DIR/bridges/livox_tcp_receiver.py"
sleep 2

echo "[5/8] Starting IMU TCP receiver..."
ensure_local "imu_tcp_receiver" "python3 $HEXPLORER_DIR/bridges/imu_tcp_receiver.py"
sleep 2

echo "[6/8] Starting TF publisher..."
ros2 run tf2_ros static_transform_publisher --x 0.3 --y 0 --z 0.2 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id base_link --child-frame-id livox_frame &
PIDS+=($!)
sleep 0.3

echo "[7/8] Starting Fast-LIO2 + odom relay..."
python3 "$HEXPLORER_DIR/bridges/odom_relay.py" &
PIDS+=($!)
ros2 launch fast_lio mapping.launch.py config_file:=hexplorer_mid360.yaml rviz:=false &
PIDS+=($!)
sleep 3

echo "[8/8] Starting detection receiver..."
ensure_local "detection_receiver" "python3 $HEXPLORER_DIR/tracking/detection_receiver.py --with-images"
sleep 2

# ─── Optional RViz ───────────────────────────────────────────────────────────

if [ "$RVIZ_MODE" = true ]; then
    echo "Starting RViz..."
    ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id camera_color_optical_frame &
    PIDS+=($!)
    python3 "$HEXPLORER_DIR/tracking/tracking_rviz_visualizer.py" &
    PIDS+=($!)
    rviz2 -d "$HEXPLORER_DIR/config/tracking_visualization.rviz" &
    PIDS+=($!)
    sleep 2
fi

# ─── Voice Demo ──────────────────────────────────────────────────────────────

echo ""
echo "========================================="
echo "  Infrastructure Ready"
echo "========================================="
echo "Say 'hey robot' to wake me up!"
echo "Press Ctrl+C to stop"
echo ""

# Note: voice_demo.py manages Jetson tracker lifecycle itself
# (starts/restarts tracker when target changes via voice command)
python3 "$HEXPLORER_DIR/voice/voice_demo.py" $DEBUG_FLAG
