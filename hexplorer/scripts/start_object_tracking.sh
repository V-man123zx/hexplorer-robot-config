#!/bin/bash
#
# Start object tracking system
# Ensures Jetson services are running (doesn't duplicate), starts local components.
#
# Usage:
#   bash start_object_tracking.sh           # Full system with robot control
#   bash start_object_tracking.sh --smart   # Smart follower with obstacle avoidance
#   bash start_object_tracking.sh --test    # Test mode - terminal visualizer (no robot)
#   bash start_object_tracking.sh --rviz    # RViz visualization mode (no robot)
#
# Detection modes:
#   DETECT_MODE=yolo         # Default. 80 COCO classes (person, bottle, chair, dog...)
#   DETECT_MODE=yolo-world   # Open vocabulary - detect anything by text description
#   DETECT_MODE=color        # Simple HSV color tracking (yellow, red, green, blue)
#
# Environment variables:
#   DETECT_MODE=yolo          # Detection mode (see above)
#   TARGET=person             # What to detect:
#                             #   yolo: person, bottle, chair, cup, dog, any... (80 COCO classes)
#                             #   yolo-world: "red toolbox", "fire extinguisher", "water bottle"...
#                             #   color: yellow, red, green, blue
#   TARGET_DISTANCE=800       # Distance to maintain (mm)
#   MAX_SPEED=0.6             # Max forward speed (m/s)
#   TURN_SPEED=0.8            # Turn speed (rad/s)
#   OBSTACLE_STOP=0.4         # Stop if obstacle closer (m) [smart mode]
#   OBSTACLE_SLOW=0.8         # Slow if obstacle closer (m) [smart mode]
#   SEARCH_SPEED=0.6          # Search speed (m/s) [smart mode]

set -e

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
HEXPLORER_DIR="$(dirname "$SCRIPT_DIR")"
source "$SCRIPT_DIR/common.sh"

# Parse arguments
TEST_MODE=false
RVIZ_MODE=false
SMART_MODE=false
for arg in "$@"; do
    case $arg in
        --test) TEST_MODE=true ;;
        --rviz) RVIZ_MODE=true ;;
        --smart) SMART_MODE=true ;;
        --slam) ;; # Legacy flag, ignored
    esac
done

echo "========================================="
echo "  Hexplorer Object Tracking System"
echo "========================================="
if [ "$RVIZ_MODE" = true ]; then
    echo "  MODE: RVIZ (visualization with RViz)"
elif [ "$TEST_MODE" = true ]; then
    echo "  MODE: TEST (terminal visualization)"
elif [ "$SMART_MODE" = true ]; then
    echo "  MODE: SMART (obstacle avoidance + visited-area search)"
    echo "  Fast-LIO2 Odometry: Enabled (LiDAR+IMU fusion)"
else
    echo "  MODE: FULL (robot will follow object)"
fi
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash
source /home/robot/robot_controller_release/ros2_packages/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Configurable parameters
DETECT_MODE="${DETECT_MODE:-yolo}"

# TARGET is the primary variable. Fall back to TARGET_COLOR for backward compat.
if [ -n "${TARGET:-}" ]; then
    : # TARGET already set by user
elif [ -n "${TARGET_COLOR:-}" ]; then
    TARGET="$TARGET_COLOR"  # backward compat
else
    # Smart defaults per mode
    case "$DETECT_MODE" in
        yolo)       TARGET="person" ;;
        yolo-world) TARGET="person" ;;
        color)      TARGET="yellow" ;;
        *)          TARGET="person" ;;
    esac
fi

TARGET_DISTANCE="${TARGET_DISTANCE:-800}"
MAX_SPEED="${MAX_SPEED:-0.6}"
TURN_SPEED="${TURN_SPEED:-0.8}"
OBSTACLE_STOP="${OBSTACLE_STOP:-0.4}"
OBSTACLE_SLOW="${OBSTACLE_SLOW:-0.8}"
SEARCH_SPEED="${SEARCH_SPEED:-0.6}"

echo "Tracking parameters:"
echo "  - Detect mode:     $DETECT_MODE"
echo "  - Target:          $TARGET"
echo "  - Target distance: ${TARGET_DISTANCE}mm"
echo "  - Max speed:       ${MAX_SPEED} m/s"
echo "  - Turn speed:      ${TURN_SPEED} rad/s"
if [ "$SMART_MODE" = true ]; then
    echo "  - Obstacle stop:   ${OBSTACLE_STOP}m"
    echo "  - Obstacle slow:   ${OBSTACLE_SLOW}m"
fi
echo ""

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

# Check Jetson connectivity
echo "[1/4] Checking Jetson connectivity..."
if ! sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 "$JETSON_USER@$JETSON_IP" "echo ok" >/dev/null 2>&1; then
    echo "ERROR: Cannot connect to Jetson at $JETSON_IP"
    exit 1
fi
echo "  Jetson connected"

# Sync tracker script
echo "[2/4] Syncing tracker script to Jetson..."
sshpass -p "$JETSON_PASS" scp -o StrictHostKeyChecking=no \
    "$HEXPLORER_DIR/tracking/jetson_object_tracker.py" \
    "$JETSON_USER@$JETSON_IP:/home/robot/jetson_object_tracker.py" 2>/dev/null

# Start LiDAR + odometry if smart mode
if [ "$SMART_MODE" = true ]; then
    source ~/fastlio_ws/install/setup.bash 2>/dev/null || true

    echo "[3/11] Ensuring Jetson LiDAR services..."
    ensure_jetson_lidar

    echo "[4/11] Starting local LiDAR receiver..."
    ensure_local "livox_tcp_receiver" "python3 $HEXPLORER_DIR/bridges/livox_tcp_receiver.py"
    sleep 2

    echo "[5/11] Starting IMU TCP receiver..."
    ensure_local "imu_tcp_receiver" "python3 $HEXPLORER_DIR/bridges/imu_tcp_receiver.py"
    sleep 2

    echo "[6/11] Starting TF publisher..."
    ros2 run tf2_ros static_transform_publisher --x 0.3 --y 0 --z 0.2 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id base_link --child-frame-id livox_frame &
    PIDS+=($!)
    sleep 0.3

    echo "[7/11] Starting Fast-LIO2 + odom relay..."
    python3 "$HEXPLORER_DIR/bridges/odom_relay.py" &
    PIDS+=($!)
    ros2 launch fast_lio mapping.launch.py config_file:=hexplorer_mid360.yaml rviz:=false &
    PIDS+=($!)
    sleep 3

    STEP_TRACKER="8"
    STEP_RECEIVER="9"
    TOTAL_STEPS="10"
else
    STEP_TRACKER="3"
    STEP_RECEIVER="4"
    TOTAL_STEPS="4"
fi

# Start object tracker on Jetson (checks if already running)
echo "[$STEP_TRACKER/$TOTAL_STEPS] Ensuring object tracker on Jetson..."
STREAM_FLAG=""
if [ "$RVIZ_MODE" = true ] || [ "$SMART_MODE" = true ] || [ "$DETECT_MODE" != "color" ]; then
    STREAM_FLAG="--stream-images"
fi
ensure_jetson_tracker "$TARGET" "$STREAM_FLAG" "$DETECT_MODE"

# Start detection receiver
echo "[$STEP_RECEIVER/$TOTAL_STEPS] Starting detection receiver..."
RECEIVER_ARGS=""
if [ "$RVIZ_MODE" = true ] || [ "$SMART_MODE" = true ] || [ "$DETECT_MODE" != "color" ]; then
    RECEIVER_ARGS="--with-images"
fi
ensure_local "detection_receiver" "python3 $HEXPLORER_DIR/tracking/detection_receiver.py $RECEIVER_ARGS"
sleep 2

echo ""
echo "========================================="
if [ "$SMART_MODE" = true ] && [ "$RVIZ_MODE" = true ]; then
    echo "  SMART + RVIZ MODE"
    echo "========================================="
    echo "Press Ctrl+C to stop (robot will sit down safely)"
    echo ""
    ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id camera_color_optical_frame &
    PIDS+=($!)
    python3 "$HEXPLORER_DIR/tracking/tracking_rviz_visualizer.py" &
    PIDS+=($!)
    sleep 1
    rviz2 -d "$HEXPLORER_DIR/config/tracking_visualization.rviz" &
    PIDS+=($!)
    sleep 2
    python3 "$HEXPLORER_DIR/tracking/smart_follower.py" \
        --target-distance "$TARGET_DISTANCE" --max-speed "$MAX_SPEED" \
        --turn-speed "$TURN_SPEED" --obstacle-stop "$OBSTACLE_STOP" \
        --obstacle-slow "$OBSTACLE_SLOW" --search-speed "$SEARCH_SPEED"

elif [ "$RVIZ_MODE" = true ]; then
    echo "  RVIZ MODE - 3D Visualization (no robot control)"
    echo "========================================="
    echo "Press Ctrl+C to stop"
    echo ""
    ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id camera_color_optical_frame &
    PIDS+=($!)
    python3 "$HEXPLORER_DIR/tracking/tracking_rviz_visualizer.py" &
    PIDS+=($!)
    sleep 1
    rviz2 -d "$HEXPLORER_DIR/config/tracking_visualization.rviz"

elif [ "$TEST_MODE" = true ]; then
    echo "  TEST MODE - Terminal Visualization"
    echo "========================================="
    echo "Press Ctrl+C to stop"
    echo ""
    python3 "$HEXPLORER_DIR/tracking/tracking_visualizer.py"

elif [ "$SMART_MODE" = true ]; then
    echo "  SMART MODE - Obstacle Avoidance + Visited-Area Search"
    echo "========================================="
    echo "Press Ctrl+C to stop (robot will sit down safely)"
    echo ""
    python3 "$HEXPLORER_DIR/tracking/smart_follower.py" \
        --target-distance "$TARGET_DISTANCE" --max-speed "$MAX_SPEED" \
        --turn-speed "$TURN_SPEED" --obstacle-stop "$OBSTACLE_STOP" \
        --obstacle-slow "$OBSTACLE_SLOW" --search-speed "$SEARCH_SPEED"

else
    echo "  Starting Robot Control"
    echo "========================================="
    echo "Press Ctrl+C to stop (robot will sit down safely)"
    echo ""
    python3 "$HEXPLORER_DIR/tracking/object_follower.py" \
        --target-distance "$TARGET_DISTANCE" --max-speed "$MAX_SPEED" \
        --turn-speed "$TURN_SPEED"
fi
