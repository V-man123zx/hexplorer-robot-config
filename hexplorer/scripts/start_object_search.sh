#!/bin/bash
#
# Start object search system
# Systematically searches for a target object using MOLA odometry + LiDAR obstacle avoidance.
#
# Usage:
#   bash start_object_search.sh                  # Search for person (default)
#   bash start_object_search.sh --rviz           # With RViz visualization
#   TARGET=bottle bash start_object_search.sh    # Search for specific object
#   DETECT_MODE=yolo-world TARGET="red toolbox" bash start_object_search.sh
#
# Environment variables:
#   DETECT_MODE=yolo          # Detection mode: yolo, yolo-world, color
#   TARGET=person             # What to search for
#   SEARCH_SPEED=0.15         # Navigation speed (m/s)
#   SCAN_SPEED=0.15           # Scan rotation speed (rad/s)
#   NAVIGATE_DISTANCE=2.0     # Meters between scans
#   STOP_DISTANCE=0.8         # Obstacle stop distance (m)
#   SLOW_DISTANCE=1.5         # Obstacle slow distance (m)
#   CONFIRM_DISTANCE=1500     # Approach distance (mm)

set -e

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
HEXPLORER_DIR="$(dirname "$SCRIPT_DIR")"
source "$SCRIPT_DIR/common.sh"

# Parse arguments
RVIZ_MODE=false
EXTRA_ARGS=""
for arg in "$@"; do
    case $arg in
        --rviz) RVIZ_MODE=true ;;
        --no-approach) EXTRA_ARGS="$EXTRA_ARGS --no-approach" ;;
        --no-sit) EXTRA_ARGS="$EXTRA_ARGS --no-sit" ;;
    esac
done

echo "========================================="
echo "  Hexplorer Object Search System"
echo "========================================="
if [ "$RVIZ_MODE" = true ]; then
    echo "  RViz visualization: Enabled"
fi
echo ""

# Source ROS2 + robot + MOLA workspaces
source /opt/ros/humble/setup.bash
source /home/robot/robot_controller_release/ros2_packages/setup.bash
source ~/MOLA-SLAM/mola_ws/install/setup.bash 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Configurable parameters
DETECT_MODE="${DETECT_MODE:-yolo}"

if [ -n "${TARGET:-}" ]; then
    : # TARGET already set
else
    case "$DETECT_MODE" in
        yolo)       TARGET="person" ;;
        yolo-world) TARGET="person" ;;
        color)      TARGET="yellow" ;;
        *)          TARGET="person" ;;
    esac
fi

SEARCH_SPEED="${SEARCH_SPEED:-0.15}"
SCAN_SPEED="${SCAN_SPEED:-0.15}"
NAVIGATE_DISTANCE="${NAVIGATE_DISTANCE:-2.0}"
STOP_DISTANCE="${STOP_DISTANCE:-0.8}"
SLOW_DISTANCE="${SLOW_DISTANCE:-1.5}"
CONFIRM_DISTANCE="${CONFIRM_DISTANCE:-1500}"

echo "Search parameters:"
echo "  - Detect mode:        $DETECT_MODE"
echo "  - Target:             $TARGET"
echo "  - Search speed:       ${SEARCH_SPEED} m/s"
echo "  - Scan speed:         ${SCAN_SPEED} rad/s"
echo "  - Navigate distance:  ${NAVIGATE_DISTANCE} m"
echo "  - Stop distance:      ${STOP_DISTANCE} m"
echo "  - Slow distance:      ${SLOW_DISTANCE} m"
echo "  - Confirm distance:   ${CONFIRM_DISTANCE} mm"
echo ""

declare -a PIDS=()

cleanup() {
    echo ""
    echo "Cleaning up..."
    for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null || true; done
    pkill -9 -f "mola-cli" 2>/dev/null || true
    pkill -9 -f "filterpass" 2>/dev/null || true
    # Kill camera processes on Jetson, leave LiDAR running
    jetson_kill_camera
    echo "Cleanup complete"
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# ─── Infrastructure (proven sequence from start_object_tracking.sh --smart) ───

echo "[1/10] Checking Jetson connectivity..."
if ! sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 "$JETSON_USER@$JETSON_IP" "echo ok" >/dev/null 2>&1; then
    echo "ERROR: Cannot connect to Jetson at $JETSON_IP"
    exit 1
fi
echo "  Jetson connected"

echo "[2/10] Syncing tracker script to Jetson..."
sshpass -p "$JETSON_PASS" scp -o StrictHostKeyChecking=no \
    "$HEXPLORER_DIR/tracking/jetson_object_tracker.py" \
    "$JETSON_USER@$JETSON_IP:/home/robot/jetson_object_tracker.py" 2>/dev/null

echo "[3/10] Ensuring Jetson LiDAR services..."
ensure_jetson_lidar

echo "[4/10] Starting local LiDAR receiver..."
ensure_local "livox_tcp_receiver" "python3 $HEXPLORER_DIR/bridges/livox_tcp_receiver.py"
sleep 2

echo "[5/10] Starting TF publishers..."
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id odom --child-frame-id base_link &
PIDS+=($!)
sleep 0.3
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.2 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id base_link --child-frame-id livox_frame &
PIDS+=($!)
sleep 0.3

echo "[6/10] Starting filterpass node..."
ensure_local "filterpass" "python3 ~/MOLA-SLAM/mola_ws/install/mola_bringup/lib/mola_bringup/filterpass.py"
sleep 2

echo "[7/10] Starting MOLA LiDAR Odometry..."
ros2 launch mola_lidar_odometry ros2-lidar-odometry-katana.launch.py \
    lidar_topic_name:=/livox/lidar_filtered \
    ignore_lidar_pose_from_tf:=true \
    use_rviz:=false \
    use_mola_gui:=False \
    use_state_estimator:=False \
    mola_lo_pipeline:=../pipelines/lidar3d-katana.yaml &
PIDS+=($!)
sleep 3

echo "[8/10] Ensuring object tracker on Jetson..."
ensure_jetson_tracker "$TARGET" "--stream-images" "$DETECT_MODE"

echo "[9/10] Starting detection receiver..."
ensure_local "detection_receiver" "python3 $HEXPLORER_DIR/tracking/detection_receiver.py --with-images"
sleep 2

echo ""
echo "========================================="
echo "  Starting Object Search"
echo "========================================="
echo "Target: $TARGET"
echo "Press Ctrl+C to stop (robot will sit down safely)"
echo ""

# Start RViz if requested
if [ "$RVIZ_MODE" = true ]; then
    echo "[10/10] Starting RViz..."
    ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id camera_color_optical_frame &
    PIDS+=($!)
    rviz2 -d "$HEXPLORER_DIR/config/search_visualization.rviz" &
    PIDS+=($!)
    sleep 2
fi

# Run the searcher
python3 "$HEXPLORER_DIR/navigation/object_searcher.py" \
    --search-speed "$SEARCH_SPEED" \
    --scan-speed "$SCAN_SPEED" \
    --navigate-distance "$NAVIGATE_DISTANCE" \
    --stop-distance "$STOP_DISTANCE" \
    --slow-distance "$SLOW_DISTANCE" \
    --confirm-distance "$CONFIRM_DISTANCE" \
    $EXTRA_ARGS
