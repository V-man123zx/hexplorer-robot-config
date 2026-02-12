#!/bin/bash
# Combined script to launch full RViz sensor demo
# Ensures Jetson services are running (doesn't duplicate), starts local components.
#
# Usage:
#   bash start_sensor_demo.sh              # All sensors + tracking (yellow)
#   bash start_sensor_demo.sh --no-track   # Sensors only, no object tracking
#   bash start_sensor_demo.sh --slam       # Add MOLA odometry/SLAM
#
# Environment variables:
#   DETECT_MODE=yolo    # Detection mode: yolo, yolo-world, color
#   TARGET=person       # What to detect (see start_object_tracking.sh for full list)

set -e

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
HEXPLORER_DIR="$(dirname "$SCRIPT_DIR")"
source "$SCRIPT_DIR/common.sh"

# Parse arguments
ENABLE_TRACKING=true
ENABLE_SLAM=false
for arg in "$@"; do
    case $arg in
        --no-track) ENABLE_TRACKING=false ;;
        --slam) ENABLE_SLAM=true ;;
    esac
done

DETECT_MODE="${DETECT_MODE:-yolo}"

if [ -n "${TARGET:-}" ]; then
    : # TARGET already set
elif [ -n "${TARGET_COLOR:-}" ]; then
    TARGET="$TARGET_COLOR"
else
    case "$DETECT_MODE" in
        yolo)       TARGET="person" ;;
        yolo-world) TARGET="person" ;;
        color)      TARGET="yellow" ;;
        *)          TARGET="person" ;;
    esac
fi

declare -a PIDS=()

cleanup() {
    echo ""
    echo "Shutting down sensor demo..."
    for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null || true; done
    pkill -9 -f "mola-cli" 2>/dev/null || true
    pkill -9 -f "filterpass" 2>/dev/null || true
    # Only kill camera processes on Jetson, leave LiDAR running
    jetson_kill_camera
    echo "Cleanup complete."
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

echo "=== Hexplorer Sensor Demo ==="
if [ "$ENABLE_TRACKING" = true ]; then
    echo "  Object Tracking: ENABLED (mode: $DETECT_MODE, target: $TARGET)"
else
    echo "  Object Tracking: DISABLED"
fi
[ "$ENABLE_SLAM" = true ] && echo "  MOLA Odometry:   ENABLED"
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash
source /home/robot/robot_controller_release/ros2_packages/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
[ "$ENABLE_SLAM" = true ] && source ~/MOLA-SLAM/mola_ws/install/setup.bash 2>/dev/null || true

# [1] Ensure Jetson LiDAR services
echo "[1/10] Ensuring Jetson LiDAR services..."
ensure_jetson_lidar

# [2] Ensure camera process on Jetson (tracker OR depth, not both)
if [ "$ENABLE_TRACKING" = true ]; then
    echo "[2/10] Syncing and starting tracker on Jetson..."
    sshpass -p "$JETSON_PASS" scp -o StrictHostKeyChecking=no \
        "$HEXPLORER_DIR/tracking/jetson_object_tracker.py" \
        robot@$JETSON_IP:/home/robot/jetson_object_tracker.py 2>/dev/null
    ensure_jetson_tracker "$TARGET" "--stream-images" "$DETECT_MODE"
else
    echo "[2/10] Ensuring depth publisher on Jetson..."
    ensure_jetson_depth
fi

# [3] Start local receivers
echo "[3/10] Starting local receivers..."
if [ "$ENABLE_TRACKING" = false ]; then
    ensure_local "depth_bridge_receiver" "python3 $HEXPLORER_DIR/bridges/depth_bridge_receiver.py"
    sleep 1
fi
ensure_local "livox_tcp_receiver" "python3 $HEXPLORER_DIR/bridges/livox_tcp_receiver.py"
sleep 2

# [4] MOLA components if SLAM enabled
if [ "$ENABLE_SLAM" = true ]; then
    echo "[4/10] Starting MOLA filterpass + odometry..."
    ensure_local "filterpass" "python3 ~/MOLA-SLAM/mola_ws/install/mola_bringup/lib/mola_bringup/filterpass.py"
    sleep 2

    ros2 launch mola_lidar_odometry ros2-lidar-odometry-katana.launch.py \
        lidar_topic_name:=/livox/lidar_filtered \
        ignore_lidar_pose_from_tf:=true \
        use_rviz:=false \
        use_mola_gui:=False \
        use_state_estimator:=False \
        mola_lo_pipeline:=../pipelines/lidar3d-katana.yaml &
    PIDS+=($!)
    sleep 3
fi

# [5] Detection receiver if tracking
if [ "$ENABLE_TRACKING" = true ]; then
    echo "[5/10] Starting detection receiver..."
    ensure_local "detection_receiver" "python3 $HEXPLORER_DIR/tracking/detection_receiver.py --with-images"
    sleep 1
fi

# [6] TF publishers
echo "[6/10] Starting TF publishers..."
if [ "$ENABLE_SLAM" = true ]; then
    ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id odom --child-frame-id base_link &
    PIDS+=($!)
    ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.2 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id base_link --child-frame-id livox_frame &
else
    ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id livox_frame &
fi
PIDS+=($!)
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 --frame-id map --child-frame-id camera_depth_optical_frame &
PIDS+=($!)
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 --frame-id map --child-frame-id camera_color_optical_frame &
PIDS+=($!)
sleep 1

# [7] Tracking visualizer if tracking
if [ "$ENABLE_TRACKING" = true ]; then
    echo "[7/10] Starting tracking RViz visualizer..."
    python3 "$HEXPLORER_DIR/tracking/tracking_rviz_visualizer.py" &
    PIDS+=($!)
    sleep 1
fi

# [8] Launch RViz
echo "[8/10] Launching RViz..."
echo ""
echo "=== Sensor Demo Running ==="
echo "Press Ctrl+C to stop all processes."
echo ""

rviz2 -d "$HEXPLORER_DIR/config/sensor_visualization.rviz"

cleanup
