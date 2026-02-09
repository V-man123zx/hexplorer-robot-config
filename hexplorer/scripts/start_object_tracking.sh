#!/bin/bash
#
# Start object tracking system
#
# Usage:
#   bash start_object_tracking.sh           # Full system with robot control
#   bash start_object_tracking.sh --smart   # Smart follower with obstacle avoidance + SLAM search
#   bash start_object_tracking.sh --slam    # Start SLAM system alongside tracking (auto with --smart)
#   bash start_object_tracking.sh --test    # Test mode - terminal visualizer (no robot)
#   bash start_object_tracking.sh --rviz    # RViz visualization mode (no robot)
#
# Environment variables:
#   TARGET_COLOR=yellow     # Color to track (yellow, red, green, blue)
#   TARGET_DISTANCE=800     # Distance to maintain (mm)
#   MAX_SPEED=0.3           # Max forward speed (m/s)
#   TURN_SPEED=0.15         # Turn speed (rad/s)
#   OBSTACLE_STOP=0.8       # Stop if obstacle closer (m) [smart mode]
#   OBSTACLE_SLOW=1.2       # Slow if obstacle closer (m) [smart mode]
#   SEARCH_SPEED=0.5        # Search speed (m/s) [smart mode]
#

set -e

# Resolve symlinks to get actual script location
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
HEXPLORER_DIR="$(dirname "$SCRIPT_DIR")"
JETSON_IP="192.168.1.20"
JETSON_USER="robot"
JETSON_PASS="123"

# Parse arguments
TEST_MODE=false
RVIZ_MODE=false
SMART_MODE=false
SLAM_MODE=false
for arg in "$@"; do
    case $arg in
        --test)
            TEST_MODE=true
            shift
            ;;
        --rviz)
            RVIZ_MODE=true
            shift
            ;;
        --smart)
            SMART_MODE=true
            SLAM_MODE=true  # Smart mode requires SLAM
            shift
            ;;
        --slam)
            SLAM_MODE=true
            shift
            ;;
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
    echo "  MODE: SMART (obstacle avoidance + SLAM-based search)"
    echo "  SLAM: Enabled (map builds continuously)"
else
    echo "  MODE: FULL (robot will follow object)"
    if [ "$SLAM_MODE" = true ]; then
        echo "  SLAM: Enabled"
    fi
fi
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash
source /home/robot/robot_controller_release/ros2_packages/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Configurable parameters
TARGET_COLOR="${TARGET_COLOR:-yellow}"
TARGET_DISTANCE="${TARGET_DISTANCE:-800}"
MAX_SPEED="${MAX_SPEED:-0.4}"
TURN_SPEED="${TURN_SPEED:-0.6}"
OBSTACLE_STOP="${OBSTACLE_STOP:-0.8}"
OBSTACLE_SLOW="${OBSTACLE_SLOW:-1.2}"
SEARCH_SPEED="${SEARCH_SPEED:-0.2}"

echo "Tracking parameters:"
echo "  - Target color:    $TARGET_COLOR"
echo "  - Target distance: ${TARGET_DISTANCE}mm"
echo "  - Max speed:       ${MAX_SPEED} m/s"
echo "  - Turn speed:      ${TURN_SPEED} rad/s"
if [ "$SMART_MODE" = true ]; then
    echo "  - Obstacle stop:   ${OBSTACLE_STOP}m"
    echo "  - Obstacle slow:   ${OBSTACLE_SLOW}m"
    echo "  - Search mode:     SLAM-based (no timeout)"
fi
echo ""

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
        "pkill -f 'jetson_object_tracker.py' 2>/dev/null; \
         pkill -f 'livox_tcp_bridge.py' 2>/dev/null; \
         pkill -f 'livox_lidar_node' 2>/dev/null" 2>/dev/null || true

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

# Copy tracker script to Jetson if needed
echo "[2/4] Syncing tracker script to Jetson..."
sshpass -p "$JETSON_PASS" scp -o StrictHostKeyChecking=no \
    "$HEXPLORER_DIR/tracking/jetson_object_tracker.py" \
    "$JETSON_USER@$JETSON_IP:/home/robot/jetson_object_tracker.py" 2>/dev/null
echo "  Script synced"

# Start LiDAR if in smart mode or slam mode (needed for obstacle avoidance and SLAM)
if [ "$SMART_MODE" = true ] || [ "$SLAM_MODE" = true ]; then
    echo "[3/9] Starting Livox LiDAR driver on Jetson..."
    sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no "$JETSON_USER@$JETSON_IP" \
        "source /opt/ros/humble/setup.bash && \
         source /home/robot/robot_controller_release/ros2_packages/setup.bash && \
         ros2 launch livox_lidar_node start_node.launch.py" &
    PIDS+=($!)
    sleep 3

    echo "[4/9] Starting Livox TCP bridge on Jetson..."
    sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no "$JETSON_USER@$JETSON_IP" \
        "source /opt/ros/humble/setup.bash && \
         source /home/robot/robot_controller_release/ros2_packages/setup.bash && \
         python3 /home/robot/livox_tcp_bridge.py" &
    PIDS+=($!)
    sleep 2

    echo "[5/9] Starting Livox TCP receiver on Mini PC..."
    python3 "$HEXPLORER_DIR/bridges/livox_tcp_receiver.py" &
    PIDS+=($!)
    sleep 1

    # Start SLAM components if in SLAM mode
    if [ "$SLAM_MODE" = true ]; then
        echo "[6/9] Starting odometry publisher..."
        python3 "$HEXPLORER_DIR/slam/odometry_publisher.py" &
        PIDS+=($!)
        sleep 0.5

        echo "[7/9] Starting static TF: base_link -> livox_frame..."
        ros2 run tf2_ros static_transform_publisher \
            --x 0.0 --y 0.0 --z 0.2 \
            --qx 0.0 --qy 0.0 --qz 0.0 --qw 1.0 \
            --frame-id base_link --child-frame-id livox_frame &
        PIDS+=($!)
        sleep 0.5

        echo "[8/9] Starting pointcloud to laserscan converter..."
        ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
            --ros-args \
            --params-file "$HEXPLORER_DIR/slam/config/pc_to_scan.yaml" \
            -r cloud_in:=/livox/pointcloud &
        PIDS+=($!)
        sleep 0.5

        echo "[9/9] Starting slam_toolbox..."
        ros2 launch slam_toolbox online_async_launch.py \
            slam_params_file:="$HEXPLORER_DIR/slam/config/slam_params.yaml" &
        PIDS+=($!)
        sleep 2

        STEP_TRACKER="10"
        STEP_RECEIVER="11"
        TOTAL_STEPS="12"
    else
        STEP_TRACKER="6"
        STEP_RECEIVER="7"
        TOTAL_STEPS="8"
    fi
else
    STEP_TRACKER="3"
    STEP_RECEIVER="4"
    TOTAL_STEPS="4"
fi

# Start object tracker on Jetson
echo "[$STEP_TRACKER/$TOTAL_STEPS] Starting object tracker on Jetson..."
TRACKER_ARGS="--mode color --target $TARGET_COLOR"
if [ "$RVIZ_MODE" = true ] || [ "$SMART_MODE" = true ]; then
    TRACKER_ARGS="$TRACKER_ARGS --stream-images"
fi
sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no "$JETSON_USER@$JETSON_IP" \
    "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH && \
     export PYTHONPATH=/usr/local/lib/python3.10/dist-packages:\$PYTHONPATH && \
     python3 /home/robot/jetson_object_tracker.py $TRACKER_ARGS" &
PIDS+=($!)
sleep 3

# Start detection receiver on Mini PC
echo "[$STEP_RECEIVER/$TOTAL_STEPS] Starting detection receiver on Mini PC..."
RECEIVER_ARGS=""
if [ "$RVIZ_MODE" = true ] || [ "$SMART_MODE" = true ]; then
    RECEIVER_ARGS="--with-images"
fi
python3 "$HEXPLORER_DIR/tracking/detection_receiver.py" $RECEIVER_ARGS &
PIDS+=($!)
sleep 2

echo ""
echo "========================================="
if [ "$RVIZ_MODE" = true ]; then
    echo "  RVIZ MODE - 3D Visualization"
    echo "========================================="
    echo ""
    echo "Starting RViz visualizer and TF publisher..."
    echo "Press Ctrl+C to stop"
    echo ""

    # Start TF publisher for camera frame
    ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
        --frame-id map --child-frame-id camera_color_optical_frame &
    PIDS+=($!)

    # Start RViz visualizer node
    python3 "$HEXPLORER_DIR/tracking/tracking_rviz_visualizer.py" &
    PIDS+=($!)
    sleep 1

    # Run RViz in foreground
    rviz2 -d "$HEXPLORER_DIR/config/tracking_visualization.rviz"

elif [ "$TEST_MODE" = true ]; then
    echo "  TEST MODE - Terminal Visualization"
    echo "========================================="
    echo ""
    echo "Press Ctrl+C to stop"
    echo ""

    # Run visualizer in foreground
    python3 "$HEXPLORER_DIR/tracking/tracking_visualizer.py"

elif [ "$SMART_MODE" = true ]; then
    echo "  SMART MODE - Obstacle Avoidance + SLAM-Based Search"
    echo "========================================="
    echo ""
    echo "Robot will stand up and follow the $TARGET_COLOR object"
    echo "Uses LiDAR for obstacle avoidance and SLAM-based frontier exploration"
    echo "Search: turn-in-place (5s) -> explore frontiers -> patrol"
    echo "Press Ctrl+C to stop (robot will sit down safely)"
    echo ""

    # Run smart follower in foreground
    python3 "$HEXPLORER_DIR/tracking/smart_follower.py" \
        --target-distance "$TARGET_DISTANCE" \
        --max-speed "$MAX_SPEED" \
        --turn-speed "$TURN_SPEED" \
        --obstacle-stop "$OBSTACLE_STOP" \
        --obstacle-slow "$OBSTACLE_SLOW" \
        --search-speed "$SEARCH_SPEED"

else
    echo "  Starting Robot Control"
    echo "========================================="
    echo ""
    echo "Robot will stand up and follow the $TARGET_COLOR object"
    echo "Press Ctrl+C to stop (robot will sit down safely)"
    echo ""

    # Run object follower in foreground
    python3 "$HEXPLORER_DIR/tracking/object_follower.py" \
        --target-distance "$TARGET_DISTANCE" \
        --max-speed "$MAX_SPEED" \
        --turn-speed "$TURN_SPEED"
fi
