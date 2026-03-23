#!/bin/bash
#
# YOLO-World Detection Menu
#
# Simplified launcher for YOLO-World open-vocabulary detection.
# Always shows RViz with full-screen tracking image.
#
# Usage:  bash ~/hexplorer/scripts/yolo_world_menu.sh
#

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
HEXPLORER_DIR="$(dirname "$SCRIPT_DIR")"
source "$SCRIPT_DIR/common.sh"

source /opt/ros/humble/setup.bash
source /home/robot/robot_controller_release/ros2_packages/setup.bash
source ~/fastlio_ws/install/setup.bash 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ─── User settings ──────────────────────────────────────────────────────────
TARGET="person"
BEHAVIOR="follow"   # follow, smart, search, test, rviz
MAX_SPEED="0.3"
TARGET_DISTANCE="800"
SEARCH_SPEED="0.15"

# ─── Internal state ─────────────────────────────────────────────────────────
INFRA_OK=false
FASTLIO_OK=false
CUR_TARGET=""
declare -a PIDS=()
declare -a RVIZ_PIDS=()

# ─── Cleanup ────────────────────────────────────────────────────────────────
kill_rviz() {
    for pid in "${RVIZ_PIDS[@]}"; do kill -9 "$pid" 2>/dev/null; done
    RVIZ_PIDS=()
    pkill -9 -f "rviz2" 2>/dev/null
    pkill -9 -f "rqt_image_view" 2>/dev/null
    pkill -9 -f "tracking_rviz_visualizer" 2>/dev/null
}

full_cleanup() {
    trap - EXIT SIGINT SIGTERM
    echo ""
    echo "Shutting down..."
    for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null; done
    kill_rviz
    pkill -9 -f "fastlio_mapping" 2>/dev/null
    pkill -9 -f "odom_relay" 2>/dev/null
    pkill -9 -f "detection_receiver" 2>/dev/null
    pkill -9 -f "object_follower.py" 2>/dev/null
    pkill -9 -f "smart_follower.py" 2>/dev/null
    pkill -9 -f "object_searcher.py" 2>/dev/null
    pkill -9 -f "tracking_visualizer" 2>/dev/null
    jetson_kill_camera 2>/dev/null
    echo "Done."
    exit 0
}
trap full_cleanup EXIT
trap 'exit 0' SIGINT SIGTERM

# ─── Infrastructure ─────────────────────────────────────────────────────────
start_infra() {
    [ "$INFRA_OK" = true ] && return 0
    echo "Checking Jetson..."
    if ! sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 \
        "$JETSON_USER@$JETSON_IP" "echo ok" >/dev/null 2>&1; then
        echo "ERROR: Cannot connect to Jetson at $JETSON_IP"
        sleep 2
        return 1
    fi
    echo "Syncing tracker to Jetson..."
    sshpass -p "$JETSON_PASS" scp -o StrictHostKeyChecking=no \
        "$HEXPLORER_DIR/tracking/jetson_object_tracker.py" \
        "$JETSON_USER@$JETSON_IP:/home/robot/jetson_object_tracker.py" 2>/dev/null
    INFRA_OK=true
}

start_fastlio() {
    [ "$FASTLIO_OK" = true ] && return 0
    echo "Starting Fast-LIO2 + LiDAR + IMU..."
    ensure_jetson_lidar
    ros2 run tf2_ros static_transform_publisher \
        --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
        --frame-id odom --child-frame-id camera_init >/dev/null 2>&1 &
    PIDS+=($!)
    ros2 run tf2_ros static_transform_publisher \
        --x 0.3 --y 0 --z 0.2 --qx 0 --qy 0 --qz 0 --qw 1 \
        --frame-id base_link --child-frame-id livox_frame >/dev/null 2>&1 &
    PIDS+=($!)
    sleep 0.5
    ensure_local "livox_tcp_receiver" "python3 $HEXPLORER_DIR/bridges/livox_tcp_receiver.py"
    sleep 2
    ensure_local "imu_tcp_receiver" "python3 $HEXPLORER_DIR/bridges/imu_tcp_receiver.py"
    sleep 2
    python3 "$HEXPLORER_DIR/bridges/odom_relay.py" >/dev/null 2>&1 &
    PIDS+=($!)
    sleep 0.5
    ros2 launch fast_lio mapping.launch.py \
        config_file:=hexplorer_mid360.yaml rviz:=false >/dev/null 2>&1 &
    PIDS+=($!)
    sleep 3
    FASTLIO_OK=true
}

receiver_alive() {
    pgrep -f "detection_receiver" >/dev/null 2>&1
}

# ─── Tracker setup ──────────────────────────────────────────────────────────
setup_tracker() {
    # If same target AND receiver alive, skip
    if [ "$CUR_TARGET" = "$TARGET" ] && receiver_alive; then
        echo "Tracker already running ($TARGET)"
        return 0
    fi

    # If same target but receiver died, just restart receiver
    if [ "$CUR_TARGET" = "$TARGET" ] && jetson_running "jetson_object_tracker.py"; then
        echo "Restarting detection receiver..."
        pkill -9 -f "detection_receiver" 2>/dev/null || true
        sleep 0.5
        python3 "$HEXPLORER_DIR/tracking/detection_receiver.py" --with-images &
        PIDS+=($!)
        sleep 2
        return 0
    fi

    echo "Starting YOLO-World tracker → $TARGET"

    # Kill old tracker
    sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no "$JETSON_USER@$JETSON_IP" \
        "pkill -9 -f jetson_object_tracker.py 2>/dev/null; true" 2>/dev/null || true
    pkill -9 -f "detection_receiver" 2>/dev/null || true

    echo "  Waiting for old tracker to stop..."
    for i in $(seq 1 10); do
        if ! jetson_running "jetson_object_tracker.py"; then
            break
        fi
        sleep 1
    done

    echo "  Starting YOLO-World on Jetson (target: $TARGET)..."
    local extra_args="--stream-images"
    sshpass -p "$JETSON_PASS" ssh -f -o StrictHostKeyChecking=no "$JETSON_USER@$JETSON_IP" \
        "nohup /home/robot/start_tracker.sh yolo-world '$TARGET' '$extra_args' > /tmp/tracker.log 2>&1 < /dev/null &" 2>/dev/null

    echo "  Waiting for YOLO-World model to load..."
    sleep 12

    # Start detection receiver
    pkill -9 -f "detection_receiver" 2>/dev/null || true
    sleep 0.5
    python3 "$HEXPLORER_DIR/tracking/detection_receiver.py" --with-images &
    PIDS+=($!)
    sleep 2

    CUR_TARGET="$TARGET"
}

# ─── Image viewer ──────────────────────────────────────────────────────────
launch_viewer() {
    kill_rviz
    # Camera TF for marker projection
    ros2 run tf2_ros static_transform_publisher \
        --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
        --frame-id map --child-frame-id camera_color_optical_frame >/dev/null 2>&1 &
    RVIZ_PIDS+=($!)
    python3 "$HEXPLORER_DIR/tracking/tracking_rviz_visualizer.py" >/dev/null 2>&1 &
    RVIZ_PIDS+=($!)
    sleep 1
    ros2 run rqt_image_view rqt_image_view /object_tracking/image 2>/dev/null &
    RVIZ_PIDS+=($!)
    sleep 2
    echo "  Image viewer launched"
}

# ─── Launch ─────────────────────────────────────────────────────────────────
do_launch() {
    start_infra || return

    # Start Fast-LIO2 only for modes that need odometry
    if [ "$BEHAVIOR" = "smart" ] || [ "$BEHAVIOR" = "search" ]; then
        start_fastlio
    fi
    setup_tracker

    echo ""
    echo "========================================="
    echo "  YOLO-World: \"$TARGET\" → $BEHAVIOR"
    echo "========================================="
    echo "Press Ctrl+C to return to menu"
    echo ""

    # Always launch RViz
    launch_viewer

    case "$BEHAVIOR" in
        follow)
            python3 "$HEXPLORER_DIR/tracking/object_follower.py" \
                --target-distance "$TARGET_DISTANCE" \
                --max-speed "$MAX_SPEED" --turn-speed 0.8 || true
            ;;
        smart)
            python3 "$HEXPLORER_DIR/tracking/smart_follower.py" \
                --target-distance "$TARGET_DISTANCE" \
                --max-speed "$MAX_SPEED" --turn-speed 0.8 || true
            ;;
        search)
            python3 "$HEXPLORER_DIR/navigation/object_searcher.py" \
                --search-speed "$SEARCH_SPEED" --scan-speed 0.15 \
                --navigate-distance 2.0 --stop-distance 0.8 \
                --slow-distance 1.5 --confirm-distance 1500 || true
            ;;
        test)
            python3 "$HEXPLORER_DIR/tracking/tracking_visualizer.py" || true
            ;;
        rviz)
            echo "RViz running. Press Ctrl+C to return to menu."
            wait "${RVIZ_PIDS[@]}" 2>/dev/null || true
            ;;
    esac

    kill_rviz
}

# ─── Menu helpers ───────────────────────────────────────────────────────────
pick_target() {
    local r
    r=$(whiptail --title "YOLO-World Target" --inputbox \
        "Describe what to detect:" 8 50 "$TARGET" 3>&1 1>&2 2>&3) || return
    [ -n "$r" ] && TARGET="$r"
}

pick_behavior() {
    local r
    r=$(whiptail --title "Behavior" --menu "Pick behavior:" 15 55 5 \
        "follow" "Follow object (robot moves)" \
        "smart"  "Smart follow (+ obstacle avoidance)" \
        "search" "Search (systematic exploration)" \
        "test"   "Terminal visualizer (no robot)" \
        "rviz"   "RViz only (no robot)" \
        3>&1 1>&2 2>&3) || return
    BEHAVIOR="$r"
}

pick_settings() {
    local r
    r=$(whiptail --title "Settings" --menu "Adjust:" 14 50 3 \
        "speed"    "Max speed: $MAX_SPEED m/s" \
        "distance" "Follow distance: ${TARGET_DISTANCE}mm" \
        "search"   "Search speed: $SEARCH_SPEED m/s" \
        3>&1 1>&2 2>&3) || return
    case "$r" in
        speed)    MAX_SPEED=$(whiptail --inputbox "Max speed (m/s):" 8 40 "$MAX_SPEED" 3>&1 1>&2 2>&3) || return ;;
        distance) TARGET_DISTANCE=$(whiptail --inputbox "Follow dist (mm):" 8 40 "$TARGET_DISTANCE" 3>&1 1>&2 2>&3) || return ;;
        search)   SEARCH_SPEED=$(whiptail --inputbox "Search speed (m/s):" 8 40 "$SEARCH_SPEED" 3>&1 1>&2 2>&3) || return ;;
    esac
}

# ─── Main loop ──────────────────────────────────────────────────────────────
while true; do
    clear
    trap 'exit 0' SIGINT

    CHOICE=$(whiptail --title "YOLO-World Detection" \
        --menu "\"$TARGET\"  →  $BEHAVIOR" 16 55 6 \
        "launch"   ">>> LAUNCH <<<" \
        "target"   "Detect: $TARGET" \
        "behavior" "Behavior: $BEHAVIOR" \
        "settings" "Speed / distance" \
        "exit"     "Exit" \
        3>&1 1>&2 2>&3) || exit 0

    case "$CHOICE" in
        launch)
            clear
            trap 'echo ""' SIGINT
            do_launch
            ;;
        target)   pick_target ;;
        behavior) pick_behavior ;;
        settings) pick_settings ;;
        exit)     exit 0 ;;
    esac
done
