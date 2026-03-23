#!/bin/bash
#
# Interactive live control menu for Hexplorer tracking/search.
# Change target, detection mode, or behavior at runtime.
# Press Ctrl+C during tracking to return to menu.
#
# Usage:  bash ~/hexplorer/scripts/tracking_menu.sh
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
DETECT_MODE="yolo"
TARGET="person"
BEHAVIOR="follow"   # follow, smart, search, test, rviz, avoid
MAX_SPEED="0.3"
TARGET_DISTANCE="800"
SEARCH_SPEED="0.15"
RVIZ_ENABLED="off"

# ─── Internal state ─────────────────────────────────────────────────────────
INFRA_OK=false
FASTLIO_OK=false
CUR_DMODE=""
CUR_TARGET=""
declare -a PIDS=()
declare -a RVIZ_PIDS=()

# ─── Cleanup ─────────────────────────────────────────────────────────────────
kill_rviz() {
    for pid in "${RVIZ_PIDS[@]}"; do kill -9 "$pid" 2>/dev/null; done
    RVIZ_PIDS=()
    pkill -9 -f "rviz2" 2>/dev/null
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
    pkill -9 -f "obstacle_avoidance" 2>/dev/null
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
    # Static TFs
    ros2 run tf2_ros static_transform_publisher \
        --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
        --frame-id odom --child-frame-id camera_init >/dev/null 2>&1 &
    PIDS+=($!)
    ros2 run tf2_ros static_transform_publisher \
        --x 0.3 --y 0 --z 0.2 --qx 0 --qy 0 --qz 0 --qw 1 \
        --frame-id base_link --child-frame-id livox_frame >/dev/null 2>&1 &
    PIDS+=($!)
    sleep 0.5
    # TCP receivers
    ensure_local "livox_tcp_receiver" "python3 $HEXPLORER_DIR/bridges/livox_tcp_receiver.py"
    sleep 2
    ensure_local "imu_tcp_receiver" "python3 $HEXPLORER_DIR/bridges/imu_tcp_receiver.py"
    sleep 2
    # Odom relay
    python3 "$HEXPLORER_DIR/bridges/odom_relay.py" >/dev/null 2>&1 &
    PIDS+=($!)
    sleep 0.5
    # Fast-LIO2
    ros2 launch fast_lio mapping.launch.py \
        config_file:=hexplorer_mid360.yaml rviz:=false >/dev/null 2>&1 &
    PIDS+=($!)
    sleep 3
    FASTLIO_OK=true
}

launch_rviz() {
    # $1 = behavior name → selects rviz config
    kill_rviz
    local rviz_config=""
    case "$1" in
        follow)  rviz_config="$HEXPLORER_DIR/config/rviz_follow.rviz" ;;
        smart)   rviz_config="$HEXPLORER_DIR/config/rviz_smart.rviz" ;;
        search)  rviz_config="$HEXPLORER_DIR/config/rviz_search.rviz" ;;
        avoid)   rviz_config="$HEXPLORER_DIR/config/rviz_avoid.rviz" ;;
        test)    rviz_config="$HEXPLORER_DIR/config/rviz_follow.rviz" ;;
        rviz)    rviz_config="$HEXPLORER_DIR/config/rviz_follow.rviz" ;;
        *)       rviz_config="$HEXPLORER_DIR/config/rviz_follow.rviz" ;;
    esac

    # Start tracking visualizer for modes that have detection data
    if [ "$1" != "avoid" ]; then
        # Camera TF for marker projection
        ros2 run tf2_ros static_transform_publisher \
            --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
            --frame-id map --child-frame-id camera_color_optical_frame >/dev/null 2>&1 &
        RVIZ_PIDS+=($!)
        python3 "$HEXPLORER_DIR/tracking/tracking_rviz_visualizer.py" >/dev/null 2>&1 &
        RVIZ_PIDS+=($!)
        sleep 1
    fi

    rviz2 -d "$rviz_config" 2>/dev/null &
    RVIZ_PIDS+=($!)
    sleep 2
    echo "  RViz launched ($1 view)"
}

receiver_alive() {
    pgrep -f "detection_receiver" >/dev/null 2>&1
}

setup_tracker() {
    local same_mode=false
    [ "$CUR_DMODE" = "$DETECT_MODE" ] && [ "$CUR_TARGET" = "$TARGET" ] && same_mode=true

    # If same mode/target AND receiver is still alive, skip full restart
    if [ "$same_mode" = true ] && receiver_alive; then
        echo "Tracker already running ($DETECT_MODE / $TARGET)"
        return 0
    fi

    # If same mode but receiver died, just restart receiver
    if [ "$same_mode" = true ] && jetson_running "jetson_object_tracker.py"; then
        echo "Restarting detection receiver..."
        pkill -9 -f "detection_receiver" 2>/dev/null || true
        sleep 0.5
        python3 "$HEXPLORER_DIR/tracking/detection_receiver.py" --with-images &
        PIDS+=($!)
        sleep 2
        return 0
    fi

    echo "Switching tracker → $DETECT_MODE / $TARGET"

    # Kill old tracker on Jetson and wait until it's actually dead
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

    # Start tracker directly (bypass ensure_jetson_tracker's "already running" check)
    echo "  Starting object tracker on Jetson (mode: $DETECT_MODE, target: $TARGET)..."
    local extra_args="--stream-images"
    sshpass -p "$JETSON_PASS" ssh -f -o StrictHostKeyChecking=no "$JETSON_USER@$JETSON_IP" \
        "nohup /home/robot/start_tracker.sh $DETECT_MODE '$TARGET' '$extra_args' > /tmp/tracker.log 2>&1 < /dev/null &" 2>/dev/null

    if [ "$DETECT_MODE" = "yolo" ] || [ "$DETECT_MODE" = "yolo-world" ]; then
        echo "  Waiting for YOLO model to load..."
        sleep 8
    else
        sleep 3
    fi

    # Start detection receiver
    pkill -9 -f "detection_receiver" 2>/dev/null || true
    sleep 0.5
    python3 "$HEXPLORER_DIR/tracking/detection_receiver.py" --with-images &
    PIDS+=($!)
    sleep 2

    CUR_DMODE="$DETECT_MODE"
    CUR_TARGET="$TARGET"
}

# ─── Launch ──────────────────────────────────────────────────────────────────
do_launch() {
    # Obstacle avoidance is standalone
    if [ "$BEHAVIOR" = "avoid" ]; then
        echo ""
        echo "=== Obstacle Avoidance | speed=$MAX_SPEED | RViz=$RVIZ_ENABLED ==="
        echo "Press Ctrl+C to return to menu"
        echo ""
        if [ "$RVIZ_ENABLED" = "on" ]; then
            start_fastlio
            launch_rviz avoid
        fi
        (FORWARD_SPEED="$MAX_SPEED" STOP_DISTANCE=0.8 \
            bash "$SCRIPT_DIR/start_obstacle_avoidance.sh") || true
        kill_rviz
        INFRA_OK=false; FASTLIO_OK=false; CUR_DMODE=""; CUR_TARGET=""
        return
    fi

    start_infra || return

    # Start Fast-LIO2 only for modes that need odometry/LiDAR
    if [ "$BEHAVIOR" = "smart" ] || [ "$BEHAVIOR" = "search" ]; then
        start_fastlio
    fi
    setup_tracker

    echo ""
    echo "========================================="
    echo "  [$DETECT_MODE] $TARGET → $BEHAVIOR"
    [ "$RVIZ_ENABLED" = "on" ] && echo "  RViz: ON"
    echo "========================================="
    echo "Press Ctrl+C to return to menu"
    echo ""

    # Launch RViz if enabled
    if [ "$RVIZ_ENABLED" = "on" ]; then
        launch_rviz "$BEHAVIOR"
    fi

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
            # Standalone RViz mode — launch and block until closed
            if [ "$RVIZ_ENABLED" != "on" ]; then
                # Force-launch RViz even if toggle is off (this IS the rviz behavior)
                start_fastlio
                launch_rviz rviz
            fi
            echo "RViz running. Press Ctrl+C to return to menu."
            wait "${RVIZ_PIDS[@]}" 2>/dev/null || true
            ;;
    esac

    # Clean up RViz when behavior exits
    kill_rviz
}

# ─── Menu helpers ────────────────────────────────────────────────────────────
pick_target() {
    local r
    case "$DETECT_MODE" in
        yolo)
            r=$(whiptail --title "YOLO Target" --menu "Pick target:" 18 45 9 \
                "person"     "Person" \
                "bottle"     "Bottle" \
                "cup"        "Cup" \
                "chair"      "Chair" \
                "dog"        "Dog" \
                "cat"        "Cat" \
                "backpack"   "Backpack" \
                "cell phone" "Cell phone" \
                "custom"     "Type custom..." \
                3>&1 1>&2 2>&3) || return
            if [ "$r" = "custom" ]; then
                r=$(whiptail --title "Custom" --inputbox "COCO class name:" 8 40 "" 3>&1 1>&2 2>&3) || return
            fi
            TARGET="$r"
            ;;
        yolo-world)
            r=$(whiptail --title "YOLO-World" --inputbox "Describe object:" 8 50 "$TARGET" 3>&1 1>&2 2>&3) || return
            TARGET="$r"
            ;;
        color)
            r=$(whiptail --title "Color" --menu "Pick color:" 12 30 4 \
                "yellow" "Yellow" "red" "Red" "green" "Green" "blue" "Blue" \
                3>&1 1>&2 2>&3) || return
            TARGET="$r"
            ;;
    esac
}

pick_detect() {
    local r
    r=$(whiptail --title "Detection Mode" --menu "Pick mode:" 12 55 3 \
        "yolo"       "YOLOv8 — 80 COCO classes (fast)" \
        "yolo-world" "YOLO-World — any text description" \
        "color"      "Color tracking (HSV)" \
        3>&1 1>&2 2>&3) || return
    DETECT_MODE="$r"
    case "$DETECT_MODE" in
        yolo)       TARGET="person" ;;
        yolo-world) TARGET="person" ;;
        color)      TARGET="yellow" ;;
    esac
}

pick_behavior() {
    local r
    r=$(whiptail --title "Behavior" --menu "Pick behavior:" 16 55 6 \
        "follow" "Follow object (robot moves)" \
        "smart"  "Smart follow (+ obstacle avoidance)" \
        "search" "Search (systematic exploration)" \
        "test"   "Terminal visualizer (no robot)" \
        "rviz"   "RViz visualization (no robot)" \
        "avoid"  "Obstacle avoidance (no tracking)" \
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

# ─── Main loop ───────────────────────────────────────────────────────────────
while true; do
    clear
    # At menu: Ctrl+C exits
    trap 'exit 0' SIGINT

    RVIZ_LABEL="OFF"
    [ "$RVIZ_ENABLED" = "on" ] && RVIZ_LABEL="ON"

    CHOICE=$(whiptail --title "Hexplorer Control" \
        --menu "[$DETECT_MODE] $TARGET  →  $BEHAVIOR  |  RViz: $RVIZ_LABEL" 20 60 8 \
        "launch"   ">>> LAUNCH <<<" \
        "target"   "Target: $TARGET" \
        "detect"   "Detection: $DETECT_MODE" \
        "behavior" "Behavior: $BEHAVIOR" \
        "rviz"     "RViz: $RVIZ_LABEL (toggle)" \
        "settings" "Speed / distance" \
        "exit"     "Exit" \
        3>&1 1>&2 2>&3) || exit 0

    case "$CHOICE" in
        launch)
            clear
            # During tracking: Ctrl+C returns to menu (children still get SIGINT)
            trap 'echo ""' SIGINT
            do_launch
            ;;
        target)   pick_target ;;
        detect)   pick_detect ;;
        behavior) pick_behavior ;;
        rviz)
            if [ "$RVIZ_ENABLED" = "on" ]; then
                RVIZ_ENABLED="off"
            else
                RVIZ_ENABLED="on"
            fi
            ;;
        settings) pick_settings ;;
        exit)     exit 0 ;;
    esac
done
