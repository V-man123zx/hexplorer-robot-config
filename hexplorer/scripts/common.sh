#!/bin/bash
# Common functions for all hexplorer launch scripts
# Source this file: source "$(dirname "$0")/common.sh"

JETSON_IP="192.168.1.20"
JETSON_USER="robot"
JETSON_PASS="123"

# Check if a process is running on Jetson
# Uses bracket trick to prevent pgrep from matching the SSH shell's own command line
jetson_running() {
    local pattern="[${1:0:1}]${1:1}"
    sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
        "$JETSON_USER@$JETSON_IP" "pgrep -f '$pattern' >/dev/null 2>&1" 2>/dev/null
}

# Ensure LiDAR services are running on Jetson (driver + TCP bridge)
# Starts them if not already running. Never duplicates.
ensure_jetson_lidar() {
    echo "  Checking Jetson LiDAR services..."
    sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
        "$JETSON_USER@$JETSON_IP" "bash /home/robot/jetson_services.sh" 2>/dev/null
}

# Ensure object tracker is running on Jetson (kills depth publisher if active since they share camera)
# Args: $1=target (default: yellow), $2=stream_flag, $3=detect_mode (default: color)
ensure_jetson_tracker() {
    local target="${1:-yellow}"
    local stream_flag="${2:-}"
    local detect_mode="${3:-color}"

    if jetson_running "jetson_object_tracker.py"; then
        echo "  Object tracker already running on Jetson"
        return 0
    fi

    # Kill depth publisher if running (shares camera)
    if jetson_running "realsense_depth_tcp_publisher.py"; then
        echo "  Stopping depth publisher (camera needed for tracker)..."
        sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no "$JETSON_USER@$JETSON_IP" \
            "pkill -9 -f realsense_depth_tcp_publisher.py 2>/dev/null; true" 2>/dev/null || true
        sleep 1
    fi

    echo "  Starting object tracker on Jetson (mode: $detect_mode, target: $target)..."
    local extra_args=""
    [ -n "$stream_flag" ] && extra_args="--stream-images"

    sshpass -p "$JETSON_PASS" ssh -f -o StrictHostKeyChecking=no "$JETSON_USER@$JETSON_IP" \
        "nohup /home/robot/start_tracker.sh $detect_mode '$target' '$extra_args' > /tmp/tracker.log 2>&1 < /dev/null &" 2>/dev/null

    # YOLO models need extra time to load
    if [ "$detect_mode" = "yolo" ] || [ "$detect_mode" = "yolo-world" ]; then
        echo "  Waiting for YOLO model to load..."
        sleep 8
    else
        sleep 3
    fi
}

# Ensure depth publisher is running on Jetson (kills tracker if active since they share camera)
ensure_jetson_depth() {
    if jetson_running "realsense_depth_tcp_publisher.py"; then
        echo "  Depth publisher already running on Jetson"
        return 0
    fi

    # Kill tracker if running (shares camera)
    if jetson_running "jetson_object_tracker.py"; then
        echo "  Stopping object tracker (camera needed for depth)..."
        sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no "$JETSON_USER@$JETSON_IP" \
            "pkill -9 -f jetson_object_tracker.py 2>/dev/null; true" 2>/dev/null || true
        sleep 1
    fi

    echo "  Starting depth publisher on Jetson..."
    sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no "$JETSON_USER@$JETSON_IP" \
        "source /opt/ros/humble/setup.bash && \
         export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH && \
         export PYTHONPATH=/usr/local/lib/python3.10/dist-packages:\$PYTHONPATH && \
         nohup python3 /home/robot/realsense_depth_tcp_publisher.py > /tmp/realsense.log 2>&1 &" 2>/dev/null
    sleep 3
}

# Ensure local process is running (by grep pattern and start command)
# Usage: ensure_local "livox_tcp_receiver" "python3 $HEXPLORER_DIR/bridges/livox_tcp_receiver.py"
ensure_local() {
    local pattern="$1"
    local cmd="$2"

    if pgrep -f "$pattern" >/dev/null 2>&1; then
        echo "  $pattern already running"
        return 0
    fi

    echo "  Starting $pattern..."
    eval "$cmd &"
    local pid=$!
    PIDS+=($pid)
    return 0
}

# Kill only camera-related Jetson processes (tracker/depth), leave LiDAR running
jetson_kill_camera() {
    sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
        "$JETSON_USER@$JETSON_IP" \
        "pkill -9 -f jetson_object_tracker.py 2>/dev/null; \
         pkill -9 -f realsense_depth_tcp_publisher.py 2>/dev/null; true" 2>/dev/null || true
}

# Kill ALL Jetson processes (use only when you want full shutdown)
jetson_kill_all() {
    sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
        "$JETSON_USER@$JETSON_IP" \
        "killall -9 python3 livox_lidar_node 2>/dev/null; true" 2>/dev/null || true
}
