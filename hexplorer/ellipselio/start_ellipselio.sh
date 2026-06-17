#!/usr/bin/env bash
# EllipseLIO live launcher — self-sufficient (brings up Livox LiDAR+IMU bridges itself).
# Separate from ~/fastlio_ws. Does NOT start Fast-LIO2 (avoids double LIO CPU load).
#
# Usage:
#   bash ~/ellipselio_ws/start_ellipselio.sh            # live, no RViz
#   bash ~/ellipselio_ws/start_ellipselio.sh --rviz     # live, with RViz
set -e

RVIZ=false
[ "$1" = "--rviz" ] && RVIZ=true

HEXPLORER_DIR="$HOME/hexplorer"
source "$HEXPLORER_DIR/scripts/common.sh"

# Match the robot stack's RMW so we see /livox topics published on cyclonedds.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Cap OpenMP so EllipseLIO can't grab all 16 cores and starve the robot control loop.
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-8}"
# RViz display (RDP session)
export DISPLAY="${DISPLAY:-:10.0}"

source /opt/ros/humble/setup.bash
source /home/robot/robot_controller_release/ros2_packages/setup.bash
source ~/ellipselio_ws/install/setup.bash

declare -a PIDS=()
cleanup() {
    echo "Shutting down EllipseLIO..."
    for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null || true; done
    pkill -9 -f "ellipselio" 2>/dev/null || true
    pkill -9 -f "component_container_mt" 2>/dev/null || true
    # Leave Jetson LiDAR + local receivers running (shared infra).
    echo "EllipseLIO stopped."
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

echo "=== EllipseLIO live (mid360.yaml, RMW=cyclonedds, OMP=$OMP_NUM_THREADS) ==="

echo "[1/5] Ensuring Jetson LiDAR + IMU services..."
ensure_jetson_lidar

echo "[2/5] Starting local LiDAR receiver..."
ensure_local "livox_tcp_receiver" "python3 $HEXPLORER_DIR/bridges/livox_tcp_receiver.py"
sleep 2

echo "[3/5] Starting local IMU receiver..."
ensure_local "imu_tcp_receiver" "python3 $HEXPLORER_DIR/bridges/imu_tcp_receiver.py"
sleep 2

echo "[4/5] Verifying /livox topics..."
timeout 6 ros2 topic hz /livox/lidar --window 3 2>/dev/null || echo "  Warning: /livox/lidar not flowing yet"
timeout 6 ros2 topic hz /livox/imu   --window 3 2>/dev/null || echo "  Warning: /livox/imu not flowing yet"

echo "[5/5] Launching EllipseLIO (rviz=$RVIZ)..."
ros2 launch ellipselio ellipselio_standalone.launch.py \
    config_file:=mid360.yaml \
    use_sim_time:=false \
    rviz:=$RVIZ

wait
