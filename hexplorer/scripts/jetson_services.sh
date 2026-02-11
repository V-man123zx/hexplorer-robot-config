#!/bin/bash
# Jetson persistent services - starts LiDAR driver and TCP bridge if not already running
# This runs at Jetson boot via systemd, and can be called by Mini PC scripts to ensure services are up.
#
# Usage (on Jetson):
#   bash /home/robot/jetson_services.sh
#
# What it starts:
#   1. Livox LiDAR driver (livox_lidar_node)
#   2. Livox TCP bridge (livox_tcp_bridge.py) on port 9998

source /opt/ros/humble/setup.bash
source /home/robot/robot_controller_release/ros2_packages/setup.bash

# Start LiDAR driver if not running
if ! pgrep -f "livox_lidar_node" >/dev/null 2>&1; then
    echo "[jetson_services] Starting Livox LiDAR driver..."
    nohup ros2 launch livox_lidar_node start_node.launch.py > /tmp/livox_driver.log 2>&1 &
    sleep 4
    echo "[jetson_services] LiDAR driver started (PID $!)"
else
    echo "[jetson_services] LiDAR driver already running"
fi

# Start TCP bridge if not running
if ! pgrep -f "livox_tcp_bridge.py" >/dev/null 2>&1; then
    echo "[jetson_services] Starting Livox TCP bridge..."
    nohup python3 /home/robot/livox_tcp_bridge.py > /tmp/livox_bridge.log 2>&1 &
    sleep 2
    echo "[jetson_services] TCP bridge started (PID $!)"
else
    echo "[jetson_services] TCP bridge already running"
fi

echo "[jetson_services] All services ready"
