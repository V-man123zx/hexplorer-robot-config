#!/bin/bash
# Start combined RealSense Depth + TCP publisher
# Run this on Jetson

source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/usr/local/lib/python3.10/dist-packages:$PYTHONPATH

echo "Starting RealSense Depth + TCP publisher..."
echo "TCP server will listen on port 9999"
python3 /home/robot/realsense_depth_tcp_publisher.py
