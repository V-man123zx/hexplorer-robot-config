#!/bin/bash
# View RealSense Camera stream from Jetson
# Run on Mini PC (192.168.1.10)

echo "Starting camera viewer with CycloneDDS..."
echo "Press Q in the window to quit"
echo ""

source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 /home/robot/camera_viewer.py
