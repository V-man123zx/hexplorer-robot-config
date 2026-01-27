#!/bin/bash
# Start RealSense Camera with CycloneDDS
# Run on Jetson (192.168.1.20)

echo "Starting RealSense Camera with CycloneDDS..."
echo ""
echo "Topic: /camera/camera/color/image_raw"
echo ""
echo "On Mini PC, run: ~/view_camera.sh"
echo ""

source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 /home/robot/realsense_v4l2_publisher.py
