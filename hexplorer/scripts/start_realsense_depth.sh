#!/bin/bash
# Start RealSense depth/pointcloud publisher on Jetson
# Uses pyrealsense2 v2.55 (which works, unlike v2.56)

source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

echo "Starting RealSense depth publisher..."
echo "Topics:"
echo "  /camera/camera/color/image_raw"
echo "  /camera/camera/depth/image_rect_raw"
echo "  /camera/camera/points"
echo ""

python3 /home/robot/realsense_depth_publisher.py
