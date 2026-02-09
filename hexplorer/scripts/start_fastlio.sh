#!/bin/bash
# Start Fast-LIO SLAM with LiDAR and IMU bridges
# Usage: bash start_fastlio.sh

set -e

# Get script directory
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
HEXPLORER_DIR="$(dirname "$SCRIPT_DIR")"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}=== Fast-LIO SLAM Demo ===${NC}"

# Cleanup function
cleanup() {
    echo -e "\n${YELLOW}Cleaning up...${NC}"

    # Kill local processes
    pkill -f "imu_tcp_receiver.py" 2>/dev/null || true
    pkill -f "livox_tcp_receiver.py" 2>/dev/null || true
    pkill -f "fast_lio" 2>/dev/null || true
    pkill -f "rviz2" 2>/dev/null || true

    # Kill remote processes on Jetson
    sshpass -p "123" ssh -o StrictHostKeyChecking=no robot@192.168.1.20 \
        "pkill -f 'livox_tcp_bridge.py' 2>/dev/null; \
         pkill -f 'imu_tcp_bridge.py' 2>/dev/null; \
         pkill -f 'livox_lidar_node' 2>/dev/null" 2>/dev/null || true

    echo -e "${GREEN}Cleanup complete${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Source ROS2
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source custom messages (for Livox)
if [ -f /home/robot/robot_controller_release/ros2_packages/setup.bash ]; then
    source /home/robot/robot_controller_release/ros2_packages/setup.bash
fi

# Source Fast-LIO workspace
if [ -f /home/robot/fastlio_ws/install/setup.bash ]; then
    source /home/robot/fastlio_ws/install/setup.bash
fi

echo -e "${YELLOW}Step 1: Starting Livox LiDAR driver on Jetson...${NC}"
sshpass -p "123" ssh -o StrictHostKeyChecking=no robot@192.168.1.20 \
    "source /opt/ros/humble/setup.bash && \
     source /home/robot/robot_controller_release/ros2_packages/setup.bash && \
     ros2 launch livox_lidar_node start_node.launch.py" &
sleep 3

echo -e "${YELLOW}Step 2: Starting Livox TCP bridge on Jetson...${NC}"
sshpass -p "123" ssh -o StrictHostKeyChecking=no robot@192.168.1.20 \
    "source /opt/ros/humble/setup.bash && \
     source /home/robot/robot_controller_release/ros2_packages/setup.bash && \
     python3 /home/robot/livox_tcp_bridge.py" &
sleep 2

echo -e "${YELLOW}Step 3: Starting IMU TCP bridge on Jetson...${NC}"
sshpass -p "123" ssh -o StrictHostKeyChecking=no robot@192.168.1.20 \
    "source /opt/ros/humble/setup.bash && \
     source /home/robot/robot_controller_release/ros2_packages/setup.bash && \
     python3 /home/robot/imu_tcp_bridge.py" &
sleep 2

echo -e "${YELLOW}Step 4: Starting Livox TCP receiver on Mini PC...${NC}"
python3 ${HEXPLORER_DIR}/bridges/livox_tcp_receiver.py &
sleep 1

echo -e "${YELLOW}Step 5: Starting IMU TCP receiver on Mini PC...${NC}"
python3 ${HEXPLORER_DIR}/bridges/imu_tcp_receiver.py &
sleep 1

echo -e "${YELLOW}Step 6: Starting Fast-LIO...${NC}"
ros2 launch fast_lio mapping.launch.py config_file:=hexplorer_mid360.yaml rviz:=false &
FASTLIO_PID=$!
sleep 3

echo -e "${YELLOW}Step 7: Starting RViz...${NC}"
rviz2 -d ${HEXPLORER_DIR}/config/fastlio_visualization.rviz &

echo -e "${GREEN}=== Fast-LIO SLAM running ===${NC}"
echo ""
echo "Topics available:"
echo "  /cloud_registered      - Registered point cloud"
echo "  /cloud_registered_body - Point cloud in body frame"
echo "  /Laser_map            - Global map (green in RViz)"
echo "  /Odometry             - Robot odometry"
echo "  /path                 - Trajectory path (red in RViz)"
echo "  /livox/pc2            - Raw LiDAR input (PointCloud2)"
echo "  /livox/imu            - IMU input"
echo ""
echo "Press Ctrl+C to stop"

# Wait
wait $FASTLIO_PID
