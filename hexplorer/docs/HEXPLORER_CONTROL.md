# Dobot Hexplorer ROS2 Control Guide

## Overview

The Dobot Hexplorer is an 18-DOF hexapod robot with 6 legs, each having 3 joints. It runs ROS2 Humble on Ubuntu 22.04.

## CRITICAL: Command Publishing Requirements

**WRONG (does not work):**
```bash
ros2 topic pub -1 /robot_cmd custom_msg/msg/RobotCommand "{target_state: 4}"
```
This sends ONE message and exits. The robot ignores single messages.

**CORRECT (works):**
Must publish commands **continuously at ~20Hz** using Python:
```python
for _ in range(40):  # 2 seconds
    cmd_pub.publish(cmd)
    time.sleep(0.05)
```

**Root Cause:** The Hexplorer controller requires continuous command publishing to maintain state transitions. A single message is received but the state machine doesn't hold - it falls back to PASSIVE mode.

## Robot States

| State | Value | Description |
|-------|-------|-------------|
| PASSIVE | 0 | Damping mode - legs compliant |
| STANDDOWN | 1 | Position folding - legs fold into position |
| STANDUP | 2 | Legs extend to standing position |
| BALANCESTAND | 3 | Force-control standing |
| WALK | 4 | Walking mode - accepts velocity commands |

## State Transition Sequence

From PASSIVE (damping) to WALK:
```
PASSIVE (0) → STANDDOWN (1) → STANDUP (2) → BALANCESTAND (3) → WALK (4)
```

**Important:** Each state requires continuous publishing (~2 seconds) before transitioning to the next.

## Network Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Network (192.168.1.x)                       │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ┌─────────────────┐         ┌─────────────────────────────────┐  │
│   │   Intel Mini PC │ <─────> │   Jetson Orin Nano              │  │
│   │   192.168.1.10  │  Eth    │   192.168.1.20                  │  │
│   │                 │         │                                 │  │
│   │ - Robot Ctrl    │         │ - realsense_camera_node         │  │
│   │ - joy_node      │         │ - livox_lidar_node              │  │
│   │ - ROS2 Master   │         │                                 │  │
│   └─────────────────┘         │   ┌───────────┐ ┌────────────┐  │  │
│                               │   │ RealSense │ │ Livox      │  │  │
│                               │   │ D435 (USB)│ │ Mid360     │  │  │
│                               │   └───────────┘ │192.168.1.190│  │  │
│                               │                 └────────────┘  │  │
│                               └─────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

### Device IPs
| Device | IP Address | Purpose |
|--------|------------|---------|
| Intel Mini PC | 192.168.1.10 | Robot controller, ROS2 master |
| Jetson Orin Nano | 192.168.1.20 | Sensor processing |
| Livox Mid360 LiDAR | 192.168.1.190 | 3D scanning |

### Jetson NFS Share
```bash
# Jetson home mounted at:
/.update_share_folder/nano/
```

## ROS2 Topics

### Control Topics
| Topic | Type | Purpose |
|-------|------|---------|
| `/robot_cmd` | `custom_msg/msg/RobotCommand` | State commands |
| `/robot_state` | `custom_msg/msg/RobotState` | Robot feedback |
| `/vel_cmd` | `geometry_msgs/msg/Twist` | Velocity commands |
| `/joy` | `sensor_msgs/msg/Joy` | Joystick input |

### Sensor Topics (from Jetson)
| Topic | Type | Purpose |
|-------|------|---------|
| `/realsense_camera_node/sn.../color/bgr/image_raw` | `sensor_msgs/msg/Image` | Color camera |
| `/realsense_camera_node/sn.../depth/image_raw` | `sensor_msgs/msg/Image` | Depth camera |
| `/livox_Lidar_node/sn.../xyz/pointcloud` | `sensor_msgs/msg/PointCloud2` | LiDAR point cloud |

## Message Definitions

### RobotCommand (`custom_msg/msg/RobotCommand`)

| Field | Type | Description |
|-------|------|-------------|
| header | std_msgs/Header | ROS header with timestamp |
| target_state | uint8 | Target state (0-4) |
| temp | float[12] | Reserved field |

### RobotState (`custom_msg/msg/RobotState`)

| Field | Type | Description |
|-------|------|-------------|
| header | std_msgs/Header | ROS header with timestamp |
| control_cmd | uint8 | Current state (0-4) |
| jpos_leg | float[18] | Joint positions (rad) |
| jvel_leg | float[18] | Joint velocities (rad/s) |
| jtau_leg | float[18] | Joint torques (Nm) |
| jerror | float[18] | Joint position errors |
| jpos_leg_des | float[18] | Desired joint positions |
| jvel_leg_des | float[18] | Desired joint velocities |
| jtau_leg_des | float[18] | Desired joint torques |
| pos_body | float[3] | Body position (x, y, z) in meters |
| vel_body | float[3] | Body velocity (m/s) |
| acc_body | float[3] | Body acceleration (m/s^2) |
| ori_body | float[4] | Body orientation (quaternion: x, y, z, w) |
| omega_body | float[3] | Angular velocity (rad/s) |
| temp | float[12] | Temperature readings |

### Velocity Command (`geometry_msgs/msg/Twist`)

| Field | Range | Description |
|-------|-------|-------------|
| linear.x | -0.3 to 0.3 m/s | Forward/backward |
| linear.y | -0.2 to 0.2 m/s | Strafe left/right |
| angular.z | -0.5 to 0.5 rad/s | Turn left/right |

## Joint Mapping

18 joints = 6 legs x 3 joints per leg

| Index Range | Leg |
|-------------|-----|
| 0-2 | Front Right |
| 3-5 | Front Left |
| 6-8 | Middle Right |
| 9-11 | Middle Left |
| 12-14 | Rear Right |
| 15-17 | Rear Left |

Each leg: Joint 0 = Hip, Joint 1 = Thigh, Joint 2 = Calf

## Working Python Examples

### Walk Forward Script

Location: `/home/robot/robot_controller_release/walk_forward.py`

```python
#!/usr/bin/env python3
"""Walk the Hexplorer robot forward a specified distance"""
import rclpy
from rclpy.node import Node
from custom_msg.msg import RobotCommand
from geometry_msgs.msg import Twist
import time
import sys

def walk(distance_m=0.5, speed=0.15):
    rclpy.init()
    node = Node('robot_walker')
    cmd_pub = node.create_publisher(RobotCommand, '/robot_cmd', 10)
    vel_pub = node.create_publisher(Twist, '/vel_cmd', 10)
    time.sleep(0.3)

    cmd = RobotCommand()
    vel = Twist()

    # STANDDOWN (1) - 2 sec
    cmd.target_state = 1
    for _ in range(40):
        cmd_pub.publish(cmd)
        time.sleep(0.05)

    # STANDUP (2) - 2 sec
    cmd.target_state = 2
    for _ in range(40):
        cmd_pub.publish(cmd)
        time.sleep(0.05)

    # BALANCE (3) - 2 sec
    cmd.target_state = 3
    for _ in range(40):
        cmd_pub.publish(cmd)
        time.sleep(0.05)

    # WALK (4) + velocity
    walk_time = abs(distance_m) / speed
    cmd.target_state = 4
    vel.linear.x = speed if distance_m > 0 else -speed

    for _ in range(int(walk_time / 0.05)):
        cmd_pub.publish(cmd)
        vel_pub.publish(vel)
        time.sleep(0.05)

    # PASSIVE (0)
    cmd.target_state = 0
    for _ in range(20):
        cmd_pub.publish(cmd)
        time.sleep(0.05)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    dist = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
    walk(dist)
```

### Yellow Ball Follower (Vision-Based Control)

```python
#!/usr/bin/env python3
"""Follow a yellow ball using RealSense camera"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from custom_msg.msg import RobotCommand
from cv_bridge import CvBridge
import cv2
import numpy as np

class YellowBallFollower(Node):
    def __init__(self):
        super().__init__('yellow_ball_follower')

        # Publishers
        self.cmd_pub = self.create_publisher(RobotCommand, '/robot_cmd', 10)
        self.vel_pub = self.create_publisher(Twist, '/vel_cmd', 10)

        # Subscriber - adjust topic for your camera serial number
        self.image_sub = self.create_subscription(
            Image,
            '/realsense_camera_node/sn408122070053/color/bgr/image_raw',
            self.image_callback, 10)

        self.bridge = CvBridge()
        self.cmd = RobotCommand()
        self.vel = Twist()

        # Yellow color range in HSV
        self.yellow_lower = np.array([20, 100, 100])
        self.yellow_upper = np.array([35, 255, 255])

        # Control parameters
        self.image_center_x = 320  # Assuming 640x480
        self.min_area = 500  # Minimum blob size to track

        # Start in walk mode
        self.cmd.target_state = 4

        # Control loop timer (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.ball_x = None
        self.ball_area = 0

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Create mask for yellow color
            mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Find largest contour
                largest = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest)

                if area > self.min_area:
                    M = cv2.moments(largest)
                    if M['m00'] > 0:
                        self.ball_x = int(M['m10'] / M['m00'])
                        self.ball_area = area
                        return

            self.ball_x = None
            self.ball_area = 0

        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

    def control_loop(self):
        # Always publish command to maintain state
        self.cmd_pub.publish(self.cmd)

        if self.ball_x is not None:
            # Calculate turn rate based on ball position
            error = self.ball_x - self.image_center_x
            turn_rate = -error * 0.002  # Proportional control
            turn_rate = max(-0.3, min(0.3, turn_rate))  # Clamp

            # Move forward if ball is visible
            self.vel.linear.x = 0.1
            self.vel.angular.z = turn_rate
        else:
            # Stop if no ball visible
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0

        self.vel_pub.publish(self.vel)

def main():
    rclpy.init()

    # First stand up the robot
    node = rclpy.create_node('standup_node')
    cmd_pub = node.create_publisher(RobotCommand, '/robot_cmd', 10)
    import time
    time.sleep(0.3)

    cmd = RobotCommand()
    for state in [1, 2, 3, 4]:
        cmd.target_state = state
        for _ in range(40):
            cmd_pub.publish(cmd)
            time.sleep(0.05)

    node.destroy_node()

    # Now run the follower
    follower = YellowBallFollower()
    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        pass
    finally:
        # Return to passive
        cmd = RobotCommand()
        cmd.target_state = 0
        for _ in range(20):
            follower.cmd_pub.publish(cmd)
            time.sleep(0.05)
        follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Gamepad Controls (Thor G30s)

| Button/Stick | Action |
|--------------|--------|
| LT + A | Toggle position mode / standing |
| START | Enter walk mode |
| Left Stick | Forward/backward/strafe |
| Right Stick | Turn |
| RT + Dpad Up | Boxing pose (gamepad-internal only) |
| RT + Dpad Down | Dance pose (gamepad-internal only) |

**Note:** Special poses like boxing are handled by the gamepad's internal state machine and cannot be triggered via ROS2 commands.

## Quick Commands

```bash
# Source environment
source /home/robot/robot_controller_release/ros2_packages/setup.bash

# Walk forward 0.5m
python3 /home/robot/robot_controller_release/walk_forward.py 0.5

# Walk backward 0.5m
python3 /home/robot/robot_controller_release/walk_forward.py -0.5

# Check robot state
ros2 topic echo /robot_state --once | grep control_cmd

# Check if controller is running
ps aux | grep "./main" | grep -v grep

# List all topics
ros2 topic list

# Monitor joint positions
ros2 topic echo /robot_state --field jpos_leg
```

## Network Configuration

Controller requires IP 192.168.1.10 on the ethernet interface:
```bash
nmcli connection modify enp2s0 ipv4.addresses 192.168.1.10/24 ipv4.method manual
nmcli connection up enp2s0
```

## ROS2 Visualization Tools

```bash
# 3D Visualization
rviz2

# Node/Topic Graph
ros2 run rqt_graph rqt_graph

# Data Plotting
ros2 run rqt_plot rqt_plot

# Image Viewer
ros2 run rqt_image_view rqt_image_view

# All-in-one GUI
rqt

# Generate TF tree PDF
ros2 run tf2_tools view_frames
```

### RViz2 Configuration
- **Fixed Frame**: Set to `robot_base_frame_defined`
- Add TF display to see coordinate frames
- Add PointCloud2 for lidar (topic: `/livox/lidar`)
- Add Image for camera (topic: `/camera/color/image_raw`)

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Controller not starting | Check IP is set to 192.168.1.10 |
| Commands ignored | Must publish continuously at 20Hz, not single messages |
| Robot falls after standing | Need to maintain state 3 or 4 with continuous publishing |
| No sensor data | Check Jetson is running, sensors connected |
| Camera topic not found | Verify camera serial number in topic name |

### Check Logs
```bash
tail -f /home/robot/robot_controller_release/executable/log*.txt
```

## Files Reference

| File | Purpose |
|------|---------|
| `/home/robot/robot_controller_release/walk_forward.py` | Walk script |
| `/home/robot/robot_controller_release/ros2_packages/setup.bash` | ROS2 environment |
| `/home/robot/robot_controller_release/executable/main` | Robot controller binary |
| `~/.claude/CLAUDE.md` | Claude memory (robot control notes) |
