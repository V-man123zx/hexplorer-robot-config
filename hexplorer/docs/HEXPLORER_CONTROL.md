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

## Robot States

| State | Value | Description |
|-------|-------|-------------|
| PASSIVE | 0 | Damping mode - legs compliant |
| STANDDOWN | 1 | Position folding |
| STANDUP | 2 | Legs extend |
| BALANCESTAND | 3 | Force-control standing |
| WALK | 4 | Walking mode - accepts velocity commands |

**Transition sequence:** `PASSIVE (0) → STANDDOWN (1) → STANDUP (2) → BALANCESTAND (3) → WALK (4)`

Each state requires ~2 seconds of continuous publishing before transitioning.

## ROS2 Topics

### Control Topics
| Topic | Type | Purpose |
|-------|------|---------|
| `/robot_cmd` | `custom_msg/msg/RobotCommand` | State commands |
| `/robot_state` | `custom_msg/msg/RobotState` | Robot feedback |
| `/vel_cmd` | `geometry_msgs/msg/Twist` | Velocity commands |
| `/joy` | `sensor_msgs/msg/Joy` | Joystick input |

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

## Gamepad Controls (Thor G30s)

| Button/Stick | Action |
|--------------|--------|
| LT + A | Toggle position mode / standing |
| START | Enter walk mode |
| Left Stick | Forward/backward/strafe |
| Right Stick | Turn |
| RT + Dpad Up | Boxing pose (gamepad-internal only, not via ROS2) |
| RT + Dpad Down | Dance pose (gamepad-internal only, not via ROS2) |

## Quick Commands

```bash
# Source environment
source /home/robot/robot_controller_release/ros2_packages/setup.bash

# Walk forward/backward 0.5m
python3 /home/robot/robot_controller_release/walk_forward.py 0.5
python3 /home/robot/robot_controller_release/walk_forward.py -0.5

# Check robot state
ros2 topic echo /robot_state --once | grep control_cmd

# Check if controller is running
ps aux | grep "./main" | grep -v grep
```

## Network Configuration

Controller requires IP 192.168.1.10 on the ethernet interface:
```bash
nmcli connection modify enp2s0 ipv4.addresses 192.168.1.10/24 ipv4.method manual
nmcli connection up enp2s0
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Controller not starting | Check IP is set to 192.168.1.10 |
| Commands ignored | Must publish continuously at 20Hz, not single messages |
| Robot falls after standing | Need to maintain state 3 or 4 with continuous publishing |
| No sensor data | Check Jetson is running, sensors connected |

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
