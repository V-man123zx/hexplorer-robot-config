#!/usr/bin/env python3
"""
Smart Object Follower with Obstacle Avoidance and SLAM-based Search

Combines object tracking with LiDAR-based obstacle avoidance and SLAM-based
frontier exploration when target is lost.

State Machine:
    INIT -> IDLE -> FOLLOWING -> (EVADE | BLOCKED) -> FOLLOWING
                 -> SEARCH (turn-in-place -> explore frontiers -> patrol)
                 -> FOLLOWING

Search Sub-states:
    1. TURN_IN_PLACE (5s): Turn toward where target was last seen
    2. EXPLORE: Navigate to frontiers (boundaries of mapped/unmapped areas)
    3. PATROL: When fully mapped, patrol random free cells

Features:
- Object following from /object_detection topic
- LiDAR-based 360-degree obstacle avoidance
- SLAM-based frontier exploration when target is lost
- No search timeout - keeps searching until target found
- Safe sit-down on Ctrl+C

IMPORTANT: SLAM system must be running separately (start_slam.sh)
The smart follower reads /map but does not control SLAM lifecycle.

Usage:
    source /home/robot/robot_controller_release/ros2_packages/setup.bash
    python3 smart_follower.py [options]

Options:
    --target-distance    Target distance to maintain from object (default: 800mm)
    --max-speed          Maximum forward speed (default: 0.3 m/s)
    --turn-speed         Angular velocity for turning (default: 0.15 rad/s)
    --obstacle-stop      Stop if obstacle closer than this (default: 0.8m)
    --obstacle-slow      Slow down if obstacle closer than this (default: 1.2m)
    --search-speed       Forward speed while searching (default: 0.1 m/s)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from custom_msg.msg import RobotCommand, LivoxPointcloud
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
import numpy as np
import struct
import json
import time
import signal
import argparse
import sys
import os
from enum import Enum, auto

# Add hexplorer navigation to path for frontier explorer
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'navigation'))
from frontier_explorer import FrontierExplorer


# Default parameters
DEFAULT_TARGET_DISTANCE = 800      # mm - maintain this distance from target
DEFAULT_CLOSE_BUFFER = 200         # mm - backup if closer than target - buffer
DEFAULT_APPROACH_BUFFER = 500      # mm - slow approach within target + buffer
DEFAULT_MAX_SPEED = 0.3            # m/s
DEFAULT_SLOW_SPEED = 0.15          # m/s
DEFAULT_BACKUP_SPEED = 0.15        # m/s
DEFAULT_TURN_SPEED = 0.15          # rad/s
DEFAULT_SEARCH_TURN_SPEED = 0.12   # rad/s
DEFAULT_SEARCH_SPEED = 0.1         # m/s

# Obstacle avoidance parameters
DEFAULT_OBSTACLE_STOP = 0.8        # m - stop if obstacle closer
DEFAULT_OBSTACLE_SLOW = 1.2        # m - slow down if obstacle closer
DEFAULT_EMERGENCY_STOP = 0.5       # m - EMERGENCY stop, never move forward

# Search parameters
TURN_IN_PLACE_DURATION = 5.0       # seconds to turn in place before exploring
GOAL_REACHED_THRESHOLD = 0.5       # meters - consider goal reached if within this distance
EXPLORE_MAX_DISTANCE = 10.0        # meters - max frontier distance to consider
PATROL_MIN_DISTANCE = 2.0          # meters - min patrol waypoint distance
PATROL_MAX_DISTANCE = 5.0          # meters - max patrol waypoint distance

# Image parameters
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
IMAGE_CENTER_X = IMAGE_WIDTH // 2
DEADZONE = 50  # pixels

# LiDAR processing parameters
LIDAR_SECTOR_ANGLE = 45            # Degrees for each direction sector (±45° = 90° total per direction)
LIDAR_HEIGHT_MIN = 0.05            # Ignore points below this (filter ground)
LIDAR_HEIGHT_MAX = 1.2             # Ignore points above this height (m)
LIDAR_MIN_DISTANCE_M = 0.3         # Ignore LiDAR points closer than this (robot body)

# Direction sectors (in degrees, 0 = forward/+X, 90 = left/+Y, -90 = right/-Y, 180 = back/-X)
# Each sector covers ±LIDAR_SECTOR_ANGLE from the center direction


class State(Enum):
    """Robot state machine states."""
    INIT = auto()
    IDLE = auto()
    FOLLOWING = auto()
    EVADE = auto()
    BLOCKED = auto()
    SEARCH = auto()
    SHUTDOWN = auto()


class SearchSubState(Enum):
    """Sub-states within SEARCH state."""
    TURN_IN_PLACE = auto()  # Turn toward last seen direction (no forward motion)
    EXPLORE = auto()        # Navigate to frontiers using SLAM map
    PATROL = auto()         # When fully mapped, patrol random free cells


class SmartFollower(Node):
    def __init__(self, args):
        super().__init__('smart_follower')

        # Store parameters
        self.target_distance = args.target_distance
        self.close_distance = self.target_distance - DEFAULT_CLOSE_BUFFER
        self.approach_distance = self.target_distance + DEFAULT_APPROACH_BUFFER
        self.max_speed = args.max_speed
        self.turn_speed = args.turn_speed
        self.slow_speed = DEFAULT_SLOW_SPEED
        self.backup_speed = DEFAULT_BACKUP_SPEED
        self.search_turn_speed = DEFAULT_SEARCH_TURN_SPEED
        self.search_speed = args.search_speed

        # Obstacle parameters
        self.obstacle_stop = args.obstacle_stop
        self.obstacle_slow = args.obstacle_slow
        self.emergency_stop = DEFAULT_EMERGENCY_STOP  # Hard safety limit

        # Publishers for robot control
        self.cmd_pub = self.create_publisher(RobotCommand, '/robot_cmd', 10)
        self.vel_pub = self.create_publisher(Twist, '/vel_cmd', 10)

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to detection topic
        self.create_subscription(
            String, '/object_detection', self.detection_callback, 10)

        # Subscribe to LiDAR (both bridged and direct topics)
        self.create_subscription(
            PointCloud2, '/livox/pointcloud', self.lidar_callback, sensor_qos)
        self.create_subscription(
            LivoxPointcloud, '/livox_Lidar_node/sn153/xyz/pointcloud',
            self.livox_callback, sensor_qos)

        # Subscribe to SLAM map and odometry for exploration
        self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, sensor_qos)

        # Frontier explorer for SLAM-based search
        self.frontier_explorer = FrontierExplorer(min_frontier_size=10)

        # Robot pose from odometry
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.last_odom_time = 0
        self._first_odom = True

        # Exploration state
        self.current_goal = None  # (x, y) goal for exploration
        self.last_map_time = 0
        self._slam_available = False

        # State machine
        self.state = State.INIT
        self.running = True
        self.is_standing = False

        # Detection state
        self.last_detection = None
        self.last_detection_time = 0
        self.last_seen_side = 0      # -1=left, 0=center, 1=right
        self.last_target_x = IMAGE_CENTER_X  # Track last position

        # LiDAR state - 360 degree obstacle detection
        self.lidar_min_distance = float('inf')      # Overall minimum (legacy, for compatibility)
        self.lidar_obstacle_side = 0                # -1=left, 0=center, 1=right (for front obstacles)
        self.lidar_front = float('inf')             # Distance to closest obstacle in front
        self.lidar_back = float('inf')              # Distance to closest obstacle behind
        self.lidar_left = float('inf')              # Distance to closest obstacle on left
        self.lidar_right = float('inf')             # Distance to closest obstacle on right
        self.last_lidar_time = 0
        self._first_lidar = True

        # Search state
        self.search_start_time = 0
        self.search_sub_state = SearchSubState.TURN_IN_PLACE
        self.search_direction = 1  # Direction to turn: -1=left, 1=right

        # Evade state
        self.evade_direction = 0

        self.get_logger().info('Smart Follower initialized')
        self.get_logger().info(f'  Target distance: {self.target_distance}mm')
        self.get_logger().info(f'  Max speed: {self.max_speed} m/s')
        self.get_logger().info(f'  Obstacle stop: {self.obstacle_stop}m')
        self.get_logger().info(f'  Obstacle slow: {self.obstacle_slow}m')
        self.get_logger().info(f'  Search mode: SLAM-based (no timeout)')
        self.get_logger().info('  NOTE: SLAM system must be running (start_slam.sh)')

    def detection_callback(self, msg):
        """Process detection message from Jetson."""
        try:
            self.last_detection = json.loads(msg.data)
            self.last_detection_time = time.time()

            if self.last_detection['detected']:
                cx = self.last_detection['center_x']
                self.last_target_x = cx
                if cx < IMAGE_CENTER_X - DEADZONE:
                    self.last_seen_side = -1  # Object on left
                elif cx > IMAGE_CENTER_X + DEADZONE:
                    self.last_seen_side = 1   # Object on right
                else:
                    self.last_seen_side = 0   # Object centered
        except json.JSONDecodeError:
            pass

    def map_callback(self, msg):
        """Process SLAM map for frontier exploration."""
        if self.frontier_explorer.update_map(msg):
            self.last_map_time = time.time()
            if not self._slam_available:
                self._slam_available = True
                self.get_logger().info('SLAM map received - frontier exploration available')

    def odom_callback(self, msg):
        """Process odometry for robot position tracking."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = np.arctan2(siny_cosp, cosy_cosp)

        self.last_odom_time = time.time()

        if self._first_odom:
            self._first_odom = False
            self.get_logger().info(f'Odometry received - position tracking active')

    def lidar_callback(self, msg):
        """Process LiDAR pointcloud (bridged PointCloud2 topic)."""
        try:
            point_step = msg.point_step
            data = bytes(msg.data) if isinstance(msg.data, list) else msg.data
            num_points = len(data) // point_step

            if num_points < 10:
                return

            # Extract x, y, z coordinates
            points = []
            for i in range(num_points):
                offset = i * point_step
                x, y, z = struct.unpack_from('fff', data, offset)
                points.append((x, y, z))

            self._process_lidar_points(np.array(points))

        except Exception as e:
            self.get_logger().warn(f'LiDAR processing error: {e}')

    def livox_callback(self, msg):
        """Process custom Livox pointcloud message (direct topic)."""
        try:
            if msg.point_num < 10:
                return

            points = []
            for p in msg.points:
                points.append((p.x, p.y, p.z))

            self._process_lidar_points(np.array(points))

        except Exception as e:
            self.get_logger().warn(f'Livox processing error: {e}')

    def _process_lidar_points(self, points):
        """
        Common LiDAR point processing logic - 360 DEGREE obstacle detection.
        Calculates minimum distance to obstacles in all directions:
        - Front (forward motion)
        - Back (reverse motion)
        - Left (left strafe)
        - Right (right strafe)
        """
        # Filter by height (ignore ground and high obstacles)
        height_mask = (points[:, 2] > LIDAR_HEIGHT_MIN) & (points[:, 2] < LIDAR_HEIGHT_MAX)
        points = points[height_mask]

        if len(points) < 10:
            self.lidar_min_distance = float('inf')
            self.lidar_front = float('inf')
            self.lidar_back = float('inf')
            self.lidar_left = float('inf')
            self.lidar_right = float('inf')
            return

        # Calculate distance and angle for each point
        # LiDAR coordinate system: +X = forward, +Y = left, +Z = up
        distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        angles_rad = np.arctan2(points[:, 1], points[:, 0])  # Angle from forward direction
        angles_deg = angles_rad * 180 / np.pi  # -180 to +180, 0 = forward

        # Filter out points too close (robot body)
        valid_mask = distances > LIDAR_MIN_DISTANCE_M
        points = points[valid_mask]
        distances = distances[valid_mask]
        angles_deg = angles_deg[valid_mask]

        if len(distances) < 5:
            self.lidar_min_distance = float('inf')
            self.lidar_front = float('inf')
            self.lidar_back = float('inf')
            self.lidar_left = float('inf')
            self.lidar_right = float('inf')
            return

        # Define sectors (each direction gets ±LIDAR_SECTOR_ANGLE)
        # Front: -45 to +45 degrees (around 0)
        # Left: +45 to +135 degrees (around +90)
        # Back: +135 to +180 and -180 to -135 (around ±180)
        # Right: -135 to -45 degrees (around -90)

        front_mask = (angles_deg >= -LIDAR_SECTOR_ANGLE) & (angles_deg <= LIDAR_SECTOR_ANGLE)
        left_mask = (angles_deg > LIDAR_SECTOR_ANGLE) & (angles_deg <= 180 - LIDAR_SECTOR_ANGLE)
        back_mask = (angles_deg > 180 - LIDAR_SECTOR_ANGLE) | (angles_deg < -180 + LIDAR_SECTOR_ANGLE)
        right_mask = (angles_deg >= -180 + LIDAR_SECTOR_ANGLE) & (angles_deg < -LIDAR_SECTOR_ANGLE)

        # Calculate minimum distance for each direction (using 10th percentile for robustness)
        front_distances = distances[front_mask]
        back_distances = distances[back_mask]
        left_distances = distances[left_mask]
        right_distances = distances[right_mask]

        self.lidar_front = np.percentile(front_distances, 10) if len(front_distances) > 3 else float('inf')
        self.lidar_back = np.percentile(back_distances, 10) if len(back_distances) > 3 else float('inf')
        self.lidar_left = np.percentile(left_distances, 10) if len(left_distances) > 3 else float('inf')
        self.lidar_right = np.percentile(right_distances, 10) if len(right_distances) > 3 else float('inf')

        # Legacy: overall minimum distance (front only for backward compatibility)
        self.lidar_min_distance = self.lidar_front

        if self._first_lidar:
            self.get_logger().info(
                f'LiDAR 360° active! F:{self.lidar_front:.1f}m B:{self.lidar_back:.1f}m '
                f'L:{self.lidar_left:.1f}m R:{self.lidar_right:.1f}m')
            self._first_lidar = False

        # Determine obstacle side (for front obstacles, used in state machine)
        front_points = points[front_mask]
        if len(front_distances) > 3:
            close_mask = front_distances < self.obstacle_stop
            if np.sum(close_mask) > 3:
                close_points = front_points[close_mask]
                avg_y = np.mean(close_points[:, 1])
                if avg_y > 0.15:
                    self.lidar_obstacle_side = 1   # Obstacle on left
                elif avg_y < -0.15:
                    self.lidar_obstacle_side = -1  # Obstacle on right
                else:
                    self.lidar_obstacle_side = 0   # Center
            else:
                self.lidar_obstacle_side = 0

        self.last_lidar_time = time.time()

    def get_target_direction(self):
        """Get which direction the target is relative to center.
        Returns: -1 (left), 0 (center), 1 (right)
        """
        if self.last_detection and self.last_detection['detected']:
            cx = self.last_detection['center_x']
            if cx < IMAGE_CENTER_X - DEADZONE:
                return -1
            elif cx > IMAGE_CENTER_X + DEADZONE:
                return 1
        return 0

    def is_obstacle_blocking_target(self):
        """Check if obstacle is between robot and target."""
        if self.lidar_min_distance > self.obstacle_stop:
            return False

        target_dir = self.get_target_direction()
        obstacle_side = self.lidar_obstacle_side

        # Obstacle is blocking if it's in the center or same side as target
        if obstacle_side == 0:
            return True  # Center obstacle blocks everything
        if target_dir == 0:
            return True  # Target centered, any close obstacle blocks
        if target_dir == obstacle_side:
            return True  # Target and obstacle on same side

        return False

    def can_evade_around_obstacle(self):
        """Check if we can steer around obstacle to reach target.
        Returns: (can_evade, direction) where direction is turn direction
        """
        if self.lidar_min_distance > self.obstacle_stop:
            return False, 0

        target_dir = self.get_target_direction()
        obstacle_side = self.lidar_obstacle_side

        # Can evade if target is on opposite side of obstacle
        if obstacle_side != 0 and target_dir != 0:
            if target_dir != obstacle_side:
                # Target is on opposite side - steer toward target
                return True, -target_dir  # Turn toward target
        return False, 0

    def apply_safety_limits(self, vel, lidar_fresh):
        """
        GLOBAL SAFETY CHECK - 360 DEGREE obstacle avoidance.
        Applies to ALL states before publishing velocity.
        Checks obstacles in the direction of intended movement.

        Returns: (modified_vel, safety_status) where safety_status is None if no intervention
        """
        safety_status = None
        safety_messages = []

        # If no fresh LiDAR data, don't allow any motion (be conservative)
        if not lidar_fresh:
            if vel.linear.x != 0 or vel.linear.y != 0:
                vel.linear.x = 0.0
                vel.linear.y = 0.0
                safety_status = 'SAFETY: No LiDAR - stopped'
            return vel, safety_status

        # ============================================================
        # CHECK FORWARD DIRECTION (vel.linear.x > 0)
        # ============================================================
        if vel.linear.x > 0:
            if self.lidar_front < self.emergency_stop:
                # EMERGENCY: Very close obstacle in front
                vel.linear.x = 0.0
                if self.lidar_obstacle_side != 0:
                    vel.angular.z = self.turn_speed * (-self.lidar_obstacle_side)
                elif vel.angular.z == 0:
                    vel.angular.z = self.turn_speed
                safety_messages.append(f'EMERGENCY FRONT {self.lidar_front:.2f}m!')

            elif self.lidar_front < self.obstacle_stop:
                # Close obstacle in front - stop forward motion
                vel.linear.x = 0.0
                if abs(vel.angular.z) < 0.01:
                    if self.lidar_obstacle_side != 0:
                        vel.angular.z = self.turn_speed * (-self.lidar_obstacle_side)
                    else:
                        vel.angular.z = self.turn_speed
                safety_messages.append(f'STOP FRONT {self.lidar_front:.2f}m')

            elif self.lidar_front < self.obstacle_slow:
                # Slow zone - reduce forward speed
                speed_factor = (self.lidar_front - self.obstacle_stop) / \
                               (self.obstacle_slow - self.obstacle_stop)
                speed_factor = max(0.0, min(1.0, speed_factor))
                vel.linear.x = vel.linear.x * speed_factor
                if speed_factor < 1.0:
                    safety_messages.append(f'SLOW FRONT {self.lidar_front:.2f}m ({speed_factor:.0%})')

        # ============================================================
        # CHECK BACKWARD DIRECTION (vel.linear.x < 0)
        # ============================================================
        if vel.linear.x < 0:
            if self.lidar_back < self.emergency_stop:
                # EMERGENCY: Very close obstacle behind
                vel.linear.x = 0.0
                safety_messages.append(f'EMERGENCY BACK {self.lidar_back:.2f}m!')

            elif self.lidar_back < self.obstacle_stop:
                # Close obstacle behind - stop backward motion
                vel.linear.x = 0.0
                safety_messages.append(f'STOP BACK {self.lidar_back:.2f}m')

            elif self.lidar_back < self.obstacle_slow:
                # Slow zone - reduce backward speed
                speed_factor = (self.lidar_back - self.obstacle_stop) / \
                               (self.obstacle_slow - self.obstacle_stop)
                speed_factor = max(0.0, min(1.0, speed_factor))
                vel.linear.x = vel.linear.x * speed_factor  # Note: already negative
                if speed_factor < 1.0:
                    safety_messages.append(f'SLOW BACK {self.lidar_back:.2f}m ({speed_factor:.0%})')

        # ============================================================
        # CHECK LEFT DIRECTION (vel.linear.y > 0, if robot can strafe)
        # ============================================================
        if hasattr(vel.linear, 'y') and vel.linear.y > 0:
            if self.lidar_left < self.emergency_stop:
                vel.linear.y = 0.0
                safety_messages.append(f'EMERGENCY LEFT {self.lidar_left:.2f}m!')

            elif self.lidar_left < self.obstacle_stop:
                vel.linear.y = 0.0
                safety_messages.append(f'STOP LEFT {self.lidar_left:.2f}m')

            elif self.lidar_left < self.obstacle_slow:
                speed_factor = (self.lidar_left - self.obstacle_stop) / \
                               (self.obstacle_slow - self.obstacle_stop)
                speed_factor = max(0.0, min(1.0, speed_factor))
                vel.linear.y = vel.linear.y * speed_factor
                if speed_factor < 1.0:
                    safety_messages.append(f'SLOW LEFT {self.lidar_left:.2f}m ({speed_factor:.0%})')

        # ============================================================
        # CHECK RIGHT DIRECTION (vel.linear.y < 0, if robot can strafe)
        # ============================================================
        if hasattr(vel.linear, 'y') and vel.linear.y < 0:
            if self.lidar_right < self.emergency_stop:
                vel.linear.y = 0.0
                safety_messages.append(f'EMERGENCY RIGHT {self.lidar_right:.2f}m!')

            elif self.lidar_right < self.obstacle_stop:
                vel.linear.y = 0.0
                safety_messages.append(f'STOP RIGHT {self.lidar_right:.2f}m')

            elif self.lidar_right < self.obstacle_slow:
                speed_factor = (self.lidar_right - self.obstacle_stop) / \
                               (self.obstacle_slow - self.obstacle_stop)
                speed_factor = max(0.0, min(1.0, speed_factor))
                vel.linear.y = vel.linear.y * speed_factor
                if speed_factor < 1.0:
                    safety_messages.append(f'SLOW RIGHT {self.lidar_right:.2f}m ({speed_factor:.0%})')

        # ============================================================
        # EMERGENCY: Obstacle very close in ANY direction - stop everything
        # ============================================================
        min_all_directions = min(self.lidar_front, self.lidar_back,
                                  self.lidar_left, self.lidar_right)
        if min_all_directions < self.emergency_stop * 0.7:  # Extra safety margin
            vel.linear.x = 0.0
            if hasattr(vel.linear, 'y'):
                vel.linear.y = 0.0
            # Find safest direction and turn toward it
            if self.lidar_front >= self.lidar_back:
                # Front is clearer, turn toward clearer side
                if self.lidar_left > self.lidar_right:
                    vel.angular.z = self.turn_speed
                else:
                    vel.angular.z = -self.turn_speed
            safety_messages.append(f'EMERGENCY ALL {min_all_directions:.2f}m!')

        # Combine safety messages
        if safety_messages:
            safety_status = ' | '.join(safety_messages)

        return vel, safety_status

    def stand_up(self):
        """Bring robot to walking-ready state."""
        cmd = RobotCommand()
        self.get_logger().info('Standing up...')

        # STANDDOWN(1) -> STANDUP(2) -> BALANCESTAND(3)
        for state in [1, 2, 3]:
            cmd.target_state = state
            state_names = {1: 'STANDDOWN', 2: 'STANDUP', 3: 'BALANCESTAND'}
            self.get_logger().info(f'  State {state} ({state_names.get(state, "?")})')

            for _ in range(40):  # 2 seconds per state
                if not self.running:
                    return False
                self.cmd_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.05)

        self.is_standing = True
        self.get_logger().info('Standing complete')
        return True

    def sit_down(self):
        """Return robot to passive/damping mode."""
        cmd = RobotCommand()
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0

        self.get_logger().info('Sitting down...')

        # Stop velocity first
        for _ in range(20):
            self.vel_pub.publish(vel)
            time.sleep(0.05)

        # Proper sit-down: BALANCESTAND(3) -> STANDDOWN(1) -> PASSIVE(0)
        for state in [3, 1, 0]:
            cmd.target_state = state
            for _ in range(40):
                self.cmd_pub.publish(cmd)
                time.sleep(0.05)

        self.is_standing = False
        self.get_logger().info('Robot in passive mode')

    def run(self):
        """Main control loop with state machine."""
        # Stand up first
        if not self.stand_up():
            return

        cmd = RobotCommand()
        vel = Twist()
        cmd.target_state = 4  # WALK mode

        self.state = State.IDLE
        log_counter = 0

        self.get_logger().info('Smart follower running - Ctrl+C to stop')
        self.get_logger().info('Waiting for detection on /object_detection and LiDAR on /livox/pointcloud')

        try:
            while self.running:
                vel.linear.x = 0.0
                vel.angular.z = 0.0
                status = ''

                now = time.time()
                detection_fresh = (now - self.last_detection_time) < 0.5
                lidar_fresh = (now - self.last_lidar_time) < 1.0
                target_detected = (detection_fresh and self.last_detection and
                                   self.last_detection['detected'])
                obstacle_close = (lidar_fresh and
                                  self.lidar_min_distance < self.obstacle_stop)

                # State machine transitions and actions
                if self.state == State.IDLE:
                    if target_detected:
                        self.state = State.FOLLOWING
                        self.get_logger().info('Target detected - FOLLOWING')
                    else:
                        status = 'IDLE - waiting for target'
                        # Even in IDLE, turn away from very close obstacles
                        if lidar_fresh and self.lidar_min_distance < self.emergency_stop:
                            if self.lidar_obstacle_side != 0:
                                vel.angular.z = self.turn_speed * (-self.lidar_obstacle_side)
                            else:
                                vel.angular.z = self.turn_speed
                            status = f'IDLE - avoiding obstacle at {self.lidar_min_distance:.2f}m'

                elif self.state == State.FOLLOWING:
                    if not target_detected:
                        # Target lost
                        self.state = State.SEARCH
                        self.search_start_time = now
                        self.search_phase = 0
                        self.search_direction = -self.last_seen_side if self.last_seen_side != 0 else 1
                        self.get_logger().info(f'Target lost - SEARCH phase 0')
                    elif obstacle_close:
                        if self.is_obstacle_blocking_target():
                            self.state = State.BLOCKED
                            self.get_logger().info(
                                f'Obstacle blocking at {self.lidar_min_distance:.2f}m - BLOCKED')
                        else:
                            can_evade, direction = self.can_evade_around_obstacle()
                            if can_evade:
                                self.state = State.EVADE
                                self.evade_direction = direction
                                self.get_logger().info(
                                    f'Evading obstacle at {self.lidar_min_distance:.2f}m - EVADE '
                                    f'{"left" if direction > 0 else "right"}')
                            else:
                                self.state = State.BLOCKED
                                self.get_logger().info(
                                    f'Obstacle at {self.lidar_min_distance:.2f}m - BLOCKED')
                    else:
                        # Normal following behavior
                        vel = self._follow_target(vel)
                        status = self._get_follow_status()

                elif self.state == State.EVADE:
                    if not target_detected:
                        self.state = State.SEARCH
                        self.search_start_time = now
                        self.search_phase = 0
                        self.search_direction = -self.last_seen_side if self.last_seen_side != 0 else 1
                        self.get_logger().info('Target lost during evade - SEARCH')
                    elif not obstacle_close:
                        self.state = State.FOLLOWING
                        self.get_logger().info('Path clear - FOLLOWING')
                    else:
                        # Steer around obstacle while tracking target
                        cx = self.last_detection['center_x']
                        error = cx - IMAGE_CENTER_X

                        # Reduce forward speed, steer around
                        vel.linear.x = self.slow_speed * 0.5
                        # Combine evade direction with centering
                        evade_turn = self.turn_speed * self.evade_direction
                        center_turn = -self.turn_speed * 0.3 * (error / 200.0) if abs(error) > DEADZONE else 0
                        vel.angular.z = evade_turn + center_turn

                        status = f'EVADE {"left" if self.evade_direction > 0 else "right"} obs={self.lidar_min_distance:.2f}m'

                elif self.state == State.BLOCKED:
                    if not target_detected:
                        self.state = State.SEARCH
                        self.search_start_time = now
                        self.search_phase = 0
                        self.get_logger().info('Target lost while blocked - SEARCH')
                    elif not obstacle_close:
                        self.state = State.FOLLOWING
                        self.get_logger().info('Path clear - FOLLOWING')
                    else:
                        can_evade, direction = self.can_evade_around_obstacle()
                        if can_evade:
                            self.state = State.EVADE
                            self.evade_direction = direction
                            self.get_logger().info(f'Can evade - switching to EVADE')
                        else:
                            # Stay stopped, keep tracking target direction
                            cx = self.last_detection['center_x']
                            error = cx - IMAGE_CENTER_X
                            if abs(error) > DEADZONE:
                                vel.angular.z = -self.turn_speed * 0.3 * np.sign(error)
                            status = f'BLOCKED at {self.lidar_min_distance:.2f}m - waiting'

                elif self.state == State.SEARCH:
                    if target_detected:
                        self.state = State.FOLLOWING
                        self.search_start_time = 0
                        self.current_goal = None
                        self.get_logger().info('Target found - FOLLOWING')
                    else:
                        search_time = now - self.search_start_time
                        vel, status = self._search_pattern(vel, search_time)

                        # Obstacle avoidance during search (handled within _search_pattern
                        # for goal navigation, but add extra safety here)
                        if obstacle_close and vel.linear.x > 0:
                            # Stop forward motion if obstacle in way
                            vel.linear.x = 0.0
                            if self.lidar_obstacle_side != 0:
                                vel.angular.z = self.turn_speed * (-self.lidar_obstacle_side)
                            else:
                                vel.angular.z = self.turn_speed * self.search_direction
                            status = f'SEARCH AVOID obs={self.lidar_min_distance:.2f}m'

                        # No timeout - keep searching until target found

                # ============================================================
                # GLOBAL SAFETY CHECK - applies to ALL states
                # This is the final safeguard - robot will NEVER hit obstacles
                # ============================================================
                vel, safety_status = self.apply_safety_limits(vel, lidar_fresh)
                if safety_status:
                    status = safety_status  # Override status with safety message

                # Publish commands
                self.cmd_pub.publish(cmd)
                self.vel_pub.publish(vel)

                # Log periodically
                log_counter += 1
                if log_counter % 25 == 0:
                    if lidar_fresh:
                        lidar_status = f'F:{self.lidar_front:.1f} B:{self.lidar_back:.1f} L:{self.lidar_left:.1f} R:{self.lidar_right:.1f}'
                    else:
                        lidar_status = 'LiDAR:?'
                    self.get_logger().info(f'{status} [{lidar_status}]')

                # Log safety events immediately (not just periodically)
                if safety_status and 'EMERGENCY' in safety_status:
                    self.get_logger().warn(safety_status)

                # Process callbacks
                rclpy.spin_once(self, timeout_sec=0.01)

                # Maintain ~25Hz
                time.sleep(0.04)

        except KeyboardInterrupt:
            self.get_logger().info('Interrupted')
        finally:
            self.sit_down()

    def _follow_target(self, vel):
        """Execute target following behavior. Returns updated velocity."""
        cx = self.last_detection['center_x']
        distance = self.last_detection['distance_mm']
        error = cx - IMAGE_CENTER_X

        # Horizontal centering (turning)
        if abs(error) > DEADZONE:
            turn_scale = min(1.0, abs(error) / 200.0)
            if error > 0:
                vel.angular.z = -self.turn_speed * turn_scale  # Turn right
            else:
                vel.angular.z = self.turn_speed * turn_scale   # Turn left

        # Distance control (forward/backward)
        if distance > 0:
            # Check for obstacles in slow zone
            if self.lidar_min_distance < self.obstacle_slow:
                # Reduce speed based on obstacle proximity
                speed_factor = max(0.3, (self.lidar_min_distance - self.obstacle_stop) /
                                   (self.obstacle_slow - self.obstacle_stop))
            else:
                speed_factor = 1.0

            if distance < self.close_distance:
                # Too close - back up (only if clear behind!)
                if self.lidar_back > self.obstacle_stop:
                    vel.linear.x = -self.backup_speed
                else:
                    vel.linear.x = 0.0  # Can't back up, obstacle behind
            elif distance < self.target_distance:
                # Within target zone - hold position
                vel.linear.x = 0.0
            elif distance < self.approach_distance:
                # Slow approach zone
                vel.linear.x = self.slow_speed * speed_factor
            else:
                # Far away - full speed (adjusted for obstacles)
                vel.linear.x = self.max_speed * speed_factor
        return vel

    def _get_follow_status(self):
        """Get status string for following mode."""
        cx = self.last_detection['center_x']
        distance = self.last_detection['distance_mm']
        error = cx - IMAGE_CENTER_X

        if distance < self.close_distance:
            return f'BACKUP dist={distance}mm err={error:+d}px'
        elif distance < self.target_distance:
            return f'HOLD dist={distance}mm err={error:+d}px'
        elif distance < self.approach_distance:
            return f'SLOW dist={distance}mm err={error:+d}px'
        else:
            return f'FOLLOW dist={distance}mm err={error:+d}px'

    def _search_pattern(self, vel, search_time):
        """
        Execute SLAM-based search pattern. Returns (velocity, status).

        Search sub-states:
        1. TURN_IN_PLACE (0-5s): Turn in place toward last seen direction
        2. EXPLORE: Navigate to frontiers using SLAM map
        3. PATROL: When fully mapped, patrol random free cells
        """
        # Sub-state 1: Turn in place (first 5 seconds)
        if search_time < TURN_IN_PLACE_DURATION:
            if self.search_sub_state != SearchSubState.TURN_IN_PLACE:
                self.search_sub_state = SearchSubState.TURN_IN_PLACE
                self.current_goal = None
                self.get_logger().info(f'Search: TURN_IN_PLACE phase')

            # Turn in place - NO forward motion
            vel.linear.x = 0.0
            vel.angular.z = self.search_turn_speed * self.search_direction
            direction = "left" if self.search_direction > 0 else "right"
            return vel, f'SEARCH: turn {direction} ({search_time:.1f}s/{TURN_IN_PLACE_DURATION}s)'

        # After turn-in-place, use SLAM-based exploration
        slam_available = self._slam_available and (time.time() - self.last_map_time) < 5.0
        odom_available = (time.time() - self.last_odom_time) < 1.0

        if not slam_available or not odom_available:
            # Fall back to simple turn if no SLAM data
            vel.linear.x = 0.0
            vel.angular.z = self.search_turn_speed * self.search_direction
            return vel, f'SEARCH: no SLAM - turning (map:{slam_available} odom:{odom_available})'

        # Sub-state 2: EXPLORE - navigate to frontiers
        if self.search_sub_state != SearchSubState.PATROL:
            # Check if we have a goal or need a new one
            if self.current_goal is None or self._goal_reached(self.current_goal):
                if self.current_goal is not None:
                    self.get_logger().info(f'Frontier goal reached at ({self.current_goal[0]:.1f}, {self.current_goal[1]:.1f})')

                # Find new frontier
                frontiers = self.frontier_explorer.find_frontiers()
                new_goal = self.frontier_explorer.select_best_frontier(
                    self.robot_x, self.robot_y, max_distance=EXPLORE_MAX_DISTANCE)

                if new_goal is not None:
                    self.current_goal = new_goal
                    self.search_sub_state = SearchSubState.EXPLORE
                    self.get_logger().info(f'New frontier goal: ({new_goal[0]:.1f}, {new_goal[1]:.1f})')
                else:
                    # No frontiers - room fully mapped, switch to patrol
                    self.search_sub_state = SearchSubState.PATROL
                    self.current_goal = None
                    self.get_logger().info('No frontiers - switching to PATROL mode')

            if self.search_sub_state == SearchSubState.EXPLORE and self.current_goal is not None:
                vel, status = self._navigate_to_goal(vel, self.current_goal)
                return vel, f'EXPLORE: {status}'

        # Sub-state 3: PATROL - room fully mapped, patrol random locations
        if self.search_sub_state == SearchSubState.PATROL:
            if self.current_goal is None or self._goal_reached(self.current_goal):
                # Pick a new random free cell
                new_goal = self.frontier_explorer.get_random_free_cell(
                    self.robot_x, self.robot_y,
                    min_distance=PATROL_MIN_DISTANCE,
                    max_distance=PATROL_MAX_DISTANCE)

                if new_goal is not None:
                    self.current_goal = new_goal
                    self.get_logger().info(f'New patrol waypoint: ({new_goal[0]:.1f}, {new_goal[1]:.1f})')
                else:
                    # No valid patrol points - just turn and look
                    vel.linear.x = 0.0
                    vel.angular.z = self.search_turn_speed * self.search_direction
                    return vel, 'PATROL: no waypoints - turning'

            if self.current_goal is not None:
                vel, status = self._navigate_to_goal(vel, self.current_goal)
                return vel, f'PATROL: {status}'

        # Fallback
        vel.linear.x = 0.0
        vel.angular.z = self.search_turn_speed * self.search_direction
        return vel, 'SEARCH: fallback turning'

    def _navigate_to_goal(self, vel, goal):
        """
        Navigate toward a goal position.
        Returns (velocity, status_string).
        """
        goal_x, goal_y = goal

        # Calculate angle to goal
        dx = goal_x - self.robot_x
        dy = goal_y - self.robot_y
        distance = np.sqrt(dx * dx + dy * dy)
        angle_to_goal = np.arctan2(dy, dx)

        # Calculate angle error (normalize to -pi, pi)
        angle_error = angle_to_goal - self.robot_yaw
        while angle_error > np.pi:
            angle_error -= 2 * np.pi
        while angle_error < -np.pi:
            angle_error += 2 * np.pi

        # Turn toward goal
        if abs(angle_error) > 0.3:  # ~17 degrees
            # Need to turn significantly - don't move forward
            vel.linear.x = 0.0
            vel.angular.z = self.search_turn_speed * np.sign(angle_error) * min(1.0, abs(angle_error))
            status = f'turning to goal ({np.degrees(angle_error):+.0f}°)'
        else:
            # Roughly facing goal - move forward while correcting
            vel.linear.x = self.search_speed
            vel.angular.z = self.turn_speed * 0.5 * angle_error  # Proportional correction
            status = f'moving to goal (d={distance:.1f}m, err={np.degrees(angle_error):+.0f}°)'

        return vel, status

    def _goal_reached(self, goal):
        """Check if robot has reached the goal."""
        if goal is None:
            return True
        dx = goal[0] - self.robot_x
        dy = goal[1] - self.robot_y
        distance = np.sqrt(dx * dx + dy * dy)
        return distance < GOAL_REACHED_THRESHOLD


def parse_args():
    parser = argparse.ArgumentParser(
        description='Smart object follower with obstacle avoidance',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('--target-distance', type=int, default=DEFAULT_TARGET_DISTANCE,
                        help='Target distance to maintain (mm)')
    parser.add_argument('--max-speed', type=float, default=DEFAULT_MAX_SPEED,
                        help='Maximum forward speed (m/s)')
    parser.add_argument('--turn-speed', type=float, default=DEFAULT_TURN_SPEED,
                        help='Angular velocity for turning (rad/s)')
    parser.add_argument('--obstacle-stop', type=float, default=DEFAULT_OBSTACLE_STOP,
                        help='Stop if obstacle closer than this (m)')
    parser.add_argument('--obstacle-slow', type=float, default=DEFAULT_OBSTACLE_SLOW,
                        help='Slow down if obstacle closer than this (m)')
    parser.add_argument('--search-speed', type=float, default=DEFAULT_SEARCH_SPEED,
                        help='Forward speed while searching (m/s)')
    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init()
    node = SmartFollower(args)

    def signal_handler(sig, frame):
        node.get_logger().info('Stopping...')
        node.running = False

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
