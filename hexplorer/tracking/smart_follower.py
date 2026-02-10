#!/usr/bin/env python3
"""
Smart Object Follower with Obstacle Avoidance and Visited-Area Search

Combines object tracking with LiDAR-based obstacle avoidance and visited-area
tracking for intelligent search when target is lost.

State Machine:
    INIT -> IDLE -> FOLLOWING -> (EVADE | BLOCKED) -> FOLLOWING
                 -> SEARCH (turn-in-place -> explore unvisited areas)
                 -> FOLLOWING

Search Sub-states:
    1. TURN_IN_PLACE (5s): Turn toward where target was last seen
    2. EXPLORE_UNVISITED: Navigate toward areas the robot hasn't visited

Features:
- Object following from /object_detection topic
- LiDAR-based 360-degree obstacle avoidance
- Visited-area tracking using MOLA odometry
- No search timeout - keeps searching until target found
- Safe sit-down on Ctrl+C

Uses MOLA odometry (/state_estimator/pose) when available, falls back to /odom.

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
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import struct
import json
import time
import signal
import argparse
import math
from enum import Enum, auto


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
EXPLORE_RAY_LENGTH = 3.0           # meters - ray length for checking unvisited areas
VISITED_CELL_SIZE = 0.5            # meters - grid cell size for visited area tracking

# Image parameters
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
IMAGE_CENTER_X = IMAGE_WIDTH // 2
DEADZONE = 50  # pixels

# LiDAR processing parameters
LIDAR_SECTOR_ANGLE = 45            # Degrees for each direction sector (±45° = 90° total per direction)
LIDAR_HEIGHT_MIN = 0.10            # Ignore points below this (filter ground) - raised from 0.05
LIDAR_HEIGHT_MAX = 1.0             # Ignore points above this height (m) - lowered from 1.2
LIDAR_MIN_DISTANCE_M = 0.35        # Ignore LiDAR points closer than this (robot body)
LIDAR_MIN_POINTS_PER_SECTOR = 8    # Minimum points needed to consider obstacle real (was 3)
LIDAR_PERCENTILE = 20              # Use 20th percentile for robustness (was 10th)

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
    TURN_IN_PLACE = auto()      # Turn toward last seen direction (no forward motion)
    EXPLORE_UNVISITED = auto()  # Navigate toward unvisited areas using odometry


class VisitedAreaTracker:
    """
    Simple grid-based tracking of visited positions using odometry.

    Divides the environment into cells and tracks which cells the robot
    has visited. Used to guide search toward unexplored areas.
    """

    def __init__(self, cell_size=VISITED_CELL_SIZE):
        self.cell_size = cell_size
        self.visited = set()  # Set of (grid_x, grid_y) tuples

    def mark_visited(self, x, y):
        """Mark a position as visited."""
        gx = int(x / self.cell_size)
        gy = int(y / self.cell_size)
        self.visited.add((gx, gy))

    def is_visited(self, x, y):
        """Check if a position has been visited."""
        gx = int(x / self.cell_size)
        gy = int(y / self.cell_size)
        return (gx, gy) in self.visited

    def count_visited_in_direction(self, robot_x, robot_y, angle_rad, ray_length=EXPLORE_RAY_LENGTH):
        """
        Count visited cells along a ray in a given direction.

        Args:
            robot_x, robot_y: Current robot position
            angle_rad: Direction angle in radians (0 = +X, pi/2 = +Y)
            ray_length: How far to check

        Returns:
            Number of visited cells along the ray
        """
        visited_count = 0
        step = self.cell_size * 0.5  # Check at half-cell intervals
        distance = 0.0

        while distance < ray_length:
            distance += step
            check_x = robot_x + distance * math.cos(angle_rad)
            check_y = robot_y + distance * math.sin(angle_rad)

            if self.is_visited(check_x, check_y):
                visited_count += 1

        return visited_count

    def get_unvisited_direction(self, robot_x, robot_y, robot_yaw, num_directions=8):
        """
        Find the direction with the fewest visited cells.

        Args:
            robot_x, robot_y: Current robot position
            robot_yaw: Current robot heading (radians)
            num_directions: Number of directions to check (default: 8)

        Returns:
            (best_angle, visited_ratio) where:
            - best_angle is the absolute angle (radians) to navigate toward
            - visited_ratio is 0.0 (all unvisited) to 1.0 (all visited)
        """
        best_angle = robot_yaw  # Default: continue forward
        min_visited = float('inf')
        max_cells = int(EXPLORE_RAY_LENGTH / (self.cell_size * 0.5))  # Max possible cells

        for i in range(num_directions):
            angle = robot_yaw + (2 * math.pi * i / num_directions)
            # Normalize angle to -pi to pi
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi

            visited_count = self.count_visited_in_direction(robot_x, robot_y, angle)

            if visited_count < min_visited:
                min_visited = visited_count
                best_angle = angle

        visited_ratio = min_visited / max_cells if max_cells > 0 else 0.0
        return best_angle, visited_ratio

    def get_stats(self):
        """Get statistics about visited area."""
        return {
            'visited_cells': len(self.visited),
            'area_m2': len(self.visited) * self.cell_size * self.cell_size
        }


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

        # Publisher for state visualization
        self.state_pub = self.create_publisher(String, '/smart_follower/state', 10)

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to detection topic
        self.create_subscription(
            String, '/object_detection', self.detection_callback, 10)

        # Subscribe to LiDAR topics (prefer MOLA's filtered output)
        # Primary: MOLA filtered lidar (cleanest data)
        self.create_subscription(
            PointCloud2, '/livox/lidar_filtered', self.lidar_callback, sensor_qos)
        # Fallback: Raw lidar from TCP bridge (if MOLA not running)
        self.create_subscription(
            PointCloud2, '/livox/lidar', self.lidar_callback, sensor_qos)
        # Direct Livox topic (if running directly on Jetson network)
        self.create_subscription(
            LivoxPointcloud, '/livox_Lidar_node/sn153/xyz/pointcloud',
            self.livox_callback, sensor_qos)

        # Subscribe to MOLA odometry (preferred) and fallback /odom
        self.create_subscription(
            Odometry, '/state_estimator/pose', self.mola_odom_callback, sensor_qos)
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, sensor_qos)

        # Visited area tracker for search
        self.visited_tracker = VisitedAreaTracker(cell_size=VISITED_CELL_SIZE)
        self._mola_odom_available = False

        # Robot pose from odometry
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.last_odom_time = 0
        self._first_odom = True

        # Exploration state
        self.current_goal = None  # (x, y) goal for exploration
        self.explore_target_angle = None  # Target angle for exploration

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
        self.get_logger().info(f'  Search mode: visited-area tracking (no timeout)')
        self.get_logger().info('  Uses MOLA odometry when available, falls back to /odom')

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

    def mola_odom_callback(self, msg):
        """Process MOLA odometry (preferred source)."""
        self._update_pose_from_odometry(msg)

        if not self._mola_odom_available:
            self._mola_odom_available = True
            self.get_logger().info('MOLA odometry received - using /state_estimator/pose')

    def odom_callback(self, msg):
        """Process standard odometry (fallback if MOLA not available)."""
        # Only use if MOLA odometry is not available
        if self._mola_odom_available:
            return

        self._update_pose_from_odometry(msg)

        if self._first_odom:
            self._first_odom = False
            self.get_logger().info('Odometry received from /odom - position tracking active')

    def _update_pose_from_odometry(self, msg):
        """Common odometry processing for both MOLA and standard odom."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = np.arctan2(siny_cosp, cosy_cosp)

        self.last_odom_time = time.time()

        # Mark current position as visited
        self.visited_tracker.mark_visited(self.robot_x, self.robot_y)

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

        # Calculate minimum distance for each direction
        # Use LIDAR_PERCENTILE (20th) with minimum LIDAR_MIN_POINTS_PER_SECTOR points
        # This reduces false positives from noise/outliers
        front_distances = distances[front_mask]
        back_distances = distances[back_mask]
        left_distances = distances[left_mask]
        right_distances = distances[right_mask]

        min_pts = LIDAR_MIN_POINTS_PER_SECTOR
        pct = LIDAR_PERCENTILE
        self.lidar_front = np.percentile(front_distances, pct) if len(front_distances) >= min_pts else float('inf')
        self.lidar_back = np.percentile(back_distances, pct) if len(back_distances) >= min_pts else float('inf')
        self.lidar_left = np.percentile(left_distances, pct) if len(left_distances) >= min_pts else float('inf')
        self.lidar_right = np.percentile(right_distances, pct) if len(right_distances) >= min_pts else float('inf')

        # Legacy: overall minimum distance (front only for backward compatibility)
        self.lidar_min_distance = self.lidar_front

        if self._first_lidar:
            self.get_logger().info(
                f'LiDAR 360° active! F:{self.lidar_front:.1f}m B:{self.lidar_back:.1f}m '
                f'L:{self.lidar_left:.1f}m R:{self.lidar_right:.1f}m')
            self._first_lidar = False

        # Determine obstacle side (for front obstacles, used in state machine)
        front_points = points[front_mask]
        if len(front_distances) >= min_pts:
            close_mask = front_distances < self.obstacle_stop
            if np.sum(close_mask) >= min_pts:
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
        self.get_logger().info('Waiting for detection on /object_detection and LiDAR on /livox/lidar_filtered or /livox/lidar')

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

                # Publish state for visualization
                state_msg = String()
                state_data = {
                    'state': self.state.name,
                    'status': status,
                    'lidar': {
                        'front': round(self.lidar_front, 2),
                        'back': round(self.lidar_back, 2),
                        'left': round(self.lidar_left, 2),
                        'right': round(self.lidar_right, 2)
                    },
                    'velocity': {
                        'linear': round(vel.linear.x, 2),
                        'angular': round(vel.angular.z, 2)
                    },
                    'target_detected': target_detected
                }
                if self.state == State.SEARCH:
                    state_data['search_sub_state'] = self.search_sub_state.name
                    stats = self.visited_tracker.get_stats()
                    state_data['visited_cells'] = stats['visited_cells']
                state_msg.data = json.dumps(state_data)
                self.state_pub.publish(state_msg)

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
        Execute visited-area-based search pattern. Returns (velocity, status).

        Search sub-states:
        1. TURN_IN_PLACE (0-5s): Turn in place toward last seen direction
        2. EXPLORE_UNVISITED: Navigate toward areas robot hasn't visited
        """
        # Sub-state 1: Turn in place (first 5 seconds)
        if search_time < TURN_IN_PLACE_DURATION:
            if self.search_sub_state != SearchSubState.TURN_IN_PLACE:
                self.search_sub_state = SearchSubState.TURN_IN_PLACE
                self.current_goal = None
                self.explore_target_angle = None
                self.get_logger().info(f'Search: TURN_IN_PLACE phase')

            # Turn in place - NO forward motion
            vel.linear.x = 0.0
            vel.angular.z = self.search_turn_speed * self.search_direction
            direction = "left" if self.search_direction > 0 else "right"
            return vel, f'SEARCH: turn {direction} ({search_time:.1f}s/{TURN_IN_PLACE_DURATION}s)'

        # Check if odometry is available
        odom_available = (time.time() - self.last_odom_time) < 1.0

        if not odom_available:
            # Fall back to simple turn if no odometry
            vel.linear.x = 0.0
            vel.angular.z = self.search_turn_speed * self.search_direction
            return vel, f'SEARCH: no odom - turning'

        # Sub-state 2: EXPLORE_UNVISITED - navigate toward unvisited areas
        if self.search_sub_state != SearchSubState.EXPLORE_UNVISITED:
            self.search_sub_state = SearchSubState.EXPLORE_UNVISITED
            self.explore_target_angle = None
            stats = self.visited_tracker.get_stats()
            self.get_logger().info(
                f'Search: EXPLORE_UNVISITED phase (visited: {stats["visited_cells"]} cells, '
                f'{stats["area_m2"]:.1f}m²)')

        # Periodically re-evaluate best direction (every ~3 seconds or when angle is None)
        should_recalculate = (
            self.explore_target_angle is None or
            int(search_time) % 3 == 0 and int(search_time * 10) % 10 == 0
        )

        if should_recalculate:
            best_angle, visited_ratio = self.visited_tracker.get_unvisited_direction(
                self.robot_x, self.robot_y, self.robot_yaw)

            if self.explore_target_angle is None or abs(best_angle - self.explore_target_angle) > 0.3:
                self.explore_target_angle = best_angle
                angle_deg = math.degrees(best_angle)
                self.get_logger().info(
                    f'Explore: target angle {angle_deg:.0f}° '
                    f'(visited ratio: {visited_ratio:.1%})')

        # Navigate toward target angle
        vel, status = self._navigate_to_angle(vel, self.explore_target_angle)
        stats = self.visited_tracker.get_stats()
        return vel, f'EXPLORE: {status} (visited: {stats["visited_cells"]} cells)'

    def _navigate_to_angle(self, vel, target_angle):
        """
        Navigate toward a target angle while moving forward.
        Returns (velocity, status_string).
        """
        # Calculate angle error (normalize to -pi, pi)
        angle_error = target_angle - self.robot_yaw
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # Turn toward target angle
        if abs(angle_error) > 0.4:  # ~23 degrees - need significant turn
            # Turn in place first
            vel.linear.x = 0.0
            vel.angular.z = self.search_turn_speed * np.sign(angle_error) * min(1.5, abs(angle_error))
            status = f'turning ({math.degrees(angle_error):+.0f}°)'
        elif abs(angle_error) > 0.15:  # ~9 degrees - turn while moving slowly
            vel.linear.x = self.search_speed * 0.5
            vel.angular.z = self.turn_speed * 0.7 * angle_error
            status = f'adjusting ({math.degrees(angle_error):+.0f}°)'
        else:
            # Roughly facing target - move forward
            vel.linear.x = self.search_speed
            vel.angular.z = self.turn_speed * 0.3 * angle_error  # Small correction
            status = f'exploring ({math.degrees(self.robot_yaw):.0f}°)'

        return vel, status


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
