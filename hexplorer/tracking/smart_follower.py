#!/usr/bin/env python3
"""
Smart Object Follower with Obstacle Avoidance for Hexplorer Robot

Combines object tracking with LiDAR-based obstacle avoidance.
Uses LiDAR (always available) for obstacle detection while following color-tracked objects.

State Machine:
    INIT -> IDLE -> FOLLOWING -> (EVADE | BLOCKED) -> FOLLOWING
                 -> SEARCH -> FOLLOWING | IDLE

Features:
- Object following from /object_detection topic
- LiDAR-based obstacle avoidance (always available during tracking)
- Active search pattern when target is lost
- Safe sit-down on Ctrl+C

Usage:
    source /home/robot/robot_controller_release/ros2_packages/setup.bash
    python3 smart_follower.py [options]

Options:
    --target-distance    Target distance to maintain from object (default: 800mm)
    --max-speed          Maximum forward speed (default: 0.3 m/s)
    --turn-speed         Angular velocity for turning (default: 0.15 rad/s)
    --obstacle-stop      Stop if obstacle closer than this (default: 0.8m)
    --obstacle-slow      Slow down if obstacle closer than this (default: 1.2m)
    --search-timeout     Give up searching after this many seconds (default: 15)
    --search-speed       Forward speed while searching (default: 0.1 m/s)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from custom_msg.msg import RobotCommand, LivoxPointcloud
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import numpy as np
import struct
import json
import time
import signal
import argparse
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
DEFAULT_SEARCH_TIMEOUT = 15.0      # seconds

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
        self.search_timeout = args.search_timeout

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
        self.search_phase = 0
        self.search_direction = 1
        self.zigzag_time = 0

        # Evade state
        self.evade_direction = 0

        self.get_logger().info('Smart Follower initialized')
        self.get_logger().info(f'  Target distance: {self.target_distance}mm')
        self.get_logger().info(f'  Max speed: {self.max_speed} m/s')
        self.get_logger().info(f'  Obstacle stop: {self.obstacle_stop}m')
        self.get_logger().info(f'  Obstacle slow: {self.obstacle_slow}m')
        self.get_logger().info(f'  Search timeout: {self.search_timeout}s')

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
                        self.get_logger().info('Target found - FOLLOWING')
                    else:
                        search_time = now - self.search_start_time
                        vel, status = self._search_pattern(vel, search_time)

                        # Obstacle avoidance during search
                        if obstacle_close:
                            # Stop forward motion, turn away from obstacle
                            vel.linear.x = 0.0
                            if self.lidar_obstacle_side != 0:
                                # Turn away from obstacle
                                vel.angular.z = self.turn_speed * (-self.lidar_obstacle_side)
                            else:
                                # Obstacle centered - turn in search direction
                                vel.angular.z = self.turn_speed * self.search_direction
                            status = f'SEARCH AVOID obs={self.lidar_min_distance:.2f}m'

                        if search_time > self.search_timeout:
                            self.state = State.IDLE
                            self.search_start_time = 0
                            self.get_logger().info('Search timeout - IDLE')

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
        """Execute active search pattern. Returns (velocity, status)."""
        now = time.time()

        # Phase 0 (0-3s): Turn toward last seen direction + slow forward
        if search_time < 3.0:
            if self.search_phase != 0:
                self.search_phase = 0
            vel.angular.z = self.search_turn_speed * self.search_direction
            vel.linear.x = self.search_speed
            direction = "left" if self.search_direction > 0 else "right"
            return vel, f'SEARCH P1: turn {direction} + forward ({search_time:.1f}s)'

        # Phase 1 (3-8s): Zigzag - walk forward, alternate turning
        elif search_time < 8.0:
            if self.search_phase != 1:
                self.search_phase = 1
                self.zigzag_time = now

            # Switch direction every 1.5 seconds
            zigzag_elapsed = now - self.zigzag_time
            if zigzag_elapsed > 1.5:
                self.search_direction *= -1
                self.zigzag_time = now

            vel.linear.x = self.search_speed
            vel.angular.z = self.search_turn_speed * self.search_direction * 0.8
            direction = "left" if self.search_direction > 0 else "right"
            return vel, f'SEARCH P2: zigzag {direction} ({search_time:.1f}s)'

        # Phase 2 (8-15s): Expanding spiral - increasing turn radius
        elif search_time < self.search_timeout:
            if self.search_phase != 2:
                self.search_phase = 2

            # Slower forward, wider turns as time goes on
            progress = (search_time - 8.0) / (self.search_timeout - 8.0)
            vel.linear.x = self.search_speed * (0.5 + 0.5 * progress)
            # Decrease turn speed for wider spiral
            vel.angular.z = self.search_turn_speed * (1.0 - 0.5 * progress) * self.search_direction
            direction = "left" if self.search_direction > 0 else "right"
            return vel, f'SEARCH P3: spiral {direction} ({search_time:.1f}s)'

        # Phase 3 (>15s): Timeout - stop and wait
        else:
            return vel, f'SEARCH timeout ({search_time:.1f}s)'


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
    parser.add_argument('--search-timeout', type=float, default=DEFAULT_SEARCH_TIMEOUT,
                        help='Give up searching after this many seconds')
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
