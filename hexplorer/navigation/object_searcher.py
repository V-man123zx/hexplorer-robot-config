#!/usr/bin/env python3
"""
Object Searcher for Hexplorer Robot

Systematically searches for a target object by scanning in place, then navigating
to unvisited areas. Uses Fast-LIO2 odometry for position tracking and LiDAR for
obstacle avoidance (proven front-cone approach from obstacle_avoidance.py).

State machine:
    STANDUP -> SCANNING -> NAVIGATING -> SCANNING -> ... (cycle)
                  |             |
                FOUND         FOUND
                  |
              APPROACH -> CONFIRMED -> SHUTDOWN

Requires:
    - Fast-LIO2 + odom_relay running (for /lidar_odometry/pose)
    - Detection receiver running (for /object_detection)
    - LiDAR data on /livox/lidar

Usage:
    source /opt/ros/humble/setup.bash
    source /home/robot/robot_controller_release/ros2_packages/setup.bash
    source ~/fastlio_ws/install/setup.bash
    python3 object_searcher.py [options]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from custom_msg.msg import RobotCommand
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point as GeoPoint
from builtin_interfaces.msg import Duration
import numpy as np
import struct
import json
import time
import signal
import argparse
import math

# LiDAR processing (from proven obstacle_avoidance.py)
LIDAR_FRONT_ANGLE = 30       # Degrees from center for "front" cone
LIDAR_BACK_ANGLE = 30        # Degrees from center for "back" cone
LIDAR_HEIGHT_MIN = 0.05      # Ignore points below this (ground)
LIDAR_HEIGHT_MAX = 1.2       # Ignore points above this
LIDAR_MIN_DISTANCE = 0.3     # Ignore points closer (robot body)

# Detection parameters
IMAGE_WIDTH = 640
IMAGE_CENTER_X = IMAGE_WIDTH // 2
DEADZONE = 50                # Pixels - consider centered if error < this
CONFIRM_FRAMES = 3           # Consecutive detections needed to confirm

# States
STATE_STANDUP = 'STANDUP'
STATE_SCANNING = 'SCANNING'
STATE_NAVIGATING = 'NAVIGATING'
STATE_FOUND = 'FOUND'
STATE_APPROACH = 'APPROACH'
STATE_CONFIRMED = 'CONFIRMED'
STATE_SHUTDOWN = 'SHUTDOWN'


class VisitedAreaTracker:
    """Grid-based coverage tracker with wall awareness for search planning."""

    WALL_HIT_THRESHOLD = 5  # Need this many LiDAR hits before a cell is considered a wall

    def __init__(self, cell_size=0.25):
        self.cell_size = cell_size
        self.visited = set()       # {(grid_x, grid_y), ...}
        self.blocked = set()       # Confirmed wall cells
        self._block_hits = {}      # {(gx,gy): hit_count} — accumulate before confirming

    def mark_visited(self, x, y):
        self.visited.add((int(math.floor(x / self.cell_size)),
                          int(math.floor(y / self.cell_size))))

    def mark_blocked(self, x, y):
        """Accumulate a wall hit. Only confirms as blocked after enough hits."""
        cell = (int(math.floor(x / self.cell_size)),
                int(math.floor(y / self.cell_size)))
        self._block_hits[cell] = self._block_hits.get(cell, 0) + 1
        if self._block_hits[cell] >= self.WALL_HIT_THRESHOLD:
            self.blocked.add(cell)

    def decay_blocked(self, seen_cells):
        """Decay hit counts for blocked cells not seen this scan. Unblock if hits drop to 0."""
        to_remove = []
        for cell in list(self._block_hits.keys()):
            if cell not in seen_cells:
                self._block_hits[cell] -= 1
                if self._block_hits[cell] <= 0:
                    del self._block_hits[cell]
                    self.blocked.discard(cell)
                    to_remove.append(cell)

    def mark_camera_cone(self, robot_x, robot_y, robot_yaw,
                         half_angle=math.radians(15), min_dist=0.9, max_dist=3.0):
        """Mark cells visible in a forward-facing camera cone.

        Args:
            half_angle: Half-width of cone (15 deg = 30 deg total FOV)
            min_dist: Near edge of cone (~3 ft)
            max_dist: Far edge of cone (~10 ft)
        """
        step = self.cell_size * 0.5
        for d_idx in range(int(min_dist / step), int(max_dist / step) + 1):
            d = d_idx * step
            # Sweep across the cone width at this distance
            arc_step = self.cell_size / max(d, 0.5)
            angle = -half_angle
            while angle <= half_angle:
                px = robot_x + d * math.cos(robot_yaw + angle)
                py = robot_y + d * math.sin(robot_yaw + angle)
                cell = (int(math.floor(px / self.cell_size)),
                        int(math.floor(py / self.cell_size)))
                # Don't mark blocked cells as visited (wall is still there)
                if cell not in self.blocked:
                    self.visited.add(cell)
                angle += arc_step

    def get_best_direction(self, robot_x, robot_y, robot_yaw, num_dirs=16,
                           exclude_angle=None, exclude_width=math.radians(45)):
        """Check num_dirs directions, return angle with most open space + unvisited cells.

        Rays terminate when they hit a blocked (wall) cell.
        Score = unvisited_cells + 0.1 * visited_cells (so open visited areas
        always beat wall-blocked directions).

        Args:
            exclude_angle: if set, directions within exclude_width of this angle
                          get score zeroed (used when replanning away from a stuck direction)
        """
        best_angle = robot_yaw
        best_score = -1.0
        ray_length = 6.0
        step = self.cell_size  # Step at cell resolution so we don't skip walls
        total_all = 0

        for i in range(num_dirs):
            angle = robot_yaw + (2 * math.pi * i / num_dirs)

            # Skip directions near the excluded angle
            if exclude_angle is not None:
                diff = abs(math.atan2(math.sin(angle - exclude_angle),
                                      math.cos(angle - exclude_angle)))
                if diff < exclude_width:
                    continue

            unvisited = 0
            visited = 0
            for d_idx in range(1, int(ray_length / step) + 1):
                d = d_idx * step
                px = robot_x + d * math.cos(angle)
                py = robot_y + d * math.sin(angle)
                gx = int(math.floor(px / self.cell_size))
                gy = int(math.floor(py / self.cell_size))
                if (gx, gy) in self.blocked:
                    break
                total_all += 1
                if (gx, gy) not in self.visited:
                    unvisited += 1
                else:
                    visited += 1

            # Unvisited cells are worth 1 point, visited cells worth 0.1
            # So open-but-visited paths still beat wall-blocked paths
            score = unvisited + 0.1 * visited
            if score > best_score:
                best_score = score
                best_angle = angle

        if best_score <= 0:
            # Completely boxed in — just pick forward
            return robot_yaw, 1.0

        visited_ratio = 1.0 - (best_score / max(total_all, 1)) if total_all > 0 else 1.0
        visited_ratio = max(0.0, min(1.0, visited_ratio))
        return best_angle, visited_ratio

    def get_coverage_stats(self):
        """Return number of visited cells and approximate area."""
        n = len(self.visited)
        area = n * self.cell_size * self.cell_size
        return n, area


class ObjectSearcher(Node):
    def __init__(self, args):
        super().__init__('object_searcher')

        # Parameters
        self.search_speed = args.search_speed
        self.scan_speed = args.scan_speed
        self.navigate_distance = args.navigate_distance
        self.stop_distance = args.stop_distance
        self.slow_distance = args.slow_distance
        self.turn_speed = args.turn_speed
        self.confirm_distance = args.confirm_distance
        self.no_approach = args.no_approach
        self.no_sit = args.no_sit

        # Publishers
        self.cmd_pub = self.create_publisher(RobotCommand, '/robot_cmd', 10)
        self.vel_pub = self.create_publisher(Twist, '/vel_cmd', 10)
        self.state_pub = self.create_publisher(String, '/object_searcher/state', 10)
        self.visited_grid_pub = self.create_publisher(
            OccupancyGrid, '/object_searcher/visited_grid', 10)
        self.goal_marker_pub = self.create_publisher(
            Marker, '/object_searcher/goal_marker', 10)
        self.path_marker_pub = self.create_publisher(
            Marker, '/object_searcher/path_marker', 10)
        self.scan_marker_pub = self.create_publisher(
            Marker, '/object_searcher/scan_marker', 10)

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.create_subscription(
            String, '/object_detection', self._detection_callback, 10)

        # LiDAR pointcloud (from TCP bridge)
        self.create_subscription(
            PointCloud2, '/livox/lidar', self._lidar_callback, sensor_qos)

        # Odometry - Fast-LIO2 via odom_relay
        self.create_subscription(
            Odometry, '/lidar_odometry/pose', self._odom_callback, sensor_qos)

        # State
        self.running = True
        self.state = STATE_STANDUP
        self.is_standing = False

        # Odometry
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False
        self.last_odom_time = 0

        # LiDAR obstacle data
        self.front_min_distance = float('inf')
        self.front_obstacle_side = 0  # -1=left, 0=center, 1=right
        self.back_min_distance = float('inf')
        self.last_lidar_time = 0
        self._first_lidar = True

        # Detection
        self.last_detection = None
        self.last_detection_time = 0
        self.last_detection_msg_time = 0  # Any message (even detected=false)
        self.consecutive_detections = 0
        self.last_seen_side = 0
        self.detection_timeout = 30.0  # Abort if no detection messages for this long

        # Search tracking
        self.visited_tracker = VisitedAreaTracker(cell_size=0.15)
        self.scan_count = 0
        self.search_start_time = 0

        # Scanning state
        self.scan_yaw_accumulated = 0.0
        self.scan_prev_yaw = None
        self.scan_start_time = 0
        self.scan_timeout = 60.0  # Max seconds for one scan
        self.scan_start_x = 0.0   # Position at scan start (LiDAR offset causes
        self.scan_start_y = 0.0   # circular drift during in-place rotation)

        # Navigating state
        self.nav_start_x = 0.0
        self.nav_start_y = 0.0
        self.nav_target_angle = 0.0
        self.nav_last_eval_time = 0
        self.nav_eval_interval = 3.0  # Re-evaluate direction every 3 seconds
        self.nav_obstacle_count = 0   # How many loops obstacle avoidance has been active
        self.nav_obstacle_replan_threshold = 50  # ~2 seconds at 25Hz -> force replan
        self.nav_stuck_replans = 0    # Consecutive stuck replans without progress
        self.nav_last_progress_x = 0.0
        self.nav_last_progress_y = 0.0
        self.nav_no_progress_time = 0  # Timestamp when we last made progress
        self.nav_no_progress_timeout = 10.0  # Seconds without moving -> stuck

        # Found/Approach state
        self.found_start_time = 0
        self.approach_lost_time = 0

        # Visualization
        self.path_points = []
        self.last_viz_time = 0
        self.last_path_time = 0

        self.get_logger().info('Object Searcher initialized')
        self.get_logger().info(f'  Search speed: {self.search_speed} m/s')
        self.get_logger().info(f'  Scan speed: {self.scan_speed} rad/s')
        self.get_logger().info(f'  Navigate distance: {self.navigate_distance} m')
        self.get_logger().info(f'  Stop distance: {self.stop_distance} m')
        self.get_logger().info(f'  Confirm distance: {self.confirm_distance} mm')
        if self.no_approach:
            self.get_logger().info('  No-approach mode: will confirm immediately on detection')

    # ─── Callbacks ────────────────────────────────────────────────────

    def _detection_callback(self, msg):
        """Process detection message from detection_receiver."""
        try:
            self.last_detection = json.loads(msg.data)
            self.last_detection_time = time.time()
            self.last_detection_msg_time = time.time()

            if self.last_detection.get('detected', False):
                self.consecutive_detections += 1
                cx = self.last_detection.get('center_x', IMAGE_CENTER_X)
                if cx < IMAGE_CENTER_X - DEADZONE:
                    self.last_seen_side = -1
                elif cx > IMAGE_CENTER_X + DEADZONE:
                    self.last_seen_side = 1
                else:
                    self.last_seen_side = 0
            else:
                self.consecutive_detections = 0
        except json.JSONDecodeError:
            pass

    def _lidar_callback(self, msg):
        """Process LiDAR pointcloud for front and back obstacle detection.

        Uses the proven front-cone approach from obstacle_avoidance.py:
        - Height filter 0.05-1.2m
        - Angle filter (front cone, back cone)
        - Min distance 0.3m (robot body)
        - 10th percentile for robustness
        """
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

            points = np.array(points)

            # Height filter (ignore ground and high obstacles)
            height_mask = ((points[:, 2] > LIDAR_HEIGHT_MIN) &
                           (points[:, 2] < LIDAR_HEIGHT_MAX))
            points = points[height_mask]

            if len(points) < 10:
                self.front_min_distance = float('inf')
                self.back_min_distance = float('inf')
                return

            distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
            angles_deg = np.arctan2(points[:, 1], points[:, 0]) * 180 / np.pi

            # Front cone: forward-facing, within angle, not too close
            front_mask = ((np.abs(angles_deg) < LIDAR_FRONT_ANGLE) &
                          (points[:, 0] > 0) &
                          (distances > LIDAR_MIN_DISTANCE))
            front_distances = distances[front_mask]
            front_points = points[front_mask]

            if len(front_distances) > 5:
                self.front_min_distance = np.percentile(front_distances, 10)

                if self._first_lidar:
                    self.get_logger().info(
                        f'LiDAR active! Front min: {self.front_min_distance:.2f}m '
                        f'({len(front_distances)} pts)')
                    self._first_lidar = False

                # Determine obstacle side (from obstacle_avoidance.py)
                close_mask = front_distances < self.stop_distance
                if np.sum(close_mask) > 3:
                    avg_y = np.mean(front_points[close_mask, 1])
                    if avg_y > 0.1:
                        self.front_obstacle_side = 1   # Left, turn right
                    elif avg_y < -0.1:
                        self.front_obstacle_side = -1  # Right, turn left
                    else:
                        self.front_obstacle_side = 0
                else:
                    self.front_obstacle_side = 0
            else:
                self.front_min_distance = float('inf')
                self.front_obstacle_side = 0

            # Back cone: rear-facing, for safe reversing
            back_mask = ((np.abs(angles_deg) > (180 - LIDAR_BACK_ANGLE)) &
                         (points[:, 0] < 0) &
                         (distances > LIDAR_MIN_DISTANCE))
            back_distances = distances[back_mask]

            if len(back_distances) > 5:
                self.back_min_distance = np.percentile(back_distances, 10)
            else:
                self.back_min_distance = float('inf')

            # Mark obstacle points as wall cells in world coordinates
            # Height filter (0.15-1.0m) to avoid ground noise
            # Mark walls up to 3m (~10ft) away so planner sees them early
            WALL_MARK_DISTANCE = 3.0
            if self.odom_received:
                wall_mask = ((distances > LIDAR_MIN_DISTANCE) &
                             (distances < WALL_MARK_DISTANCE) &
                             (points[:, 2] > 0.15) &
                             (points[:, 2] < 1.0))
                wall_pts = points[wall_mask]
                seen_cells = set()
                if len(wall_pts) > 0:
                    cos_yaw = math.cos(self.robot_yaw)
                    sin_yaw = math.sin(self.robot_yaw)
                    for pt in wall_pts:
                        wx = self.robot_x + pt[0] * cos_yaw - pt[1] * sin_yaw
                        wy = self.robot_y + pt[0] * sin_yaw + pt[1] * cos_yaw
                        self.visited_tracker.mark_blocked(wx, wy)
                        cell = (int(math.floor(wx / self.visited_tracker.cell_size)),
                                int(math.floor(wy / self.visited_tracker.cell_size)))
                        seen_cells.add(cell)
                # Decay blocked cells not reinforced this scan
                self.visited_tracker.decay_blocked(seen_cells)

            self.last_lidar_time = time.time()

        except Exception as e:
            self.get_logger().warn(f'LiDAR processing error: {e}')

    def _odom_callback(self, msg):
        """Process odometry for position tracking."""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        self.robot_x = pos.x
        self.robot_y = pos.y

        # Quaternion to yaw
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(
                f'Odometry active! Position: ({self.robot_x:.2f}, {self.robot_y:.2f})')

        self.last_odom_time = time.time()

    # ─── Robot Control (from proven obstacle_avoidance.py / object_follower.py) ───

    def stand_up(self):
        """Bring robot to walking-ready state."""
        cmd = RobotCommand()
        self.get_logger().info('Standing up...')

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

        # BALANCESTAND(3) -> STANDDOWN(1) -> PASSIVE(0)
        for state_val in [3, 1, 0]:
            cmd.target_state = state_val
            for _ in range(40):
                self.cmd_pub.publish(cmd)
                time.sleep(0.05)

        self.is_standing = False
        self.get_logger().info('Robot in passive mode')

    # ─── Detection Helpers ────────────────────────────────────────────

    def _detection_is_fresh(self):
        """Check if we have a fresh, positive detection."""
        return (time.time() - self.last_detection_time < 0.5 and
                self.last_detection is not None and
                self.last_detection.get('detected', False))

    def _check_for_target(self):
        """Check if target is detected with enough confidence."""
        if self._detection_is_fresh() and self.consecutive_detections >= CONFIRM_FRAMES:
            return True
        return False

    # ─── State Handlers ───────────────────────────────────────────────

    def _enter_scanning(self):
        """Begin a 360-degree scan in place."""
        self.state = STATE_SCANNING
        self.scan_yaw_accumulated = 0.0
        self.scan_prev_yaw = self.robot_yaw if self.odom_received else None
        self.scan_start_time = time.time()
        self.scan_start_x = self.robot_x
        self.scan_start_y = self.robot_y
        self.scan_count += 1
        self.get_logger().info(
            f'SCANNING #{self.scan_count} at ({self.robot_x:.1f}, {self.robot_y:.1f})')

    def _handle_scanning(self, vel):
        """Rotate 360 degrees, checking for target. Returns velocity."""
        # Track accumulated rotation via odometry yaw
        if self.odom_received and self.scan_prev_yaw is not None:
            yaw_delta = self.robot_yaw - self.scan_prev_yaw
            # Normalize to [-pi, pi]
            while yaw_delta > math.pi:
                yaw_delta -= 2 * math.pi
            while yaw_delta < -math.pi:
                yaw_delta += 2 * math.pi
            self.scan_yaw_accumulated += abs(yaw_delta)
        self.scan_prev_yaw = self.robot_yaw

        # Check completion: full rotation or timeout
        elapsed = time.time() - self.scan_start_time
        if self.scan_yaw_accumulated >= 2 * math.pi:
            self.get_logger().info(
                f'Scan complete ({self.scan_yaw_accumulated:.1f} rad in {elapsed:.1f}s)')
            self._enter_navigating()
            return vel

        if elapsed > self.scan_timeout:
            self.get_logger().warn(
                f'Scan timeout ({elapsed:.0f}s, {self.scan_yaw_accumulated:.1f} rad)')
            self._enter_navigating()
            return vel

        # Rotate in place
        vel.linear.x = 0.0
        vel.angular.z = self.scan_speed
        return vel

    def _enter_navigating(self):
        """Begin navigating toward unvisited area."""
        self.state = STATE_NAVIGATING
        self.nav_start_x = self.scan_start_x
        self.nav_start_y = self.scan_start_y
        self.nav_last_eval_time = time.time()
        self.nav_obstacle_count = 0
        self.nav_stuck_replans = 0
        self.nav_last_progress_x = self.robot_x
        self.nav_last_progress_y = self.robot_y
        self.nav_no_progress_time = time.time()

        # Pick best direction (wall-aware)
        angle, visited_ratio = self.visited_tracker.get_best_direction(
            self.robot_x, self.robot_y, self.robot_yaw)
        self.nav_target_angle = angle

        self.get_logger().info(
            f'NAVIGATING toward {math.degrees(angle):.0f} deg '
            f'(visited ratio: {visited_ratio:.0%})')

    def _handle_navigating(self, vel):
        """Move toward unvisited area with obstacle avoidance. Returns velocity."""
        now = time.time()

        # Check if we've traveled enough
        dx = self.robot_x - self.nav_start_x
        dy = self.robot_y - self.nav_start_y
        distance_traveled = math.sqrt(dx * dx + dy * dy)

        if distance_traveled >= self.navigate_distance:
            self.get_logger().info(
                f'Reached navigate distance ({distance_traveled:.1f}m)')
            self._enter_scanning()
            return vel

        # Re-evaluate direction periodically
        if now - self.nav_last_eval_time > self.nav_eval_interval:
            angle, visited_ratio = self.visited_tracker.get_best_direction(
                self.robot_x, self.robot_y, self.robot_yaw)
            self.nav_target_angle = angle
            self.nav_last_eval_time = now
            self.nav_obstacle_count = 0  # Reset stuck counter on replan

        # Check front obstacle (proven pattern from obstacle_avoidance.py)
        lidar_fresh = (now - self.last_lidar_time) < 1.0

        if lidar_fresh and self.front_min_distance < self.stop_distance:
            # Obstacle too close - stop and turn away
            vel.linear.x = 0.0
            turn_dir = -self.front_obstacle_side if self.front_obstacle_side != 0 else 1
            vel.angular.z = self.turn_speed * turn_dir

            # Detect stuck loop: obstacle avoidance keeps firing
            self.nav_obstacle_count += 1
            if self.nav_obstacle_count >= self.nav_obstacle_replan_threshold:
                self.nav_stuck_replans += 1
                self.nav_obstacle_count = 0

                if self.nav_stuck_replans >= 3:
                    # Repeatedly stuck — backtrack toward where we came from
                    back_angle = math.atan2(
                        self.nav_start_y - self.robot_y,
                        self.nav_start_x - self.robot_x)
                    self.nav_target_angle = back_angle
                    self.nav_last_eval_time = now
                    self.get_logger().info(
                        f'Stuck {self.nav_stuck_replans}x — backtracking toward '
                        f'{math.degrees(back_angle):.0f} deg')
                    # Reset nav start so we travel away from here
                    self.nav_start_x = self.robot_x
                    self.nav_start_y = self.robot_y
                    self.nav_stuck_replans = 0
                else:
                    # Try replanning — exclude the direction that got us stuck
                    self.get_logger().info(
                        f'Stuck at wall ({self.nav_stuck_replans}/3), '
                        f'replanning away from {math.degrees(self.nav_target_angle):.0f} deg')
                    angle, visited_ratio = self.visited_tracker.get_best_direction(
                        self.robot_x, self.robot_y, self.robot_yaw,
                        exclude_angle=self.nav_target_angle)
                    self.nav_target_angle = angle
                    self.nav_last_eval_time = now
                    self.get_logger().info(
                        f'New direction: {math.degrees(angle):.0f} deg '
                        f'(visited ratio: {visited_ratio:.0%})')
            return vel
        else:
            # Decay obstacle count instead of hard reset (handles flickering LiDAR)
            self.nav_obstacle_count = max(0, self.nav_obstacle_count - 1)

        # Track position-based progress
        dx_prog = self.robot_x - self.nav_last_progress_x
        dy_prog = self.robot_y - self.nav_last_progress_y
        if math.sqrt(dx_prog*dx_prog + dy_prog*dy_prog) > 0.5:
            self.nav_stuck_replans = 0
            self.nav_last_progress_x = self.robot_x
            self.nav_last_progress_y = self.robot_y
            self.nav_no_progress_time = now
        elif now - self.nav_no_progress_time > self.nav_no_progress_timeout:
            # Haven't moved 0.5m in 10 seconds — we're stuck
            self.nav_stuck_replans += 1
            self.nav_no_progress_time = now
            if self.nav_stuck_replans >= 3:
                back_angle = math.atan2(
                    self.nav_start_y - self.robot_y,
                    self.nav_start_x - self.robot_x)
                self.nav_target_angle = back_angle
                self.nav_start_x = self.robot_x
                self.nav_start_y = self.robot_y
                self.get_logger().info(
                    f'No progress for {self.nav_stuck_replans}x — backtracking '
                    f'toward {math.degrees(back_angle):.0f} deg')
                self.nav_stuck_replans = 0
            else:
                angle, visited_ratio = self.visited_tracker.get_best_direction(
                    self.robot_x, self.robot_y, self.robot_yaw,
                    exclude_angle=self.nav_target_angle)
                self.nav_target_angle = angle
                self.get_logger().info(
                    f'No progress ({self.nav_stuck_replans}/3), '
                    f'replanning to {math.degrees(angle):.0f} deg '
                    f'(visited ratio: {visited_ratio:.0%})')
            self.nav_last_eval_time = now

        if lidar_fresh and self.front_min_distance < self.slow_distance:
            # Slow zone - reduce speed
            speed_factor = ((self.front_min_distance - self.stop_distance) /
                            (self.slow_distance - self.stop_distance))
            speed_factor = max(0.3, min(1.0, speed_factor))
            vel.linear.x = self.search_speed * speed_factor
        else:
            vel.linear.x = self.search_speed

        # Turn toward target angle
        angle_error = self.nav_target_angle - self.robot_yaw
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        if abs(angle_error) > 0.1:  # ~6 degrees
            turn_scale = min(1.0, abs(angle_error) / 1.0)
            vel.angular.z = self.turn_speed * turn_scale * (1 if angle_error > 0 else -1)
            # Reduce forward speed while turning significantly
            if abs(angle_error) > 0.5:  # ~30 degrees
                vel.linear.x *= 0.3
        else:
            vel.angular.z = 0.0

        return vel

    def _enter_found(self):
        """Target detected - stop and confirm."""
        self.state = STATE_FOUND
        self.found_start_time = time.time()
        label = self.last_detection.get('label', '?') if self.last_detection else '?'
        dist = self.last_detection.get('distance_mm', 0) if self.last_detection else 0
        self.get_logger().info(
            f'TARGET FOUND: {label} at {dist}mm! Confirming...')

    def _handle_found(self, vel):
        """Center target in frame and confirm detection. Returns velocity."""
        vel.linear.x = 0.0
        vel.angular.z = 0.0

        if not self._detection_is_fresh():
            # Lost detection during confirmation - go back to scanning
            if time.time() - self.found_start_time > 2.0:
                self.get_logger().info('Lost target during confirmation, resuming scan')
                self._enter_scanning()
            return vel

        cx = self.last_detection.get('center_x', IMAGE_CENTER_X)
        error = cx - IMAGE_CENTER_X

        # Center target in frame
        if abs(error) > DEADZONE:
            turn_scale = min(1.0, abs(error) / 200.0)
            vel.angular.z = (-self.turn_speed * turn_scale if error > 0
                             else self.turn_speed * turn_scale)

        # Check confirmation: enough consecutive detections
        if self.consecutive_detections >= CONFIRM_FRAMES:
            if self.no_approach:
                self._enter_confirmed()
            else:
                self._enter_approach()

        return vel

    def _enter_approach(self):
        """Move toward confirmed target."""
        self.state = STATE_APPROACH
        self.approach_lost_time = 0
        self.get_logger().info('APPROACHING target...')

    def _handle_approach(self, vel):
        """Slowly approach target while keeping it centered. Returns velocity."""
        now = time.time()

        if not self._detection_is_fresh():
            # Lost detection during approach
            if self.approach_lost_time == 0:
                self.approach_lost_time = now
            elif now - self.approach_lost_time > 3.0:
                self.get_logger().info('Lost target during approach, resuming scan')
                self.approach_lost_time = 0
                self._enter_scanning()
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            return vel

        self.approach_lost_time = 0
        distance = self.last_detection.get('distance_mm', 0)
        cx = self.last_detection.get('center_x', IMAGE_CENTER_X)
        error = cx - IMAGE_CENTER_X

        # Center target
        if abs(error) > DEADZONE:
            turn_scale = min(1.0, abs(error) / 200.0)
            vel.angular.z = (-self.turn_speed * turn_scale if error > 0
                             else self.turn_speed * turn_scale)

        # Distance control
        if distance > 0 and distance <= self.confirm_distance:
            self._enter_confirmed()
            vel.linear.x = 0.0
            return vel

        # Move forward slowly
        vel.linear.x = 0.1  # Slow approach speed

        # Front obstacle check during approach
        lidar_fresh = (now - self.last_lidar_time) < 1.0
        if lidar_fresh and self.front_min_distance < self.stop_distance:
            vel.linear.x = 0.0
            self.get_logger().info('Obstacle during approach - stopping')

        return vel

    def _enter_confirmed(self):
        """Search complete - target confirmed."""
        self.state = STATE_CONFIRMED
        elapsed = time.time() - self.search_start_time
        cells, area = self.visited_tracker.get_coverage_stats()
        label = self.last_detection.get('label', '?') if self.last_detection else '?'
        dist = self.last_detection.get('distance_mm', 0) if self.last_detection else 0

        self.get_logger().info('=' * 50)
        self.get_logger().info('  SEARCH COMPLETE - TARGET CONFIRMED')
        self.get_logger().info(f'  Target: {label}')
        self.get_logger().info(f'  Distance: {dist}mm')
        self.get_logger().info(f'  Robot position: ({self.robot_x:.2f}, {self.robot_y:.2f})')
        self.get_logger().info(f'  Scans completed: {self.scan_count}')
        self.get_logger().info(f'  Area covered: {area:.1f} m2 ({cells} cells)')
        self.get_logger().info(f'  Search time: {elapsed:.1f}s')
        self.get_logger().info('=' * 50)

    # ─── Visualization ────────────────────────────────────────────────

    def _publish_visited_grid(self):
        """Publish visited area as OccupancyGrid for RViz.

        Values: -1 = unknown/unvisited, 0 = free/visited, 100 = blocked (wall)
        """
        all_cells = self.visited_tracker.visited | self.visited_tracker.blocked
        if not all_cells:
            return

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'odom'
        grid.info.resolution = self.visited_tracker.cell_size

        cells = list(all_cells)
        min_x = min(c[0] for c in cells) - 2
        min_y = min(c[1] for c in cells) - 2
        max_x = max(c[0] for c in cells) + 2
        max_y = max(c[1] for c in cells) + 2

        grid.info.width = max_x - min_x + 1
        grid.info.height = max_y - min_y + 1
        grid.info.origin.position.x = float(min_x * self.visited_tracker.cell_size)
        grid.info.origin.position.y = float(min_y * self.visited_tracker.cell_size)
        grid.info.origin.position.z = 0.0

        # -1 = unknown, 0 = visited/free, 100 = blocked/wall
        data = [-1] * (grid.info.width * grid.info.height)
        for gx, gy in self.visited_tracker.visited:
            col = gx - min_x
            row = gy - min_y
            idx = row * grid.info.width + col
            if 0 <= idx < len(data):
                data[idx] = 0
        for gx, gy in self.visited_tracker.blocked:
            col = gx - min_x
            row = gy - min_y
            idx = row * grid.info.width + col
            if 0 <= idx < len(data):
                data[idx] = 100  # Walls show as dark in RViz
        grid.data = data
        self.visited_grid_pub.publish(grid)

    def _publish_goal_marker(self):
        """Publish arrow marker showing current navigate direction."""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'odom'
        marker.ns = 'goal'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Arrow from robot position in navigate direction
        start = GeoPoint()
        start.x = self.robot_x
        start.y = self.robot_y
        start.z = 0.1

        end = GeoPoint()
        end.x = self.robot_x + 1.5 * math.cos(self.nav_target_angle)
        end.y = self.robot_y + 1.5 * math.sin(self.nav_target_angle)
        end.z = 0.1

        marker.points = [start, end]
        marker.scale.x = 0.08  # Shaft diameter
        marker.scale.y = 0.15  # Head diameter
        marker.scale.z = 0.15  # Head length

        # Color: green=navigating, yellow=scanning, red=obstacle
        if self.state == STATE_NAVIGATING:
            lidar_fresh = (time.time() - self.last_lidar_time) < 1.0
            if lidar_fresh and self.front_min_distance < self.stop_distance:
                marker.color.r = 1.0
                marker.color.g = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
        elif self.state == STATE_SCANNING:
            marker.color.r = 1.0
            marker.color.g = 1.0
        else:
            marker.color.r = 0.0
            marker.color.g = 0.5

        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = Duration(sec=2, nanosec=0)

        self.goal_marker_pub.publish(marker)

    def _publish_path_marker(self):
        """Publish LINE_STRIP showing search path."""
        if len(self.path_points) < 2:
            return

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'odom'
        marker.ns = 'path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.03  # Line width

        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.8

        for px, py in self.path_points:
            p = GeoPoint()
            p.x = px
            p.y = py
            p.z = 0.05
            marker.points.append(p)

        self.path_marker_pub.publish(marker)

    def _publish_scan_marker(self):
        """Publish cylinder at current scan location."""
        if self.state != STATE_SCANNING:
            # Delete marker when not scanning
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'odom'
            marker.ns = 'scan'
            marker.id = 0
            marker.action = Marker.DELETE
            self.scan_marker_pub.publish(marker)
            return

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'odom'
        marker.ns = 'scan'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = self.robot_x
        marker.pose.position.y = self.robot_y
        marker.pose.position.z = 0.01

        # Radius represents scan coverage area
        marker.scale.x = 2.0  # Diameter
        marker.scale.y = 2.0
        marker.scale.z = 0.02  # Thin disk

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.15

        marker.lifetime = Duration(sec=2, nanosec=0)
        self.scan_marker_pub.publish(marker)

    def _publish_state(self):
        """Publish current state as JSON."""
        now = time.time()
        elapsed = now - self.search_start_time if self.search_start_time > 0 else 0
        cells, area = self.visited_tracker.get_coverage_stats()

        state_data = {
            'state': self.state,
            'robot_x': round(self.robot_x, 2),
            'robot_y': round(self.robot_y, 2),
            'robot_yaw_deg': round(math.degrees(self.robot_yaw), 1),
            'front_obstacle_m': round(self.front_min_distance, 2),
            'scan_count': self.scan_count,
            'area_covered_m2': round(area, 1),
            'elapsed_s': round(elapsed, 1),
            'detection_fresh': self._detection_is_fresh(),
            'consecutive_detections': self.consecutive_detections,
        }

        msg = String()
        msg.data = json.dumps(state_data)
        self.state_pub.publish(msg)

    def _update_visualizations(self):
        """Publish all visualization markers at ~1 Hz."""
        now = time.time()

        # Update path points at ~1 Hz (use scan-start pos during scanning)
        if now - self.last_path_time > 1.0 and self.odom_received:
            if self.state == STATE_SCANNING:
                self.path_points.append((self.scan_start_x, self.scan_start_y))
            else:
                self.path_points.append((self.robot_x, self.robot_y))
            # Limit path to 1000 points
            if len(self.path_points) > 1000:
                self.path_points = self.path_points[-1000:]
            self.last_path_time = now

        # Publish visualizations at ~1 Hz
        if now - self.last_viz_time > 1.0:
            self._publish_visited_grid()
            self._publish_goal_marker()
            self._publish_path_marker()
            self._publish_scan_marker()
            self._publish_state()
            self.last_viz_time = now

    # ─── Main Loop ────────────────────────────────────────────────────

    def run(self):
        """Main control loop."""
        # Stand up first
        if not self.stand_up():
            return

        self.search_start_time = time.time()
        cmd = RobotCommand()
        vel = Twist()
        cmd.target_state = 4  # WALK mode

        # Wait for LiDAR data before doing anything (REQUIRED for safe operation)
        self.get_logger().info('Waiting for LiDAR data...')
        wait_start = time.time()
        while self.last_lidar_time == 0 and self.running:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.time() - wait_start
            if elapsed > 60:
                self.get_logger().error('No LiDAR data after 60s - aborting for safety')
                self.sit_down()
                return
            if int(elapsed) % 10 == 0 and int(elapsed) > 0:
                self.get_logger().info(f'  Still waiting for LiDAR... ({int(elapsed)}s)')

        if not self.running:
            return

        self.get_logger().info('LiDAR data received!')

        # Wait for odometry (helpful but not strictly required)
        self.get_logger().info('Waiting for odometry...')
        wait_start = time.time()
        while not self.odom_received and self.running:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - wait_start > 30:
                self.get_logger().warn('No odometry after 30s, starting without it')
                break

        if self.odom_received:
            self.get_logger().info(
                f'Odometry OK. Starting search from ({self.robot_x:.1f}, {self.robot_y:.1f})')
        else:
            self.get_logger().warn('Starting search without odometry (limited functionality)')

        # Wait for detection receiver
        self.get_logger().info('Waiting for detection receiver...')
        wait_start = time.time()
        while self.last_detection_msg_time == 0 and self.running:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - wait_start > 30:
                self.get_logger().error(
                    'No detection messages after 30s - aborting (detection receiver not running?)')
                self.sit_down()
                return

        if not self.running:
            return
        self.get_logger().info('Detection receiver OK!')

        # Begin with first scan
        self._enter_scanning()

        log_counter = 0

        try:
            while self.running:
                vel.linear.x = 0.0
                vel.angular.z = 0.0

                # Mark cells the camera can see (30-deg cone, 3-10 ft ahead)
                if self.odom_received:
                    self.visited_tracker.mark_camera_cone(
                        self.robot_x, self.robot_y, self.robot_yaw)

                # Check for target in any state (except CONFIRMED/SHUTDOWN)
                if self.state not in (STATE_CONFIRMED, STATE_SHUTDOWN, STATE_FOUND,
                                      STATE_APPROACH):
                    if self._check_for_target():
                        self._enter_found()

                # Safety: check detection receiver health
                # If we got at least one message but then nothing for detection_timeout,
                # the receiver has likely crashed or disconnected
                if (self.last_detection_msg_time > 0 and
                        self.state not in (STATE_CONFIRMED, STATE_SHUTDOWN)):
                    detection_gap = time.time() - self.last_detection_msg_time
                    if detection_gap > self.detection_timeout:
                        self.get_logger().error(
                            f'No detection messages for {detection_gap:.0f}s '
                            f'- detection receiver lost. Aborting search.')
                        self.running = False
                        break

                # Safety: check LiDAR freshness during navigation/approach
                lidar_stale = (time.time() - self.last_lidar_time) > 3.0
                if lidar_stale and self.state in (STATE_NAVIGATING, STATE_APPROACH):
                    vel.linear.x = 0.0
                    vel.angular.z = 0.0
                    self.cmd_pub.publish(cmd)
                    self.vel_pub.publish(vel)
                    if log_counter % 25 == 0:
                        self.get_logger().warn(
                            f'LiDAR data stale ({time.time() - self.last_lidar_time:.0f}s) '
                            f'- stopping movement')
                    rclpy.spin_once(self, timeout_sec=0.01)
                    time.sleep(0.04)
                    log_counter += 1
                    continue

                # State machine
                if self.state == STATE_SCANNING:
                    vel = self._handle_scanning(vel)
                elif self.state == STATE_NAVIGATING:
                    vel = self._handle_navigating(vel)
                elif self.state == STATE_FOUND:
                    vel = self._handle_found(vel)
                elif self.state == STATE_APPROACH:
                    vel = self._handle_approach(vel)
                elif self.state == STATE_CONFIRMED:
                    vel.linear.x = 0.0
                    vel.angular.z = 0.0
                    # Stay standing briefly to show we're done
                    if not hasattr(self, '_confirm_time'):
                        self._confirm_time = time.time()
                    if time.time() - self._confirm_time > 3.0:
                        self.running = False
                    # Publish commands to keep walking state active
                    self.cmd_pub.publish(cmd)
                    self.vel_pub.publish(vel)
                    self._update_visualizations()
                    rclpy.spin_once(self, timeout_sec=0.01)
                    time.sleep(0.04)
                    continue

                # Publish commands
                self.cmd_pub.publish(cmd)
                self.vel_pub.publish(vel)

                # Update visualizations
                self._update_visualizations()

                # Log periodically
                log_counter += 1
                if log_counter % 25 == 0:  # Every ~1 second
                    # During scanning show scan-start pos (raw odom drifts in circle)
                    if self.state == STATE_SCANNING:
                        px, py = self.scan_start_x, self.scan_start_y
                    else:
                        px, py = self.robot_x, self.robot_y
                    status = (f'{self.state} | pos=({px:.1f},{py:.1f}) '
                              f'front={self.front_min_distance:.1f}m '
                              f'det={self.consecutive_detections}')
                    if self.state == STATE_SCANNING:
                        status += f' yaw_acc={self.scan_yaw_accumulated:.1f}rad'
                    self.get_logger().info(status)

                # Process callbacks
                rclpy.spin_once(self, timeout_sec=0.01)

                # Maintain ~25Hz
                time.sleep(0.04)

        except KeyboardInterrupt:
            self.get_logger().info('Interrupted by user')
        except Exception as e:
            self.get_logger().error(f'Error in main loop: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            if self.no_sit:
                self.get_logger().info('--no-sit: leaving robot standing')
                # Stop motion
                vel.linear.x = 0.0
                vel.angular.z = 0.0
                for _ in range(20):
                    self.vel_pub.publish(vel)
                    time.sleep(0.05)
            else:
                self.sit_down()

            # Print final stats
            elapsed = time.time() - self.search_start_time
            cells, area = self.visited_tracker.get_coverage_stats()
            self.get_logger().info(f'Search stats: {self.scan_count} scans, '
                                   f'{area:.1f}m2 covered, {elapsed:.1f}s elapsed')


def parse_args():
    parser = argparse.ArgumentParser(
        description='Object searcher for Hexplorer robot',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('--search-speed', type=float, default=0.15,
                        help='Forward speed while navigating (m/s)')
    parser.add_argument('--scan-speed', type=float, default=0.15,
                        help='Rotation speed during 360-degree scans (rad/s)')
    parser.add_argument('--navigate-distance', type=float, default=4.0,
                        help='Distance between scan points (m)')
    parser.add_argument('--stop-distance', type=float, default=0.8,
                        help='Obstacle stop distance (m)')
    parser.add_argument('--slow-distance', type=float, default=1.5,
                        help='Obstacle slow distance (m)')
    parser.add_argument('--turn-speed', type=float, default=0.1,
                        help='Turn speed for obstacle avoidance (rad/s)')
    parser.add_argument('--confirm-distance', type=int, default=1500,
                        help='Approach to this distance (mm)')
    parser.add_argument('--no-approach', action='store_true',
                        help='Skip approach, confirm immediately on detection')
    parser.add_argument('--no-sit', action='store_true',
                        help='Stay standing after finding target')
    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init()
    node = ObjectSearcher(args)

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
