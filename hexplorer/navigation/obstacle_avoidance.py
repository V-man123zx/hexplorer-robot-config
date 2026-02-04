#!/usr/bin/env python3
"""
Obstacle Avoidance for Hexplorer Robot

Uses RealSense depth camera and Livox LiDAR to navigate while avoiding obstacles.
Requires sensors to be running via: bash /home/robot/start_sensor_demo.sh

Usage:
    source /home/robot/robot_controller_release/ros2_packages/setup.bash
    python3 /home/robot/obstacle_avoidance.py [options]

Options:
    --stop-distance    Distance to stop/turn (default: 1.2m)
    --slow-distance    Distance to slow down (default: 1.8m)
    --forward-speed    Normal forward speed (default: 0.5 m/s)
    --slow-speed       Reduced speed near obstacles (default: 0.24 m/s)
    --turn-speed       Angular velocity for turning (default: 0.1 rad/s)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from custom_msg.msg import RobotCommand, RobotState, LivoxPointcloud
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, PointCloud2
import numpy as np
import struct
import time
import signal
import sys
import argparse

# Default parameters (can be overridden via command line)
DEFAULT_OBSTACLE_DISTANCE_M = 1.2   # Stop/turn if obstacle closer than this (meters)
DEFAULT_SLOW_DISTANCE_M = 1.8       # Slow down if obstacle closer than this
DEFAULT_CLEAR_DISTANCE_M = 2.0      # Consider clear if nothing closer than this
DEFAULT_FORWARD_SPEED = 0.5         # Normal forward speed (m/s)
DEFAULT_SLOW_SPEED = 0.24           # Reduced speed when obstacle nearby
DEFAULT_TURN_SPEED = 0.1            # Angular velocity for turning (rad/s)

# Depth image processing
DEPTH_ROI_X_START = 180         # Region of interest in depth image
DEPTH_ROI_X_END = 460
DEPTH_ROI_Y_START = 80          # Look higher up (less ground)
DEPTH_ROI_Y_END = 300
DEPTH_MIN_VALID_M = 0.5         # Ignore depth readings closer than this (robot body/ground)

# LiDAR processing
LIDAR_FRONT_ANGLE = 30          # Degrees from center to consider "front"
LIDAR_HEIGHT_MIN = 0.05         # Ignore points below this (filter ground) - relative to LiDAR
LIDAR_HEIGHT_MAX = 1.2          # Ignore points above this height (m)
LIDAR_MIN_DISTANCE_M = 0.3      # Ignore LiDAR points closer than this (robot body)


class ObstacleAvoidance(Node):
    def __init__(self, args):
        super().__init__('obstacle_avoidance')

        # Store parameters from command line args
        self.obstacle_distance = args.stop_distance
        self.slow_distance = args.slow_distance
        self.clear_distance = args.slow_distance + 0.2  # Slightly beyond slow distance
        self.forward_speed = args.forward_speed
        self.slow_speed = args.slow_speed
        self.turn_speed = args.turn_speed

        # Publishers for robot control
        self.cmd_pub = self.create_publisher(RobotCommand, '/robot_cmd', 10)
        self.vel_pub = self.create_publisher(Twist, '/vel_cmd', 10)

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers for sensor data (subscribe to both direct and bridged topics)
        # Depth camera - bridged topic
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, sensor_qos)
        # Also try the direct official RealSense topic
        self.depth_sub2 = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, sensor_qos)

        # LiDAR - bridged topic (PointCloud2)
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/livox/pointcloud', self.lidar_callback, sensor_qos)
        # Direct Livox topic uses custom message type
        self.lidar_sub2 = self.create_subscription(
            LivoxPointcloud, '/livox_Lidar_node/sn153/xyz/pointcloud', self.livox_callback, sensor_qos)

        self.state_sub = self.create_subscription(
            RobotState, '/robot_state', self.state_callback, 10)

        # State
        self.depth_min_distance = float('inf')  # Minimum distance from depth camera
        self.lidar_min_distance = float('inf')  # Minimum distance from LiDAR
        self.depth_obstacle_side = 0            # -1=left, 0=center, 1=right
        self.lidar_obstacle_side = 0
        self.robot_state = 0
        self.running = True
        self.is_standing = False

        # Timestamps for sensor freshness
        self.last_depth_time = 0
        self.last_lidar_time = 0
        self._first_depth = True
        self._first_lidar = True

        self.get_logger().info('Obstacle Avoidance Node initialized')
        self.get_logger().info(f'Stop distance: {self.obstacle_distance}m, Slow distance: {self.slow_distance}m')
        self.get_logger().info(f'Forward speed: {self.forward_speed} m/s, Turn speed: {self.turn_speed} rad/s')

    def depth_callback(self, msg):
        """Process depth image to find closest obstacle."""
        try:
            # Convert to numpy array (16-bit, millimeters)
            depth_data = np.frombuffer(bytes(msg.data), dtype=np.uint16)
            depth_image = depth_data.reshape((msg.height, msg.width))

            # Extract region of interest (center of image, looking forward not down)
            roi = depth_image[DEPTH_ROI_Y_START:DEPTH_ROI_Y_END,
                              DEPTH_ROI_X_START:DEPTH_ROI_X_END]

            # Convert to meters and filter:
            # - Ignore zero (invalid)
            # - Ignore too close (ground/robot body)
            # - Ignore too far (> 5m)
            roi_m = roi.astype(np.float32) / 1000.0
            valid_mask = (roi_m > DEPTH_MIN_VALID_M) & (roi_m < 5.0)
            valid_depths = roi_m[valid_mask]

            if len(valid_depths) > 100:
                # Find minimum distance using 10th percentile for robustness
                self.depth_min_distance = np.percentile(valid_depths, 10)

                # Log first depth data received
                if self._first_depth:
                    self.get_logger().info(f'Depth camera active! Min distance: {self.depth_min_distance:.2f}m (filtered)')
                    self._first_depth = False

                # Determine which side has the obstacle
                left_roi = roi_m[0:roi_m.shape[0], 0:roi_m.shape[1]//2]
                right_roi = roi_m[0:roi_m.shape[0], roi_m.shape[1]//2:roi_m.shape[1]]

                left_valid = left_roi[(left_roi > DEPTH_MIN_VALID_M) & (left_roi < 5.0)]
                right_valid = right_roi[(right_roi > DEPTH_MIN_VALID_M) & (right_roi < 5.0)]

                left_min = np.percentile(left_valid, 10) if len(left_valid) > 50 else float('inf')
                right_min = np.percentile(right_valid, 10) if len(right_valid) > 50 else float('inf')

                # -1 = obstacle on left (turn right), 1 = obstacle on right (turn left)
                if left_min < right_min - 0.3:
                    self.depth_obstacle_side = -1
                elif right_min < left_min - 0.3:
                    self.depth_obstacle_side = 1
                else:
                    self.depth_obstacle_side = 0

                self.last_depth_time = time.time()
            else:
                self.depth_min_distance = float('inf')

        except Exception as e:
            self.get_logger().warn(f'Depth processing error: {e}')

    def lidar_callback(self, msg):
        """Process LiDAR pointcloud to find closest obstacle in front."""
        try:
            # Parse pointcloud data (x, y, z, intensity - each float32)
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

            # Filter by height (ignore ground and high obstacles)
            height_mask = (points[:, 2] > LIDAR_HEIGHT_MIN) & (points[:, 2] < LIDAR_HEIGHT_MAX)
            points = points[height_mask]

            if len(points) < 10:
                self.lidar_min_distance = float('inf')
                return

            # Filter to front cone (based on angle from forward direction)
            # LiDAR x is forward, y is left
            distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
            angles = np.abs(np.arctan2(points[:, 1], points[:, 0])) * 180 / np.pi

            # Filter: front cone, forward direction, and not too close (robot body)
            front_mask = ((angles < LIDAR_FRONT_ANGLE) &
                         (points[:, 0] > 0) &
                         (distances > LIDAR_MIN_DISTANCE_M))
            front_points = points[front_mask]
            front_distances = distances[front_mask]

            if len(front_distances) > 5:
                self.lidar_min_distance = np.percentile(front_distances, 10)

                # Log first LiDAR data received
                if self._first_lidar:
                    self.get_logger().info(f'LiDAR active! Min distance: {self.lidar_min_distance:.2f}m ({len(front_distances)} front points)')
                    self._first_lidar = False

                # Determine obstacle side
                front_y = front_points[:, 1]
                close_mask = front_distances < self.obstacle_distance
                if np.sum(close_mask) > 3:
                    avg_y = np.mean(front_y[close_mask])
                    if avg_y > 0.1:
                        self.lidar_obstacle_side = 1   # Obstacle on left, turn right
                    elif avg_y < -0.1:
                        self.lidar_obstacle_side = -1  # Obstacle on right, turn left
                    else:
                        self.lidar_obstacle_side = 0

                self.last_lidar_time = time.time()
            else:
                self.lidar_min_distance = float('inf')

        except Exception as e:
            self.get_logger().warn(f'LiDAR processing error: {e}')

    def livox_callback(self, msg):
        """Process custom Livox pointcloud message."""
        try:
            if msg.point_num < 10:
                return

            # Extract points from custom message
            points = []
            for p in msg.points:
                points.append((p.x, p.y, p.z))

            points = np.array(points)

            # Filter by height (ignore ground and high obstacles)
            height_mask = (points[:, 2] > LIDAR_HEIGHT_MIN) & (points[:, 2] < LIDAR_HEIGHT_MAX)
            points = points[height_mask]

            if len(points) < 10:
                self.lidar_min_distance = float('inf')
                return

            # Filter to front cone
            distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
            angles = np.abs(np.arctan2(points[:, 1], points[:, 0])) * 180 / np.pi

            # Filter: front cone, forward direction, and not too close (robot body)
            front_mask = ((angles < LIDAR_FRONT_ANGLE) &
                         (points[:, 0] > 0) &
                         (distances > LIDAR_MIN_DISTANCE_M))
            front_points = points[front_mask]
            front_distances = distances[front_mask]

            if len(front_distances) > 5:
                self.lidar_min_distance = np.percentile(front_distances, 10)

                if self._first_lidar:
                    self.get_logger().info(f'Livox LiDAR active! Min: {self.lidar_min_distance:.2f}m ({len(front_distances)} pts)')
                    self._first_lidar = False

                # Determine obstacle side
                close_mask = front_distances < self.obstacle_distance
                if np.sum(close_mask) > 3:
                    avg_y = np.mean(front_points[close_mask, 1])
                    if avg_y > 0.1:
                        self.lidar_obstacle_side = 1
                    elif avg_y < -0.1:
                        self.lidar_obstacle_side = -1
                    else:
                        self.lidar_obstacle_side = 0

                self.last_lidar_time = time.time()
            else:
                self.lidar_min_distance = float('inf')

        except Exception as e:
            self.get_logger().warn(f'Livox processing error: {e}')

    def state_callback(self, msg):
        """Track robot state."""
        self.robot_state = msg.control_cmd

    def get_combined_obstacle_distance(self):
        """Combine depth and LiDAR readings for robust obstacle detection."""
        now = time.time()

        # Check sensor freshness (use data only if < 1 second old)
        depth_fresh = (now - self.last_depth_time) < 1.0
        lidar_fresh = (now - self.last_lidar_time) < 1.0

        # Prefer LiDAR (better filtering), use depth as backup or for confirmation
        if lidar_fresh and self.lidar_min_distance < 10:
            min_distance = self.lidar_min_distance
            obstacle_side = self.lidar_obstacle_side

            # If depth also sees something close, use the average
            if depth_fresh and self.depth_min_distance < 10:
                # Only trust depth if it's reasonably close to LiDAR reading
                # or if it sees something very close (real obstacle)
                if abs(self.depth_min_distance - self.lidar_min_distance) < 0.5:
                    min_distance = (self.lidar_min_distance + self.depth_min_distance) / 2
                elif self.depth_min_distance < 0.8:  # Depth sees very close obstacle
                    min_distance = min(self.lidar_min_distance, self.depth_min_distance)
                    obstacle_side = self.depth_obstacle_side

        elif depth_fresh and self.depth_min_distance < 10:
            # Only depth available
            min_distance = self.depth_min_distance
            obstacle_side = self.depth_obstacle_side
        else:
            return float('inf'), 0  # No valid data, assume clear

        return min_distance, obstacle_side

    def stand_up(self):
        """Bring robot to walking-ready state."""
        cmd = RobotCommand()

        self.get_logger().info('Standing up (takes ~6 seconds)...')

        # STANDDOWN (1) -> STANDUP (2) -> BALANCESTAND (3)
        for state in [1, 2, 3]:
            cmd.target_state = state
            state_names = {1: 'STANDDOWN', 2: 'STANDUP', 3: 'BALANCESTAND'}
            self.get_logger().info(f'  State {state} ({state_names.get(state, "?")})')

            for _ in range(40):  # 2 seconds per state (40 * 0.05 = 2s)
                if not self.running:
                    return False
                self.cmd_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.05)  # Ensure proper timing

        self.is_standing = True
        self.get_logger().info('Standing complete, ready to walk')
        return True

    def sit_down(self):
        """Return robot to passive/damping mode."""
        cmd = RobotCommand()
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0

        self.get_logger().info('Sitting down...')

        # Stop any velocity first
        for _ in range(20):
            self.vel_pub.publish(vel)
            time.sleep(0.05)

        # Proper sit-down sequence: BALANCESTAND(3) → STANDDOWN(1) → PASSIVE(0)
        state_names = {3: 'BALANCESTAND', 1: 'STANDDOWN', 0: 'PASSIVE'}
        for state in [3, 1, 0]:
            cmd.target_state = state
            self.get_logger().info(f'  State {state} ({state_names.get(state, "?")})')
            for _ in range(40):  # 2 seconds per state
                self.cmd_pub.publish(cmd)
                time.sleep(0.05)

        self.is_standing = False
        self.get_logger().info('Robot in passive/damping mode')

    def run(self):
        """Main control loop."""
        # Stand up first
        if not self.stand_up():
            return

        cmd = RobotCommand()
        vel = Twist()
        cmd.target_state = 4  # WALK mode

        turn_direction = 1  # 1 = left, -1 = right
        consecutive_clear = 0  # Count of consecutive clear readings
        state = 'FORWARD'  # FORWARD, TURNING
        turn_start_time = 0
        MIN_TURN_TIME = 1.5  # Minimum seconds to turn before checking clear
        CLEAR_READINGS_NEEDED = 8  # Need this many clear readings before going forward
        stuck_count = 0

        self.get_logger().info('Starting obstacle avoidance navigation')
        self.get_logger().info('Press Ctrl+C to stop')

        try:
            while self.running:
                # Always reset velocity at start of loop
                vel.linear.x = 0.0
                vel.angular.z = 0.0
                status = ''

                try:
                    # Get combined obstacle distance
                    min_distance, obstacle_side = self.get_combined_obstacle_distance()
                except Exception as e:
                    self.get_logger().error(f'Error getting obstacle distance: {e}')
                    min_distance = float('inf')
                    obstacle_side = 0

                current_time = time.time()

                # State machine for movement
                if state == 'FORWARD':
                    if min_distance < self.obstacle_distance:
                        # Obstacle detected - start turning
                        state = 'TURNING'
                        turn_start_time = current_time
                        consecutive_clear = 0
                        stuck_count = 0

                        # Choose turn direction based on obstacle side
                        if obstacle_side != 0:
                            turn_direction = -obstacle_side

                        self.get_logger().info(f'OBSTACLE at {min_distance:.2f}m! Turning {"left" if turn_direction > 0 else "right"}')
                        # Set turning velocity for this iteration
                        vel.linear.x = 0.0
                        vel.angular.z = self.turn_speed * turn_direction
                        status = f'TURNING {"left" if turn_direction > 0 else "right"}'

                    elif min_distance < self.slow_distance:
                        # In slow zone - move forward slowly
                        vel.linear.x = self.slow_speed
                        vel.angular.z = 0.0
                        status = f'SLOW at {min_distance:.2f}m'

                    else:
                        # Clear - full speed forward
                        vel.linear.x = self.forward_speed
                        vel.angular.z = 0.0
                        status = f'FORWARD (clear: {min_distance:.2f}m)'

                elif state == 'TURNING':
                    turn_elapsed = current_time - turn_start_time

                    # Keep turning for minimum time
                    if turn_elapsed < MIN_TURN_TIME:
                        vel.linear.x = 0.0
                        vel.angular.z = self.turn_speed * turn_direction
                        status = f'TURNING {"left" if turn_direction > 0 else "right"} ({turn_elapsed:.1f}s)'

                    else:
                        # After minimum turn, check if clear
                        if min_distance > self.slow_distance:
                            consecutive_clear += 1
                            if consecutive_clear >= CLEAR_READINGS_NEEDED:
                                # Confirmed clear - go forward
                                state = 'FORWARD'
                                vel.linear.x = self.forward_speed
                                vel.angular.z = 0.0
                                self.get_logger().info(f'Path clear at {min_distance:.2f}m - moving forward')
                                status = f'FORWARD (clear: {min_distance:.2f}m)'
                            else:
                                # Keep slow turning while verifying clear
                                vel.linear.x = 0.0
                                vel.angular.z = self.turn_speed * turn_direction * 0.3
                                status = f'VERIFYING clear ({consecutive_clear}/{CLEAR_READINGS_NEEDED})'

                        elif min_distance < self.obstacle_distance:
                            # Still blocked - keep turning
                            consecutive_clear = 0
                            stuck_count += 1
                            vel.linear.x = 0.0
                            vel.angular.z = self.turn_speed * turn_direction
                            status = f'BLOCKED at {min_distance:.2f}m - turning'

                            # If stuck too long, reverse briefly then try other direction
                            if stuck_count > 200:  # ~5 seconds of being stuck
                                self.get_logger().info('Stuck! Reversing briefly...')
                                # Reverse for 1.5 seconds
                                for _ in range(30):
                                    vel.linear.x = -self.slow_speed
                                    vel.angular.z = 0.0
                                    self.cmd_pub.publish(cmd)
                                    self.vel_pub.publish(vel)
                                    rclpy.spin_once(self, timeout_sec=0.01)
                                    time.sleep(0.04)
                                # Switch turn direction
                                turn_direction *= -1
                                turn_start_time = current_time
                                stuck_count = 0
                                self.get_logger().info(f'Now turning {"left" if turn_direction > 0 else "right"}')

                        else:
                            # In slow zone - keep turning but slower
                            consecutive_clear = 0
                            vel.linear.x = 0.0
                            vel.angular.z = self.turn_speed * turn_direction * 0.5
                            status = f'TURNING (slow zone {min_distance:.2f}m)'

                # Publish commands
                self.cmd_pub.publish(cmd)
                self.vel_pub.publish(vel)

                # Log status periodically
                if not hasattr(self, '_log_counter'):
                    self._log_counter = 0
                    self._no_sensor_warned = False

                self._log_counter += 1

                if self._log_counter % 20 == 0:  # Every ~1 second
                    self.get_logger().info(status)

                    # Warn if no sensor data
                    if min_distance == float('inf') and not self._no_sensor_warned:
                        self.get_logger().warn('No sensor data! Start sensors with: bash /home/robot/start_sensor_demo.sh')
                        self._no_sensor_warned = True

                # Process callbacks
                rclpy.spin_once(self, timeout_sec=0.01)

                # Maintain 20Hz rate
                time.sleep(0.04)

        except KeyboardInterrupt:
            self.get_logger().info('Interrupted by user')
        except Exception as e:
            self.get_logger().error(f'Unexpected error in main loop: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            self.sit_down()


def parse_args():
    parser = argparse.ArgumentParser(
        description='Obstacle avoidance for Hexplorer robot',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('--stop-distance', type=float, default=DEFAULT_OBSTACLE_DISTANCE_M,
                        help='Distance (m) to stop and turn')
    parser.add_argument('--slow-distance', type=float, default=DEFAULT_SLOW_DISTANCE_M,
                        help='Distance (m) to slow down')
    parser.add_argument('--forward-speed', type=float, default=DEFAULT_FORWARD_SPEED,
                        help='Normal forward speed (m/s)')
    parser.add_argument('--slow-speed', type=float, default=DEFAULT_SLOW_SPEED,
                        help='Reduced speed near obstacles (m/s)')
    parser.add_argument('--turn-speed', type=float, default=DEFAULT_TURN_SPEED,
                        help='Angular velocity for turning (rad/s)')
    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init()
    node = ObstacleAvoidance(args)

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
