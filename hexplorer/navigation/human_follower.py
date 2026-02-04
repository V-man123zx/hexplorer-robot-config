#!/usr/bin/env python3
"""
Human Follower for Hexplorer Robot

Uses depth camera to detect and follow a human, maintaining ~1m distance.
The robot tracks a human-shaped object (vertical blob) and follows it.

Usage:
    source /home/robot/robot_controller_release/ros2_packages/setup.bash
    python3 /home/robot/human_follower.py
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

# Following parameters
TARGET_DISTANCE_M = 1.0         # Desired distance from human (meters)
MIN_DISTANCE_M = 0.6            # Too close - stop/backup
MAX_DISTANCE_M = 2.5            # Too far - stop following (lost human)
DISTANCE_TOLERANCE = 0.15       # Acceptable error in distance

# Human detection parameters
HUMAN_MIN_HEIGHT_M = 0.8        # Minimum height to consider as human
HUMAN_MAX_HEIGHT_M = 2.2        # Maximum height
HUMAN_MIN_WIDTH_M = 0.3         # Minimum width
HUMAN_MAX_WIDTH_M = 1.0         # Maximum width
MIN_HUMAN_PIXELS = 5000         # Minimum pixels to detect human

# Robot movement parameters
FORWARD_SPEED = 0.15            # Forward speed when following (m/s)
BACKUP_SPEED = 0.08             # Backup speed when too close
TURN_SPEED = 0.35               # Angular velocity for turning (rad/s)
CENTER_TOLERANCE = 50           # Pixels from center before turning

# Depth image parameters
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
IMAGE_CENTER_X = IMAGE_WIDTH // 2

# Detection region (focus on center area where human likely is)
DETECT_X_START = 100
DETECT_X_END = 540
DETECT_Y_START = 50
DETECT_Y_END = 430


class HumanFollower(Node):
    def __init__(self):
        super().__init__('human_follower')

        # Publishers for robot control
        self.cmd_pub = self.create_publisher(RobotCommand, '/robot_cmd', 10)
        self.vel_pub = self.create_publisher(Twist, '/vel_cmd', 10)

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers for depth camera
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, sensor_qos)
        self.depth_sub2 = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, sensor_qos)

        # State
        self.human_detected = False
        self.human_distance = 0.0       # Distance to human in meters
        self.human_center_x = IMAGE_CENTER_X  # X position in image
        self.human_width_pixels = 0
        self.running = True
        self.is_standing = False
        self.last_detection_time = 0
        self._first_depth = True

        self.get_logger().info('Human Follower Node initialized')
        self.get_logger().info(f'Target distance: {TARGET_DISTANCE_M}m')

    def depth_callback(self, msg):
        """Process depth image to detect and locate human."""
        try:
            # Convert to numpy array (16-bit, millimeters)
            depth_data = np.frombuffer(bytes(msg.data), dtype=np.uint16)
            depth_image = depth_data.reshape((msg.height, msg.width))

            # Extract detection region
            roi = depth_image[DETECT_Y_START:DETECT_Y_END,
                              DETECT_X_START:DETECT_X_END]

            # Find pixels in human distance range (0.5m to 3m)
            valid_mask = (roi > 500) & (roi < 3000)  # 0.5m to 3m in mm
            valid_depths = roi[valid_mask]

            if len(valid_depths) < MIN_HUMAN_PIXELS:
                self.human_detected = False
                return

            # Find the dominant depth (human should be the closest large object)
            # Use histogram to find the main cluster
            hist, bin_edges = np.histogram(valid_depths, bins=50, range=(500, 3000))
            peak_idx = np.argmax(hist)
            peak_depth = (bin_edges[peak_idx] + bin_edges[peak_idx + 1]) / 2

            # Get pixels within the peak depth range (+/- 200mm)
            human_mask = (roi > peak_depth - 200) & (roi < peak_depth + 200) & (roi > 0)
            human_pixels = np.sum(human_mask)

            if human_pixels < MIN_HUMAN_PIXELS:
                self.human_detected = False
                return

            # Find the bounding box of the human
            rows, cols = np.where(human_mask)
            if len(rows) == 0:
                self.human_detected = False
                return

            min_row, max_row = np.min(rows), np.max(rows)
            min_col, max_col = np.min(cols), np.max(cols)

            # Calculate human dimensions
            height_pixels = max_row - min_row
            width_pixels = max_col - min_col

            # Estimate real-world dimensions using depth
            # Approximate: at 1m, 1 pixel ≈ 1.5mm (for 640x480 with ~60° FOV)
            pixel_size_m = (peak_depth / 1000.0) * 0.0015
            height_m = height_pixels * pixel_size_m
            width_m = width_pixels * pixel_size_m

            # Check if dimensions match a human
            if (HUMAN_MIN_HEIGHT_M < height_m < HUMAN_MAX_HEIGHT_M and
                HUMAN_MIN_WIDTH_M < width_m < HUMAN_MAX_WIDTH_M):

                self.human_detected = True
                self.human_distance = peak_depth / 1000.0  # Convert to meters
                self.human_center_x = DETECT_X_START + (min_col + max_col) // 2
                self.human_width_pixels = width_pixels
                self.last_detection_time = time.time()

                if self._first_depth:
                    self.get_logger().info(f'Human detected! Distance: {self.human_distance:.2f}m')
                    self._first_depth = False
            else:
                # Object detected but doesn't match human shape
                self.human_detected = False

        except Exception as e:
            self.get_logger().warn(f'Depth processing error: {e}')

    def stand_up(self):
        """Bring robot to walking-ready state."""
        cmd = RobotCommand()

        self.get_logger().info('Standing up (takes ~6 seconds)...')

        for state in [1, 2, 3]:
            cmd.target_state = state
            state_names = {1: 'STANDDOWN', 2: 'STANDUP', 3: 'BALANCESTAND'}
            self.get_logger().info(f'  State {state} ({state_names.get(state, "?")})')

            for _ in range(40):
                if not self.running:
                    return False
                self.cmd_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.05)

        self.is_standing = True
        self.get_logger().info('Standing complete, ready to follow')
        return True

    def sit_down(self):
        """Return robot to passive/damping mode."""
        cmd = RobotCommand()
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0

        self.get_logger().info('Sitting down...')

        for _ in range(20):
            self.vel_pub.publish(vel)
            time.sleep(0.05)

        state_names = {3: 'BALANCESTAND', 1: 'STANDDOWN', 0: 'PASSIVE'}
        for state in [3, 1, 0]:
            cmd.target_state = state
            self.get_logger().info(f'  State {state} ({state_names.get(state, "?")})')
            for _ in range(40):
                self.cmd_pub.publish(cmd)
                time.sleep(0.05)

        self.is_standing = False
        self.get_logger().info('Robot in passive/damping mode')

    def run(self):
        """Main control loop."""
        if not self.stand_up():
            return

        cmd = RobotCommand()
        vel = Twist()
        cmd.target_state = 4  # WALK mode

        lost_human_count = 0
        log_counter = 0

        self.get_logger().info('Starting human following mode')
        self.get_logger().info('Stand in front of the robot to be detected')
        self.get_logger().info('Press Ctrl+C to stop')

        try:
            while self.running:
                try:
                    # Check if human detection is recent (within 0.5 seconds)
                    detection_fresh = (time.time() - self.last_detection_time) < 0.5

                    if self.human_detected and detection_fresh:
                        lost_human_count = 0

                        # Calculate distance error
                        distance_error = self.human_distance - TARGET_DISTANCE_M

                        # Calculate angular error (how far from center)
                        center_error = self.human_center_x - IMAGE_CENTER_X

                        # Determine forward/backward velocity
                        if self.human_distance < MIN_DISTANCE_M:
                            # Too close - back up
                            vel.linear.x = -BACKUP_SPEED
                            status = f'TOO CLOSE ({self.human_distance:.2f}m) - backing up'
                        elif abs(distance_error) < DISTANCE_TOLERANCE:
                            # At target distance - stop forward motion
                            vel.linear.x = 0.0
                            status = f'FOLLOWING at {self.human_distance:.2f}m - holding'
                        elif distance_error > 0:
                            # Too far - move forward (speed proportional to error)
                            speed = min(FORWARD_SPEED, FORWARD_SPEED * distance_error)
                            vel.linear.x = max(0.05, speed)
                            status = f'FOLLOWING at {self.human_distance:.2f}m - approaching'
                        else:
                            # Slightly too close - slow down
                            vel.linear.x = 0.0
                            status = f'FOLLOWING at {self.human_distance:.2f}m - holding'

                        # Determine turn velocity to keep human centered
                        if abs(center_error) > CENTER_TOLERANCE:
                            # Turn towards human (negative error = human on left = turn left)
                            turn_factor = center_error / (IMAGE_WIDTH / 2)
                            vel.angular.z = -TURN_SPEED * turn_factor
                            status += f' (turning {"right" if center_error > 0 else "left"})'
                        else:
                            vel.angular.z = 0.0

                    else:
                        # No human detected
                        lost_human_count += 1
                        vel.linear.x = 0.0
                        vel.angular.z = 0.0

                        if lost_human_count < 40:  # ~2 seconds
                            status = 'SEARCHING - human not detected'
                        else:
                            status = 'WAITING - stand in front of robot'

                    # Publish commands
                    self.cmd_pub.publish(cmd)
                    self.vel_pub.publish(vel)

                    # Log periodically
                    log_counter += 1
                    if log_counter % 20 == 0:
                        self.get_logger().info(status)

                    # Process callbacks
                    rclpy.spin_once(self, timeout_sec=0.01)
                    time.sleep(0.04)

                except Exception as e:
                    self.get_logger().error(f'Error in control loop: {e}')
                    vel.linear.x = 0.0
                    vel.angular.z = 0.0

        except KeyboardInterrupt:
            self.get_logger().info('Interrupted by user')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            self.sit_down()


def main():
    rclpy.init()
    node = HumanFollower()

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
