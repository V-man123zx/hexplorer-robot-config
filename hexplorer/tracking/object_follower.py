#!/usr/bin/env python3
"""
Object Follower for Hexplorer Robot

Subscribes to /object_detection and controls the robot to follow the detected object.

Features:
- Distance-based speed control
- Search behavior when object lost
- Safe sit-down on Ctrl+C
- Configurable parameters

Usage:
    source /home/robot/robot_controller_release/ros2_packages/setup.bash
    python3 object_follower.py [options]

Options:
    --target-distance    Target distance to maintain (default: 800mm)
    --max-speed          Maximum forward speed (default: 0.3 m/s)
    --turn-speed         Angular velocity for turning (default: 0.15 rad/s)
"""

import rclpy
from rclpy.node import Node
from custom_msg.msg import RobotCommand
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import time
import signal
import argparse

# Default parameters
DEFAULT_TARGET_DISTANCE = 800      # mm - try to maintain this distance
DEFAULT_CLOSE_BUFFER = 200         # mm - backup if closer than target - buffer
DEFAULT_APPROACH_BUFFER = 500      # mm - slow approach within target + buffer
DEFAULT_MAX_SPEED = 0.3            # m/s
DEFAULT_SLOW_SPEED = 0.15          # m/s
DEFAULT_BACKUP_SPEED = 0.15        # m/s
DEFAULT_TURN_SPEED = 0.15          # rad/s
DEFAULT_SEARCH_TURN_SPEED = 0.08   # rad/s

# Image parameters
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
IMAGE_CENTER_X = IMAGE_WIDTH // 2
DEADZONE = 50  # pixels


class ObjectFollower(Node):
    def __init__(self, args):
        super().__init__('object_follower')

        # Store parameters
        self.target_distance = args.target_distance
        self.close_distance = self.target_distance - DEFAULT_CLOSE_BUFFER
        self.approach_distance = self.target_distance + DEFAULT_APPROACH_BUFFER
        self.max_speed = args.max_speed
        self.turn_speed = args.turn_speed
        self.slow_speed = DEFAULT_SLOW_SPEED
        self.backup_speed = DEFAULT_BACKUP_SPEED
        self.search_turn_speed = DEFAULT_SEARCH_TURN_SPEED

        # Publishers for robot control
        self.cmd_pub = self.create_publisher(RobotCommand, '/robot_cmd', 10)
        self.vel_pub = self.create_publisher(Twist, '/vel_cmd', 10)

        # Subscribe to detection topic
        self.create_subscription(
            String, '/object_detection', self.detection_callback, 10)

        # State
        self.running = True
        self.is_standing = False
        self.last_detection = None
        self.last_detection_time = 0
        self.last_seen_side = 0  # -1=left, 0=center, 1=right
        self.search_start_time = 0
        self.search_direction = 1

        self.get_logger().info('Object Follower initialized')
        self.get_logger().info(f'  Target distance: {self.target_distance}mm')
        self.get_logger().info(f'  Max speed: {self.max_speed} m/s')
        self.get_logger().info(f'  Turn speed: {self.turn_speed} rad/s')

    def detection_callback(self, msg):
        """Process detection message."""
        try:
            self.last_detection = json.loads(msg.data)
            self.last_detection_time = time.time()

            if self.last_detection['detected']:
                cx = self.last_detection['center_x']
                if cx < IMAGE_CENTER_X - DEADZONE:
                    self.last_seen_side = -1  # Object on left
                elif cx > IMAGE_CENTER_X + DEADZONE:
                    self.last_seen_side = 1   # Object on right
                else:
                    self.last_seen_side = 0   # Object centered
        except json.JSONDecodeError:
            pass

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
        """Main control loop."""
        if not self.stand_up():
            return

        cmd = RobotCommand()
        vel = Twist()
        cmd.target_state = 4  # WALK mode

        self.get_logger().info('Following object - Ctrl+C to stop')
        self.get_logger().info('Waiting for detection data on /object_detection...')

        log_counter = 0

        try:
            while self.running:
                vel.linear.x = 0.0
                vel.angular.z = 0.0
                status = ''

                now = time.time()
                detection_fresh = (now - self.last_detection_time) < 0.5

                if detection_fresh and self.last_detection and self.last_detection['detected']:
                    # Object detected - follow it
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
                        if distance < self.close_distance:
                            # Too close - back up
                            vel.linear.x = -self.backup_speed
                            status = f'BACKUP dist={distance}mm err={error:+d}px'
                        elif distance < self.target_distance:
                            # Within target zone - hold position
                            vel.linear.x = 0.0
                            status = f'HOLD dist={distance}mm err={error:+d}px'
                        elif distance < self.approach_distance:
                            # Slow approach zone
                            vel.linear.x = self.slow_speed
                            status = f'SLOW dist={distance}mm err={error:+d}px'
                        else:
                            # Far away - full speed
                            vel.linear.x = self.max_speed
                            status = f'FOLLOW dist={distance}mm err={error:+d}px'
                    else:
                        # No depth data - just track horizontally
                        status = f'TRACK (no depth) err={error:+d}px'

                    # Reset search state
                    self.search_start_time = 0

                elif not detection_fresh and self.last_seen_side != 0:
                    # Object lost - search in last seen direction
                    if self.search_start_time == 0:
                        self.search_start_time = now
                        self.search_direction = -self.last_seen_side

                    search_duration = now - self.search_start_time

                    # Search pattern: turn toward last seen side
                    if search_duration < 3.0:
                        vel.angular.z = self.search_turn_speed * self.search_direction
                        direction = "left" if self.search_direction > 0 else "right"
                        status = f'SEARCH {direction} ({search_duration:.1f}s)'
                    elif search_duration < 6.0:
                        # Try opposite direction
                        vel.angular.z = -self.search_turn_speed * self.search_direction
                        direction = "right" if self.search_direction > 0 else "left"
                        status = f'SEARCH {direction} ({search_duration:.1f}s)'
                    else:
                        # Give up and reset
                        self.search_start_time = 0
                        self.last_seen_side = 0
                        status = 'LOST - waiting'

                else:
                    # No detection and no last seen side - wait
                    status = 'WAITING for detection'

                # Publish commands
                self.cmd_pub.publish(cmd)
                self.vel_pub.publish(vel)

                # Log periodically
                log_counter += 1
                if log_counter % 25 == 0:
                    self.get_logger().info(status)

                # Process callbacks
                rclpy.spin_once(self, timeout_sec=0.01)

                # Maintain ~25Hz
                time.sleep(0.04)

        except KeyboardInterrupt:
            self.get_logger().info('Interrupted')
        finally:
            self.sit_down()


def parse_args():
    parser = argparse.ArgumentParser(
        description='Object follower for Hexplorer robot',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('--target-distance', type=int, default=DEFAULT_TARGET_DISTANCE,
                        help='Target distance to maintain (mm)')
    parser.add_argument('--max-speed', type=float, default=DEFAULT_MAX_SPEED,
                        help='Maximum forward speed (m/s)')
    parser.add_argument('--turn-speed', type=float, default=DEFAULT_TURN_SPEED,
                        help='Angular velocity for turning (rad/s)')
    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init()
    node = ObjectFollower(args)

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
