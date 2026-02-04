#!/usr/bin/env python3
"""
Yellow Object Follower with Depth Filtering for Hexplorer Robot

Detects a yellow object using the color camera and depth sensor.
Only tracks yellow objects that are close (within MAX_DISTANCE).
Robot does NOT walk - only rotates in place.

Usage:
    # Start sensors first:
    bash /home/robot/start_sensor_demo.sh

    # In another terminal:
    source /home/robot/robot_controller_release/ros2_packages/setup.bash
    python3 /home/robot/follow_white_box.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from custom_msg.msg import RobotCommand
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
import time
import signal

# Yellow detection parameters
YELLOW_CH0_MIN = 90         # Channel 0 high
YELLOW_CH1_MIN = 40         # Channel 1 medium
YELLOW_CH2_MAX = 30         # Channel 2 very low
MIN_BOX_AREA = 500          # Minimum contour area
MAX_BOX_AREA = 80000        # Maximum contour area
ASPECT_RATIO_MIN = 0.2
ASPECT_RATIO_MAX = 5.0

# Depth filtering parameters
MAX_DISTANCE = 2000         # Maximum distance in mm (2 meters)
MIN_DISTANCE = 100          # Minimum distance in mm (10 cm) - filter noise

# Control parameters
TURN_SPEED = 0.15
DEADZONE = 50
SEARCH_TURN_SPEED = 0.08

# Image parameters
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
IMAGE_CENTER_X = IMAGE_WIDTH // 2


class YellowObjectFollower(Node):
    def __init__(self):
        super().__init__('yellow_object_follower')

        # Publishers for robot control
        self.cmd_pub = self.create_publisher(RobotCommand, '/robot_cmd', 10)
        self.vel_pub = self.create_publisher(Twist, '/vel_cmd', 10)

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to color image
        self.create_subscription(
            Image, '/camera/color/image_raw', self.color_callback, sensor_qos)
        self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.color_callback, sensor_qos)

        # Subscribe to depth image
        self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, sensor_qos)
        self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, sensor_qos)

        # State
        self.color_img = None
        self.depth_img = None
        self.box_detected = False
        self.box_center_x = IMAGE_CENTER_X
        self.box_center_y = IMAGE_HEIGHT // 2
        self.box_area = 0
        self.box_distance = 0
        self.last_detection_time = 0
        self.running = True
        self.is_standing = False
        self._first_color = True
        self._first_depth = True
        self._last_seen_side = 0

        self.get_logger().info('Yellow Object Follower with Depth initialized')
        self.get_logger().info(f'Max distance: {MAX_DISTANCE}mm, Deadzone: {DEADZONE}px')

    def color_callback(self, msg):
        """Store latest color image."""
        if self._first_color:
            self.get_logger().info(f'Color camera active: {msg.width}x{msg.height}')
            self._first_color = False
        self.color_img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape((msg.height, msg.width, 3))
        self._process_detection()

    def depth_callback(self, msg):
        """Store latest depth image."""
        if self._first_depth:
            self.get_logger().info(f'Depth camera active: {msg.width}x{msg.height}, encoding: {msg.encoding}')
            self._first_depth = False
        # Depth is 16-bit unsigned (mm)
        self.depth_img = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape((msg.height, msg.width))

    def _process_detection(self):
        """Detect yellow objects that are close."""
        if self.color_img is None:
            return

        img = self.color_img
        h, w = img.shape[:2]

        # Create yellow mask
        yellow_mask = (
            (img[:, :, 0] > YELLOW_CH0_MIN) &
            (img[:, :, 1] > YELLOW_CH1_MIN) &
            (img[:, :, 2] < YELLOW_CH2_MAX)
        ).astype(np.uint8)

        # If we have depth, filter by distance
        if self.depth_img is not None and self.depth_img.shape[:2] == (h, w):
            depth = self.depth_img
            close_mask = (depth > MIN_DISTANCE) & (depth < MAX_DISTANCE)
            yellow_mask = yellow_mask & close_mask.astype(np.uint8)

        # Find yellow pixels
        ys, xs = np.where(yellow_mask > 0)

        if len(xs) >= MIN_BOX_AREA:
            # Calculate centroid
            cx = int(np.mean(xs))
            cy = int(np.mean(ys))

            # Get bounding box
            x_min, x_max = xs.min(), xs.max()
            y_min, y_max = ys.min(), ys.max()
            width = x_max - x_min
            height = y_max - y_min

            if height > 0:
                aspect = width / height
                if ASPECT_RATIO_MIN < aspect < ASPECT_RATIO_MAX:
                    area = len(xs)
                    if area <= MAX_BOX_AREA:
                        self.box_center_x = cx
                        self.box_center_y = cy
                        self.box_area = area
                        self.box_detected = True
                        self.last_detection_time = time.time()

                        # Get distance at center
                        if self.depth_img is not None:
                            self.box_distance = int(self.depth_img[cy, cx])
                        else:
                            self.box_distance = 0

                        # Remember side
                        if cx < IMAGE_CENTER_X - DEADZONE:
                            self._last_seen_side = -1
                        elif cx > IMAGE_CENTER_X + DEADZONE:
                            self._last_seen_side = 1
                        return

        self.box_detected = False
        self.box_area = 0

    def stand_up(self):
        """Bring robot to walking-ready state."""
        cmd = RobotCommand()
        self.get_logger().info('Standing up...')

        for state in [1, 2, 3]:
            cmd.target_state = state
            for _ in range(40):
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
        for _ in range(20):
            self.vel_pub.publish(vel)
            time.sleep(0.05)

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
        cmd.target_state = 4

        self.get_logger().info('Tracking yellow object (close only) - Ctrl+C to stop')

        log_counter = 0

        try:
            while self.running:
                vel.linear.x = 0.0
                vel.angular.z = 0.0
                status = ''

                now = time.time()
                fresh = (now - self.last_detection_time) < 0.5

                if fresh and self.box_detected:
                    error = self.box_center_x - IMAGE_CENTER_X
                    dist_str = f'{self.box_distance}mm' if self.box_distance > 0 else '?'

                    if abs(error) < DEADZONE:
                        vel.angular.z = 0.0
                        status = f'CENTERED dist={dist_str} area={self.box_area}'
                    else:
                        turn_scale = min(1.0, abs(error) / 200.0)
                        if error > 0:
                            vel.angular.z = -TURN_SPEED * turn_scale
                            status = f'TURN RIGHT err={error:+d}px dist={dist_str}'
                        else:
                            vel.angular.z = TURN_SPEED * turn_scale
                            status = f'TURN LEFT err={error:+d}px dist={dist_str}'

                elif not fresh and self._last_seen_side != 0:
                    vel.angular.z = -SEARCH_TURN_SPEED * self._last_seen_side
                    direction = "right" if self._last_seen_side > 0 else "left"
                    status = f'SEARCHING {direction}'

                else:
                    vel.angular.z = 0.0
                    status = 'NO CLOSE YELLOW OBJECT'

                self.cmd_pub.publish(cmd)
                self.vel_pub.publish(vel)

                log_counter += 1
                if log_counter % 20 == 0:
                    self.get_logger().info(status)

                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.04)

        except KeyboardInterrupt:
            pass
        finally:
            self.sit_down()


def main():
    rclpy.init()
    node = YellowObjectFollower()

    def signal_handler(sig, frame):
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
