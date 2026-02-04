#!/usr/bin/env python3
"""
Tracking Visualizer for Mini PC

Visualizes object detection results from the Jetson tracker.
Does NOT control the robot - just displays detection status.

Usage:
    source /opt/ros/humble/setup.bash
    python3 tracking_visualizer.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
import json
import time
import sys

# Image parameters (for display purposes)
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
IMAGE_CENTER_X = IMAGE_WIDTH // 2
DEADZONE = 50


class TrackingVisualizer(Node):
    def __init__(self):
        super().__init__('tracking_visualizer')

        # Subscribe to detection topic
        self.create_subscription(
            String, '/object_detection', self.detection_callback, 10)

        # State
        self.last_detection = None
        self.last_detection_time = 0
        self.detection_count = 0
        self.frame_count = 0
        self.start_time = time.time()

        # Display timer (10Hz)
        self.timer = self.create_timer(0.1, self.display_status)

        self.get_logger().info('Tracking Visualizer started')
        self.get_logger().info('Waiting for /object_detection topic...')

    def detection_callback(self, msg):
        """Process detection message."""
        try:
            self.last_detection = json.loads(msg.data)
            self.last_detection_time = time.time()
            self.frame_count += 1
            if self.last_detection['detected']:
                self.detection_count += 1
        except json.JSONDecodeError:
            pass

    def display_status(self):
        """Display current tracking status."""
        now = time.time()
        fresh = (now - self.last_detection_time) < 0.5

        # Clear line and move cursor
        sys.stdout.write('\r' + ' ' * 100 + '\r')

        if not fresh or self.last_detection is None:
            elapsed = now - self.start_time
            fps = self.frame_count / elapsed if elapsed > 0 else 0
            sys.stdout.write(f'[WAITING] Frames: {self.frame_count} | Detections: {self.detection_count} | FPS: {fps:.1f}')
        else:
            d = self.last_detection
            if d['detected']:
                cx = d['center_x']
                cy = d['center_y']
                dist = d['distance_mm']
                conf = d['confidence']
                label = d['label']
                bbox_w = d['bbox_w']
                bbox_h = d['bbox_h']

                # Calculate direction
                error = cx - IMAGE_CENTER_X
                if error < -DEADZONE:
                    direction = "<-- LEFT "
                elif error > DEADZONE:
                    direction = " RIGHT -->"
                else:
                    direction = " CENTER  "

                # Visual position bar
                bar_width = 40
                pos_ratio = cx / IMAGE_WIDTH
                bar_pos = int(pos_ratio * bar_width)
                bar = '[' + '-' * bar_pos + 'O' + '-' * (bar_width - bar_pos - 1) + ']'

                sys.stdout.write(
                    f'[DETECTED] {label} | {bar} {direction} | '
                    f'Dist: {dist:4d}mm | Size: {bbox_w}x{bbox_h} | Conf: {conf:.2f}'
                )
            else:
                elapsed = now - self.start_time
                fps = self.frame_count / elapsed if elapsed > 0 else 0
                sys.stdout.write(f'[NO OBJECT] Frames: {self.frame_count} | Detections: {self.detection_count} | FPS: {fps:.1f}')

        sys.stdout.flush()


def main():
    print("=" * 60)
    print("  Object Tracking Visualizer")
    print("=" * 60)
    print("Displays detection results from Jetson tracker")
    print("Press Ctrl+C to stop")
    print("")

    rclpy.init()
    node = TrackingVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        # Print final stats
        elapsed = time.time() - node.start_time
        fps = node.frame_count / elapsed if elapsed > 0 else 0
        print(f"\nFinal stats:")
        print(f"  Total frames: {node.frame_count}")
        print(f"  Total detections: {node.detection_count}")
        print(f"  Average FPS: {fps:.1f}")
        print(f"  Detection rate: {100 * node.detection_count / max(1, node.frame_count):.1f}%")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
