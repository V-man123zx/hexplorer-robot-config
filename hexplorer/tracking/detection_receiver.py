#!/usr/bin/env python3
"""
Detection Receiver for Mini PC

Receives object detection data via TCP from Jetson and publishes to ROS2 topics.

Topics published:
- /object_detection (std_msgs/String) - JSON detection data
- /object_position (geometry_msgs/Point) - 3D position (x=center_x, y=center_y, z=distance_mm)
- /camera/color/image_raw (sensor_msgs/Image) - Color image (when --with-images)

Usage:
    source /opt/ros/humble/setup.bash
    python3 detection_receiver.py
    python3 detection_receiver.py --with-images  # Also receive and publish images
"""

import socket
import struct
import threading
import json
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

JETSON_IP = '192.168.1.20'
PORT = 9997
IMAGE_PORT = 9996

# Detection message format (57 bytes total)
DETECTION_FORMAT = '!BHHHHHHIfI32s'
DETECTION_SIZE = struct.calcsize(DETECTION_FORMAT)

# Image header format
IMAGE_HEADER_FORMAT = '!III'
IMAGE_HEADER_SIZE = struct.calcsize(IMAGE_HEADER_FORMAT)


class DetectionReceiver(Node):
    def __init__(self, with_images=False):
        super().__init__('detection_receiver')

        self._with_images = with_images

        # Publishers
        self.detection_pub = self.create_publisher(String, '/object_detection', 10)
        self.position_pub = self.create_publisher(Point, '/object_position', 10)
        if with_images:
            self.image_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)

        # State
        self._running = True
        self._connected = False
        self._image_connected = False
        self._recv_count = 0
        self._detection_count = 0
        self._image_count = 0

        # Start detection receiver thread
        self._recv_thread = threading.Thread(target=self._receive_data, daemon=True)
        self._recv_thread.start()

        # Start image receiver thread if enabled
        if with_images:
            self._image_thread = threading.Thread(target=self._receive_images, daemon=True)
            self._image_thread.start()

        # Stats timer
        self.timer = self.create_timer(5.0, self._log_stats)

        self.get_logger().info(f'Detection Receiver starting...')
        self.get_logger().info(f'Connecting to Jetson at {JETSON_IP}:{PORT}')
        if with_images:
            self.get_logger().info(f'Image streaming enabled, connecting to port {IMAGE_PORT}')

    def _log_stats(self):
        status = "connected" if self._connected else "disconnected"
        msg = f'Status: {status}, Received: {self._recv_count}, Detections: {self._detection_count}'
        if self._with_images:
            img_status = "connected" if self._image_connected else "disconnected"
            msg += f', Images: {self._image_count} ({img_status})'
        self.get_logger().info(msg)

    def _receive_data(self):
        """Receive detection data from Jetson via TCP."""
        import time

        while self._running:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((JETSON_IP, PORT))
                self._connected = True
                self.get_logger().info('Connected to Jetson object tracker!')

                # Set longer timeout for ongoing communication
                sock.settimeout(2.0)

                while self._running:
                    # Receive detection message (fixed size)
                    data = b''
                    while len(data) < DETECTION_SIZE:
                        chunk = sock.recv(DETECTION_SIZE - len(data))
                        if not chunk:
                            raise ConnectionError("Connection closed")
                        data += chunk

                    # Unpack detection
                    (detected, center_x, center_y, bbox_x, bbox_y, bbox_w, bbox_h,
                     distance_mm, confidence, timestamp, label_bytes) = struct.unpack(
                        DETECTION_FORMAT, data
                    )

                    # Decode label
                    label = label_bytes.rstrip(b'\x00').decode('utf-8', errors='ignore')

                    self._recv_count += 1

                    # Create detection dict
                    detection = {
                        'detected': bool(detected),
                        'center_x': center_x,
                        'center_y': center_y,
                        'bbox_x': bbox_x,
                        'bbox_y': bbox_y,
                        'bbox_w': bbox_w,
                        'bbox_h': bbox_h,
                        'distance_mm': distance_mm,
                        'confidence': confidence,
                        'timestamp': timestamp,
                        'label': label
                    }

                    if detected:
                        self._detection_count += 1

                    # Publish JSON detection
                    json_msg = String()
                    json_msg.data = json.dumps(detection)
                    self.detection_pub.publish(json_msg)

                    # Publish position (center_x, center_y in pixels, distance in mm)
                    pos_msg = Point()
                    pos_msg.x = float(center_x)
                    pos_msg.y = float(center_y)
                    pos_msg.z = float(distance_mm)
                    self.position_pub.publish(pos_msg)

            except socket.timeout:
                self.get_logger().warn('Connection timed out, reconnecting...')
                self._connected = False
            except ConnectionRefusedError:
                self.get_logger().warn('Connection refused, retrying in 2s...')
                self._connected = False
            except ConnectionError as e:
                self.get_logger().warn(f'Connection error: {e}, reconnecting...')
                self._connected = False
            except Exception as e:
                self.get_logger().error(f'Error: {e}')
                self._connected = False

            try:
                sock.close()
            except:
                pass

            if self._running:
                time.sleep(2)

    def _receive_images(self):
        """Receive color images from Jetson via TCP."""
        import time

        while self._running:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((JETSON_IP, IMAGE_PORT))
                self._image_connected = True
                self.get_logger().info('Connected to Jetson image stream!')

                sock.settimeout(2.0)

                while self._running:
                    # Receive header
                    header = b''
                    while len(header) < IMAGE_HEADER_SIZE:
                        chunk = sock.recv(IMAGE_HEADER_SIZE - len(header))
                        if not chunk:
                            raise ConnectionError("Image connection closed")
                        header += chunk

                    width, height, data_len = struct.unpack(IMAGE_HEADER_FORMAT, header)

                    # Receive image data
                    data = b''
                    while len(data) < data_len:
                        chunk = sock.recv(min(65536, data_len - len(data)))
                        if not chunk:
                            raise ConnectionError("Image data interrupted")
                        data += chunk

                    # Publish image
                    msg = Image()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'camera_color_optical_frame'
                    msg.width = width
                    msg.height = height
                    msg.encoding = 'bgr8'
                    msg.is_bigendian = False
                    msg.step = width * 3
                    msg.data = list(data)
                    self.image_pub.publish(msg)
                    self._image_count += 1

            except socket.timeout:
                self._image_connected = False
            except ConnectionRefusedError:
                self._image_connected = False
            except ConnectionError:
                self._image_connected = False
            except Exception as e:
                self.get_logger().error(f'Image error: {e}')
                self._image_connected = False

            try:
                sock.close()
            except:
                pass

            if self._running:
                time.sleep(2)

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def parse_args():
    parser = argparse.ArgumentParser(description='Detection receiver for Mini PC')
    parser.add_argument('--with-images', action='store_true',
                        help='Also receive and publish color images')
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = DetectionReceiver(with_images=args.with_images)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
