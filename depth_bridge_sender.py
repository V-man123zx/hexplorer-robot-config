#!/usr/bin/env python3
"""
TCP bridge sender - runs on Jetson, sends depth data to Mini PC.
"""

import socket
import struct
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import numpy as np

HOST = '0.0.0.0'
PORT = 9999

class DepthBridgeSender(Node):
    def __init__(self):
        super().__init__('depth_bridge_sender')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw',
            self.depth_callback, qos)
        self.points_sub = self.create_subscription(
            PointCloud2, '/camera/camera/points',
            self.points_callback, qos)

        self._tcp_clients = []
        self._tcp_lock = threading.Lock()
        self._depth_data = None
        self._points_data = None

        # Start TCP server
        self.server_thread = threading.Thread(target=self.run_server, daemon=True)
        self.server_thread.start()

        # Periodic sender
        self.timer = self.create_timer(0.1, self.send_data)  # 10 Hz

        self.get_logger().info(f'Depth bridge sender started on port {PORT}')

    def run_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen(5)

        while True:
            client, addr = server.accept()
            self.get_logger().info(f'Client connected: {addr}')
            with self._tcp_lock:
                self._tcp_clients.append(client)

    def depth_callback(self, msg):
        # Store depth as numpy array
        self._depth_data = {
            'width': msg.width,
            'height': msg.height,
            'data': bytes(msg.data),
            'stamp_sec': msg.header.stamp.sec,
            'stamp_nsec': msg.header.stamp.nanosec
        }

    def points_callback(self, msg):
        self._points_data = {
            'width': msg.width,
            'height': msg.height,
            'point_step': msg.point_step,
            'row_step': msg.row_step,
            'data': bytes(msg.data),
            'stamp_sec': msg.header.stamp.sec,
            'stamp_nsec': msg.header.stamp.nanosec
        }

    def send_data(self):
        if not self._depth_data:
            return

        with self._tcp_lock:
            dead_clients = []
            for client in self._tcp_clients:
                try:
                    # Send depth image
                    d = self._depth_data
                    header = struct.pack('!BIIIII',
                        1,  # type: depth
                        d['width'], d['height'],
                        d['stamp_sec'], d['stamp_nsec'],
                        len(d['data']))
                    client.sendall(header + d['data'])

                    # Send pointcloud if available
                    if self._points_data:
                        p = self._points_data
                        header = struct.pack('!BIIIIII',
                            2,  # type: pointcloud
                            p['width'], p['height'],
                            p['point_step'], p['row_step'],
                            p['stamp_sec'], p['stamp_nsec'])
                        size = struct.pack('!I', len(p['data']))
                        client.sendall(header + size + p['data'])

                except Exception as e:
                    self.get_logger().warn(f'Client error: {e}')
                    dead_clients.append(client)

            for c in dead_clients:
                self._tcp_clients.remove(c)


def main():
    rclpy.init()
    node = DepthBridgeSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
