#!/usr/bin/env python3
"""
TCP bridge for Livox LiDAR - runs on Jetson, sends pointcloud to Mini PC.
"""

import socket
import struct
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from custom_msg.msg import LivoxPointcloud

HOST = '0.0.0.0'
PORT = 9998  # Different port from camera bridge

class LivoxTCPBridge(Node):
    def __init__(self):
        super().__init__('livox_tcp_bridge')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(
            LivoxPointcloud,
            '/livox_Lidar_node/sn153/xyz/pointcloud',
            self.callback,
            qos
        )

        self._tcp_clients = []
        self._tcp_lock = threading.Lock()
        self.server_thread = threading.Thread(target=self.run_server, daemon=True)
        self.server_thread.start()

        self.frame_count = 0
        self.get_logger().info(f'Livox TCP bridge started on port {PORT}')

    def run_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen(5)
        self.get_logger().info(f'TCP server listening on {HOST}:{PORT}')

        while True:
            client, addr = server.accept()
            self.get_logger().info(f'Client connected: {addr}')
            with self._tcp_lock:
                self._tcp_clients.append(client)

    def callback(self, msg):
        self.frame_count += 1

        with self._tcp_lock:
            if not self._tcp_clients:
                if self.frame_count % 100 == 0:
                    self.get_logger().info(f'Received {self.frame_count} frames, no clients')
                return

            # Convert to XYZI format (x, y, z, intensity as floats)
            points_data = []
            for pt in msg.points:
                points_data.append(struct.pack('ffff', pt.x, pt.y, pt.z, float(pt.reflectivity)))

            data = b''.join(points_data)

            # Header: stamp_sec(4), stamp_nsec(4), num_points(4), data_len(4)
            header = struct.pack('!IIII',
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                len(msg.points),
                len(data)
            )

            dead_clients = []
            for client in self._tcp_clients:
                try:
                    client.sendall(header + data)
                except Exception as e:
                    self.get_logger().warn(f'Client error: {e}')
                    dead_clients.append(client)

            for c in dead_clients:
                self._tcp_clients.remove(c)

            if self.frame_count % 100 == 0:
                self.get_logger().info(f'Sent {self.frame_count} frames, {len(msg.points)} points, {len(self._tcp_clients)} clients')


def main():
    rclpy.init()
    node = LivoxTCPBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
