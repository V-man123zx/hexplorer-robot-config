#!/usr/bin/env python3
"""
IMU TCP Bridge - Runs on Jetson
Subscribes to /livox_Lidar_node/sn.../imu/raw_data and sends via TCP to Mini PC
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import socket
import struct
import time
import threading


class ImuTcpBridge(Node):
    def __init__(self):
        super().__init__('imu_tcp_bridge')

        self.declare_parameter('imu_topic', '/livox_Lidar_node/sn153/imu/raw_data')
        self.declare_parameter('tcp_port', 9995)

        self.imu_topic = self.get_parameter('imu_topic').value
        self.tcp_port = self.get_parameter('tcp_port').value

        # TCP server
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('0.0.0.0', self.tcp_port))
        self.server_socket.listen(1)
        self.server_socket.settimeout(0.1)

        self.client = None
        self.client_lock = threading.Lock()

        # Accept thread
        self.accept_thread = threading.Thread(target=self._accept_clients, daemon=True)
        self.accept_thread.start()

        # Subscribe to IMU
        self.sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10
        )

        self.get_logger().info(f'IMU TCP Bridge started')
        self.get_logger().info(f'  Subscribing to: {self.imu_topic}')
        self.get_logger().info(f'  TCP port: {self.tcp_port}')

        self.msg_count = 0

    def _accept_clients(self):
        while rclpy.ok():
            try:
                client, addr = self.server_socket.accept()
                with self.client_lock:
                    if self.client:
                        self.client.close()
                    self.client = client
                self.get_logger().info(f'Client connected: {addr}')
            except socket.timeout:
                pass
            except Exception as e:
                self.get_logger().error(f'Accept error: {e}')

    def imu_callback(self, msg):
        with self.client_lock:
            if not self.client:
                return

            try:
                # Pack IMU data: timestamp (8) + orientation (32) + angular_vel (24) + linear_acc (24) = 88 bytes
                data = struct.pack('!Q',  # Network byte order
                    msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)

                # Orientation quaternion (x, y, z, w)
                data += struct.pack('!4d',
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w)

                # Angular velocity (x, y, z)
                data += struct.pack('!3d',
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z)

                # Linear acceleration (x, y, z)
                data += struct.pack('!3d',
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z)

                # Send with length prefix
                self.client.sendall(struct.pack('!I', len(data)) + data)

                self.msg_count += 1
                if self.msg_count % 200 == 0:
                    self.get_logger().info(f'Sent {self.msg_count} IMU messages')

            except (BrokenPipeError, ConnectionResetError):
                self.get_logger().warn('Client disconnected')
                self.client = None
            except Exception as e:
                self.get_logger().error(f'Send error: {e}')
                self.client = None


def main():
    rclpy.init()
    node = ImuTcpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
