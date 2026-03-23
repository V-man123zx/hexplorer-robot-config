#!/usr/bin/env python3
"""
IMU TCP Receiver - Runs on Mini PC
Receives IMU data via TCP and publishes to /livox/imu
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import socket
import struct
import threading


class ImuTcpReceiver(Node):
    def __init__(self):
        super().__init__('imu_tcp_receiver')

        self.declare_parameter('jetson_ip', '192.168.1.20')
        self.declare_parameter('tcp_port', 9995)
        self.declare_parameter('output_topic', '/livox/imu')

        self.jetson_ip = self.get_parameter('jetson_ip').value
        self.tcp_port = self.get_parameter('tcp_port').value
        self.output_topic = self.get_parameter('output_topic').value

        # Publisher
        self.pub = self.create_publisher(Imu, self.output_topic, 10)
        self.msg_count = 0

        # TCP client thread
        self.connected = False
        self.recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.recv_thread.start()

        self.get_logger().info(f'IMU TCP Receiver started')
        self.get_logger().info(f'  Connecting to: {self.jetson_ip}:{self.tcp_port}')
        self.get_logger().info(f'  Publishing to: {self.output_topic}')

        self.msg_count = 0

    def _receive_loop(self):
        while rclpy.ok():
            try:
                # Connect to Jetson
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((self.jetson_ip, self.tcp_port))
                self.connected = True
                self.get_logger().info('Connected to Jetson IMU bridge')

                while rclpy.ok():
                    # Read length prefix
                    length_data = self._recv_exact(sock, 4)
                    if not length_data:
                        break
                    length = struct.unpack('!I', length_data)[0]

                    # Read data
                    data = self._recv_exact(sock, length)
                    if not data:
                        break

                    # Unpack and publish
                    self._publish_imu(data)

            except socket.timeout:
                self.get_logger().warn('Connection timeout, retrying...')
            except ConnectionRefusedError:
                self.get_logger().warn('Connection refused, retrying in 2s...')
            except Exception as e:
                self.get_logger().error(f'Receive error: {e}')

            self.connected = False
            # Wait before retry
            import time
            time.sleep(2.0)

    def _recv_exact(self, sock, n):
        data = b''
        while len(data) < n:
            chunk = sock.recv(n - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    def _publish_imu(self, data):
        msg = Imu()

        # Use current time (Mini PC time) to avoid clock sync issues
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = 'livox_frame'

        # Unpack: timestamp (8) + orientation (32) + angular_vel (24) + linear_acc (24)
        offset = 0

        # Skip original timestamp (we use local time)
        offset += 8

        # Orientation
        ox, oy, oz, ow = struct.unpack_from('!4d', data, offset)
        msg.orientation.x = ox
        msg.orientation.y = oy
        msg.orientation.z = oz
        msg.orientation.w = ow
        offset += 32

        # Angular velocity
        avx, avy, avz = struct.unpack_from('!3d', data, offset)
        msg.angular_velocity.x = avx
        msg.angular_velocity.y = avy
        msg.angular_velocity.z = avz
        offset += 24

        # Linear acceleration
        lax, lay, laz = struct.unpack_from('!3d', data, offset)
        msg.linear_acceleration.x = lax
        msg.linear_acceleration.y = lay
        msg.linear_acceleration.z = laz

        # Covariance (unknown)
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        self.pub.publish(msg)

        self.msg_count += 1
        if self.msg_count % 200 == 0:
            self.get_logger().info(f'Published {self.msg_count} IMU messages')


def main():
    rclpy.init()
    node = ImuTcpReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
