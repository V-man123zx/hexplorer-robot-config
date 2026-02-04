#!/usr/bin/env python3
"""
TCP receiver for Livox LiDAR - runs on Mini PC, receives and publishes to ROS2.
"""

import socket
import struct
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

JETSON_IP = '192.168.1.20'
PORT = 9998

class LivoxTCPReceiver(Node):
    def __init__(self):
        super().__init__('livox_tcp_receiver')

        self.pub = self.create_publisher(PointCloud2, '/livox/pointcloud', 10)

        self._running = True
        self._recv_thread = threading.Thread(target=self.receive_data, daemon=True)
        self._recv_thread.start()

        self._frame_count = 0
        self.timer = self.create_timer(5.0, self.log_stats)
        self.get_logger().info(f'Connecting to Jetson at {JETSON_IP}:{PORT}...')

    def log_stats(self):
        self.get_logger().info(f'Received {self._frame_count} LiDAR frames')

    def receive_data(self):
        import time
        while self._running:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((JETSON_IP, PORT))
                self.get_logger().info('Connected to Jetson Livox bridge!')

                while self._running:
                    # Read header: stamp_sec, stamp_nsec, num_points, data_len
                    header = b''
                    while len(header) < 16:
                        chunk = sock.recv(16 - len(header))
                        if not chunk:
                            break
                        header += chunk

                    if len(header) < 16:
                        break

                    stamp_sec, stamp_nsec, num_points, data_len = struct.unpack('!IIII', header)

                    # Read point data
                    data = b''
                    while len(data) < data_len:
                        chunk = sock.recv(min(65536, data_len - len(data)))
                        if not chunk:
                            break
                        data += chunk

                    if len(data) == data_len:
                        # Create PointCloud2 message
                        msg = PointCloud2()
                        msg.header.stamp.sec = stamp_sec
                        msg.header.stamp.nanosec = stamp_nsec
                        msg.header.frame_id = 'livox_frame'

                        msg.fields = [
                            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                        ]

                        msg.height = 1
                        msg.width = num_points
                        msg.is_dense = True
                        msg.is_bigendian = False
                        msg.point_step = 16
                        msg.row_step = 16 * num_points
                        msg.data = data

                        self.pub.publish(msg)
                        self._frame_count += 1

                sock.close()

            except socket.timeout:
                self.get_logger().warn('Connection timed out, retrying...')
            except ConnectionRefusedError:
                self.get_logger().warn('Connection refused, retrying in 2s...')
            except Exception as e:
                self.get_logger().error(f'Error: {e}')

            time.sleep(2)

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main():
    rclpy.init()
    node = LivoxTCPReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
