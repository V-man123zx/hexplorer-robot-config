#!/usr/bin/env python3
"""
TCP bridge receiver - runs on Mini PC, receives depth data and publishes to ROS2.
"""

import socket
import struct
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

JETSON_IP = '192.168.1.20'
PORT = 9999

class DepthBridgeReceiver(Node):
    def __init__(self):
        super().__init__('depth_bridge_receiver')

        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.points_pub = self.create_publisher(PointCloud2, '/camera/points', 10)
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)

        self._running = True
        self._recv_thread = threading.Thread(target=self.receive_data, daemon=True)
        self._recv_thread.start()

        self._depth_count = 0
        self._points_count = 0
        self._color_count = 0
        self.timer = self.create_timer(5.0, self.log_stats)

        self.get_logger().info(f'Connecting to Jetson at {JETSON_IP}:{PORT}...')

    def log_stats(self):
        self.get_logger().info(f'Received: {self._color_count} color, {self._depth_count} depth, {self._points_count} pointcloud frames')

    def receive_data(self):
        import time
        while self._running:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((JETSON_IP, PORT))
                self.get_logger().info('Connected to Jetson!')

                while self._running:
                    # Read message type (1 byte)
                    type_byte = sock.recv(1)
                    if not type_byte:
                        break
                    msg_type = struct.unpack('!B', type_byte)[0]

                    if msg_type == 1:  # Depth image
                        header = sock.recv(20)
                        if len(header) < 20:
                            continue
                        width, height, stamp_sec, stamp_nsec, data_len = struct.unpack('!IIIII', header)

                        data = b''
                        while len(data) < data_len:
                            chunk = sock.recv(min(65536, data_len - len(data)))
                            if not chunk:
                                break
                            data += chunk

                        if len(data) == data_len:
                            msg = Image()
                            msg.header.stamp.sec = stamp_sec
                            msg.header.stamp.nanosec = stamp_nsec
                            msg.header.frame_id = 'camera_depth_optical_frame'
                            msg.width = width
                            msg.height = height
                            msg.encoding = '16UC1'
                            msg.is_bigendian = False
                            msg.step = width * 2
                            msg.data = list(data)
                            self.depth_pub.publish(msg)
                            self._depth_count += 1

                    elif msg_type == 3:  # Color image
                        header = sock.recv(20)
                        if len(header) < 20:
                            continue
                        width, height, stamp_sec, stamp_nsec, data_len = struct.unpack('!IIIII', header)

                        data = b''
                        while len(data) < data_len:
                            chunk = sock.recv(min(65536, data_len - len(data)))
                            if not chunk:
                                break
                            data += chunk

                        if len(data) == data_len:
                            msg = Image()
                            msg.header.stamp.sec = stamp_sec
                            msg.header.stamp.nanosec = stamp_nsec
                            msg.header.frame_id = 'camera_color_optical_frame'
                            msg.width = width
                            msg.height = height
                            msg.encoding = 'bgr8'
                            msg.is_bigendian = False
                            msg.step = width * 3
                            msg.data = list(data)
                            self.color_pub.publish(msg)
                            self._color_count += 1

                    elif msg_type == 2:  # Pointcloud
                        header = sock.recv(24)
                        if len(header) < 24:
                            continue
                        width, height, point_step, row_step, stamp_sec, stamp_nsec = struct.unpack('!IIIIII', header)

                        size_bytes = sock.recv(4)
                        if len(size_bytes) < 4:
                            continue
                        data_len = struct.unpack('!I', size_bytes)[0]

                        data = b''
                        while len(data) < data_len:
                            chunk = sock.recv(min(65536, data_len - len(data)))
                            if not chunk:
                                break
                            data += chunk

                        if len(data) == data_len:
                            msg = PointCloud2()
                            msg.header.stamp.sec = stamp_sec
                            msg.header.stamp.nanosec = stamp_nsec
                            msg.header.frame_id = 'camera_depth_optical_frame'
                            msg.width = width
                            msg.height = height
                            msg.point_step = point_step
                            msg.row_step = row_step
                            msg.is_dense = True
                            msg.is_bigendian = False

                            # Define fields
                            msg.fields = [
                                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
                            ]
                            msg.data = list(data)
                            self.points_pub.publish(msg)
                            self._points_count += 1

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
    node = DepthBridgeReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
