#!/usr/bin/env python3
"""
Combined RealSense Depth Publisher + TCP Bridge Sender

Publishes depth data to ROS2 locally AND sends via TCP to Mini PC.
This avoids CycloneDDS discovery issues by keeping everything in one process.
"""

import socket
import struct
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
import pyrealsense2 as rs
import numpy as np

HOST = '0.0.0.0'
PORT = 9999


class RealSenseDepthTCPPublisher(Node):
    def __init__(self):
        super().__init__('realsense_depth_tcp_node')

        # QoS profile for local publishing
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        # ROS2 Publishers (local)
        self.color_pub = self.create_publisher(Image, '/camera/camera/color/image_raw', qos)
        self.depth_pub = self.create_publisher(Image, '/camera/camera/depth/image_rect_raw', qos)
        self.points_pub = self.create_publisher(PointCloud2, '/camera/camera/points', qos)
        self.color_info_pub = self.create_publisher(CameraInfo, '/camera/camera/color/camera_info', qos)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/camera/camera/depth/camera_info', qos)

        # TCP server for remote streaming
        self._tcp_clients = []
        self._tcp_lock = threading.Lock()
        self.server_thread = threading.Thread(target=self.run_tcp_server, daemon=True)
        self.server_thread.start()

        # Configure RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Start pipeline
        self.get_logger().info('Starting RealSense pipeline...')
        self.profile = self.pipeline.start(config)

        # Create pointcloud processor
        self.pc = rs.pointcloud()

        # Get intrinsics
        color_profile = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        depth_profile = self.profile.get_stream(rs.stream.depth).as_video_stream_profile()
        self.color_intrinsics = color_profile.get_intrinsics()
        self.depth_intrinsics = depth_profile.get_intrinsics()

        # Warm up camera
        self.get_logger().info('Warming up camera...')
        for i in range(10):
            self.pipeline.wait_for_frames(timeout_ms=5000)

        self.frame_count = 0
        self.tcp_frame_count = 0
        self.get_logger().info(f'RealSense depth+TCP publisher started on port {PORT}')

    def run_tcp_server(self):
        """TCP server thread - accepts client connections"""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen(5)
        self.get_logger().info(f'TCP server listening on {HOST}:{PORT}')

        while True:
            try:
                client, addr = server.accept()
                self.get_logger().info(f'TCP client connected: {addr}')
                with self._tcp_lock:
                    self._tcp_clients.append(client)
            except Exception as e:
                self.get_logger().error(f'TCP accept error: {e}')

    def create_camera_info(self, intrinsics, stamp, frame_id):
        """Create CameraInfo message from intrinsics"""
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.width = intrinsics.width
        msg.height = intrinsics.height
        msg.k = [intrinsics.fx, 0.0, intrinsics.ppx,
                 0.0, intrinsics.fy, intrinsics.ppy,
                 0.0, 0.0, 1.0]
        msg.distortion_model = 'plumb_bob'
        msg.d = list(intrinsics.coeffs)
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
                 0.0, intrinsics.fy, intrinsics.ppy, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        return msg

    def _create_pointcloud_fields(self):
        """Create PointCloud2 field definitions"""
        fields = []
        for name, offset in [('x', 0), ('y', 4), ('z', 8)]:
            f = PointField()
            f.name = name
            f.offset = offset
            f.datatype = PointField.FLOAT32
            f.count = 1
            fields.append(f)

        f = PointField()
        f.name = 'rgb'
        f.offset = 12
        f.datatype = PointField.UINT32
        f.count = 1
        fields.append(f)
        return fields

    def send_tcp_data(self, depth_data, points_data, color_data, stamp_sec, stamp_nsec):
        """Send depth, pointcloud, and color data to all TCP clients"""
        with self._tcp_lock:
            if not self._tcp_clients:
                return

            dead_clients = []
            for client in self._tcp_clients:
                try:
                    # Send color image (type 3)
                    if color_data:
                        header = struct.pack('!BIIIII',
                            3,  # type: color
                            color_data['width'], color_data['height'],
                            stamp_sec, stamp_nsec,
                            len(color_data['data']))
                        client.sendall(header + color_data['data'])

                    # Send depth image (type 1)
                    header = struct.pack('!BIIIII',
                        1,  # type: depth
                        depth_data['width'], depth_data['height'],
                        stamp_sec, stamp_nsec,
                        len(depth_data['data']))
                    client.sendall(header + depth_data['data'])

                    # Send pointcloud if available (type 2)
                    if points_data:
                        header = struct.pack('!BIIIIII',
                            2,  # type: pointcloud
                            points_data['width'], points_data['height'],
                            points_data['point_step'], points_data['row_step'],
                            stamp_sec, stamp_nsec)
                        size = struct.pack('!I', len(points_data['data']))
                        client.sendall(header + size + points_data['data'])

                    self.tcp_frame_count += 1

                except Exception as e:
                    self.get_logger().warn(f'TCP client error: {e}')
                    dead_clients.append(client)

            for c in dead_clients:
                self._tcp_clients.remove(c)

    def publish_frame(self):
        """Publish one frame to ROS2 and TCP"""
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=5000)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                return False

            stamp = self.get_clock().now().to_msg()
            stamp_sec = stamp.sec
            stamp_nsec = stamp.nanosec

            # Convert to numpy
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_bytes = depth_image.tobytes()

            # Publish color to ROS2
            color_msg = Image()
            color_msg.header.stamp = stamp
            color_msg.header.frame_id = 'camera_color_optical_frame'
            color_msg.height = color_image.shape[0]
            color_msg.width = color_image.shape[1]
            color_msg.encoding = 'bgr8'
            color_msg.is_bigendian = False
            color_msg.step = color_image.shape[1] * 3
            color_msg.data = color_image.tobytes()
            self.color_pub.publish(color_msg)

            # Publish depth to ROS2
            depth_msg = Image()
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = 'camera_depth_optical_frame'
            depth_msg.height = depth_image.shape[0]
            depth_msg.width = depth_image.shape[1]
            depth_msg.encoding = '16UC1'
            depth_msg.is_bigendian = False
            depth_msg.step = depth_image.shape[1] * 2
            depth_msg.data = depth_bytes
            self.depth_pub.publish(depth_msg)

            # Publish camera info
            self.color_info_pub.publish(
                self.create_camera_info(self.color_intrinsics, stamp, 'camera_color_optical_frame'))
            self.depth_info_pub.publish(
                self.create_camera_info(self.depth_intrinsics, stamp, 'camera_depth_optical_frame'))

            # Prepare TCP color data
            color_bytes = color_image.tobytes()
            tcp_color_data = {
                'width': color_image.shape[1],
                'height': color_image.shape[0],
                'data': color_bytes
            }

            # Prepare TCP depth data
            tcp_depth_data = {
                'width': depth_image.shape[1],
                'height': depth_image.shape[0],
                'data': depth_bytes
            }

            # Publish pointcloud every 3rd frame (10 Hz)
            tcp_points_data = None
            self.frame_count += 1
            if self.frame_count % 3 == 0:
                self.pc.map_to(color_frame)
                points = self.pc.calculate(depth_frame)
                vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)

                # Filter valid points
                valid_mask = ~np.isnan(vertices[:, 2]) & (vertices[:, 2] > 0)
                valid_points = vertices[valid_mask]
                valid_colors = color_image.reshape(-1, 3)[valid_mask]

                if len(valid_points) > 0:
                    # Build pointcloud data
                    pc_data = []
                    for i, pt in enumerate(valid_points):
                        x, y, z = pt
                        b, g, r = valid_colors[i]
                        rgb = struct.unpack('I', struct.pack('BBBB', int(b), int(g), int(r), 255))[0]
                        pc_data.append(struct.pack('fffI', x, y, z, rgb))
                    pc_bytes = b''.join(pc_data)

                    # ROS2 pointcloud message
                    pc_msg = PointCloud2()
                    pc_msg.header.stamp = stamp
                    pc_msg.header.frame_id = 'camera_depth_optical_frame'
                    pc_msg.fields = self._create_pointcloud_fields()
                    pc_msg.height = 1
                    pc_msg.width = len(valid_points)
                    pc_msg.is_dense = True
                    pc_msg.is_bigendian = False
                    pc_msg.point_step = 16
                    pc_msg.row_step = 16 * len(valid_points)
                    pc_msg.data = pc_bytes
                    self.points_pub.publish(pc_msg)

                    # TCP pointcloud data
                    tcp_points_data = {
                        'width': len(valid_points),
                        'height': 1,
                        'point_step': 16,
                        'row_step': 16 * len(valid_points),
                        'data': pc_bytes
                    }

            # Send to TCP clients
            self.send_tcp_data(tcp_depth_data, tcp_points_data, tcp_color_data, stamp_sec, stamp_nsec)

            if self.frame_count % 100 == 0:
                with self._tcp_lock:
                    num_clients = len(self._tcp_clients)
                self.get_logger().info(f'Published {self.frame_count} frames, TCP sent: {self.tcp_frame_count}, clients: {num_clients}')

            return True

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            return False

    def stop(self):
        self.get_logger().info('Stopping RealSense pipeline...')
        self.pipeline.stop()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseDepthTCPPublisher()

    try:
        while rclpy.ok():
            node.publish_frame()
            rclpy.spin_once(node, timeout_sec=0.001)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
