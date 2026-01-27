#!/usr/bin/env python3
"""
RealSense D435 Depth/PointCloud Publisher for ROS2

Publishes color, depth, and pointcloud data using pyrealsense2 v2.55
which avoids the "RGB modules inconsistency" bug in v2.56.

Topics published:
- /camera/camera/color/image_raw (sensor_msgs/Image) - 640x480 BGR8
- /camera/camera/depth/image_rect_raw (sensor_msgs/Image) - 640x480 Z16 (mm)
- /camera/camera/points (sensor_msgs/PointCloud2) - XYZ pointcloud
- /camera/camera/color/camera_info (sensor_msgs/CameraInfo)
- /camera/camera/depth/camera_info (sensor_msgs/CameraInfo)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
import pyrealsense2 as rs
import numpy as np
import struct


class RealSenseDepthPublisher(Node):
    def __init__(self):
        super().__init__('realsense_depth_node')

        # QoS profile for reliable cross-machine streaming
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publishers with explicit QoS
        self.color_pub = self.create_publisher(Image, '/camera/camera/color/image_raw', qos)
        self.depth_pub = self.create_publisher(Image, '/camera/camera/depth/image_rect_raw', qos)
        self.points_pub = self.create_publisher(PointCloud2, '/camera/camera/points', qos)
        self.color_info_pub = self.create_publisher(CameraInfo, '/camera/camera/color/camera_info', qos)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/camera/camera/depth/camera_info', qos)

        # Configure RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Start pipeline
        self.get_logger().info('Starting RealSense pipeline...')
        self.profile = self.pipeline.start(config)

        # Create align object to align depth to color
        self.align = rs.align(rs.stream.color)

        # Create pointcloud processor
        self.pc = rs.pointcloud()

        # Get intrinsics
        color_profile = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        depth_profile = self.profile.get_stream(rs.stream.depth).as_video_stream_profile()

        self.color_intrinsics = color_profile.get_intrinsics()
        self.depth_intrinsics = depth_profile.get_intrinsics()

        # Warm up - discard first few frames
        self.get_logger().info('Warming up camera...')
        for i in range(10):
            self.pipeline.wait_for_frames(timeout_ms=5000)

        self.frame_count = 0
        self.get_logger().info('RealSense depth publisher started!')

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

    def create_pointcloud_msg(self, points, colors, stamp, frame_id):
        """Create PointCloud2 message from points and colors"""
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id

        # Filter valid points
        valid_mask = ~np.isnan(points[:, 2]) & (points[:, 2] > 0)
        valid_points = points[valid_mask]
        valid_colors = colors[valid_mask] if colors is not None else None

        if len(valid_points) == 0:
            msg.height = 1
            msg.width = 0
            msg.is_dense = True
            msg.is_bigendian = False
            msg.point_step = 16
            msg.row_step = 0
            msg.fields = self._create_pointcloud_fields()
            msg.data = bytes()
            return msg

        msg.fields = self._create_pointcloud_fields()
        msg.height = 1
        msg.width = len(valid_points)
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width

        # Pack data
        data = []
        for i, pt in enumerate(valid_points):
            x, y, z = pt
            if valid_colors is not None:
                b, g, r = valid_colors[i]
                rgb = struct.unpack('I', struct.pack('BBBB', int(b), int(g), int(r), 255))[0]
            else:
                rgb = struct.unpack('I', struct.pack('BBBB', 255, 255, 255, 255))[0]
            data.append(struct.pack('fffI', x, y, z, rgb))

        msg.data = b''.join(data)
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

    def publish_frame(self):
        """Publish one frame of color, depth, and pointcloud"""
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=5000)

            # Use unaligned frames (depth intrinsics are different from color)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                return False

            stamp = self.get_clock().now().to_msg()

            # Convert to numpy
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Publish color
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

            # Publish depth
            depth_msg = Image()
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = 'camera_depth_optical_frame'
            depth_msg.height = depth_image.shape[0]
            depth_msg.width = depth_image.shape[1]
            depth_msg.encoding = '16UC1'
            depth_msg.is_bigendian = False
            depth_msg.step = depth_image.shape[1] * 2
            depth_msg.data = depth_image.tobytes()
            self.depth_pub.publish(depth_msg)

            # Publish camera info
            self.color_info_pub.publish(
                self.create_camera_info(self.color_intrinsics, stamp, 'camera_color_optical_frame'))
            self.depth_info_pub.publish(
                self.create_camera_info(self.depth_intrinsics, stamp, 'camera_depth_optical_frame'))

            # Publish pointcloud every 10th frame to reduce CPU load
            self.frame_count += 1
            if self.frame_count % 10 == 0:
                self.pc.map_to(color_frame)
                points = self.pc.calculate(depth_frame)
                vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
                pc_msg = self.create_pointcloud_msg(
                    vertices,
                    color_image.reshape(-1, 3),
                    stamp,
                    'camera_depth_optical_frame'
                )
                self.points_pub.publish(pc_msg)

            if self.frame_count % 100 == 0:
                self.get_logger().info(f'Published {self.frame_count} frames')

            return True

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            return False

    def stop(self):
        self.get_logger().info('Stopping RealSense pipeline...')
        self.pipeline.stop()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseDepthPublisher()

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
