#!/usr/bin/env python3
"""
Relay node that subscribes to depth topics locally and republishes
with explicit QoS settings to help cross-machine discovery.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, CameraInfo


class DepthRelay(Node):
    def __init__(self):
        super().__init__('depth_relay')

        # QoS profile optimized for cross-machine transfer
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscribers (local topics)
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw',
            self.depth_callback, qos)
        self.points_sub = self.create_subscription(
            PointCloud2, '/camera/camera/points',
            self.points_callback, qos)
        self.depth_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/depth/camera_info',
            self.depth_info_callback, qos)

        # Publishers (relay topics)
        self.depth_pub = self.create_publisher(
            Image, '/relay/depth/image_raw', qos)
        self.points_pub = self.create_publisher(
            PointCloud2, '/relay/points', qos)
        self.depth_info_pub = self.create_publisher(
            CameraInfo, '/relay/depth/camera_info', qos)

        self.frame_count = 0
        self.get_logger().info('Depth relay node started!')

    def depth_callback(self, msg):
        self.depth_pub.publish(msg)
        self.frame_count += 1
        if self.frame_count % 100 == 0:
            self.get_logger().info(f'Relayed {self.frame_count} depth frames')

    def points_callback(self, msg):
        self.points_pub.publish(msg)

    def depth_info_callback(self, msg):
        self.depth_info_pub.publish(msg)


def main():
    rclpy.init()
    node = DepthRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
