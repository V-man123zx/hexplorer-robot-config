#!/usr/bin/env python3
"""
Converts Livox custom pointcloud to standard sensor_msgs/PointCloud2 for RViz.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from custom_msg.msg import LivoxPointcloud
import struct

class LivoxToPointCloud2(Node):
    def __init__(self):
        super().__init__('livox_to_pointcloud2')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub = self.create_subscription(
            LivoxPointcloud,
            '/livox_Lidar_node/sn153/xyz/pointcloud',
            self.callback,
            qos
        )

        self.pub = self.create_publisher(
            PointCloud2,
            '/livox/pointcloud',
            10
        )

        self.frame_count = 0
        self.get_logger().info('Livox to PointCloud2 converter started')

    def callback(self, msg):
        pc2 = PointCloud2()
        pc2.header = msg.header
        if not pc2.header.frame_id:
            pc2.header.frame_id = 'livox_frame'

        # Define fields: x, y, z, intensity
        pc2.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        pc2.height = 1
        pc2.width = len(msg.points)
        pc2.is_dense = True
        pc2.is_bigendian = False
        pc2.point_step = 16  # 4 floats * 4 bytes
        pc2.row_step = pc2.point_step * pc2.width

        # Pack point data
        data = []
        for pt in msg.points:
            data.append(struct.pack('ffff', pt.x, pt.y, pt.z, float(pt.reflectivity)))

        pc2.data = b''.join(data)
        self.pub.publish(pc2)

        self.frame_count += 1
        if self.frame_count % 100 == 0:
            self.get_logger().info(f'Converted {self.frame_count} frames, {len(msg.points)} points')


def main():
    rclpy.init()
    node = LivoxToPointCloud2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
