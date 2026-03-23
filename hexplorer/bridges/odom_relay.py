#!/usr/bin/env python3
"""
Odometry Relay Node - Remaps Fast-LIO2 output to Hexplorer conventions.

Fast-LIO2 publishes:  /Odometry  (frames: camera_init -> body)
Downstream expects:   /lidar_odometry/pose  (frames: odom -> base_link)

Also broadcasts TF: odom -> base_link
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomRelay(Node):
    def __init__(self):
        super().__init__('odom_relay')

        self.pub = self.create_publisher(Odometry, '/lidar_odometry/pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10
        )

        self.msg_count = 0
        self.get_logger().info('Odom relay: /Odometry -> /lidar_odometry/pose')

    def odom_callback(self, msg):
        # Remap frame IDs
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # Republish
        self.pub.publish(msg)

        # Broadcast TF
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        self.msg_count += 1
        if self.msg_count % 100 == 0:
            self.get_logger().info(f'Relayed {self.msg_count} odom messages')


def main():
    rclpy.init()
    node = OdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
