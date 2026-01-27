#!/usr/bin/env python3
"""
ROS2 Camera Viewer - Displays images from /camera/camera/color/image_raw
Uses CycloneDDS for reliable cross-machine image transfer
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
import cv2
import numpy as np

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            qos
        )
        self.get_logger().info('Camera viewer started. Press Q to quit.')
        self.frame_count = 0

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = np_arr.reshape((msg.height, msg.width, 3))

            self.frame_count += 1
            cv2.putText(frame, f'Frame: {self.frame_count}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow('RealSense Camera (CycloneDDS)', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('Quit requested')
                raise SystemExit()

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main():
    rclpy.init()
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
