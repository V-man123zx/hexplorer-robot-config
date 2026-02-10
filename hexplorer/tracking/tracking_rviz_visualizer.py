#!/usr/bin/env python3
"""
RViz Visualization for Object Tracking

Publishes detection results as RViz markers and overlaid images.
Also displays smart_follower state when available.

Topics published:
- /object_tracking/marker (visualization_msgs/Marker) - 3D marker at detected object
- /object_tracking/image (sensor_msgs/Image) - Camera image with detection overlay
- /object_tracking/text (visualization_msgs/Marker) - Text label with distance
- /object_tracking/state_text (visualization_msgs/Marker) - Robot state display

Subscribes to:
- /object_detection (std_msgs/String) - Detection data from receiver
- /camera/color/image_raw (sensor_msgs/Image) - Camera image (optional)
- /smart_follower/state (std_msgs/String) - Robot state from smart follower

Usage:
    source /opt/ros/humble/setup.bash
    python3 tracking_rviz_visualizer.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, ColorRGBA
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import json
import numpy as np
import time

# Camera parameters (approximate for RealSense D435)
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
IMAGE_CENTER_X = IMAGE_WIDTH // 2
IMAGE_CENTER_Y = IMAGE_HEIGHT // 2
FOV_H = 69.0  # Horizontal FOV in degrees
FOV_V = 42.0  # Vertical FOV in degrees


class TrackingRVizVisualizer(Node):
    def __init__(self):
        super().__init__('tracking_rviz_visualizer')

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.marker_pub = self.create_publisher(Marker, '/object_tracking/marker', 10)
        self.text_pub = self.create_publisher(Marker, '/object_tracking/text', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, '/object_tracking/markers', 10)
        self.image_pub = self.create_publisher(Image, '/object_tracking/image', 10)
        self.state_text_pub = self.create_publisher(Marker, '/object_tracking/state_text', 10)

        # Subscribers
        self.create_subscription(
            String, '/object_detection', self.detection_callback, 10)

        # Subscribe to smart follower state
        self.create_subscription(
            String, '/smart_follower/state', self.state_callback, 10)

        # Subscribe to camera images (multiple possible topics)
        self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, sensor_qos)
        self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, sensor_qos)

        # State
        self.last_detection = None
        self.last_detection_time = 0
        self.last_image = None
        self.last_image_time = 0
        self.last_robot_state = None
        self.last_robot_state_time = 0

        # Timer for publishing markers even without new detections
        self.timer = self.create_timer(0.1, self.publish_visualization)

        self.get_logger().info('RViz Tracking Visualizer started')
        self.get_logger().info('Waiting for /object_detection topic...')
        self.get_logger().info('Add these displays in RViz:')
        self.get_logger().info('  - Marker: /object_tracking/marker')
        self.get_logger().info('  - Marker: /object_tracking/text')
        self.get_logger().info('  - Image: /object_tracking/image')

    def detection_callback(self, msg):
        """Process detection message."""
        try:
            self.last_detection = json.loads(msg.data)
            self.last_detection_time = time.time()
        except json.JSONDecodeError:
            pass

    def state_callback(self, msg):
        """Process smart follower state message."""
        try:
            self.last_robot_state = json.loads(msg.data)
            self.last_robot_state_time = time.time()
        except json.JSONDecodeError:
            pass

    def image_callback(self, msg):
        """Store latest camera image."""
        self.last_image = msg
        self.last_image_time = time.time()

    def pixel_to_3d(self, cx, cy, distance_mm):
        """Convert pixel coordinates and depth to 3D position in camera frame."""
        if distance_mm <= 0:
            distance_mm = 1000  # Default 1m if no depth

        distance_m = distance_mm / 1000.0

        # Calculate angles from center
        angle_h = (cx - IMAGE_CENTER_X) / IMAGE_WIDTH * np.radians(FOV_H)
        angle_v = (cy - IMAGE_CENTER_Y) / IMAGE_HEIGHT * np.radians(FOV_V)

        # Convert to 3D (camera optical frame: Z forward, X right, Y down)
        x = distance_m * np.tan(angle_h)
        y = distance_m * np.tan(angle_v)
        z = distance_m

        return x, y, z

    def publish_visualization(self):
        """Publish visualization markers and overlaid image."""
        now = time.time()
        detection_fresh = (now - self.last_detection_time) < 0.5
        image_fresh = (now - self.last_image_time) < 1.0

        stamp = self.get_clock().now().to_msg()

        if detection_fresh and self.last_detection:
            d = self.last_detection

            if d['detected']:
                # Get 3D position
                x, y, z = self.pixel_to_3d(
                    d['center_x'], d['center_y'], d['distance_mm']
                )

                # Publish sphere marker at detection location
                marker = Marker()
                marker.header.stamp = stamp
                marker.header.frame_id = 'camera_color_optical_frame'
                marker.ns = 'object_detection'
                marker.id = 0
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = z
                marker.pose.orientation.w = 1.0

                # Scale based on bounding box size
                scale = max(d['bbox_w'], d['bbox_h']) / 200.0
                scale = max(0.1, min(0.5, scale))
                marker.scale.x = scale
                marker.scale.y = scale
                marker.scale.z = scale

                # Color based on target
                marker.color = self._get_target_color(d['label'])
                marker.color.a = 0.8
                marker.lifetime.sec = 0
                marker.lifetime.nanosec = 500000000  # 0.5 sec

                self.marker_pub.publish(marker)

                # Publish text marker with info
                text_marker = Marker()
                text_marker.header.stamp = stamp
                text_marker.header.frame_id = 'camera_color_optical_frame'
                text_marker.ns = 'object_detection_text'
                text_marker.id = 1
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = x
                text_marker.pose.position.y = y - 0.15
                text_marker.pose.position.z = z
                text_marker.scale.z = 0.1  # Text height
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                text_marker.text = f'{d["label"]}: {d["distance_mm"]}mm'
                text_marker.lifetime.sec = 0
                text_marker.lifetime.nanosec = 500000000

                self.text_pub.publish(text_marker)

                # Publish bounding box as line strip
                bbox_marker = Marker()
                bbox_marker.header.stamp = stamp
                bbox_marker.header.frame_id = 'camera_color_optical_frame'
                bbox_marker.ns = 'object_bbox'
                bbox_marker.id = 2
                bbox_marker.type = Marker.LINE_STRIP
                bbox_marker.action = Marker.ADD
                bbox_marker.scale.x = 0.01  # Line width

                # Convert bbox corners to 3D
                bx, by, bw, bh = d['bbox_x'], d['bbox_y'], d['bbox_w'], d['bbox_h']
                dist = d['distance_mm']
                corners = [
                    (bx, by),
                    (bx + bw, by),
                    (bx + bw, by + bh),
                    (bx, by + bh),
                    (bx, by)  # Close the box
                ]

                for cx, cy in corners:
                    px, py, pz = self.pixel_to_3d(cx, cy, dist)
                    p = Point()
                    p.x = px
                    p.y = py
                    p.z = pz
                    bbox_marker.points.append(p)

                bbox_marker.color = self._get_target_color(d['label'])
                bbox_marker.color.a = 1.0
                bbox_marker.lifetime.sec = 0
                bbox_marker.lifetime.nanosec = 500000000

                # Publish as marker array
                marker_array = MarkerArray()
                marker_array.markers = [marker, text_marker, bbox_marker]
                self.marker_array_pub.publish(marker_array)

            else:
                # No detection - publish delete markers
                self._publish_delete_markers(stamp)
        else:
            # Detection stale - delete markers
            self._publish_delete_markers(stamp)

        # Publish robot state text if available
        robot_state_fresh = (now - self.last_robot_state_time) < 1.0
        if robot_state_fresh and self.last_robot_state:
            self._publish_state_marker(stamp)

        # Publish overlaid image if we have both detection and image
        if image_fresh and self.last_image is not None:
            self._publish_overlaid_image(detection_fresh, robot_state_fresh)

    def _get_target_color(self, label):
        """Get RViz color for target label."""
        color = ColorRGBA()
        if label == 'yellow':
            color.r = 1.0
            color.g = 1.0
            color.b = 0.0
        elif label == 'red':
            color.r = 1.0
            color.g = 0.0
            color.b = 0.0
        elif label == 'green':
            color.r = 0.0
            color.g = 1.0
            color.b = 0.0
        elif label == 'blue':
            color.r = 0.0
            color.g = 0.0
            color.b = 1.0
        else:
            color.r = 1.0
            color.g = 0.5
            color.b = 0.0
        color.a = 1.0
        return color

    def _publish_state_marker(self, stamp):
        """Publish robot state as text marker in RViz."""
        s = self.last_robot_state

        # Build state text
        state = s.get('state', '?')
        status = s.get('status', '')

        # Color based on state
        if state == 'FOLLOWING':
            r, g, b = 0.0, 1.0, 0.0  # Green
        elif state == 'SEARCH':
            r, g, b = 1.0, 1.0, 0.0  # Yellow
            sub_state = s.get('search_sub_state', '')
            if sub_state:
                state = f'{state}/{sub_state}'
        elif state == 'EVADE':
            r, g, b = 1.0, 0.5, 0.0  # Orange
        elif state == 'BLOCKED':
            r, g, b = 1.0, 0.0, 0.0  # Red
        elif state == 'IDLE':
            r, g, b = 0.5, 0.5, 0.5  # Gray
        else:
            r, g, b = 1.0, 1.0, 1.0  # White

        # Build display text
        lidar = s.get('lidar', {})
        vel = s.get('velocity', {})
        lines = [
            f'STATE: {state}',
            f'{status}',
            f'LiDAR: F={lidar.get("front", "?"):.1f} B={lidar.get("back", "?"):.1f}',
            f'       L={lidar.get("left", "?"):.1f} R={lidar.get("right", "?"):.1f}',
            f'Vel: {vel.get("linear", 0):.2f} m/s, {vel.get("angular", 0):.2f} rad/s',
        ]
        if 'visited_cells' in s:
            lines.append(f'Visited: {s["visited_cells"]} cells')

        text = '\n'.join(lines)

        # Create marker - position in top-left of view
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = 'camera_color_optical_frame'
        marker.ns = 'robot_state'
        marker.id = 10
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = -0.3
        marker.pose.position.y = -0.4
        marker.pose.position.z = 1.0
        marker.scale.z = 0.08  # Text height
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0
        marker.text = text
        marker.lifetime.sec = 1
        marker.lifetime.nanosec = 0

        self.state_text_pub.publish(marker)

    def _publish_delete_markers(self, stamp):
        """Publish markers with DELETE action."""
        for ns, id in [('object_detection', 0), ('object_detection_text', 1), ('object_bbox', 2)]:
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = 'camera_color_optical_frame'
            marker.ns = ns
            marker.id = id
            marker.action = Marker.DELETE
            if ns == 'object_detection':
                self.marker_pub.publish(marker)
            elif ns == 'object_detection_text':
                self.text_pub.publish(marker)

    def _publish_overlaid_image(self, detection_fresh, robot_state_fresh=False):
        """Publish camera image with detection and state overlay."""
        if self.last_image is None:
            return

        # Convert image to numpy array
        img_msg = self.last_image
        if img_msg.encoding == 'bgr8':
            img = np.frombuffer(bytes(img_msg.data), dtype=np.uint8).reshape(
                (img_msg.height, img_msg.width, 3)).copy()
        elif img_msg.encoding == 'rgb8':
            img = np.frombuffer(bytes(img_msg.data), dtype=np.uint8).reshape(
                (img_msg.height, img_msg.width, 3)).copy()
            # Convert RGB to BGR for OpenCV
            img = img[:, :, ::-1]
        else:
            # Unsupported encoding, just forward the image
            self.image_pub.publish(img_msg)
            return

        # Draw robot state in top-left corner
        if robot_state_fresh and self.last_robot_state:
            s = self.last_robot_state
            state = s.get('state', '?')
            status = s.get('status', '')
            sub_state = s.get('search_sub_state', '')

            # Color based on state (BGR)
            if state == 'FOLLOWING':
                state_color = (0, 255, 0)  # Green
            elif state == 'SEARCH':
                state_color = (0, 255, 255)  # Yellow
                if sub_state:
                    state = f'{state}/{sub_state}'
            elif state == 'EVADE':
                state_color = (0, 165, 255)  # Orange
            elif state == 'BLOCKED':
                state_color = (0, 0, 255)  # Red
            elif state == 'IDLE':
                state_color = (128, 128, 128)  # Gray
            else:
                state_color = (255, 255, 255)  # White

            # Draw state banner at top
            img[0:30, 0:img_msg.width] = (40, 40, 40)  # Dark background
            # Draw colored state indicator bar
            img[0:30, 0:8] = state_color

            # Draw LiDAR bars at bottom
            lidar = s.get('lidar', {})
            bar_height = 15
            bar_y = img_msg.height - bar_height
            img[bar_y:img_msg.height, 0:img_msg.width] = (40, 40, 40)

            # Show LiDAR distances as colored bars
            # Front (center top section)
            front_dist = lidar.get('front', 10)
            front_width = min(200, int(front_dist * 50))
            front_color = (0, 255, 0) if front_dist > 1.2 else ((0, 255, 255) if front_dist > 0.8 else (0, 0, 255))
            cx = img_msg.width // 2
            img[bar_y:img_msg.height, cx-front_width//2:cx+front_width//2] = front_color

        # Draw detection overlay if fresh
        if detection_fresh and self.last_detection and self.last_detection['detected']:
            d = self.last_detection

            # Get color for drawing (BGR)
            label = d['label']
            if label == 'yellow':
                color = (0, 255, 255)
            elif label == 'red':
                color = (0, 0, 255)
            elif label == 'green':
                color = (0, 255, 0)
            elif label == 'blue':
                color = (255, 0, 0)
            else:
                color = (0, 165, 255)  # Orange

            # Draw bounding box
            x, y, w, h = d['bbox_x'], d['bbox_y'], d['bbox_w'], d['bbox_h']
            # Top line
            img[max(0, y):min(img_msg.height, y+3), max(0, x):min(img_msg.width, x+w)] = color
            # Bottom line
            img[max(0, y+h-3):min(img_msg.height, y+h), max(0, x):min(img_msg.width, x+w)] = color
            # Left line
            img[max(0, y):min(img_msg.height, y+h), max(0, x):min(img_msg.width, x+3)] = color
            # Right line
            img[max(0, y):min(img_msg.height, y+h), max(0, x+w-3):min(img_msg.width, x+w)] = color

            # Draw center crosshair
            cx, cy = d['center_x'], d['center_y']
            # Horizontal line
            img[max(0, cy-1):min(img_msg.height, cy+2), max(0, cx-10):min(img_msg.width, cx+10)] = color
            # Vertical line
            img[max(0, cy-10):min(img_msg.height, cy+10), max(0, cx-1):min(img_msg.width, cx+2)] = color

            # Draw text background and text (simple rectangle for label)
            text = f'{label}: {d["distance_mm"]}mm'
            text_x = max(0, x)
            text_y = max(30, y - 25)  # Avoid state banner
            # Black background
            img[text_y:text_y+20, text_x:min(text_x+len(text)*8, img_msg.width)] = (0, 0, 0)
            # Note: For proper text, would need cv2.putText, but keeping dependencies minimal

        # Create output message
        out_msg = Image()
        out_msg.header = img_msg.header
        out_msg.height = img_msg.height
        out_msg.width = img_msg.width
        out_msg.encoding = 'bgr8'
        out_msg.is_bigendian = False
        out_msg.step = img_msg.width * 3
        out_msg.data = img.tobytes()

        self.image_pub.publish(out_msg)


def main():
    rclpy.init()
    node = TrackingRVizVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
