#!/usr/bin/env python3
"""
Object Tracker for Jetson Orin Nano

Detects objects using the RealSense camera and sends detection results
via TCP to the Mini PC for robot control.

Features:
- Color-based detection (yellow, red, green, blue, or custom HSV)
- Direct RealSense access via pyrealsense2 v2.55
- Depth at detection centroid
- TCP server on port 9997

Usage:
    export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
    python3 jetson_object_tracker.py --mode color --target yellow
    python3 jetson_object_tracker.py --mode color --target red
    python3 jetson_object_tracker.py --mode color --target custom --hsv-min 20,100,100 --hsv-max 40,255,255
"""

import socket
import struct
import threading
import time
import argparse
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    print("ERROR: pyrealsense2 not found. Make sure to set:")
    print("  export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH")
    exit(1)

try:
    import cv2
except ImportError:
    print("ERROR: opencv-python not found. Install with: pip3 install opencv-python")
    exit(1)

HOST = '0.0.0.0'
PORT = 9997
IMAGE_PORT = 9996  # Separate port for image streaming

# Detection message format (57 bytes total)
# B: detected (1 byte)
# H: center_x (2 bytes)
# H: center_y (2 bytes)
# H: bbox_x (2 bytes)
# H: bbox_y (2 bytes)
# H: bbox_w (2 bytes)
# H: bbox_h (2 bytes)
# I: distance_mm (4 bytes)
# f: confidence (4 bytes)
# I: timestamp (4 bytes)
# 32s: label (32 bytes)
DETECTION_FORMAT = '!BHHHHHHIfI32s'
DETECTION_SIZE = struct.calcsize(DETECTION_FORMAT)  # Should be 57 bytes

# Image message header format
# I: width (4 bytes)
# I: height (4 bytes)
# I: data_length (4 bytes)
IMAGE_HEADER_FORMAT = '!III'
IMAGE_HEADER_SIZE = struct.calcsize(IMAGE_HEADER_FORMAT)

# Color presets (all using HSV for better detection)
COLOR_PRESETS = {
    'yellow': {
        'method': 'hsv',
        'params': {'h_min': 20, 'h_max': 40, 's_min': 80, 'v_min': 80}
    },
    'orange': {
        'method': 'hsv',
        'params': {'h_min': 10, 'h_max': 25, 's_min': 100, 'v_min': 100}
    },
    'red': {
        'method': 'hsv',
        'params': {'h_min': 0, 'h_max': 10, 's_min': 100, 'v_min': 100}
    },
    'red2': {  # Red wraps around in HSV
        'method': 'hsv',
        'params': {'h_min': 170, 'h_max': 180, 's_min': 100, 'v_min': 100}
    },
    'green': {
        'method': 'hsv',
        'params': {'h_min': 35, 'h_max': 85, 's_min': 80, 'v_min': 80}
    },
    'blue': {
        'method': 'hsv',
        'params': {'h_min': 100, 'h_max': 130, 's_min': 80, 'v_min': 80}
    }
}

# Detection parameters
MIN_CONTOUR_AREA = 500
MAX_CONTOUR_AREA = 80000
ASPECT_RATIO_MIN = 0.2
ASPECT_RATIO_MAX = 5.0
MAX_DETECTION_DISTANCE = 5000  # mm


class ObjectTracker:
    def __init__(self, args):
        self.args = args
        self.running = True
        self.stream_images = args.stream_images

        # TCP server for detections
        self._tcp_clients = []
        self._tcp_lock = threading.Lock()

        # TCP server for images (optional)
        self._image_clients = []
        self._image_lock = threading.Lock()

        # Frame counter
        self.frame_count = 0
        self.detection_count = 0
        self.start_time = time.time()

        # Parse custom HSV if provided
        self.custom_hsv_min = None
        self.custom_hsv_max = None
        if args.target == 'custom':
            if args.hsv_min and args.hsv_max:
                self.custom_hsv_min = tuple(map(int, args.hsv_min.split(',')))
                self.custom_hsv_max = tuple(map(int, args.hsv_max.split(',')))
            else:
                print("ERROR: --hsv-min and --hsv-max required for custom target")
                exit(1)

        print(f"Object Tracker initializing...")
        print(f"  Mode: {args.mode}")
        print(f"  Target: {args.target}")
        if self.custom_hsv_min:
            print(f"  HSV min: {self.custom_hsv_min}")
            print(f"  HSV max: {self.custom_hsv_max}")

        # Initialize RealSense
        self._init_realsense()

        # Start TCP server for detections
        self.server_thread = threading.Thread(target=self._run_tcp_server, daemon=True)
        self.server_thread.start()

        # Start image TCP server if streaming enabled
        if self.stream_images:
            self.image_server_thread = threading.Thread(target=self._run_image_server, daemon=True)
            self.image_server_thread.start()
            print(f"Image streaming enabled on port {IMAGE_PORT}")

        print(f"Object Tracker ready")

    def _init_realsense(self):
        """Initialize RealSense pipeline."""
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        print("Starting RealSense pipeline...")
        self.profile = self.pipeline.start(config)

        # Create align object to align depth to color
        self.align = rs.align(rs.stream.color)

        # Warm up camera
        print("Warming up camera...")
        for _ in range(15):
            self.pipeline.wait_for_frames(timeout_ms=5000)

        print("RealSense pipeline started")

    def _run_tcp_server(self):
        """TCP server thread - accepts client connections for detections."""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen(5)
        print(f"Detection TCP server listening on {HOST}:{PORT}")

        while self.running:
            try:
                client, addr = server.accept()
                print(f"Detection client connected: {addr}")
                with self._tcp_lock:
                    self._tcp_clients.append(client)
            except Exception as e:
                if self.running:
                    print(f"TCP accept error: {e}")

    def _run_image_server(self):
        """TCP server thread for image streaming."""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, IMAGE_PORT))
        server.listen(5)
        print(f"Image TCP server listening on {HOST}:{IMAGE_PORT}")

        while self.running:
            try:
                client, addr = server.accept()
                print(f"Image client connected: {addr}")
                with self._image_lock:
                    self._image_clients.append(client)
            except Exception as e:
                if self.running:
                    print(f"Image TCP accept error: {e}")

    def _send_detection(self, detection):
        """Send detection to all TCP clients."""
        with self._tcp_lock:
            if not self._tcp_clients:
                return

            # Pack detection data
            label_bytes = detection['label'].encode('utf-8')[:32].ljust(32, b'\x00')

            data = struct.pack(
                DETECTION_FORMAT,
                1 if detection['detected'] else 0,
                detection['center_x'],
                detection['center_y'],
                detection['bbox_x'],
                detection['bbox_y'],
                detection['bbox_w'],
                detection['bbox_h'],
                detection['distance_mm'],
                detection['confidence'],
                detection['timestamp'],
                label_bytes
            )

            dead_clients = []
            for client in self._tcp_clients:
                try:
                    client.sendall(data)
                except Exception as e:
                    print(f"TCP client error: {e}")
                    dead_clients.append(client)

            for c in dead_clients:
                self._tcp_clients.remove(c)

    def _send_image(self, color_image):
        """Send color image to all image TCP clients."""
        if not self.stream_images:
            return

        with self._image_lock:
            if not self._image_clients:
                return

            height, width = color_image.shape[:2]
            image_bytes = color_image.tobytes()

            # Pack header and data
            header = struct.pack(IMAGE_HEADER_FORMAT, width, height, len(image_bytes))

            dead_clients = []
            for client in self._image_clients:
                try:
                    client.sendall(header + image_bytes)
                except Exception as e:
                    print(f"Image client error: {e}")
                    dead_clients.append(client)

            for c in dead_clients:
                self._image_clients.remove(c)

    def _detect_color_bgr(self, image, preset):
        """Detect color using direct BGR thresholds (for yellow)."""
        params = preset['params']

        mask = (
            (image[:, :, 0] > params['ch0_min']) &
            (image[:, :, 1] > params['ch1_min']) &
            (image[:, :, 2] < params['ch2_max'])
        ).astype(np.uint8) * 255

        return mask

    def _detect_color_hsv(self, image, h_min, h_max, s_min, v_min):
        """Detect color using HSV range."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)

        return mask

    def _detect_object(self, color_image, depth_image):
        """Detect object in frame and return detection info."""
        detection = {
            'detected': False,
            'center_x': 0,
            'center_y': 0,
            'bbox_x': 0,
            'bbox_y': 0,
            'bbox_w': 0,
            'bbox_h': 0,
            'distance_mm': 0,
            'confidence': 0.0,
            'timestamp': int(time.time()),
            'label': self.args.target
        }

        # Generate mask based on target
        mask = None

        if self.args.target == 'custom':
            # Custom HSV range
            mask = self._detect_color_hsv(
                color_image,
                self.custom_hsv_min[0], self.custom_hsv_max[0],
                self.custom_hsv_min[1], self.custom_hsv_min[2]
            )
        elif self.args.target == 'yellow':
            # Yellow uses HSV detection
            preset = COLOR_PRESETS['yellow']
            mask = self._detect_color_hsv(
                color_image,
                preset['params']['h_min'], preset['params']['h_max'],
                preset['params']['s_min'], preset['params']['v_min']
            )
        elif self.args.target == 'red':
            # Red wraps around HSV, need two ranges
            preset1 = COLOR_PRESETS['red']
            preset2 = COLOR_PRESETS['red2']
            mask1 = self._detect_color_hsv(
                color_image,
                preset1['params']['h_min'], preset1['params']['h_max'],
                preset1['params']['s_min'], preset1['params']['v_min']
            )
            mask2 = self._detect_color_hsv(
                color_image,
                preset2['params']['h_min'], preset2['params']['h_max'],
                preset2['params']['s_min'], preset2['params']['v_min']
            )
            mask = cv2.bitwise_or(mask1, mask2)
        elif self.args.target in COLOR_PRESETS:
            preset = COLOR_PRESETS[self.args.target]
            if preset['method'] == 'hsv':
                mask = self._detect_color_hsv(
                    color_image,
                    preset['params']['h_min'], preset['params']['h_max'],
                    preset['params']['s_min'], preset['params']['v_min']
                )
            else:
                mask = self._detect_color_bgr(color_image, preset)
        else:
            print(f"Unknown target: {self.args.target}")
            return detection

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return detection

        # Find best contour (largest that meets criteria)
        best_contour = None
        best_area = 0

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < MIN_CONTOUR_AREA or area > MAX_CONTOUR_AREA:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            if h == 0:
                continue

            aspect = w / h
            if aspect < ASPECT_RATIO_MIN or aspect > ASPECT_RATIO_MAX:
                continue

            # Check depth at center
            cx = x + w // 2
            cy = y + h // 2

            if depth_image is not None:
                depth_val = depth_image[cy, cx]
                if depth_val > 0 and depth_val < MAX_DETECTION_DISTANCE:
                    if area > best_area:
                        best_area = area
                        best_contour = (contour, x, y, w, h, cx, cy, depth_val)
                elif depth_val == 0:
                    # No depth data, still track if area is large enough
                    if area > best_area:
                        best_area = area
                        best_contour = (contour, x, y, w, h, cx, cy, 0)
            else:
                if area > best_area:
                    best_area = area
                    best_contour = (contour, x, y, w, h, cx, cy, 0)

        if best_contour:
            _, x, y, w, h, cx, cy, depth_val = best_contour

            detection['detected'] = True
            detection['center_x'] = cx
            detection['center_y'] = cy
            detection['bbox_x'] = x
            detection['bbox_y'] = y
            detection['bbox_w'] = w
            detection['bbox_h'] = h
            detection['distance_mm'] = int(depth_val)
            detection['confidence'] = min(1.0, best_area / MAX_CONTOUR_AREA)

            self.detection_count += 1

        return detection

    def run(self):
        """Main processing loop."""
        print("Starting object detection loop...")
        print("Press Ctrl+C to stop")

        try:
            while self.running:
                # Get frames
                frames = self.pipeline.wait_for_frames(timeout_ms=5000)

                # Align depth to color
                aligned_frames = self.align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()

                if not color_frame:
                    continue

                # Convert to numpy
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data()) if depth_frame else None

                # Detect object
                detection = self._detect_object(color_image, depth_image)

                # Send detection to clients
                self._send_detection(detection)

                # Send image to clients if streaming enabled
                if self.stream_images:
                    self._send_image(color_image)

                self.frame_count += 1

                # Log status periodically
                if self.frame_count % 100 == 0:
                    elapsed = time.time() - self.start_time
                    fps = self.frame_count / elapsed
                    with self._tcp_lock:
                        num_clients = len(self._tcp_clients)
                    print(f"Frames: {self.frame_count}, Detections: {self.detection_count}, "
                          f"FPS: {fps:.1f}, Clients: {num_clients}")

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.stop()

    def stop(self):
        """Clean shutdown."""
        self.running = False
        print("Stopping RealSense pipeline...")
        self.pipeline.stop()
        print("Object Tracker stopped")


def parse_args():
    parser = argparse.ArgumentParser(
        description='Object tracker for Jetson Orin Nano',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('--mode', choices=['color', 'yolo'], default='color',
                        help='Detection mode (yolo not yet implemented)')
    parser.add_argument('--target', default='yellow',
                        help='Target to detect: yellow, red, green, blue, or custom')
    parser.add_argument('--hsv-min', type=str, default=None,
                        help='HSV minimum for custom target (H,S,V)')
    parser.add_argument('--hsv-max', type=str, default=None,
                        help='HSV maximum for custom target (H,S,V)')
    parser.add_argument('--port', type=int, default=PORT,
                        help='TCP server port for detections')
    parser.add_argument('--stream-images', action='store_true',
                        help='Also stream color images via TCP (port 9996)')
    return parser.parse_args()


def main():
    args = parse_args()

    global PORT
    PORT = args.port

    tracker = ObjectTracker(args)
    tracker.run()


if __name__ == '__main__':
    main()
