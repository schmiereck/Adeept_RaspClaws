#!/usr/bin/env python3
# File name   : FPV_ROS2.py
# Description : ROS 2 Camera Stream via ZMQ Bridge
# Author      : GitHub Copilot + Claude
# Date        : 2026-02-05
# Modified    : ZMQ Bridge instead of direct Picamera2 (for RoboStack Python 3.11)

"""
ROS 2 Camera Publisher for Adeept RaspClaws Robot

Receives camera frames from GUIServer's ZMQ stream (Port 5555)
and publishes them to ROS 2 topics.

This approach allows:
- GUIServer runs in system-Python 3.13 with Picamera2
- ROS2 Node runs in RoboStack ros_env with Python 3.11
- Both can access camera data simultaneously

Topics Published:
- /raspclaws/camera/image_raw (sensor_msgs/Image): Uncompressed RGB image
- /raspclaws/camera/image_compressed (sensor_msgs/CompressedImage): JPEG compressed image
- /raspclaws/camera/camera_info (sensor_msgs/CameraInfo): Camera calibration info
"""

import threading
import time
import cv2
import numpy as np
import zmq

# ROS 2 imports
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CompressedImage, CameraInfo
    from std_msgs.msg import Header
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: ROS 2 not available: {e}")
    ROS2_AVAILABLE = False


# ==================== Configuration ====================

class CameraConfig:
    """
    Camera configuration constants.
    Modify these to adjust camera behavior.
    """

    # ZMQ Connection
    ZMQ_HOST = '127.0.0.1'  # GUIServer runs on same machine
    ZMQ_PORT = 5555         # GUIServer's FPV ZMQ port
    ZMQ_TIMEOUT = 5000      # 5 seconds timeout

    # Expected resolution from GUIServer
    WIDTH = 640
    HEIGHT = 480

    # Frame rate (FPS) - target for publishing
    # Note: Actual FPS depends on GUIServer's camera feed
    TARGET_FPS = 30

    # JPEG Compression quality (0-100)
    # Higher = better quality, larger files
    # Lower = worse quality, smaller files
    JPEG_QUALITY = 85

    # Publish modes
    PUBLISH_RAW = True          # Publish uncompressed image (larger bandwidth)
    PUBLISH_COMPRESSED = True   # Publish JPEG compressed image (smaller bandwidth)
    PUBLISH_CAMERA_INFO = True  # Publish camera calibration info

    # Topic names
    TOPIC_RAW = '/raspclaws/camera/image_raw'
    TOPIC_COMPRESSED = '/raspclaws/camera/image_compressed'
    TOPIC_CAMERA_INFO = '/raspclaws/camera/camera_info'

    # Frame ID (for TF transformations)
    FRAME_ID = 'camera_link'

    # Camera info (basic - replace with real calibration if available)
    # K = camera matrix (fx, fy, cx, cy)
    # D = distortion coefficients (k1, k2, t1, t2, k3)
    CAMERA_MATRIX = [
        WIDTH * 1.0, 0.0, WIDTH / 2.0,
        0.0, HEIGHT * 1.0, HEIGHT / 2.0,
        0.0, 0.0, 1.0
    ]
    DISTORTION_COEFFS = [0.0, 0.0, 0.0, 0.0, 0.0]


# ==================== ROS 2 Camera Publisher ====================

class CameraPublisher(Node):
    """ROS 2 node that publishes camera frames from ZMQ stream"""

    def __init__(self):
        super().__init__('camera_publisher')

        self.get_logger().info('Initializing Camera Publisher (ZMQ Bridge)...')

        # Configuration
        self.config = CameraConfig()

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Camera state
        self.camera_paused = False
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps = 0.0
        self.zmq_connected = False

        # Create publishers
        self.create_publishers()

        # Initialize ZMQ connection
        self.init_zmq()

        # Start capture thread
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()

        self.get_logger().info('✓ Camera Publisher (ZMQ Bridge) initialized successfully')

    def create_publishers(self):
        """Create ROS 2 publishers for camera topics"""
        if self.config.PUBLISH_RAW:
            self.image_raw_pub = self.create_publisher(
                Image,
                self.config.TOPIC_RAW,
                10
            )
            self.get_logger().info(f'✓ Publisher created: {self.config.TOPIC_RAW}')

        if self.config.PUBLISH_COMPRESSED:
            self.image_compressed_pub = self.create_publisher(
                CompressedImage,
                self.config.TOPIC_COMPRESSED,
                10
            )
            self.get_logger().info(f'✓ Publisher created: {self.config.TOPIC_COMPRESSED}')

        if self.config.PUBLISH_CAMERA_INFO:
            self.camera_info_pub = self.create_publisher(
                CameraInfo,
                self.config.TOPIC_CAMERA_INFO,
                10
            )
            self.get_logger().info(f'✓ Publisher created: {self.config.TOPIC_CAMERA_INFO}')

    def init_zmq(self):
        """Initialize ZMQ subscriber to GUIServer's camera stream"""
        try:
            self.get_logger().info('Connecting to ZMQ camera stream...')

            # Create ZMQ context and socket
            self.zmq_context = zmq.Context()
            self.zmq_socket = self.zmq_context.socket(zmq.SUB)

            # Connect to GUIServer's video stream
            zmq_address = f'tcp://{self.config.ZMQ_HOST}:{self.config.ZMQ_PORT}'
            self.zmq_socket.connect(zmq_address)

            # Subscribe to all messages (empty filter)
            self.zmq_socket.setsockopt_string(zmq.SUBSCRIBE, '')

            # Set timeout
            self.zmq_socket.setsockopt(zmq.RCVTIMEO, self.config.ZMQ_TIMEOUT)

            self.get_logger().info(f'✓ Connected to ZMQ stream at {zmq_address}')
            self.get_logger().info(f'   Waiting for frames from GUIServer...')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize ZMQ connection: {e}')
            self.get_logger().error('Make sure GUIServer is running with camera enabled')
            raise

    def pause(self):
        """Pause camera streaming (saves CPU/power)"""
        self.camera_paused = True
        self.get_logger().info('📷 Camera stream PAUSED')

    def resume(self):
        """Resume camera streaming"""
        self.camera_paused = False
        self.get_logger().info('📷 Camera stream RESUMED')

    def is_paused(self):
        """Check if camera is paused"""
        return self.camera_paused

    def capture_loop(self):
        """
        Main capture loop (runs in separate thread).
        Receives frames from ZMQ and publishes to ROS 2 topics.
        """
        self.get_logger().info('Starting ZMQ capture loop...')

        frame_delay = 1.0 / self.config.TARGET_FPS

        while rclpy.ok():
            try:
                loop_start = time.time()

                # Receive frame from ZMQ
                try:
                    message = self.zmq_socket.recv()

                    # Decode JPEG image
                    frame = cv2.imdecode(
                        np.frombuffer(message, dtype=np.uint8),
                        cv2.IMREAD_COLOR
                    )

                    if frame is None:
                        self.get_logger().warn('Failed to decode frame from ZMQ')
                        continue

                    # Convert BGR to RGB (OpenCV uses BGR, ROS uses RGB)
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                    # Mark as connected on first successful frame
                    if not self.zmq_connected:
                        self.zmq_connected = True
                        self.get_logger().info('✓ Receiving frames from GUIServer')

                except zmq.Again:
                    # Timeout - no frame received
                    if self.zmq_connected:
                        self.get_logger().warn('ZMQ timeout - no frames from GUIServer')
                        self.zmq_connected = False
                    time.sleep(0.5)
                    continue

                # Skip publishing if paused
                if self.camera_paused:
                    time.sleep(frame_delay)
                    continue

                # Create timestamp
                timestamp = self.get_clock().now().to_msg()

                # Publish raw image
                if self.config.PUBLISH_RAW:
                    self.publish_raw_image(frame, timestamp)

                # Publish compressed image
                if self.config.PUBLISH_COMPRESSED:
                    self.publish_compressed_image(frame, timestamp)

                # Publish camera info
                if self.config.PUBLISH_CAMERA_INFO:
                    self.publish_camera_info(timestamp)

                # Update FPS counter
                self.update_fps()

                # Sleep to maintain target FPS
                elapsed = time.time() - loop_start
                sleep_time = max(0.0, frame_delay - elapsed)
                time.sleep(sleep_time)

            except Exception as e:
                self.get_logger().error(f'Error in capture loop: {e}')
                time.sleep(1.0)  # Avoid tight error loop

    def publish_raw_image(self, frame, timestamp):
        """Publish uncompressed image"""
        try:
            # Convert numpy array to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
            msg.header.stamp = timestamp
            msg.header.frame_id = self.config.FRAME_ID

            self.image_raw_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish raw image: {e}')

    def publish_compressed_image(self, frame, timestamp):
        """Publish JPEG compressed image"""
        try:
            # Convert RGB to BGR (cv2.imencode expects BGR)
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # Encode as JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.config.JPEG_QUALITY]
            _, buffer = cv2.imencode('.jpg', frame_bgr, encode_param)

            # Create CompressedImage message
            msg = CompressedImage()
            msg.header.stamp = timestamp
            msg.header.frame_id = self.config.FRAME_ID
            msg.format = 'jpeg'
            msg.data = buffer.tobytes()

            self.image_compressed_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish compressed image: {e}')

    def publish_camera_info(self, timestamp):
        """Publish camera calibration info"""
        try:
            msg = CameraInfo()
            msg.header.stamp = timestamp
            msg.header.frame_id = self.config.FRAME_ID

            msg.width = self.config.WIDTH
            msg.height = self.config.HEIGHT

            # Camera matrix (K)
            msg.k = self.config.CAMERA_MATRIX

            # Distortion coefficients (D)
            msg.d = self.config.DISTORTION_COEFFS

            # Distortion model
            msg.distortion_model = 'plumb_bob'

            self.camera_info_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish camera info: {e}')

    def update_fps(self):
        """Update FPS counter"""
        self.frame_count += 1

        current_time = time.time()
        elapsed = current_time - self.last_fps_time

        if elapsed >= 1.0:  # Update every second
            self.fps = self.frame_count / elapsed
            self.get_logger().debug(f'Camera FPS: {self.fps:.1f}')

            self.frame_count = 0
            self.last_fps_time = current_time

    def get_fps(self):
        """Get current FPS"""
        return self.fps

    def shutdown(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down camera publisher...')
        try:
            if hasattr(self, 'zmq_socket'):
                self.zmq_socket.close()
            if hasattr(self, 'zmq_context'):
                self.zmq_context.term()
            self.get_logger().info('✓ ZMQ connection closed')
        except Exception as e:
            self.get_logger().error(f'Error during ZMQ shutdown: {e}')


# ==================== Main Entry Point ====================

def main(args=None):
    """Main entry point"""

    if not ROS2_AVAILABLE:
        print("ERROR: ROS 2 is not available. Please install ROS 2 Humble.")
        return 1

    # Initialize ROS 2
    rclpy.init(args=args)

    node = None
    try:
        node = CameraPublisher()

        # Spin (process callbacks)
        rclpy.spin(node)

    except KeyboardInterrupt:
        print('Keyboard interrupt received')
    except Exception as e:
        print(f"FATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        # Cleanup
        if node is not None:
            try:
                node.shutdown()
                node.destroy_node()
            except Exception as e:
                print(f"Warning: Error during cleanup: {e}")

        # Shutdown ROS 2
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Warning: Error during ROS 2 shutdown: {e}")

    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
