#!/usr/bin/env/python3
# File name   : FPV_ROS2_simple.py
# Description : ROS 2 Camera wrapper for existing FPV.py
# Author      : GitHub Copilot
# Date        : 2026-01-29

"""
Simple ROS2 Camera Publisher - wraps existing FPV.py
No picamera2 import needed - uses existing capture from FPV.py

Topics Published:
- /raspclaws/camera/image_compressed (sensor_msgs/CompressedImage): JPEG compressed image
- /raspclaws/camera/camera_info (sensor_msgs/CameraInfo): Camera calibration info
"""

import threading
import time
import cv2

# ROS 2 imports
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage, CameraInfo
    from std_msgs.msg import Header
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

    # Resolution (from FPV.py configuration)
    WIDTH = 640
    HEIGHT = 480

    # Frame rate (FPS)
    # Higher = smoother video, but more CPU/bandwidth
    # Lower = less smooth, but saves resources
    TARGET_FPS = 30

    # JPEG Compression quality (0-100)
    # Higher = better quality, larger files
    # Lower = worse quality, smaller files
    JPEG_QUALITY = 85

    # Publish modes
    PUBLISH_COMPRESSED = True   # Publish JPEG compressed image (smaller bandwidth)
    PUBLISH_CAMERA_INFO = True  # Publish camera calibration info

    # Topic names
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

# Global variable for sharing frames between FPV.py and ROS2
latest_frame = None
frame_lock = threading.Lock()

def set_latest_frame(frame):
    """Called from FPV.py to share the frame with ROS2"""
    global latest_frame
    with frame_lock:
        latest_frame = frame.copy() if frame is not None else None

def get_latest_frame():
    """Get the latest frame for ROS2 publishing"""
    global latest_frame
    with frame_lock:
        return latest_frame.copy() if latest_frame is not None else None


class SimpleCameraPublisher(Node):
    """
    Simple ROS 2 Camera Publisher
    Uses frames from existing FPV.py (no picamera2 dependency!)
    """

    def __init__(self):
        super().__init__('camera_publisher')

        self.get_logger().info('Initializing Simple Camera Publisher...')

        # Configuration
        self.config = CameraConfig()
        self.camera_paused = False
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps = 0.0

        # Create publishers
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

        self.get_logger().info(f'✓ Configuration: {self.config.WIDTH}x{self.config.HEIGHT} @ {self.config.TARGET_FPS} FPS, JPEG Quality: {self.config.JPEG_QUALITY}')

        # Start publish thread
        self.publish_thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.publish_thread.start()

        self.get_logger().info('✓ Simple Camera Publisher initialized successfully')

    def pause(self):
        """Pause camera streaming"""
        self.camera_paused = True
        self.get_logger().info('📷 Camera stream PAUSED')

    def resume(self):
        """Resume camera streaming"""
        self.camera_paused = False
        self.get_logger().info('📷 Camera stream RESUMED')

    def publish_loop(self):
        """
        Main publish loop (runs in separate thread)
        Gets frames from FPV.py and publishes to ROS2
        """
        self.get_logger().info('Starting publish loop...')

        frame_delay = 1.0 / self.config.TARGET_FPS

        while rclpy.ok():
            try:
                loop_start = time.time()

                # Skip if paused
                if self.camera_paused:
                    time.sleep(frame_delay)
                    continue

                # Get frame from FPV.py
                frame = get_latest_frame()

                if frame is None:
                    # No frame yet, wait a bit
                    time.sleep(0.1)
                    continue

                # Create timestamp
                timestamp = self.get_clock().now().to_msg()

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
                self.get_logger().error(f'Error in publish loop: {e}')
                time.sleep(1.0)

    def publish_compressed_image(self, frame, timestamp):
        """Publish JPEG compressed image"""
        try:
            # frame from FPV.py is RGB, cv2.imencode expects BGR
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


# ==================== Integration with FPV.py ====================

def integrate_with_fpv():
    """
    This function should be called FROM FPV.py capture_thread
    to share frames with ROS2

    Add this to FPV.py capture_thread, after frame capture:

        frame_image = picam2.capture_array()

        # Share frame with ROS2 (if available)
        try:
            from FPV_ROS2_simple import set_latest_frame
            set_latest_frame(frame_image)
        except:
            pass  # ROS2 not available, ignore

        # ... rest of FPV.py code ...
    """
    pass


# ==================== Main Entry Point ====================

def main(args=None):
    """Main entry point"""

    if not ROS2_AVAILABLE:
        print("ERROR: ROS 2 is not available")
        return 1

    # Initialize ROS 2
    rclpy.init(args=args)

    node = None
    try:
        node = SimpleCameraPublisher()

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
