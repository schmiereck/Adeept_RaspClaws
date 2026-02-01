#!/usr/bin/env python3
# File name   : ROSServer.py
# Description : ROS 2 Server for Adeept RaspClaws Robot
# Author      : GitHub Copilot
# Date        : 2026/01/18

"""
ROS 2 Server for Adeept RaspClaws Robot

This server provides ROS 2 topics and services for controlling the robot.
Designed to run in a Docker container with ros-humble-ros-base on Raspberry Pi.

Topics Published:
- /raspclaws/battery (std_msgs/Float32): Battery voltage in Volts
- /raspclaws/cpu_temp (std_msgs/Float32): CPU temperature in °C
- /raspclaws/cpu_usage (std_msgs/Float32): CPU usage percentage (0-100)
- /raspclaws/ram_usage (std_msgs/Float32): RAM usage percentage (0-100)
- /raspclaws/servo_positions (std_msgs/String): Current servo positions (formatted string)
- /raspclaws/imu (sensor_msgs/Imu): MPU6050 gyro/accelerometer data (structured) **PREFERRED**
- /raspclaws/gyro_data (std_msgs/String): MPU6050 data (formatted string) **DEPRECATED - use /raspclaws/imu**
- /raspclaws/status (std_msgs/String): Robot status messages
- /raspclaws/camera/image_raw (sensor_msgs/Image): Camera feed (uncompressed)
- /raspclaws/camera/image_compressed (sensor_msgs/CompressedImage): Camera feed (JPEG compressed)
- /raspclaws/camera/camera_info (sensor_msgs/CameraInfo): Camera calibration info

Topics Subscribed:
- /raspclaws/cmd_vel (geometry_msgs/Twist): Movement commands
- /raspclaws/head_cmd (geometry_msgs/Point): Head position commands

Services:
- /raspclaws/reset_servos (std_srvs/Trigger): Reset all servos to default
- /raspclaws/set_smooth_mode (std_srvs/SetBool): Enable/disable smooth mode
- /raspclaws/set_smooth_cam (std_srvs/SetBool): Enable/disable smooth camera mode
- /raspclaws/set_servo_standby (std_srvs/SetBool): Set servo standby mode (True=Standby, False=Wakeup)
- /raspclaws/set_camera_pause (std_srvs/SetBool): Pause/resume camera stream (True=Pause, False=Resume)
"""

import sys
import os
import time
import threading

# Add parent directory to path for protocol.py import
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from protocol import (
    CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, CMD_RIGHT,
    CMD_FAST, CMD_SLOW, MOVE_STAND,
    CMD_SMOOTH_CAM, CMD_SMOOTH_CAM_OFF
)

# ROS 2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from rclpy.executors import ExternalShutdownException

    from std_msgs.msg import Float32, String
    from geometry_msgs.msg import Twist, Point, Vector3, Quaternion
    from sensor_msgs.msg import Image, Imu
    from std_srvs.srv import Trigger, SetBool

    ROS2_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: ROS 2 not available: {e}")
    print("Running in MOCK MODE - ROS 2 features disabled")
    ROS2_AVAILABLE = False
    # Create mock exception for when ROS2 is not available
    class ExternalShutdownException(Exception):
        pass

# Import existing robot modules
try:
    import Move as move
    import RPIservo
    import Switch as switch
    import RobotLight as robotLight
    import Info
    ROBOT_MODULES_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: Robot modules not available: {e}")
    print("Running in MOCK MODE - robot control disabled")
    ROBOT_MODULES_AVAILABLE = False
    move = None
    RPIservo = None
    switch = None
    robotLight = None
    Info = None

# Import camera module separately (optional, may fail if libcamera not available)
CameraPublisher = None
try:
    from FPV_ROS2 import CameraPublisher
    print("✓ Camera module available")
except ImportError as e:
    print(f"⚠ Camera module not available: {e}")
    print("  (Camera features will be disabled, but robot control still works)")
    CameraPublisher = None


# ==================== ROS 2 Node ====================

class RaspClawsNode(Node):
    """Main ROS 2 node for RaspClaws robot control"""

    def __init__(self):
        super().__init__('raspclaws_node')

        self.get_logger().info('Initializing RaspClaws ROS 2 Node...')

        # QoS Profile for reliable communication
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Robot state
        self.smooth_mode = False
        self.smooth_cam_mode = False
        self.servo_standby_active = False
        self.current_twist = Twist()
        self.current_head_pos = Point()

        # Lazy initialization state
        self.hardware_initialized = False
        self.ws2812 = None
        self.camera_publisher = None  # Camera publisher (separate node)

        self.get_logger().info('💤 Lazy initialization enabled - hardware will be initialized on first command')
        self.get_logger().info('   (Servos stay soft until first movement/head command)')

        # Create publishers
        self.create_publishers()

        # Create subscribers
        self.create_subscribers()

        # Create services
        self.create_services()

        # Create timers for periodic tasks
        self.create_timers()

        self.get_logger().info('RaspClaws ROS 2 Node initialized successfully!')

    def init_robot_hardware(self):
        """Initialize robot hardware components (called on first movement command)"""
        if self.hardware_initialized:
            return  # Already initialized

        try:
            self.get_logger().info('🤖 Initializing robot hardware on first command...')

            # ⚡ JETZT ERST Servos aktivieren!
            self.get_logger().info('⚡ Aktiviere PCA9685 Servo-Controller...')
            RPIservo.initialize_pwm()  # <--- EXPLIZITER AUFRUF

            # Initialize switches
            switch.switchSetup()
            switch.set_all_switch_off()

            # Initialize servos
            self.get_logger().info('🔧 Initialisiere Servo-Positionen...')
            move.init_all()

            # Initialize LEDs (optional)
            try:
                self.ws2812 = robotLight.Adeept_SPI_LedPixel(16, 255)
                if self.ws2812.check_spi_state() != 0:
                    self.ws2812.start()
                    self.ws2812.breath(70, 70, 255)
                    self.get_logger().info('✓ WS2812 LEDs initialized')
                else:
                    self.ws2812 = None
            except:
                self.ws2812 = None
                self.get_logger().warn('WS2812 LEDs not available')

            # Initialize camera publisher (optional)
            try:
                if CameraPublisher is not None:
                    self.get_logger().info('🎥 Initializing camera publisher...')
                    self.camera_publisher = CameraPublisher()
                    self.get_logger().info('✓ Camera publisher initialized')
                else:
                    self.get_logger().warn('Camera publisher not available')
            except Exception as e:
                self.get_logger().warn(f'Camera publisher initialization failed: {e}')
                self.camera_publisher = None

            self.hardware_initialized = True
            self.get_logger().info('✓ Robot hardware initialized successfully')
            self.get_logger().info('🔥 Servos sind jetzt AKTIV und STEIF!')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize robot hardware: {e}')
            raise

    def create_publishers(self):
        """Create ROS 2 publishers"""
        # Battery voltage
        self.battery_pub = self.create_publisher(
            Float32,
            '/raspclaws/battery',
            self.qos_profile
        )

        # CPU temperature
        self.cpu_temp_pub = self.create_publisher(
            Float32,
            '/raspclaws/cpu_temp',
            self.qos_profile
        )

        # CPU usage
        self.cpu_pub = self.create_publisher(
            Float32,
            '/raspclaws/cpu_usage',
            self.qos_profile
        )

        # RAM usage
        self.ram_pub = self.create_publisher(
            Float32,
            '/raspclaws/ram_usage',
            self.qos_profile
        )

        # Servo positions (as formatted string)
        self.servo_positions_pub = self.create_publisher(
            String,
            '/raspclaws/servo_positions',
            self.qos_profile
        )

        # MPU6050 Gyro/Accelerometer data (as formatted string) - DEPRECATED, use /raspclaws/imu instead
        self.gyro_pub = self.create_publisher(
            String,
            '/raspclaws/gyro_data',
            self.qos_profile
        )

        # MPU6050 IMU data (structured) - PREFERRED
        self.imu_pub = self.create_publisher(
            Imu,
            '/raspclaws/imu',
            self.qos_profile
        )

        # Status messages
        self.status_pub = self.create_publisher(
            String,
            '/raspclaws/status',
            self.qos_profile
        )

        # Camera image (placeholder - will be implemented with FPV.py integration)
        # self.image_pub = self.create_publisher(
        #     Image,
        #     '/raspclaws/camera/image',
        #     self.qos_profile
        # )

        self.get_logger().info('Publishers created')

    def create_subscribers(self):
        """Create ROS 2 subscribers"""
        # Movement commands (cmd_vel)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/raspclaws/cmd_vel',
            self.cmd_vel_callback,
            self.qos_profile
        )

        # Head position commands
        self.head_cmd_sub = self.create_subscription(
            Point,
            '/raspclaws/head_cmd',
            self.head_cmd_callback,
            self.qos_profile
        )

        self.get_logger().info('Subscribers created')

    def create_services(self):
        """Create ROS 2 services"""
        # Reset servos service
        self.reset_srv = self.create_service(
            Trigger,
            '/raspclaws/reset_servos',
            self.reset_servos_callback
        )

        # Smooth mode service
        self.smooth_mode_srv = self.create_service(
            SetBool,
            '/raspclaws/set_smooth_mode',
            self.set_smooth_mode_callback
        )

        # Smooth camera mode service
        self.smooth_cam_srv = self.create_service(
            SetBool,
            '/raspclaws/set_smooth_cam',
            self.set_smooth_cam_callback
        )

        # Servo standby/wakeup service
        self.servo_standby_srv = self.create_service(
            SetBool,
            '/raspclaws/set_servo_standby',
            self.set_servo_standby_callback
        )

        # Camera pause/resume service
        self.camera_pause_srv = self.create_service(
            SetBool,
            '/raspclaws/set_camera_pause',
            self.set_camera_pause_callback
        )

        self.get_logger().info('Services created')

    def create_timers(self):
        """Create timers for periodic tasks"""
        # Publish system info every 2 seconds
        self.info_timer = self.create_timer(2.0, self.publish_system_info)

        self.get_logger().info('Timers created')

    # ==================== Callbacks ====================

    def cmd_vel_callback(self, msg):
        """
        Handle movement commands (Twist messages)

        Mapping:
        - msg.linear.z: Forward/backward speed (-1.0 to 1.0)
                       > 0: forward, < 0: backward, 0: no linear movement
        - msg.angular.y: Turn/arc factor (-1.0 to 1.0)
                        If linear.z != 0 and angular.y != 0: Arc movement (curved path)
                        If linear.z == 0 and angular.y != 0: Turn in place
                        > 0: turn right, < 0: turn left

        Examples:
        1. Straight forward:  linear.z=0.5, angular.y=0     → forward at 50% speed
        2. Straight backward: linear.z=-0.5, angular.y=0    → backward at 50% speed
        3. Forward-right arc: linear.z=0.5, angular.y=0.3   → forward curve right
        4. Forward-left arc:  linear.z=0.5, angular.y=-0.3  → forward curve left
        5. Turn right:        linear.z=0, angular.y=0.5     → pivot turn right
        6. Turn left:         linear.z=0, angular.y=-0.5    → pivot turn left
        7. Stop:              linear.z=0, angular.y=0       → stand
        """
        self.current_twist = msg

        if not ROBOT_MODULES_AVAILABLE:
            self.get_logger().debug(
                f'MOCK: cmd_vel received: linear.z={msg.linear.z:.2f}, angular.y={msg.angular.y:.2f}'
            )
            return

        try:
            # Lazy initialization: Initialize hardware on first movement command
            if not self.hardware_initialized:
                self.init_robot_hardware()

            # Extract velocity components
            linear_z = msg.linear.z     # Forward/backward speed (-1.0 to 1.0)
            angular_y = msg.angular.y   # Turn/arc factor (-1.0 to 1.0)

            # Threshold for considering a value as non-zero
            threshold = 0.05

            # Determine movement type based on linear and angular components
            has_linear = abs(linear_z) > threshold
            has_angular = abs(angular_y) > threshold

            if has_linear and not has_angular:
                # Case 1: Straight forward/backward (no turning)
                # Map linear.z to movement speed (0-100 scale)
                speed = int(abs(linear_z) * 100)
                speed = max(10, min(100, speed))  # Clamp to 10-100
                move.set_movement_speed(speed)

                if linear_z > 0:
                    move.commandInput(CMD_FORWARD)
                    self.get_logger().debug(f'Straight forward: speed={speed}')
                else:
                    move.commandInput(CMD_BACKWARD)
                    self.get_logger().debug(f'Straight backward: speed={speed}')

            elif has_linear and has_angular:
                # Case 2: Arc movement (curved forward/backward)
                # Map linear.z to speed
                speed = int(abs(linear_z) * 100)
                speed = max(10, min(100, speed))
                move.set_movement_speed(speed)

                # Map angular.y to arc factor (0.0 to 1.0)
                # Higher angular.y = tighter turn
                arc_factor = abs(angular_y)
                arc_factor = max(0.0, min(1.0, arc_factor))
                move.set_arc_factor(arc_factor)

                # Determine direction
                if linear_z > 0:
                    # Forward arc
                    if angular_y > 0:
                        move.commandInput(CMD_FORWARD_RIGHT_ARC)
                        self.get_logger().debug(f'Forward-right arc: speed={speed}, arc={arc_factor:.2f}')
                    else:
                        move.commandInput(CMD_FORWARD_LEFT_ARC)
                        self.get_logger().debug(f'Forward-left arc: speed={speed}, arc={arc_factor:.2f}')
                else:
                    # Backward arc (if supported)
                    # Note: Check if Move.py supports backward arcs, otherwise use straight backward
                    if angular_y > 0:
                        # Backward-right would need CMD_BACKWARD_RIGHT_ARC
                        # Fallback to backward for now
                        move.commandInput(CMD_BACKWARD)
                        self.get_logger().warning('Backward arc not fully supported, using straight backward')
                    else:
                        # Backward-left would need CMD_BACKWARD_LEFT_ARC
                        move.commandInput(CMD_BACKWARD)
                        self.get_logger().warning('Backward arc not fully supported, using straight backward')

            elif not has_linear and has_angular:
                # Case 3: Turn in place (pivot)
                # Map angular.y to turn speed
                speed = int(abs(angular_y) * 100)
                speed = max(10, min(100, speed))
                move.set_movement_speed(speed)

                if angular_y > 0:
                    move.commandInput(CMD_RIGHT)
                    self.get_logger().debug(f'Turn right in place: speed={speed}')
                else:
                    move.commandInput(CMD_LEFT)
                    self.get_logger().debug(f'Turn left in place: speed={speed}')

            else:
                # Case 4: Stop (no linear, no angular)
                move.commandInput(MOVE_STAND)
                self.get_logger().debug('Stop/Stand')

        except Exception as e:
            self.get_logger().error(f'Error executing movement command: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def head_cmd_callback(self, msg):
        """Handle head position commands (Point messages)"""
        self.current_head_pos = msg

        if not ROBOT_MODULES_AVAILABLE:
            self.get_logger().debug(f'MOCK: head_cmd received: x={msg.x}, y={msg.y}')
            return

        try:
            # Lazy initialization: Initialize hardware on first head command
            if not self.hardware_initialized:
                self.init_robot_hardware()

            # msg.x: left/right (-1.0 to 1.0)
            # msg.y: up/down (-1.0 to 1.0)

            if msg.x > 0.1:
                move.look_right()
            elif msg.x < -0.1:
                move.look_left()

            if msg.y > 0.1:
                move.look_up()
            elif msg.y < -0.1:
                move.look_down()

            self.get_logger().debug(f'Head command executed: x={msg.x:.2f}, y={msg.y:.2f}')

        except Exception as e:
            self.get_logger().error(f'Error executing head command: {e}')

    def reset_servos_callback(self, request, response):
        """Service callback to reset all servos"""
        try:
            if ROBOT_MODULES_AVAILABLE:
                # Lazy initialization: Initialize hardware on first service call
                if not self.hardware_initialized:
                    self.init_robot_hardware()

                move.look_home()
                self.get_logger().info('Servos reset to home position')
                response.success = True
                response.message = 'Servos reset successfully'
            else:
                self.get_logger().info('MOCK: Servos reset')
                response.success = True
                response.message = 'MOCK: Servos reset'
        except Exception as e:
            self.get_logger().error(f'Error resetting servos: {e}')
            response.success = False
            response.message = f'Error: {e}'

        return response

    def set_smooth_mode_callback(self, request, response):
        """Service callback to set smooth movement mode"""
        try:
            self.smooth_mode = request.data

            if ROBOT_MODULES_AVAILABLE:
                if self.smooth_mode:
                    move.commandInput(CMD_SLOW)
                else:
                    move.commandInput(CMD_FAST)

            self.get_logger().info(f'Smooth mode: {self.smooth_mode}')
            response.success = True
            response.message = f'Smooth mode set to {self.smooth_mode}'
        except Exception as e:
            self.get_logger().error(f'Error setting smooth mode: {e}')
            response.success = False
            response.message = f'Error: {e}'

        return response

    def set_smooth_cam_callback(self, request, response):
        """Service callback to set smooth camera mode"""
        try:
            self.smooth_cam_mode = request.data

            if ROBOT_MODULES_AVAILABLE:
                if self.smooth_cam_mode:
                    move.commandInput(CMD_SMOOTH_CAM)
                else:
                    move.commandInput(CMD_SMOOTH_CAM_OFF)

            self.get_logger().info(f'Smooth camera mode: {self.smooth_cam_mode}')
            response.success = True
            response.message = f'Smooth camera mode set to {self.smooth_cam_mode}'
        except Exception as e:
            self.get_logger().error(f'Error setting smooth camera mode: {e}')
            response.success = False
            response.message = f'Error: {e}'

        return response

    def set_servo_standby_callback(self, request, response):
        """Service callback to set servo standby mode"""
        try:
            standby_requested = request.data  # True = Standby, False = Wakeup

            if not ROBOT_MODULES_AVAILABLE:
                self.get_logger().info(f'MOCK: Servo standby set to {standby_requested}')
                self.servo_standby_active = standby_requested
                response.success = True
                response.message = f'MOCK: Servo standby set to {standby_requested}'
                return response

            # Lazy initialization: Initialize hardware before standby/wakeup
            if not self.hardware_initialized:
                self.init_robot_hardware()

            if standby_requested:
                # STANDBY: Put servos to sleep (soft, low power)
                self.get_logger().info('🔋 SERVO STANDBY - Stopping PWM signals')
                move.standby()
                self.servo_standby_active = True
                response.success = True
                response.message = 'Servos in STANDBY mode - legs are soft, low power'
            else:
                # WAKEUP: Restore servos to stand position
                self.get_logger().info('⚡ SERVO WAKEUP - Restoring servo positions')
                move.wakeup()
                self.servo_standby_active = False
                response.success = True
                response.message = 'Servos WAKEUP - robot ready in stand position'

            self.get_logger().info(f'Servo standby mode: {self.servo_standby_active}')

        except Exception as e:
            self.get_logger().error(f'Error setting servo standby mode: {e}')
            response.success = False
            response.message = f'Error: {e}'

        return response

    def set_camera_pause_callback(self, request, response):
        """Service callback to pause/resume camera stream"""
        try:
            pause_requested = request.data  # True = Pause, False = Resume

            if not ROBOT_MODULES_AVAILABLE or self.camera_publisher is None:
                self.get_logger().info(f'MOCK: Camera pause set to {pause_requested}')
                response.success = True
                response.message = f'MOCK: Camera pause set to {pause_requested}'
                return response

            if pause_requested:
                # PAUSE: Stop camera streaming (save CPU/power)
                self.get_logger().info('📷 CAMERA PAUSE - Stopping video stream')
                self.camera_publisher.pause()
                response.success = True
                response.message = 'Camera stream PAUSED - saving CPU/power'
            else:
                # RESUME: Restart camera streaming
                self.get_logger().info('📷 CAMERA RESUME - Restarting video stream')
                self.camera_publisher.resume()
                response.success = True
                response.message = 'Camera stream RESUMED'

            self.get_logger().info(f'Camera pause mode: {pause_requested}')

        except Exception as e:
            self.get_logger().error(f'Error setting camera pause mode: {e}')
            response.success = False
            response.message = f'Error: {e}'

        return response

    # ==================== Periodic Tasks ====================

    def publish_system_info(self):
        """Publish system information periodically"""
        self.get_logger().info('Publishing system info...')
        try:
            if not ROBOT_MODULES_AVAILABLE:
                # MOCK MODE: Publish placeholder data
                battery_msg = Float32()
                battery_msg.data = 7.4
                self.battery_pub.publish(battery_msg)

                cpu_temp_msg = Float32()
                cpu_temp_msg.data = 45.0
                self.cpu_temp_pub.publish(cpu_temp_msg)

                cpu_msg = Float32()
                cpu_msg.data = 25.0
                self.cpu_pub.publish(cpu_msg)

                ram_msg = Float32()
                ram_msg.data = 50.0
                self.ram_pub.publish(ram_msg)

                status_msg = String()
                servo_status = "STANDBY" if self.servo_standby_active else "ACTIVE"
                status_msg.data = f'MOCK MODE - Servos: {servo_status}, Smooth: {self.smooth_mode}, SmoothCam: {self.smooth_cam_mode}'
                self.status_pub.publish(status_msg)

                return

            # ==================== REAL DATA ====================

            # Battery voltage (from Info.py)
            battery_voltage_str = Info.get_battery_voltage()
            battery_msg = Float32()
            try:
                battery_msg.data = float(battery_voltage_str)
            except ValueError:
                battery_msg.data = 0.0
            self.battery_pub.publish(battery_msg)

            # CPU temperature (from Info.py)
            cpu_temp_str = Info.get_cpu_tempfunc()
            cpu_temp_msg = Float32()
            try:
                cpu_temp_msg.data = float(cpu_temp_str)
            except ValueError:
                cpu_temp_msg.data = 0.0
            self.cpu_temp_pub.publish(cpu_temp_msg)

            # CPU usage (from Info.py)
            cpu_use_str = Info.get_cpu_use()
            cpu_msg = Float32()
            try:
                cpu_msg.data = float(cpu_use_str)
            except ValueError:
                cpu_msg.data = 0.0
            self.cpu_pub.publish(cpu_msg)

            # RAM usage (from Info.py)
            ram_use_str = Info.get_ram_info()
            ram_msg = Float32()
            try:
                ram_msg.data = float(ram_use_str)
            except ValueError:
                ram_msg.data = 0.0
            self.ram_pub.publish(ram_msg)

            # MPU6050 Gyro/Accelerometer data (from Move.py)
            # Gyro sensor is independent of servos, so we can publish even before hardware init
            try:
                gyro_data = move.get_mpu6050_data()

                # Publish as String (DEPRECATED - for backward compatibility)
                gyro_msg = String()
                gyro_msg.data = gyro_data
                self.gyro_pub.publish(gyro_msg)

                # Publish as structured IMU message (PREFERRED)
                if gyro_data != "MPU:N/A" and gyro_data != "MPU:ERROR":
                    # Parse gyro_data string: "G:x,y,z A:x,y,z"
                    try:
                        parts = gyro_data.split(' ')
                        gyro_part = parts[0].split(':')[1]  # "x,y,z"
                        accel_part = parts[1].split(':')[1]  # "x,y,z"

                        gyro_values = [float(x) for x in gyro_part.split(',')]
                        accel_values = [float(x) for x in accel_part.split(',')]

                        # Create IMU message
                        imu_msg = Imu()

                        # Header
                        imu_msg.header.stamp = self.get_clock().now().to_msg()
                        imu_msg.header.frame_id = 'imu_link'

                        # Angular velocity (rad/s) - convert from deg/s
                        imu_msg.angular_velocity.x = gyro_values[0] * 0.017453293  # deg to rad
                        imu_msg.angular_velocity.y = gyro_values[1] * 0.017453293
                        imu_msg.angular_velocity.z = gyro_values[2] * 0.017453293

                        # Linear acceleration (m/s²) - already in m/s²
                        imu_msg.linear_acceleration.x = accel_values[0]
                        imu_msg.linear_acceleration.y = accel_values[1]
                        imu_msg.linear_acceleration.z = accel_values[2]

                        # Orientation (not available from MPU6050 directly - set to unknown)
                        imu_msg.orientation.x = 0.0
                        imu_msg.orientation.y = 0.0
                        imu_msg.orientation.z = 0.0
                        imu_msg.orientation.w = 0.0  # Invalid quaternion = orientation unknown

                        # Covariance (set first element to -1 to indicate unknown)
                        imu_msg.orientation_covariance[0] = -1.0
                        imu_msg.angular_velocity_covariance[0] = 0.0  # Assume known
                        imu_msg.linear_acceleration_covariance[0] = 0.0  # Assume known

                        self.imu_pub.publish(imu_msg)
                        self.get_logger().debug(f'Published IMU data: gyro={gyro_values}, accel={accel_values}')
                    except Exception as parse_error:
                        self.get_logger().warn(f'Failed to parse gyro_data for IMU message: {parse_error}')

                self.get_logger().debug(f'Published gyro_data: {gyro_data}')
            except Exception as e:
                self.get_logger().error(f'Error getting/publishing gyro data: {e}')

            # Servo positions (from Move.py) - only if hardware is initialized
            if self.hardware_initialized:
                servo_positions = move.get_servo_positions_info()
                servo_msg = String()
                servo_msg.data = servo_positions
                self.servo_positions_pub.publish(servo_msg)


            # Status message
            status_msg = String()
            hw_status = "HW_READY" if self.hardware_initialized else "HW_LAZY"
            servo_status = "STANDBY" if self.servo_standby_active else "ACTIVE"
            status_msg.data = f'{hw_status} - Servos: {servo_status}, Smooth: {self.smooth_mode}, SmoothCam: {self.smooth_cam_mode}'
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing system info: {e}')

    def shutdown(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down RaspClaws node...')

        if ROBOT_MODULES_AVAILABLE:
            try:
                # Cleanup camera
                if self.camera_publisher is not None:
                    self.camera_publisher.shutdown()

                # Cleanup movement
                move.clean_all()

                # Cleanup LEDs
                if self.ws2812:
                    self.ws2812.led_close()
            except Exception as e:
                self.get_logger().error(f'Error during cleanup: {e}')

        self.get_logger().info('RaspClaws node shutdown complete')


# ==================== Main Entry Point ====================

def main(args=None):
    """Main entry point"""

    if not ROS2_AVAILABLE:
        print("ERROR: ROS 2 is not available. Please install ROS 2 Humble.")
        print("See: https://docs.ros.org/en/humble/Installation.html")
        return 1

    # Initialize ROS 2
    rclpy.init(args=args)

    # Kurze Pause, damit die Middleware den Server findet
    time.sleep(1.0)

    # Create node
    node = None
    try:
        node = RaspClawsNode()

        # Spin (process callbacks)
        try:
            node.get_logger().info('Spinning node...')
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Keyboard interrupt received')
        except ExternalShutdownException:
            # This is a normal ROS 2 shutdown signal - not an error
            node.get_logger().info('External shutdown signal received - shutting down gracefully')
        except Exception as e:
            node.get_logger().error(f'Unexpected error during spin: {e}')
            import traceback
            traceback.print_exc()
            raise

    except Exception as e:
        print(f"FATAL ERROR: Failed to create node:")
        print(f"{type(e).__name__}: {e}")
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
    sys.exit(main())
