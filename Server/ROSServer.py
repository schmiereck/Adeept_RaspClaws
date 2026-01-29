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
- /raspclaws/gyro_data (std_msgs/String): MPU6050 gyro/accelerometer data (formatted string)
- /raspclaws/status (std_msgs/String): Robot status messages
- /raspclaws/camera/image (sensor_msgs/Image): Camera feed (TODO)

Topics Subscribed:
- /raspclaws/cmd_vel (geometry_msgs/Twist): Movement commands
- /raspclaws/head_cmd (geometry_msgs/Point): Head position commands

Services:
- /raspclaws/reset_servos (std_srvs/Trigger): Reset all servos to default
- /raspclaws/set_smooth_mode (std_srvs/SetBool): Enable/disable smooth mode
- /raspclaws/set_smooth_cam (std_srvs/SetBool): Enable/disable smooth camera mode
- /raspclaws/set_servo_standby (std_srvs/SetBool): Set servo standby mode (True=Standby, False=Wakeup)
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
    from geometry_msgs.msg import Twist, Point
    from sensor_msgs.msg import Image
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

        # MPU6050 Gyro/Accelerometer data (as formatted string)
        self.gyro_pub = self.create_publisher(
            String,
            '/raspclaws/gyro_data',
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

        self.get_logger().info('Services created')

    def create_timers(self):
        """Create timers for periodic tasks"""
        # Publish system info every 2 seconds
        self.info_timer = self.create_timer(2.0, self.publish_system_info)

        self.get_logger().info('Timers created')

    # ==================== Callbacks ====================

    def cmd_vel_callback(self, msg):
        """Handle movement commands (Twist messages)"""
        self.current_twist = msg

        if not ROBOT_MODULES_AVAILABLE:
            self.get_logger().debug(f'MOCK: cmd_vel received: linear.x={msg.linear.x}, angular.z={msg.angular.z}')
            return

        try:
            # Lazy initialization: Initialize hardware on first movement command
            if not self.hardware_initialized:
                self.init_robot_hardware()

            # Convert Twist to robot commands
            linear_x = msg.linear.x    # Forward/backward (-1.0 to 1.0)
            angular_z = msg.angular.z  # Left/right rotation (-1.0 to 1.0)

            # Map to robot movement commands
            if abs(linear_x) > 0.1:
                if linear_x > 0:
                    move.commandInput(CMD_FORWARD)
                else:
                    move.commandInput(CMD_BACKWARD)
            elif abs(angular_z) > 0.1:
                if angular_z > 0:
                    move.commandInput(CMD_LEFT)
                else:
                    move.commandInput(CMD_RIGHT)
            else:
                move.commandInput(MOVE_STAND)

            self.get_logger().debug(f'Movement command executed: linear={linear_x:.2f}, angular={angular_z:.2f}')

        except Exception as e:
            self.get_logger().error(f'Error executing movement command: {e}')

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

    # ==================== Periodic Tasks ====================

    def publish_system_info(self):
        """Publish system information periodically"""
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

            # Servo positions (from Move.py) - only if hardware is initialized
            if self.hardware_initialized:
                servo_positions = move.get_servo_positions_info()
                servo_msg = String()
                servo_msg.data = servo_positions
                self.servo_positions_pub.publish(servo_msg)

            # MPU6050 Gyro/Accelerometer data (from Move.py) - only if hardware is initialized
            if self.hardware_initialized:
                gyro_data = move.get_mpu6050_data()
                gyro_msg = String()
                gyro_msg.data = gyro_data
                self.gyro_pub.publish(gyro_msg)

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
                move.clean_all()
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
