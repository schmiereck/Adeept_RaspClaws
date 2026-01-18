#!/usr/bin/env python3
# File name   : ros2_test_client.py
# Description : Simple ROS 2 Test Client for RaspClaws (PC/Windows)
# Author      : GitHub Copilot
# Date        : 2026/01/18

"""
ROS 2 Test Client for RaspClaws

Simple Python tool to test ROS 2 API without installing Cosmos Reason 2 or ROS Planner.
Can be run on Windows/Linux/Mac with rclpy installed.

Usage:
    python ros2_test_client.py list              # List available commands
    python ros2_test_client.py forward           # Move forward
    python ros2_test_client.py status            # Show robot status
    python ros2_test_client.py battery           # Show battery voltage
"""

import sys
import time
import argparse

# ROS 2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

    from std_msgs.msg import Float32, String
    from geometry_msgs.msg import Twist, Point
    from std_srvs.srv import Trigger, SetBool

    ROS2_AVAILABLE = True
except ImportError as e:
    print(f"ERROR: ROS 2 not available: {e}")
    print("\nInstallation instructions:")
    print("1. Install ROS 2 Humble: https://docs.ros.org/en/humble/Installation.html")
    print("2. Source ROS 2: source /opt/ros/humble/setup.bash")
    print("3. Install Python dependencies: pip install rclpy")
    ROS2_AVAILABLE = False
    sys.exit(1)


# ==================== Test Client Node ====================

class RaspClawsTestClient(Node):
    """Simple test client for RaspClaws ROS 2 API"""

    def __init__(self):
        super().__init__('raspclaws_test_client')

        # QoS Profile
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/raspclaws/cmd_vel',
            self.qos_profile
        )

        self.head_cmd_pub = self.create_publisher(
            Point,
            '/raspclaws/head_cmd',
            self.qos_profile
        )

        # Subscribers (for monitoring)
        self.battery_sub = self.create_subscription(
            Float32,
            '/raspclaws/battery',
            self.battery_callback,
            self.qos_profile
        )

        self.cpu_sub = self.create_subscription(
            Float32,
            '/raspclaws/cpu_usage',
            self.cpu_callback,
            self.qos_profile
        )

        self.status_sub = self.create_subscription(
            String,
            '/raspclaws/status',
            self.status_callback,
            self.qos_profile
        )

        # Service clients
        self.reset_client = self.create_client(Trigger, '/raspclaws/reset_servos')
        self.smooth_client = self.create_client(SetBool, '/raspclaws/set_smooth_mode')
        self.smooth_cam_client = self.create_client(SetBool, '/raspclaws/set_smooth_cam')

        # Latest values
        self.latest_battery = None
        self.latest_cpu = None
        self.latest_status = None

        self.get_logger().info('RaspClaws Test Client initialized')

    # ==================== Callbacks ====================

    def battery_callback(self, msg):
        self.latest_battery = msg.data

    def cpu_callback(self, msg):
        self.latest_cpu = msg.data

    def status_callback(self, msg):
        self.latest_status = msg.data

    # ==================== Movement Commands ====================

    def send_twist(self, linear_x=0.0, angular_z=0.0):
        """Send movement command"""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f'Sent: linear.x={linear_x}, angular.z={angular_z}')

    def forward(self, speed=0.5):
        """Move forward"""
        self.send_twist(linear_x=speed)

    def backward(self, speed=0.5):
        """Move backward"""
        self.send_twist(linear_x=-speed)

    def left(self, speed=0.5):
        """Turn left"""
        self.send_twist(angular_z=speed)

    def right(self, speed=0.5):
        """Turn right"""
        self.send_twist(angular_z=-speed)

    def stop(self):
        """Stop movement"""
        self.send_twist(0.0, 0.0)

    # ==================== Head Commands ====================

    def send_head_cmd(self, x=0.0, y=0.0):
        """Send head position command"""
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.0
        self.head_cmd_pub.publish(msg)
        self.get_logger().info(f'Head: x={x}, y={y}')

    def head_left(self):
        """Look left"""
        self.send_head_cmd(x=0.5)

    def head_right(self):
        """Look right"""
        self.send_head_cmd(x=-0.5)

    def head_up(self):
        """Look up"""
        self.send_head_cmd(y=0.5)

    def head_down(self):
        """Look down"""
        self.send_head_cmd(y=-0.5)

    def head_center(self):
        """Center head"""
        self.send_head_cmd(0.0, 0.0)

    # ==================== Service Calls ====================

    def call_reset_servos(self):
        """Call reset servos service"""
        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Reset service not available')
            return False

        request = Trigger.Request()
        future = self.reset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Reset: {response.message}')
            return response.success
        else:
            self.get_logger().error('Service call failed')
            return False

    def call_smooth_mode(self, enable=True):
        """Call smooth mode service"""
        if not self.smooth_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Smooth mode service not available')
            return False

        request = SetBool.Request()
        request.data = enable
        future = self.smooth_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Smooth mode: {response.message}')
            return response.success
        else:
            self.get_logger().error('Service call failed')
            return False

    def call_smooth_cam(self, enable=True):
        """Call smooth camera service"""
        if not self.smooth_cam_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Smooth cam service not available')
            return False

        request = SetBool.Request()
        request.data = enable
        future = self.smooth_cam_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Smooth cam: {response.message}')
            return response.success
        else:
            self.get_logger().error('Service call failed')
            return False

    # ==================== Monitoring ====================

    def print_status(self):
        """Print current status"""
        print("\n" + "="*50)
        print("RaspClaws Robot Status")
        print("="*50)
        print(f"Battery:     {self.latest_battery:.2f} V" if self.latest_battery else "Battery:     N/A")
        print(f"CPU Usage:   {self.latest_cpu:.1f} %" if self.latest_cpu else "CPU Usage:   N/A")
        print(f"Status:      {self.latest_status}" if self.latest_status else "Status:      N/A")
        print("="*50 + "\n")


# ==================== Command Line Interface ====================

def run_command(client, command, args):
    """Execute a command"""

    commands = {
        # Movement
        'forward': lambda: client.forward(float(args.speed) if args.speed else 0.5),
        'backward': lambda: client.backward(float(args.speed) if args.speed else 0.5),
        'left': lambda: client.left(float(args.speed) if args.speed else 0.5),
        'right': lambda: client.right(float(args.speed) if args.speed else 0.5),
        'stop': lambda: client.stop(),

        # Head
        'head_left': lambda: client.head_left(),
        'head_right': lambda: client.head_right(),
        'head_up': lambda: client.head_up(),
        'head_down': lambda: client.head_down(),
        'head_center': lambda: client.head_center(),

        # Services
        'reset': lambda: client.call_reset_servos(),
        'smooth_on': lambda: client.call_smooth_mode(True),
        'smooth_off': lambda: client.call_smooth_mode(False),
        'smooth_cam_on': lambda: client.call_smooth_cam(True),
        'smooth_cam_off': lambda: client.call_smooth_cam(False),

        # Monitoring
        'status': lambda: monitor_status(client, duration=3),
        'battery': lambda: monitor_battery(client, duration=3),
        'monitor': lambda: monitor_all(client, duration=int(args.duration) if args.duration else 10),
    }

    if command in commands:
        commands[command]()
    else:
        print(f"Unknown command: {command}")
        print("Use 'list' to see available commands")
        return False

    return True


def monitor_status(client, duration=3):
    """Monitor status for a few seconds"""
    print(f"Monitoring status for {duration} seconds...")
    start_time = time.time()

    while (time.time() - start_time) < duration:
        rclpy.spin_once(client, timeout_sec=0.1)

    client.print_status()


def monitor_battery(client, duration=3):
    """Monitor battery for a few seconds"""
    print(f"Reading battery voltage...")
    start_time = time.time()

    while (time.time() - start_time) < duration:
        rclpy.spin_once(client, timeout_sec=0.1)
        if client.latest_battery is not None:
            break

    if client.latest_battery:
        print(f"\nBattery: {client.latest_battery:.2f} V\n")
    else:
        print("\nNo battery data received\n")


def monitor_all(client, duration=10):
    """Monitor all topics"""
    print(f"Monitoring all topics for {duration} seconds (Ctrl+C to stop)...")
    print("="*50)

    start_time = time.time()
    last_print = 0

    try:
        while (time.time() - start_time) < duration:
            rclpy.spin_once(client, timeout_sec=0.1)

            # Print every second
            if time.time() - last_print >= 1.0:
                elapsed = int(time.time() - start_time)
                print(f"[{elapsed}s] Battery: {client.latest_battery:.2f}V | "
                      f"CPU: {client.latest_cpu:.1f}% | "
                      f"Status: {client.latest_status}")
                last_print = time.time()

    except KeyboardInterrupt:
        print("\nMonitoring stopped")


def list_commands():
    """Print available commands"""
    print("\n" + "="*50)
    print("RaspClaws ROS 2 Test Client - Available Commands")
    print("="*50)
    print("\nMovement Commands:")
    print("  forward [--speed 0.5]     - Move forward")
    print("  backward [--speed 0.5]    - Move backward")
    print("  left [--speed 0.5]        - Turn left")
    print("  right [--speed 0.5]       - Turn right")
    print("  stop                      - Stop movement")
    print("\nHead Commands:")
    print("  head_left                 - Look left")
    print("  head_right                - Look right")
    print("  head_up                   - Look up")
    print("  head_down                 - Look down")
    print("  head_center               - Center head")
    print("\nService Commands:")
    print("  reset                     - Reset all servos to home")
    print("  smooth_on                 - Enable smooth movement mode")
    print("  smooth_off                - Disable smooth movement mode")
    print("  smooth_cam_on             - Enable smooth camera mode")
    print("  smooth_cam_off            - Disable smooth camera mode")
    print("\nMonitoring Commands:")
    print("  status                    - Show current status")
    print("  battery                   - Show battery voltage")
    print("  monitor [--duration 10]   - Monitor all topics")
    print("\nExamples:")
    print("  python ros2_test_client.py forward")
    print("  python ros2_test_client.py forward --speed 0.3")
    print("  python ros2_test_client.py status")
    print("  python ros2_test_client.py monitor --duration 30")
    print("="*50 + "\n")


def interactive_mode(client):
    """Interactive command mode"""
    print("\n" + "="*50)
    print("RaspClaws ROS 2 Test Client - Interactive Mode")
    print("="*50)
    print("Type 'help' for commands, 'quit' to exit\n")

    class Args:
        speed = None
        duration = None

    args = Args()

    while True:
        try:
            # Spin once to receive messages
            rclpy.spin_once(client, timeout_sec=0.1)

            # Get user input
            cmd = input("raspclaws> ").strip().lower()

            if not cmd:
                continue

            if cmd in ['quit', 'exit', 'q']:
                print("Goodbye!")
                break

            if cmd in ['help', 'list', '?']:
                list_commands()
                continue

            # Parse speed/duration from command
            parts = cmd.split()
            command = parts[0]

            args.speed = None
            args.duration = None

            if len(parts) > 1:
                try:
                    args.speed = parts[1]
                    args.duration = parts[1]
                except:
                    pass

            run_command(client, command, args)

        except KeyboardInterrupt:
            print("\nUse 'quit' to exit")
        except Exception as e:
            print(f"Error: {e}")


# ==================== Main Entry Point ====================

def main():
    """Main entry point"""

    parser = argparse.ArgumentParser(
        description='RaspClaws ROS 2 Test Client',
        epilog='Use "list" command to see all available commands'
    )

    parser.add_argument('command', nargs='?', default='interactive',
                        help='Command to execute (or "interactive" for interactive mode)')
    parser.add_argument('--speed', type=float, help='Movement speed (0.0-1.0)')
    parser.add_argument('--duration', type=int, help='Monitoring duration in seconds')

    args = parser.parse_args()

    # Initialize ROS 2
    rclpy.init()

    try:
        # Create client node
        client = RaspClawsTestClient()

        # Wait for robot node
        print("Waiting for RaspClaws node...")
        start_time = time.time()
        while (time.time() - start_time) < 5.0:
            rclpy.spin_once(client, timeout_sec=0.1)
            # Simple check - could be improved

        print("✓ Connected to RaspClaws\n")

        # Execute command
        if args.command == 'list':
            list_commands()
        elif args.command == 'interactive':
            interactive_mode(client)
        else:
            run_command(client, args.command, args)

        # Cleanup
        client.destroy_node()

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
