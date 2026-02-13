#!/usr/bin/env python3
"""
Test Script for RobotControlNode
--------------------------------
Executes a sequence of actions to verify the RobotControlNode functionality.
Run this on the Raspberry Pi (or a machine connected to the ROS2 network).

Usage:
    python3 test_robot_control.py
"""

import time
import rclpy
from RobotControlNode import RobotControlNode

def main():
    print("Initializing ROS2...")
    rclpy.init()

    print("Creating RobotControlNode...")
    node = RobotControlNode()

    # Wait a bit for connections
    print("Waiting for connections...")
    time.sleep(2)

    try:
        # 1. Test Head Movement (Pan/Tilt)
        print("\n--- Test 1: Head Movement ---")
        print("Looking Left/Up...")
        node.set_head_pos(pan=45.0, tilt=20.0, sync=False) # Async fire-and-forget
        time.sleep(1.0)
        
        print("Looking Right/Down...")
        node.set_head_pos(pan=-45.0, tilt=-20.0, sync=False)
        time.sleep(1.0)
        
        print("Centering Head...")
        node.set_head_pos(pan=0.0, tilt=0.0, sync=False)
        time.sleep(1.0)

        # 2. Test Linear Move
        print("\n--- Test 2: Linear Move ---")
        print("Moving Forward 10cm...")
        result = node.move_linear(distance_cm=10.0, speed=50.0, sync=True)
        if result and result.success:
            print("✓ Linear Move Success")
        else:
            print("✗ Linear Move Failed")

        # 3. Test Rotate
        print("\n--- Test 3: Rotate ---")
        print("Rotating 45 degrees Right...")
        result = node.rotate(angle_degrees=45.0, speed=50.0, sync=True)
        if result and result.success:
            print("✓ Rotate Success")
        else:
            print("✗ Rotate Failed")

        # 4. Test Arc Move
        print("\n--- Test 4: Arc Move ---")
        print("Moving in Arc (Left curve)...")
        result = node.move_arc(distance_cm=10.0, arc_factor=0.5, speed=50.0, sync=True)
        if result and result.success:
            print("✓ Arc Move Success")
        else:
            print("✗ Arc Move Failed")

    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    except Exception as e:
        print(f"\nERROR: {e}")
    finally:
        print("\nShutting down...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
