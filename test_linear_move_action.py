#!/usr/bin/env python3
"""
Test Client for RaspClaws LinearMove Action

This script sends a LinearMove goal to the action server and displays progress.
Useful for testing robot movement with specific speed and cycle counts.

Usage:
    python test_linear_move_action.py

Author: RaspClaws Test Suite
Date: 2026-02-22
"""

import sys
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

try:
    from raspclaws_interfaces.action import LinearMove
except ImportError as e:
    print(f"ERROR: Cannot import LinearMove action interface!")
    print(f"       {e}")
    print("")
    print("Please ensure:")
    print("  1. raspclaws_interfaces package is built (colcon build)")
    print("  2. Source the ROS2 workspace (source install/setup.bash)")
    sys.exit(1)


class LinearMoveTestClient(Node):
    """Test client for LinearMove action."""

    def __init__(self):
        super().__init__('linear_move_test_client')
        
        # Create action client
        self._action_client = ActionClient(
            self,
            LinearMove,
            '/raspclaws/linear_move'
        )
        
        self.get_logger().info("LinearMove Test Client initialized")
        self.get_logger().info("Waiting for action server...")
        
    def send_goal(self, distance_cm: float, speed: float, step_size_cm: float = 5.0):
        """
        Send a LinearMove goal to the action server.
        
        Args:
            distance_cm: Distance in centimeters (positive = forward, negative = backward)
            speed: Movement speed (10.0 - 80.0)
            step_size_cm: Feedback interval in centimeters
        """
        # Wait for server to be available (timeout 10 seconds)
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available!")
            self.get_logger().error("Please ensure ROSServer.py is running.")
            return False
        
        self.get_logger().info(f"Action server available!")
        
        # Create goal message
        goal_msg = LinearMove.Goal()
        goal_msg.distance_cm = distance_cm
        goal_msg.speed = speed
        goal_msg.step_size_cm = step_size_cm
        
        # Log goal parameters
        direction = "FORWARD" if distance_cm > 0 else "BACKWARD"
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"SENDING GOAL: Move {direction}")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  Distance:      {abs(distance_cm):.1f} cm")
        self.get_logger().info(f"  Speed:         {speed:.1f}")
        self.get_logger().info(f"  Step Size:     {step_size_cm:.1f} cm (feedback interval)")
        self.get_logger().info("=" * 60)
        self.get_logger().info("")
        
        # Send goal with callbacks
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True
    
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Goal REJECTED by server!")
            self.get_logger().error("Check parameters (speed: 10-80, distance > 0.1cm)")
            return
        
        self.get_logger().info("Goal ACCEPTED by server")
        self.get_logger().info("Movement started...")
        
        # Get result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Handle progress feedback."""
        feedback = feedback_msg.feedback
        
        # Display progress bar
        progress_pct = feedback.progress * 100.0
        bar_length = 40
        filled_length = int(bar_length * feedback.progress)
        bar = '█' * filled_length + '░' * (bar_length - filled_length)
        
        self.get_logger().info(
            f"Progress: [{bar}] {progress_pct:5.1f}% | "
            f"Distance: {feedback.distance_traveled:6.1f}cm | "
            f"Steps: {feedback.steps_completed:3d}"
        )
    
    def get_result_callback(self, future):
        """Handle final result."""
        result = future.result().result
        
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("MOVEMENT COMPLETED")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  Success:           {result.success}")
        self.get_logger().info(f"  Message:           {result.message}")
        self.get_logger().info(f"  Distance Traveled: {result.distance_traveled:.2f} cm")
        self.get_logger().info(f"  Speed Used:        {result.final_speed:.1f}")
        self.get_logger().info(f"  Steps Taken:       {result.steps_taken}")
        self.get_logger().info("=" * 60)
        self.get_logger().info("")


def main():
    """Main function."""
    print("")
    print("=" * 60)
    print("      RaspClaws LinearMove Action Test Client")
    print("=" * 60)
    print("")
    
    # Initialize ROS2
    rclpy.init()
    
    # Create test client
    client = LinearMoveTestClient()
    
    # === TEST CONFIGURATION ===
    # Modify these values to test different movements:
    
    TEST_DISTANCE_CM = 30.0    # Distance in cm (positive = forward, negative = backward)
    TEST_SPEED = 50.0          # Speed (10.0 - 80.0)
    TEST_STEP_SIZE = 5.0       # Feedback interval in cm
    
    # ===========================
    
    print("Test Configuration:")
    print(f"  Distance:  {TEST_DISTANCE_CM:.1f} cm")
    print(f"  Speed:     {TEST_SPEED:.1f}")
    print(f"  Step Size: {TEST_STEP_SIZE:.1f} cm")
    print("")
    print("Starting test in 2 seconds...")
    print("(Press Ctrl+C to cancel)")
    print("")
    
    time.sleep(2.0)
    
    # Send goal
    success = client.send_goal(
        distance_cm=TEST_DISTANCE_CM,
        speed=TEST_SPEED,
        step_size_cm=TEST_STEP_SIZE
    )
    
    if not success:
        print("Failed to send goal!")
        rclpy.shutdown()
        return
    
    # Spin until complete (or Ctrl+C)
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        print("")
        print("Interrupted by user!")
    
    # Cleanup
    client.destroy_node()
    rclpy.shutdown()
    
    print("")
    print("Test completed.")
    print("")


if __name__ == '__main__':
    main()
