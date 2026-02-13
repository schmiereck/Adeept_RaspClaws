#!/usr/bin/env python3
"""
Robot Control Node (Phase 1)
----------------------------
Provides a high-level interface to control the RaspClaws robot via ROS2 Actions.
Designed to be used by autonomous agents (e.g., Neural Networks).

Features:
- Action Clients for: HeadPosition, LinearMove, Rotate, ArcMove
- Synchronous and Asynchronous execution methods
- Feedback handling

Author: RaspClaws Project
Date: 2026-02-13
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future

# Import action interfaces
try:
    from raspclaws_interfaces.action import HeadPosition, LinearMove, Rotate, ArcMove
    ACTIONS_AVAILABLE = True
except ImportError:
    print("WARNING: raspclaws_interfaces not found. Make sure you have sourced the workspace.")
    ACTIONS_AVAILABLE = False

class RobotControlNode(Node):
    """
    ROS2 Node acting as a client controller for the robot.
    Abstracts ROS2 Actions into simple method calls.
    """

    def __init__(self):
        super().__init__('robot_control_node')
        
        if not ACTIONS_AVAILABLE:
            self.get_logger().error("Action interfaces missing! Cannot function.")
            return

        self.get_logger().info("Initializing RobotControlNode...")

        # --- Action Clients ---
        self._head_client = ActionClient(self, HeadPosition, '/raspclaws/head_position')
        self._linear_client = ActionClient(self, LinearMove, '/raspclaws/linear_move')
        self._rotate_client = ActionClient(self, Rotate, '/raspclaws/rotate')
        self._arc_client = ActionClient(self, ArcMove, '/raspclaws/arc_move')

        # Wait for servers (non-blocking in constructor, but methods will check)
        self.server_check_timer = self.create_timer(1.0, self._check_servers)
        self._servers_ready = False

    def _check_servers(self):
        """Periodically check if action servers are available."""
        if self._servers_ready:
            self.server_check_timer.cancel()
            return

        head_ready = self._head_client.wait_for_server(timeout_sec=0.1)
        linear_ready = self._linear_client.wait_for_server(timeout_sec=0.1)
        rotate_ready = self._rotate_client.wait_for_server(timeout_sec=0.1)
        arc_ready = self._arc_client.wait_for_server(timeout_sec=0.1)

        if head_ready and linear_ready and rotate_ready and arc_ready:
            self.get_logger().info("✅ All Action Servers connected!")
            self._servers_ready = True
            self.server_check_timer.cancel()
        else:
            self.get_logger().info("Waiting for action servers...", throttle_duration_sec=5.0)

    # =========================================================================
    # Linear Move
    # =========================================================================
    def move_linear(self, distance_cm: float, speed: float = 50.0, sync: bool = True):
        """
        Move the robot linearly.
        
        Args:
            distance_cm: Distance in cm (positive=forward, negative=backward)
            speed: Speed (10-80)
            sync: If True, waits for completion. If False, returns the goal handle future.
        """
        goal_msg = LinearMove.Goal()
        goal_msg.distance_cm = float(distance_cm)
        goal_msg.speed = float(speed)
        goal_msg.step_size_cm = 5.0  # Default feedback interval

        self.get_logger().info(f"Sending LinearMove goal: {distance_cm}cm")
        
        return self._send_goal(self._linear_client, goal_msg, sync)

    # =========================================================================
    # Rotate
    # =========================================================================
    def rotate(self, angle_degrees: float, speed: float = 50.0, use_imu: bool = True, sync: bool = True):
        """
        Rotate the robot.
        
        Args:
            angle_degrees: Angle in degrees (positive=right, negative=left)
            speed: Speed (10-80)
            use_imu: Whether to use IMU for correction
            sync: If True, waits for completion.
        """
        goal_msg = Rotate.Goal()
        goal_msg.angle_degrees = float(angle_degrees)
        goal_msg.speed = float(speed)
        goal_msg.step_size_deg = 10.0  # Default feedback interval
        goal_msg.use_imu = use_imu

        self.get_logger().info(f"Sending Rotate goal: {angle_degrees}°")
        
        return self._send_goal(self._rotate_client, goal_msg, sync)

    # =========================================================================
    # Arc Move
    # =========================================================================
    def move_arc(self, distance_cm: float, arc_factor: float, speed: float = 50.0, sync: bool = True):
        """
        Move the robot in an arc.
        
        Args:
            distance_cm: Distance in cm
            arc_factor: Curvature factor (0.0 - 1.0)
            speed: Speed (10-80)
            sync: If True, waits for completion.
        """
        goal_msg = ArcMove.Goal()
        goal_msg.distance_cm = float(distance_cm)
        goal_msg.arc_factor = float(arc_factor)
        goal_msg.speed = float(speed)
        goal_msg.step_size_cm = 5.0

        self.get_logger().info(f"Sending ArcMove goal: {distance_cm}cm, factor={arc_factor}")
        
        return self._send_goal(self._arc_client, goal_msg, sync)

    # =========================================================================
    # Head Position
    # =========================================================================
    def set_head_pos(self, pan: float, tilt: float, smooth: bool = True, sync: bool = False):
        """
        Move the camera head.
        
        Args:
            pan: Pan angle (-100 to 100)
            tilt: Tilt angle (-67 to 67)
            smooth: Smooth movement
            sync: Defaults to False for head movement (usually fire-and-forget)
        """
        goal_msg = HeadPosition.Goal()
        goal_msg.pan_degrees = float(pan)
        goal_msg.tilt_degrees = float(tilt)
        goal_msg.smooth = smooth

        self.get_logger().info(f"Sending HeadPosition goal: pan={pan}, tilt={tilt}")
        
        return self._send_goal(self._head_client, goal_msg, sync)

    # =========================================================================
    # Internal Helpers
    # =========================================================================
    def _send_goal(self, client: ActionClient, goal_msg, sync: bool):
        if not client.server_is_ready():
            self.get_logger().error(f"Action server for {client._action_name} not ready!")
            return None

        send_goal_future = client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)

        if sync:
            # Synchronous execution: Wait for Goal Acceptance, then Result
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                self.get_logger().error("Goal rejected :(")
                return None

            self.get_logger().info("Goal accepted, waiting for result...")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            
            result = result_future.result()
            self._log_result(result)
            return result
        else:
            # Asynchronous: Return the future
            return send_goal_future

    def _feedback_callback(self, feedback_msg):
        """Generic feedback handler."""
        feedback = feedback_msg.feedback
        # We can implement specific logging based on feedback type if needed
        # For now, just debug log
        self.get_logger().debug(f"Received feedback: {feedback}")

    def _log_result(self, result):
        status = result.status
        # status 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        if status == 4:
            self.get_logger().info("Action SUCCEEDED")
        else:
            self.get_logger().warn(f"Action finished with status: {status}")

def main(args=None):
    rclpy.init(args=args)
    
    node = RobotControlNode()
    
    # Keep node alive
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
