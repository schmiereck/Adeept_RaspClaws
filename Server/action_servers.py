#!/usr/bin/env python3
"""
ROS2 Action Servers for RaspClaws Robot

Implements 4 action servers for precise robot control:
- HeadPosition: Move camera to target angles
- LinearMove: Move forward/backward for specified distance
- Rotate: Rotate in place for specified angle
- ArcMove: Move in arc trajectory

Author: RaspClaws Actions System
Date: 2026-02-12
"""

import time
import threading
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

# Import custom action messages (will be generated from .action files)
try:
    from raspclaws_interfaces.action import HeadPosition, LinearMove, Rotate, ArcMove
    ACTIONS_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: Action interfaces not available: {e}")
    print("Please build raspclaws_interfaces package first!")
    ACTIONS_AVAILABLE = False

from calibration import RobotCalibration

# Import Move module for hardware control
try:
    import Move as move_module
    MOVE_MODULE_AVAILABLE = True
except ImportError:
    print("WARNING: Move module not available")
    MOVE_MODULE_AVAILABLE = False
    move_module = None


class ActionServerManager:
    """
    Manages all action servers for the RaspClaws robot.

    This class creates and manages 4 action servers:
    - /raspclaws/head_position
    - /raspclaws/linear_move
    - /raspclaws/rotate
    - /raspclaws/arc_move
    """

    def __init__(self, node: Node):
        """
        Initialize action server manager.

        Args:
            node: ROS2 node instance (ROSServer)
        """
        if not ACTIONS_AVAILABLE:
            raise ImportError("Action interfaces not available. Build raspclaws_interfaces first!")

        self.node = node
        self.logger = node.get_logger()

        # Use Move module directly (it's a module, not a node attribute)
        if not MOVE_MODULE_AVAILABLE or move_module is None:
            raise RuntimeError("Move module not available - cannot initialize action servers")

        self.move = move_module  # Reference to Move module

        # Ensure robot hardware is initialized (for action servers)
        if hasattr(node, 'init_robot_hardware') and not node.hardware_initialized:
            self.logger.info('Action servers require hardware - initializing now...')
            node.init_robot_hardware()

        # Load calibration data
        try:
            self.calibration = RobotCalibration()
            self.logger.info("Calibration data loaded successfully")
        except FileNotFoundError:
            self.logger.warn("Calibration file not found. Using default values.")
            self.logger.warn("Run calibrate_robot.py for accurate measurements!")
            self.calibration = None

        # Callback group for concurrent action execution
        self.callback_group = ReentrantCallbackGroup()

        # Track active goals (only one active goal per action type)
        self.active_goals = {
            'head': None,
            'linear': None,
            'rotate': None,
            'arc': None
        }

        # Create action servers
        self.create_action_servers()

    def create_action_servers(self):
        """Create all 4 action servers."""
        self.logger.info("Creating action servers...")

        # HeadPosition Action Server
        self.head_position_server = ActionServer(
            self.node,
            HeadPosition,
            '/raspclaws/head_position',
            execute_callback=self.execute_head_position,
            goal_callback=self.goal_callback_head,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        self.logger.info("  ✓ HeadPosition action server created")

        # LinearMove Action Server
        self.linear_move_server = ActionServer(
            self.node,
            LinearMove,
            '/raspclaws/linear_move',
            execute_callback=self.execute_linear_move,
            goal_callback=self.goal_callback_linear,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        self.logger.info("  ✓ LinearMove action server created")

        # Rotate Action Server
        self.rotate_server = ActionServer(
            self.node,
            Rotate,
            '/raspclaws/rotate',
            execute_callback=self.execute_rotate,
            goal_callback=self.goal_callback_rotate,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        self.logger.info("  ✓ Rotate action server created")

        # ArcMove Action Server
        self.arc_move_server = ActionServer(
            self.node,
            ArcMove,
            '/raspclaws/arc_move',
            execute_callback=self.execute_arc_move,
            goal_callback=self.goal_callback_arc,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        self.logger.info("  ✓ ArcMove action server created")

        self.logger.info("All action servers ready!")

    # === Goal and Cancel Callbacks ===

    def goal_callback_head(self, goal_request):
        """Accept or reject HeadPosition goal."""
        self.logger.info(f"HeadPosition goal request: pan={goal_request.pan_degrees:.1f}°, "
                         f"tilt={goal_request.tilt_degrees:.1f}°")

        # Validate goal parameters
        if goal_request.pan_degrees < -100.0 or goal_request.pan_degrees > 100.0:
            self.logger.warn("Pan angle out of range (-100 to +100)")
            return GoalResponse.REJECT

        if goal_request.tilt_degrees < -67.0 or goal_request.tilt_degrees > 67.0:
            self.logger.warn("Tilt angle out of range (-67 to +67)")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def goal_callback_linear(self, goal_request):
        """Accept or reject LinearMove goal."""
        self.logger.info(f"LinearMove goal request: distance={goal_request.distance_cm:.1f}cm, "
                         f"speed={goal_request.speed:.1f}")

        # Validate goal parameters
        if goal_request.speed < 10.0 or goal_request.speed > 80.0:
            self.logger.warn("Speed out of range (10-80)")
            return GoalResponse.REJECT

        if abs(goal_request.distance_cm) < 0.1:
            self.logger.warn("Distance too small")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def goal_callback_rotate(self, goal_request):
        """Accept or reject Rotate goal."""
        self.logger.info(f"Rotate goal request: angle={goal_request.angle_degrees:.1f}°, "
                         f"speed={goal_request.speed:.1f}, use_imu={goal_request.use_imu}")

        # Validate goal parameters
        if goal_request.speed < 10.0 or goal_request.speed > 80.0:
            self.logger.warn("Speed out of range (10-80)")
            return GoalResponse.REJECT

        if abs(goal_request.angle_degrees) < 0.1:
            self.logger.warn("Angle too small")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def goal_callback_arc(self, goal_request):
        """Accept or reject ArcMove goal."""
        self.logger.info(f"ArcMove goal request: distance={goal_request.distance_cm:.1f}cm, "
                         f"arc_factor={goal_request.arc_factor:.2f}, speed={goal_request.speed:.1f}")

        # Validate goal parameters
        if goal_request.speed < 10.0 or goal_request.speed > 80.0:
            self.logger.warn("Speed out of range (10-80)")
            return GoalResponse.REJECT

        if goal_request.arc_factor < 0.0 or goal_request.arc_factor > 1.0:
            self.logger.warn("Arc factor out of range (0.0-1.0)")
            return GoalResponse.REJECT

        if abs(goal_request.distance_cm) < 0.1:
            self.logger.warn("Distance too small")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle cancel requests for any action."""
        self.logger.info(f"Cancel request received for {goal_handle.request}")
        return CancelResponse.ACCEPT

    # === HeadPosition Action Implementation ===

    async def execute_head_position(self, goal_handle):
        """
        Execute HeadPosition action.

        Moves camera head to target pan/tilt angles.
        Uses PWM control for servos on channels 12 (pan) and 13 (tilt).
        """
        self.logger.info("Executing HeadPosition action...")
        goal = goal_handle.request
        feedback = HeadPosition.Feedback()
        result = HeadPosition.Result()

        # Check if calibration is available
        if self.calibration is None:
            result.success = False
            result.message = "Calibration not available"
            return result

        # Get current position
        current_pan_pwm = self.move.Left_Right_input
        current_tilt_pwm = self.move.Up_Down_input

        current_pan_deg = self.calibration.pwm_to_pan_degrees(current_pan_pwm)
        current_tilt_deg = self.calibration.pwm_to_tilt_degrees(current_tilt_pwm)

        # Get target position
        target_pan_deg = goal.pan_degrees
        target_tilt_deg = goal.tilt_degrees

        target_pan_pwm = self.calibration.pan_degrees_to_pwm(target_pan_deg)
        target_tilt_pwm = self.calibration.tilt_degrees_to_pwm(target_tilt_deg)

        self.logger.info(f"Moving from ({current_pan_deg:.1f}°, {current_tilt_deg:.1f}°) "
                         f"to ({target_pan_deg:.1f}°, {target_tilt_deg:.1f}°)")

        # Smooth movement or direct jump
        if goal.smooth:
            # Smooth interpolation: 20 steps over 1 second (50ms per step)
            num_steps = 20
            step_delay = 0.05  # 50ms

            for step in range(num_steps + 1):
                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    self.logger.info("HeadPosition action cancelled")
                    result.success = False
                    result.message = "Cancelled by user"
                    goal_handle.canceled()
                    return result

                # Interpolate position
                t = step / num_steps
                current_pan_pwm = int(current_pan_pwm + (target_pan_pwm - current_pan_pwm) * t)
                current_tilt_pwm = int(current_tilt_pwm + (target_tilt_pwm - current_tilt_pwm) * t)

                # Set servo positions
                self.move.pwm.set_pwm(12, 0, current_pan_pwm)  # Pan servo
                self.move.pwm.set_pwm(13, 0, current_tilt_pwm)  # Tilt servo

                # Update internal state
                self.move.Left_Right_input = current_pan_pwm
                self.move.Up_Down_input = current_tilt_pwm

                # Send feedback
                feedback.current_pan = self.calibration.pwm_to_pan_degrees(current_pan_pwm)
                feedback.current_tilt = self.calibration.pwm_to_tilt_degrees(current_tilt_pwm)
                feedback.progress = t
                goal_handle.publish_feedback(feedback)

                # Wait before next step
                if step < num_steps:
                    time.sleep(step_delay)

        else:
            # Direct jump to target
            self.move.pwm.set_pwm(12, 0, target_pan_pwm)  # Pan servo
            self.move.pwm.set_pwm(13, 0, target_tilt_pwm)  # Tilt servo

            # Update internal state
            self.move.Left_Right_input = target_pan_pwm
            self.move.Up_Down_input = target_tilt_pwm

            # Send feedback
            feedback.current_pan = target_pan_deg
            feedback.current_tilt = target_tilt_deg
            feedback.progress = 1.0
            goal_handle.publish_feedback(feedback)

        # Verify final position
        final_pan_deg = self.calibration.pwm_to_pan_degrees(self.move.Left_Right_input)
        final_tilt_deg = self.calibration.pwm_to_tilt_degrees(self.move.Up_Down_input)

        # Success
        result.final_pan = final_pan_deg
        result.final_tilt = final_tilt_deg
        result.success = True
        result.message = "Head position reached"
        goal_handle.succeed()

        self.logger.info(f"HeadPosition completed: ({final_pan_deg:.1f}°, {final_tilt_deg:.1f}°)")
        return result

    # === LinearMove Action Implementation ===

    async def execute_linear_move(self, goal_handle):
        """
        Execute LinearMove action.

        Moves robot forward/backward for specified distance.
        Uses calibrated step counting to measure distance.
        """
        self.logger.info("Executing LinearMove action...")
        goal = goal_handle.request
        feedback = LinearMove.Feedback()
        result = LinearMove.Result()

        # Check if calibration is available
        if self.calibration is None:
            result.success = False
            result.message = "Calibration not available"
            return result

        # Get parameters
        target_distance = goal.distance_cm
        speed = goal.speed
        step_size_cm = goal.step_size_cm

        # Calculate steps needed
        cm_per_step = self.calibration.get_cm_per_step(speed)
        total_steps = max(1, int(abs(target_distance) / cm_per_step))

        self.logger.info(f"Moving {target_distance:.1f}cm at speed {speed:.1f} "
                         f"({total_steps} steps, {cm_per_step:.2f}cm/step)")

        # Determine direction
        if target_distance > 0:
            direction = 'forward'
        else:
            direction = 'backward'

        # Set movement speed
        self.move.set_movement_speed(int(speed))

        # Start movement
        self.move.commandInput(direction)

        # Track progress
        steps_completed = 0
        last_phase = self.move.gait_phase
        distance_traveled = 0.0
        last_feedback_distance = 0.0

        # Movement loop
        while steps_completed < total_steps:
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.move.commandInput('stand')
                self.logger.info("LinearMove action cancelled")
                result.distance_traveled = distance_traveled
                result.steps_taken = steps_completed
                result.success = False
                result.message = "Cancelled by user"
                goal_handle.canceled()
                return result

            # Poll gait phase (10ms interval)
            time.sleep(0.01)
            current_phase = self.move.gait_phase

            # Detect cycle wrap (1.0 -> 0.0 = one complete gait cycle)
            if last_phase > 0.8 and current_phase < 0.2:
                steps_completed += 1
                distance_traveled = steps_completed * cm_per_step

                # Send feedback at step_size_cm intervals
                if distance_traveled - last_feedback_distance >= step_size_cm:
                    feedback.distance_traveled = distance_traveled
                    feedback.progress = steps_completed / total_steps
                    feedback.steps_completed = steps_completed
                    goal_handle.publish_feedback(feedback)
                    last_feedback_distance = distance_traveled

                self.logger.debug(f"Step {steps_completed}/{total_steps}, "
                                  f"distance: {distance_traveled:.1f}cm")

            last_phase = current_phase

        # Stop movement
        self.move.commandInput('stand')
        self.logger.info("LinearMove completed, stopping...")

        # Final result
        result.distance_traveled = distance_traveled
        result.final_speed = speed
        result.steps_taken = steps_completed
        result.success = True
        result.message = f"Moved {distance_traveled:.1f}cm in {steps_completed} steps"
        goal_handle.succeed()

        self.logger.info(f"LinearMove completed: {distance_traveled:.1f}cm")
        return result

    # === Rotate Action Implementation ===

    async def execute_rotate(self, goal_handle):
        """
        Execute Rotate action.

        Rotates robot in place for specified angle.
        Primary: Calibrated step counting
        Secondary: MPU6050 IMU for error detection (if use_imu=True)
        """
        self.logger.info("Executing Rotate action...")
        goal = goal_handle.request
        feedback = Rotate.Feedback()
        result = Rotate.Result()

        # Check if calibration is available
        if self.calibration is None:
            result.success = False
            result.message = "Calibration not available"
            return result

        # Get parameters
        target_angle = goal.angle_degrees
        speed = goal.speed
        step_size_deg = goal.step_size_deg
        use_imu = goal.use_imu

        # Calculate steps needed
        deg_per_step = self.calibration.get_degrees_per_step(speed)
        total_steps = max(1, int(abs(target_angle) / deg_per_step))

        self.logger.info(f"Rotating {target_angle:.1f}° at speed {speed:.1f} "
                         f"({total_steps} steps, {deg_per_step:.2f}°/step)")

        # Determine direction
        if target_angle > 0:
            direction = 'right'
        else:
            direction = 'left'

        # IMU setup (if requested and available)
        imu_angle = 0.0
        imu_available = use_imu and self.move.mpu6050_connection
        if use_imu and not imu_available:
            self.logger.warn("IMU requested but not available")

        # Set movement speed
        self.move.set_movement_speed(int(speed))

        # Start rotation
        self.move.commandInput(direction)

        # Track progress
        steps_completed = 0
        last_phase = self.move.gait_phase
        angle_rotated = 0.0
        last_feedback_angle = 0.0
        last_imu_time = time.time()

        # Movement loop
        while steps_completed < total_steps:
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.move.commandInput('no')
                self.logger.info("Rotate action cancelled")
                result.angle_rotated = angle_rotated
                result.imu_angle = imu_angle
                result.angle_error = abs(target_angle - angle_rotated)
                result.steps_taken = steps_completed
                result.success = False
                result.message = "Cancelled by user"
                goal_handle.canceled()
                return result

            # Poll gait phase (10ms interval)
            time.sleep(0.01)
            current_phase = self.move.gait_phase

            # Update IMU if available
            if imu_available:
                current_time = time.time()
                dt = current_time - last_imu_time
                last_imu_time = current_time

                # Read gyro data and integrate
                try:
                    mpu_data = self.move.get_mpu6050_data()
                    if mpu_data and 'gyro' in mpu_data:
                        gyro_z = mpu_data['gyro'].get('z', 0.0)
                        # Integrate angular velocity
                        imu_angle += gyro_z * dt
                except Exception as e:
                    self.logger.warn(f"IMU read error: {e}")

            # Detect cycle wrap (1.0 -> 0.0 = one complete gait cycle)
            if last_phase > 0.8 and current_phase < 0.2:
                steps_completed += 1
                angle_rotated = steps_completed * deg_per_step

                # Send feedback at step_size_deg intervals
                if abs(angle_rotated - last_feedback_angle) >= step_size_deg:
                    feedback.angle_rotated = angle_rotated
                    feedback.imu_angle = imu_angle if imu_available else 0.0
                    feedback.progress = steps_completed / total_steps
                    feedback.steps_completed = steps_completed
                    goal_handle.publish_feedback(feedback)
                    last_feedback_angle = angle_rotated

                self.logger.debug(f"Step {steps_completed}/{total_steps}, "
                                  f"angle: {angle_rotated:.1f}° (IMU: {imu_angle:.1f}°)")

            last_phase = current_phase

        # Stop rotation
        self.move.commandInput('no')
        self.logger.info("Rotate completed, stopping...")

        # Calculate error
        angle_error = abs(target_angle - angle_rotated)
        if imu_available:
            imu_error = abs(target_angle - imu_angle)
            self.logger.info(f"Step counting error: {angle_error:.1f}°, IMU error: {imu_error:.1f}°")

        # Final result
        result.angle_rotated = angle_rotated
        result.imu_angle = imu_angle if imu_available else 0.0
        result.angle_error = angle_error
        result.steps_taken = steps_completed
        result.success = True
        result.message = f"Rotated {angle_rotated:.1f}° in {steps_completed} steps"
        goal_handle.succeed()

        self.logger.info(f"Rotate completed: {angle_rotated:.1f}° (error: {angle_error:.1f}°)")
        return result

    # === ArcMove Action Implementation ===

    async def execute_arc_move(self, goal_handle):
        """
        Execute ArcMove action.

        Moves robot in arc trajectory with specified curvature.
        Similar to LinearMove but uses arc_factor for curved path.
        """
        self.logger.info("Executing ArcMove action...")
        goal = goal_handle.request
        feedback = ArcMove.Feedback()
        result = ArcMove.Result()

        # Check if calibration is available
        if self.calibration is None:
            result.success = False
            result.message = "Calibration not available"
            return result

        # Get parameters
        target_distance = goal.distance_cm
        arc_factor = goal.arc_factor
        speed = goal.speed
        step_size_cm = goal.step_size_cm

        # Calculate steps needed (use linear calibration for arc distance)
        cm_per_step = self.calibration.get_cm_per_step(speed)
        total_steps = max(1, int(abs(target_distance) / cm_per_step))

        self.logger.info(f"Moving {target_distance:.1f}cm in arc (factor={arc_factor:.2f}) "
                         f"at speed {speed:.1f} ({total_steps} steps)")

        # Determine direction based on arc_factor sign
        if target_distance > 0:
            if arc_factor >= 0:
                direction = 'forward_right_arc'
            else:
                direction = 'forward_left_arc'
        else:
            if arc_factor >= 0:
                direction = 'backward_right_arc'
            else:
                direction = 'backward_left_arc'

        # Set arc factor (use absolute value)
        self.move.set_arc_factor(abs(arc_factor))

        # Set movement speed
        self.move.set_movement_speed(int(speed))

        # Start movement
        self.move.commandInput(direction)

        # Track progress
        steps_completed = 0
        last_phase = self.move.gait_phase
        distance_traveled = 0.0
        last_feedback_distance = 0.0

        # Movement loop (identical to LinearMove)
        while steps_completed < total_steps:
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.move.commandInput('stand')
                self.move.set_arc_factor(0.7)  # Reset to default
                self.logger.info("ArcMove action cancelled")
                result.distance_traveled = distance_traveled
                result.arc_factor_used = arc_factor
                result.steps_taken = steps_completed
                result.success = False
                result.message = "Cancelled by user"
                goal_handle.canceled()
                return result

            # Poll gait phase (10ms interval)
            time.sleep(0.01)
            current_phase = self.move.gait_phase

            # Detect cycle wrap (1.0 -> 0.0 = one complete gait cycle)
            if last_phase > 0.8 and current_phase < 0.2:
                steps_completed += 1
                distance_traveled = steps_completed * cm_per_step

                # Send feedback at step_size_cm intervals
                if distance_traveled - last_feedback_distance >= step_size_cm:
                    feedback.distance_traveled = distance_traveled
                    feedback.progress = steps_completed / total_steps
                    feedback.steps_completed = steps_completed
                    goal_handle.publish_feedback(feedback)
                    last_feedback_distance = distance_traveled

                self.logger.debug(f"Step {steps_completed}/{total_steps}, "
                                  f"distance: {distance_traveled:.1f}cm")

            last_phase = current_phase

        # Stop movement and reset arc factor
        self.move.commandInput('stand')
        self.move.set_arc_factor(0.7)  # Reset to default
        self.logger.info("ArcMove completed, stopping...")

        # Final result
        result.distance_traveled = distance_traveled
        result.arc_factor_used = arc_factor
        result.steps_taken = steps_completed
        result.success = True
        result.message = f"Moved {distance_traveled:.1f}cm in arc (factor={arc_factor:.2f})"
        goal_handle.succeed()

        self.logger.info(f"ArcMove completed: {distance_traveled:.1f}cm")
        return result


# === Standalone Test ===

def main():
    """Standalone test for action servers."""
    print("This module should be imported by ROSServer.py")
    print("Not intended for standalone execution.")


if __name__ == '__main__':
    main()
