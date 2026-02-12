#!/usr/bin/env python3
"""
Robot Calibration Data Management

Handles loading and interpolation of calibration data for:
- Linear movement (distance per gait cycle at different speeds)
- Rotation (angle per gait cycle at different speeds)
- Camera servo PWM to degree conversion

Author: RaspClaws Actions System
Date: 2026-02-12
"""

import os
import yaml
import numpy as np
from scipy.interpolate import interp1d
from typing import Dict, Tuple


class RobotCalibration:
    """Manages robot calibration data and provides interpolation functions."""

    def __init__(self, calibration_file: str = None):
        """
        Initialize calibration manager.

        Args:
            calibration_file: Path to calibration.yaml. If None, uses default location.
        """
        if calibration_file is None:
            # Default location: same directory as this script
            script_dir = os.path.dirname(os.path.abspath(__file__))
            calibration_file = os.path.join(script_dir, 'calibration.yaml')

        self.calibration_file = calibration_file
        self.data = self.load_calibration()

        # Create interpolation functions
        self._create_interpolators()

    def load_calibration(self) -> Dict:
        """
        Load calibration data from YAML file.

        Returns:
            Dictionary with calibration data

        Raises:
            FileNotFoundError: If calibration file doesn't exist
            ValueError: If calibration data is invalid
        """
        if not os.path.exists(self.calibration_file):
            raise FileNotFoundError(
                f"Calibration file not found: {self.calibration_file}\n"
                f"Please run calibrate_robot.py first!"
            )

        with open(self.calibration_file, 'r') as f:
            data = yaml.safe_load(f)

        # Validate required sections
        required_sections = ['linear_calibration', 'rotation_calibration', 'camera_calibration']
        for section in required_sections:
            if section not in data:
                raise ValueError(f"Missing required calibration section: {section}")

        return data

    def _create_interpolators(self):
        """Create scipy interpolation functions for smooth speed interpolation."""
        # Linear movement interpolator
        linear_cal = self.data['linear_calibration']
        speeds = [int(k.split('_')[1]) for k in linear_cal.keys()]
        distances = [linear_cal[f'speed_{s}'] for s in speeds]
        self._linear_interpolator = interp1d(
            speeds, distances,
            kind='linear',
            bounds_error=False,
            fill_value=(distances[0], distances[-1])  # Clamp to edges
        )

        # Rotation interpolator
        rotation_cal = self.data['rotation_calibration']
        speeds = [int(k.split('_')[1]) for k in rotation_cal.keys()]
        angles = [rotation_cal[f'speed_{s}'] for s in speeds]
        self._rotation_interpolator = interp1d(
            speeds, angles,
            kind='linear',
            bounds_error=False,
            fill_value=(angles[0], angles[-1])  # Clamp to edges
        )

    # === Linear Movement ===

    def get_cm_per_step(self, speed: float) -> float:
        """
        Get distance traveled per gait cycle for given speed.

        Args:
            speed: Movement speed (10.0 - 80.0)

        Returns:
            Distance in centimeters per gait cycle
        """
        # Clamp speed to valid range
        speed = max(10.0, min(80.0, speed))
        return float(self._linear_interpolator(speed))

    def get_steps_for_distance(self, distance_cm: float, speed: float) -> int:
        """
        Calculate number of gait cycles needed for target distance.

        Args:
            distance_cm: Target distance in centimeters
            speed: Movement speed (10.0 - 80.0)

        Returns:
            Number of gait cycles needed
        """
        cm_per_step = self.get_cm_per_step(speed)
        return max(1, int(abs(distance_cm) / cm_per_step))

    # === Rotation ===

    def get_degrees_per_step(self, speed: float) -> float:
        """
        Get angle rotated per gait cycle for given speed.

        Args:
            speed: Rotation speed (10.0 - 80.0)

        Returns:
            Angle in degrees per gait cycle
        """
        # Clamp speed to valid range
        speed = max(10.0, min(80.0, speed))
        return float(self._rotation_interpolator(speed))

    def get_steps_for_rotation(self, angle_degrees: float, speed: float) -> int:
        """
        Calculate number of gait cycles needed for target rotation.

        Args:
            angle_degrees: Target angle in degrees
            speed: Rotation speed (10.0 - 80.0)

        Returns:
            Number of gait cycles needed
        """
        deg_per_step = self.get_degrees_per_step(speed)
        return max(1, int(abs(angle_degrees) / deg_per_step))

    # === Camera Servo Conversion ===

    def pan_degrees_to_pwm(self, degrees: float) -> int:
        """
        Convert pan angle to PWM duty cycle.

        Args:
            degrees: Pan angle (-100.0 to +100.0)

        Returns:
            PWM duty cycle value
        """
        cam_cal = self.data['camera_calibration']

        # Clamp to valid range
        degrees = max(cam_cal['pan_deg_min'], min(cam_cal['pan_deg_max'], degrees))

        # Linear mapping: degrees -> PWM
        deg_range = cam_cal['pan_deg_max'] - cam_cal['pan_deg_min']
        pwm_range = cam_cal['pan_pwm_max'] - cam_cal['pan_pwm_min']

        # Normalize degrees to 0-1 range
        normalized = (degrees - cam_cal['pan_deg_min']) / deg_range

        # Map to PWM range
        pwm = cam_cal['pan_pwm_min'] + (normalized * pwm_range)

        return int(pwm)

    def pwm_to_pan_degrees(self, pwm: int) -> float:
        """
        Convert PWM duty cycle to pan angle.

        Args:
            pwm: PWM duty cycle value

        Returns:
            Pan angle in degrees
        """
        cam_cal = self.data['camera_calibration']

        # Clamp to valid range
        pwm = max(cam_cal['pan_pwm_min'], min(cam_cal['pan_pwm_max'], pwm))

        # Linear mapping: PWM -> degrees
        pwm_range = cam_cal['pan_pwm_max'] - cam_cal['pan_pwm_min']
        deg_range = cam_cal['pan_deg_max'] - cam_cal['pan_deg_min']

        # Normalize PWM to 0-1 range
        normalized = (pwm - cam_cal['pan_pwm_min']) / pwm_range

        # Map to degree range
        degrees = cam_cal['pan_deg_min'] + (normalized * deg_range)

        return float(degrees)

    def tilt_degrees_to_pwm(self, degrees: float) -> int:
        """
        Convert tilt angle to PWM duty cycle.

        Args:
            degrees: Tilt angle (-67.0 to +67.0)

        Returns:
            PWM duty cycle value
        """
        cam_cal = self.data['camera_calibration']

        # Clamp to valid range
        degrees = max(cam_cal['tilt_deg_min'], min(cam_cal['tilt_deg_max'], degrees))

        # Linear mapping: degrees -> PWM
        deg_range = cam_cal['tilt_deg_max'] - cam_cal['tilt_deg_min']
        pwm_range = cam_cal['tilt_pwm_max'] - cam_cal['tilt_pwm_min']

        # Normalize degrees to 0-1 range
        normalized = (degrees - cam_cal['tilt_deg_min']) / deg_range

        # Map to PWM range
        pwm = cam_cal['tilt_pwm_min'] + (normalized * pwm_range)

        return int(pwm)

    def pwm_to_tilt_degrees(self, pwm: int) -> float:
        """
        Convert PWM duty cycle to tilt angle.

        Args:
            pwm: PWM duty cycle value

        Returns:
            Tilt angle in degrees
        """
        cam_cal = self.data['camera_calibration']

        # Clamp to valid range
        pwm = max(cam_cal['tilt_pwm_min'], min(cam_cal['tilt_pwm_max'], pwm))

        # Linear mapping: PWM -> degrees
        pwm_range = cam_cal['tilt_pwm_max'] - cam_cal['tilt_pwm_min']
        deg_range = cam_cal['tilt_deg_max'] - cam_cal['tilt_deg_min']

        # Normalize PWM to 0-1 range
        normalized = (pwm - cam_cal['tilt_pwm_min']) / pwm_range

        # Map to degree range
        degrees = cam_cal['tilt_deg_min'] + (normalized * deg_range)

        return float(degrees)

    def get_camera_center(self) -> Tuple[int, int]:
        """
        Get PWM values for camera center position (0°, 0°).

        Returns:
            Tuple of (pan_pwm, tilt_pwm)
        """
        cam_cal = self.data['camera_calibration']
        return (cam_cal['pan_pwm_center'], cam_cal['tilt_pwm_center'])

    # === Utility ===

    def print_calibration_summary(self):
        """Print a summary of loaded calibration data."""
        print("=" * 60)
        print("ROBOT CALIBRATION SUMMARY")
        print("=" * 60)

        print("\nLinear Movement (cm per gait cycle):")
        for speed in [10, 20, 30, 40, 50, 60, 70, 80]:
            cm = self.get_cm_per_step(speed)
            print(f"  Speed {speed:2d}: {cm:.2f} cm/cycle")

        print("\nRotation (degrees per gait cycle):")
        for speed in [10, 20, 30, 40, 50, 60, 70, 80]:
            deg = self.get_degrees_per_step(speed)
            print(f"  Speed {speed:2d}: {deg:.2f} deg/cycle")

        print("\nCamera Calibration:")
        cam_cal = self.data['camera_calibration']
        print(f"  Pan:  {cam_cal['pan_deg_min']:+6.1f}° to {cam_cal['pan_deg_max']:+6.1f}°  "
              f"(PWM {cam_cal['pan_pwm_min']}-{cam_cal['pan_pwm_max']}, center={cam_cal['pan_pwm_center']})")
        print(f"  Tilt: {cam_cal['tilt_deg_min']:+6.1f}° to {cam_cal['tilt_deg_max']:+6.1f}°  "
              f"(PWM {cam_cal['tilt_pwm_min']}-{cam_cal['tilt_pwm_max']}, center={cam_cal['tilt_pwm_center']})")

        if 'calibration_info' in self.data:
            info = self.data['calibration_info']
            print(f"\nCalibration Info:")
            print(f"  Last calibrated: {info.get('last_calibrated', 'unknown')}")
            print(f"  Surface type: {info.get('surface_type', 'unknown')}")
            if 'notes' in info:
                print(f"  Notes: {info['notes']}")

        print("=" * 60)


# Example usage
if __name__ == '__main__':
    try:
        cal = RobotCalibration()
        cal.print_calibration_summary()

        # Test conversions
        print("\nTest Conversions:")
        print(f"Pan 45° -> PWM {cal.pan_degrees_to_pwm(45.0)}")
        print(f"PWM 400 -> Pan {cal.pwm_to_pan_degrees(400):.1f}°")
        print(f"Speed 35.0 -> {cal.get_cm_per_step(35.0):.2f} cm/cycle")
        print(f"Speed 35.0 -> {cal.get_degrees_per_step(35.0):.2f} deg/cycle")

    except FileNotFoundError as e:
        print(f"ERROR: {e}")
