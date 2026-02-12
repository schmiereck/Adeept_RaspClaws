#!/usr/bin/env python3
"""
Interactive Robot Calibration Script

This script helps calibrate the RaspClaws robot by running controlled
movements and recording the measured distances/angles.

Usage:
    python3 calibrate_robot.py

Requirements:
    - Robot hardware connected and functional
    - Measuring tape or ruler for distance measurements
    - Protractor or angle markings for rotation measurements

Author: RaspClaws Actions System
Date: 2026-02-12
"""

import os
import sys
import time
import yaml
from datetime import datetime

# Import robot hardware control
try:
    from Move import Move
    HARDWARE_AVAILABLE = True
except ImportError:
    print("WARNING: Move.py not available. Running in simulation mode.")
    HARDWARE_AVAILABLE = False


class RobotCalibrator:
    """Interactive calibration tool for RaspClaws robot."""

    def __init__(self):
        """Initialize calibrator."""
        self.calibration_data = {
            'linear_calibration': {},
            'rotation_calibration': {},
            'camera_calibration': {
                'pan_pwm_center': 300,
                'pan_pwm_min': 100,
                'pan_pwm_max': 500,
                'pan_deg_min': -100.0,
                'pan_deg_max': 100.0,
                'tilt_pwm_center': 365,
                'tilt_pwm_min': 230,
                'tilt_pwm_max': 500,
                'tilt_deg_min': -67.0,
                'tilt_deg_max': 67.0,
            },
            'calibration_info': {
                'last_calibrated': datetime.now().strftime('%Y-%m-%d'),
                'surface_type': 'unknown',
                'notes': ''
            }
        }

        if HARDWARE_AVAILABLE:
            self.move = Move()
            print("Hardware initialized successfully!")
        else:
            self.move = None
            print("Running in SIMULATION mode (no hardware)")

    def wait_for_user(self, message: str = "Press ENTER to continue..."):
        """Wait for user input."""
        input(f"\n{message}")

    def get_float_input(self, prompt: str, default: float = None) -> float:
        """Get validated float input from user."""
        while True:
            try:
                if default is not None:
                    user_input = input(f"{prompt} [default: {default}]: ").strip()
                    if not user_input:
                        return default
                else:
                    user_input = input(f"{prompt}: ").strip()

                return float(user_input)
            except ValueError:
                print("Invalid input. Please enter a number.")

    def calibrate_linear_movement(self):
        """Calibrate linear movement at different speeds."""
        print("\n" + "=" * 60)
        print("LINEAR MOVEMENT CALIBRATION")
        print("=" * 60)
        print("\nThis will test forward movement at different speeds.")
        print("You will need to measure the distance traveled with a tape measure.")
        print("\nFor each speed level, the robot will:")
        print("  1. Move forward for 10 gait cycles")
        print("  2. You measure the total distance traveled")
        print("  3. We calculate cm per gait cycle")

        self.wait_for_user("Prepare measuring tape and press ENTER to start...")

        speeds = [10, 20, 30, 40, 50, 60, 70, 80]
        cycles_per_test = 10

        for speed in speeds:
            print(f"\n--- Testing Speed {speed} ---")

            if HARDWARE_AVAILABLE:
                print(f"Robot will move forward at speed {speed} for {cycles_per_test} cycles...")
                print("Mark the starting position NOW!")
                self.wait_for_user("Press ENTER when ready to start movement...")

                # Set speed and move forward
                self.move.set_movement_speed(speed)
                self.move.commandInput('forward')

                # Count cycles
                last_phase = self.move.gait_phase
                cycle_count = 0

                while cycle_count < cycles_per_test:
                    time.sleep(0.01)  # 10ms polling
                    current_phase = self.move.gait_phase

                    # Detect cycle wrap (1.0 -> 0.0)
                    if last_phase > 0.8 and current_phase < 0.2:
                        cycle_count += 1
                        print(f"  Cycle {cycle_count}/{cycles_per_test} completed")

                    last_phase = current_phase

                # Stop
                self.move.commandInput('stand')
                print("Movement complete!")

            else:
                print(f"[SIMULATION] Robot would move at speed {speed} for {cycles_per_test} cycles")

            # Get measurement from user
            distance_cm = self.get_float_input(
                f"Measure total distance traveled (cm)",
                default=None
            )

            # Calculate cm per cycle
            cm_per_cycle = distance_cm / cycles_per_test
            self.calibration_data['linear_calibration'][f'speed_{speed}'] = round(cm_per_cycle, 2)

            print(f"✓ Recorded: {cm_per_cycle:.2f} cm/cycle at speed {speed}")

        print("\n✓ Linear movement calibration complete!")

    def calibrate_rotation(self):
        """Calibrate rotation at different speeds."""
        print("\n" + "=" * 60)
        print("ROTATION CALIBRATION")
        print("=" * 60)
        print("\nThis will test rotation at different speeds.")
        print("You will need to measure the angle rotated.")
        print("\nFor each speed level, the robot will:")
        print("  1. Rotate right for 5 gait cycles")
        print("  2. You measure the total angle rotated")
        print("  3. We calculate degrees per gait cycle")
        print("\nTip: Place a piece of tape on the robot pointing forward,")
        print("     and mark angles on the ground (0°, 45°, 90°, etc.)")

        self.wait_for_user("Prepare angle markings and press ENTER to start...")

        speeds = [10, 20, 30, 40, 50, 60, 70, 80]
        cycles_per_test = 5

        for speed in speeds:
            print(f"\n--- Testing Speed {speed} ---")

            if HARDWARE_AVAILABLE:
                print(f"Robot will rotate right at speed {speed} for {cycles_per_test} cycles...")
                print("Note the starting angle NOW!")
                self.wait_for_user("Press ENTER when ready to start rotation...")

                # Set speed and rotate right
                self.move.set_movement_speed(speed)
                self.move.commandInput('right')

                # Count cycles
                last_phase = self.move.gait_phase
                cycle_count = 0

                while cycle_count < cycles_per_test:
                    time.sleep(0.01)  # 10ms polling
                    current_phase = self.move.gait_phase

                    # Detect cycle wrap (1.0 -> 0.0)
                    if last_phase > 0.8 and current_phase < 0.2:
                        cycle_count += 1
                        print(f"  Cycle {cycle_count}/{cycles_per_test} completed")

                    last_phase = current_phase

                # Stop
                self.move.commandInput('no')
                print("Rotation complete!")

            else:
                print(f"[SIMULATION] Robot would rotate at speed {speed} for {cycles_per_test} cycles")

            # Get measurement from user
            angle_deg = self.get_float_input(
                f"Measure total angle rotated (degrees)",
                default=None
            )

            # Calculate degrees per cycle
            deg_per_cycle = angle_deg / cycles_per_test
            self.calibration_data['rotation_calibration'][f'speed_{speed}'] = round(deg_per_cycle, 2)

            print(f"✓ Recorded: {deg_per_cycle:.2f} deg/cycle at speed {speed}")

        print("\n✓ Rotation calibration complete!")

    def collect_metadata(self):
        """Collect calibration metadata from user."""
        print("\n" + "=" * 60)
        print("CALIBRATION METADATA")
        print("=" * 60)

        # Surface type
        print("\nWhat surface was used for calibration?")
        print("  1. Teppich (carpet)")
        print("  2. Hartboden (hard floor)")
        print("  3. Mixed")
        surface_choice = input("Enter choice (1-3) [default: 3]: ").strip()
        surface_map = {'1': 'teppich', '2': 'hartboden', '3': 'mixed'}
        self.calibration_data['calibration_info']['surface_type'] = surface_map.get(surface_choice, 'mixed')

        # Notes
        notes = input("\nAny additional notes (optional): ").strip()
        if notes:
            self.calibration_data['calibration_info']['notes'] = notes
        else:
            self.calibration_data['calibration_info']['notes'] = "Calibrated using calibrate_robot.py"

    def save_calibration(self):
        """Save calibration data to YAML file."""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_file = os.path.join(script_dir, 'calibration.yaml')

        print(f"\nSaving calibration to: {output_file}")

        with open(output_file, 'w') as f:
            yaml.dump(self.calibration_data, f, default_flow_style=False, sort_keys=False)

        print("✓ Calibration saved successfully!")

    def print_summary(self):
        """Print calibration summary."""
        print("\n" + "=" * 60)
        print("CALIBRATION SUMMARY")
        print("=" * 60)

        print("\nLinear Movement (cm/cycle):")
        for key in sorted(self.calibration_data['linear_calibration'].keys()):
            speed = key.split('_')[1]
            value = self.calibration_data['linear_calibration'][key]
            print(f"  Speed {speed:>2}: {value:>5.2f} cm/cycle")

        print("\nRotation (deg/cycle):")
        for key in sorted(self.calibration_data['rotation_calibration'].keys()):
            speed = key.split('_')[1]
            value = self.calibration_data['rotation_calibration'][key]
            print(f"  Speed {speed:>2}: {value:>5.2f} deg/cycle")

        print("\nCalibration Info:")
        info = self.calibration_data['calibration_info']
        print(f"  Date: {info['last_calibrated']}")
        print(f"  Surface: {info['surface_type']}")
        print(f"  Notes: {info['notes']}")

        print("=" * 60)

    def run_full_calibration(self):
        """Run complete calibration procedure."""
        print("\n" + "=" * 60)
        print("RASPCLAWS ROBOT CALIBRATION")
        print("=" * 60)
        print("\nThis script will calibrate:")
        print("  1. Linear movement (forward/backward)")
        print("  2. Rotation (left/right)")
        print("\nThe camera calibration uses fixed values from hardware specs.")
        print("\nEstimated time: 20-30 minutes")

        self.wait_for_user("Press ENTER to begin calibration...")

        # Run calibrations
        self.calibrate_linear_movement()
        self.calibrate_rotation()
        self.collect_metadata()

        # Show summary
        self.print_summary()

        # Save
        save = input("\nSave calibration data? (y/n) [default: y]: ").strip().lower()
        if save in ['', 'y', 'yes']:
            self.save_calibration()
            print("\n✓ Calibration complete! You can now use action servers.")
        else:
            print("\nCalibration NOT saved.")

    def quick_test(self):
        """Quick test mode for validation."""
        print("\n" + "=" * 60)
        print("QUICK TEST MODE")
        print("=" * 60)

        if not HARDWARE_AVAILABLE:
            print("Hardware not available. Cannot run quick test.")
            return

        print("\nThis will perform a quick movement test:")
        print("  - Move forward 20cm at speed 35")
        print("  - Rotate 90° right at speed 35")

        self.wait_for_user("Press ENTER to start test...")

        # Test linear movement
        print("\n1. Moving forward...")
        self.move.set_movement_speed(35)
        self.move.commandInput('forward')
        time.sleep(3)  # ~3 seconds
        self.move.commandInput('stand')
        print("   Measure distance traveled!")

        time.sleep(2)

        # Test rotation
        print("\n2. Rotating right...")
        self.move.set_movement_speed(35)
        self.move.commandInput('right')
        time.sleep(2)  # ~2 seconds
        self.move.commandInput('no')
        print("   Measure angle rotated!")

        print("\n✓ Quick test complete!")


def main():
    """Main entry point."""
    print("RaspClaws Robot Calibration Tool")
    print("-" * 60)

    calibrator = RobotCalibrator()

    if len(sys.argv) > 1 and sys.argv[1] == '--test':
        calibrator.quick_test()
    else:
        calibrator.run_full_calibration()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nCalibration interrupted by user.")
        sys.exit(0)
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
