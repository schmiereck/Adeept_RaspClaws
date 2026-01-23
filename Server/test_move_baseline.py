"""
Unit Tests für Move.py - Baseline Tests für FT40 Implementation
Diese Tests sichern den aktuellen Zustand ab.

Datum: 2026-01-23
Zweck: Regression-Tests während FT40 Verbesserungen

WICHTIG: Diese Tests laufen auf Windows ohne Raspberry Pi Hardware!
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Mock all hardware modules BEFORE importing Move
sys.modules['mpu6050'] = MagicMock()
sys.modules['Adafruit_PCA9685'] = MagicMock()
sys.modules['RPi'] = MagicMock()
sys.modules['RPi.GPIO'] = MagicMock()
sys.modules['picamera'] = MagicMock()
sys.modules['smbus'] = MagicMock()

# Now we can safely import Move
import Move


class TestMoveBaseline(unittest.TestCase):
    """Baseline tests für die aktuelle Move.py Implementierung"""

    def setUp(self):
        """Setup vor jedem Test"""
        # Reset global state
        Move.gait_phase = 0.0
        Move._leg_positions = {
            'L1': 0, 'L2': 0, 'L3': 0,
            'R1': 0, 'R2': 0, 'R3': 0
        }
        Move._last_command = None
        Move._last_speed_sign = 0
        Move._steps_since_change = 0
        Move._direction_changed = False
        Move._stop_counter = 0
        Move.direction_command = Move.MOVE_NO
        Move.turn_command = Move.MOVE_NO
        Move.movement_speed = 35
        Move.steadyMode = 0
        Move.abort_current_movement = False

    # ==================== Test: Leg Positions Dictionary ====================

    def test_leg_positions_exist(self):
        """Test: _leg_positions Dictionary existiert und hat korrekte Struktur"""
        self.assertIsInstance(Move._leg_positions, dict)
        self.assertEqual(len(Move._leg_positions), 6)

        expected_keys = ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']
        for key in expected_keys:
            self.assertIn(key, Move._leg_positions)
            self.assertIsInstance(Move._leg_positions[key], int)

    def test_leg_positions_initial_zero(self):
        """Test: _leg_positions sind initial alle 0"""
        for leg, pos in Move._leg_positions.items():
            self.assertEqual(pos, 0, f"Leg {leg} should start at position 0")

    # ==================== Test: Calculate Target Positions ====================

    def test_calculate_target_positions_exists(self):
        """Test: calculate_target_positions Funktion existiert"""
        self.assertTrue(hasattr(Move, 'calculate_target_positions'))
        self.assertTrue(callable(Move.calculate_target_positions))

    def test_calculate_target_positions_forward(self):
        """Test: calculate_target_positions für Forward-Bewegung"""
        # Forward: speed_left = speed_right = negative
        positions = Move.calculate_target_positions(
            phase=0.0,
            speed_left=-35,
            speed_right=-35
        )

        # Check structure
        self.assertEqual(len(positions), 6)
        for leg in ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']:
            self.assertIn(leg, positions)
            self.assertIn('h', positions[leg])
            self.assertIn('v', positions[leg])
            self.assertIsInstance(positions[leg]['h'], int)
            self.assertIsInstance(positions[leg]['v'], int)

    def test_calculate_target_positions_backward(self):
        """Test: calculate_target_positions für Backward-Bewegung"""
        # Backward: speed_left = speed_right = positive
        positions = Move.calculate_target_positions(
            phase=0.0,
            speed_left=35,
            speed_right=35
        )

        self.assertEqual(len(positions), 6)

    def test_calculate_target_positions_turn_left(self):
        """Test: calculate_target_positions für Links-Drehung"""
        # Turn left: speed_left = positive, speed_right = negative
        positions = Move.calculate_target_positions(
            phase=0.5,
            speed_left=35,
            speed_right=-35
        )

        self.assertEqual(len(positions), 6)

    # ==================== Test: Apply Leg Position ====================

    def test_apply_leg_position_exists(self):
        """Test: apply_leg_position Funktion existiert"""
        self.assertTrue(hasattr(Move, 'apply_leg_position'))
        self.assertTrue(callable(Move.apply_leg_position))

    # ==================== Test: Gait Phase ====================

    def test_gait_phase_initial_zero(self):
        """Test: gait_phase ist initial 0.0"""
        self.assertEqual(Move.gait_phase, 0.0)

    # ==================== Test: Tracking Variables ====================

    def test_last_command_exists(self):
        """Test: _last_command Variable existiert"""
        self.assertTrue(hasattr(Move, '_last_command'))

    def test_last_speed_sign_exists(self):
        """Test: _last_speed_sign Variable existiert"""
        self.assertTrue(hasattr(Move, '_last_speed_sign'))

    def test_last_command_initial_none(self):
        """Test: _last_command ist initial None"""
        self.assertIsNone(Move._last_command)

    def test_last_speed_sign_initial_zero(self):
        """Test: _last_speed_sign ist initial 0"""
        self.assertEqual(Move._last_speed_sign, 0)


if __name__ == '__main__':
    # Run tests with verbose output
    unittest.main(verbosity=2)


    # ==================== Test: Calculate Target Positions ====================

    def test_calculate_target_positions_exists(self):
        """Test: calculate_target_positions Funktion existiert"""
        self.assertTrue(hasattr(self.Move, 'calculate_target_positions'))
        self.assertTrue(callable(self.Move.calculate_target_positions))

    def test_calculate_target_positions_forward(self):
        """Test: calculate_target_positions für Forward-Bewegung"""
        # Forward: speed_left = speed_right = negative
        positions = self.Move.calculate_target_positions(
            phase=0.0,
            speed_left=-35,
            speed_right=-35
        )

        # Check structure
        self.assertEqual(len(positions), 6)
        for leg in ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']:
            self.assertIn(leg, positions)
            self.assertIn('h', positions[leg])
            self.assertIn('v', positions[leg])
            self.assertIsInstance(positions[leg]['h'], int)
            self.assertIsInstance(positions[leg]['v'], int)

    def test_calculate_target_positions_backward(self):
        """Test: calculate_target_positions für Backward-Bewegung"""
        # Backward: speed_left = speed_right = positive
        positions = self.Move.calculate_target_positions(
            phase=0.0,
            speed_left=35,
            speed_right=35
        )

        # Check that positions are calculated
        self.assertEqual(len(positions), 6)
        # At phase 0.0, group_a should be in air, group_b on ground
        # For backward (positive speed), horizontal should be positive for air group

    def test_calculate_target_positions_turn_left(self):
        """Test: calculate_target_positions für Links-Drehung"""
        # Turn left: speed_left = positive, speed_right = negative
        positions = self.Move.calculate_target_positions(
            phase=0.5,
            speed_left=35,
            speed_right=-35
        )

        self.assertEqual(len(positions), 6)
        # Left and right legs should have opposite horizontal movements

    # ==================== Test: Apply Leg Position ====================

    def test_apply_leg_position_exists(self):
        """Test: apply_leg_position Funktion existiert"""
        self.assertTrue(hasattr(self.Move, 'apply_leg_position'))
        self.assertTrue(callable(self.Move.apply_leg_position))

    @patch('Move.dove_Left_I')
    def test_apply_leg_position_l1(self, mock_dove):
        """Test: apply_leg_position ruft korrekte Funktion für L1 auf"""
        self.Move.apply_leg_position('L1', 10, 5)
        mock_dove.assert_called_once_with(10, 5)

    @patch('Move.dove_Right_III')
    def test_apply_leg_position_r3(self, mock_dove):
        """Test: apply_leg_position ruft korrekte Funktion für R3 auf"""
        self.Move.apply_leg_position('R3', -15, 8)
        mock_dove.assert_called_once_with(-15, 8)

    # ==================== Test: Gait Phase ====================

    def test_gait_phase_initial_zero(self):
        """Test: gait_phase ist initial 0.0"""
        self.assertEqual(self.Move.gait_phase, 0.0)

    def test_gait_phase_advances(self):
        """Test: gait_phase wird in move_thread erhöht"""
        # Setup for forward movement
        self.Move.direction_command = self.Move.CMD_FORWARD
        self.Move.turn_command = self.Move.MOVE_NO

        initial_phase = self.Move.gait_phase

        # Call move_thread once
        with patch('Move.calculate_target_positions') as mock_calc, \
             patch('Move.apply_leg_position') as mock_apply:
            mock_calc.return_value = {
                leg: {'h': 0, 'v': 0} for leg in ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']
            }
            self.Move.move_thread()

        # Phase should have advanced
        self.assertGreater(self.Move.gait_phase, initial_phase)

    def test_gait_phase_wraps_around(self):
        """Test: gait_phase wraps von 1.0 zurück zu 0.0"""
        self.Move.gait_phase = 0.99
        self.Move.direction_command = self.Move.CMD_FORWARD
        self.Move.turn_command = self.Move.MOVE_NO

        with patch('Move.calculate_target_positions') as mock_calc, \
             patch('Move.apply_leg_position') as mock_apply:
            mock_calc.return_value = {
                leg: {'h': 0, 'v': 0} for leg in ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']
            }
            self.Move.move_thread()

        # Phase should have wrapped
        self.assertLess(self.Move.gait_phase, 0.5)

    # ==================== Test: Movement Commands ====================

    def test_forward_command_sets_speed(self):
        """Test: Forward-Kommando setzt korrekte Geschwindigkeiten"""
        self.Move.direction_command = self.Move.CMD_FORWARD
        self.Move.turn_command = self.Move.MOVE_NO
        self.Move.movement_speed = 35

        with patch('Move.calculate_target_positions') as mock_calc, \
             patch('Move.apply_leg_position') as mock_apply:
            mock_calc.return_value = {
                leg: {'h': 0, 'v': 0} for leg in ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']
            }
            self.Move.move_thread()

            # Should have been called with negative speeds for forward
            args = mock_calc.call_args[0]
            self.assertEqual(args[1], -35)  # speed_left
            self.assertEqual(args[2], -35)  # speed_right

    def test_backward_command_sets_speed(self):
        """Test: Backward-Kommando setzt korrekte Geschwindigkeiten"""
        self.Move.direction_command = self.Move.CMD_BACKWARD
        self.Move.turn_command = self.Move.MOVE_NO
        self.Move.movement_speed = 35

        with patch('Move.calculate_target_positions') as mock_calc, \
             patch('Move.apply_leg_position') as mock_apply:
            mock_calc.return_value = {
                leg: {'h': 0, 'v': 0} for leg in ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']
            }
            self.Move.move_thread()

            # Should have been called with positive speeds for backward
            args = mock_calc.call_args[0]
            self.assertEqual(args[1], 35)  # speed_left
            self.assertEqual(args[2], 35)  # speed_right

    def test_no_movement_resets_phase(self):
        """Test: Keine Bewegung setzt Phase auf 0.0 zurück (aktuelles Verhalten)"""
        self.Move.gait_phase = 0.5
        self.Move.direction_command = self.Move.MOVE_NO
        self.Move.turn_command = self.Move.MOVE_NO

        with patch('Move.handle_stand_or_steady'):
            self.Move.move_thread()

        # This is the CURRENT behavior that might change in Phase 4!
        self.assertEqual(self.Move.gait_phase, 0.0)

    # ==================== Test: Interpolation ====================

    def test_interpolation_updates_leg_positions(self):
        """Test: Interpolation aktualisiert _leg_positions"""
        self.Move.direction_command = self.Move.CMD_FORWARD
        self.Move.turn_command = self.Move.MOVE_NO

        # Set initial positions
        self.Move._leg_positions = {
            'L1': 0, 'L2': 0, 'L3': 0,
            'R1': 0, 'R2': 0, 'R3': 0
        }

        with patch('Move.calculate_target_positions') as mock_calc, \
             patch('Move.apply_leg_position') as mock_apply:
            # Return non-zero target positions
            mock_calc.return_value = {
                'L1': {'h': 20, 'v': 5},
                'L2': {'h': -20, 'v': -10},
                'L3': {'h': 20, 'v': 5},
                'R1': {'h': -20, 'v': -10},
                'R2': {'h': 20, 'v': 5},
                'R3': {'h': -20, 'v': -10},
            }

            self.Move.move_thread()

        # Leg positions should have been updated (interpolated)
        # With alpha=0.9, new_pos = 0 + 0.9 * (20 - 0) = 18
        self.assertNotEqual(self.Move._leg_positions['L1'], 0)
        self.assertGreater(self.Move._leg_positions['L1'], 0)
        self.assertLess(self.Move._leg_positions['L1'], 20)  # Should be interpolated

    def test_interpolation_alpha_value(self):
        """Test: Interpolation verwendet aktuell alpha=0.9"""
        self.Move.direction_command = self.Move.CMD_FORWARD
        self.Move.turn_command = self.Move.MOVE_NO

        self.Move._leg_positions['L1'] = 0

        with patch('Move.calculate_target_positions') as mock_calc, \
             patch('Move.apply_leg_position') as mock_apply:
            mock_calc.return_value = {
                leg: {'h': 10 if leg == 'L1' else 0, 'v': 0}
                for leg in ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']
            }

            self.Move.move_thread()

        # With alpha=0.9: new_pos = 0 + 0.9 * (10 - 0) = 9
        self.assertEqual(self.Move._leg_positions['L1'], 9)

    # ==================== Test: Tracking Variables ====================

    def test_last_command_exists(self):
        """Test: _last_command Variable existiert"""
        self.assertTrue(hasattr(self.Move, '_last_command'))

    def test_last_speed_sign_exists(self):
        """Test: _last_speed_sign Variable existiert"""
        self.assertTrue(hasattr(self.Move, '_last_speed_sign'))

    def test_last_command_initial_none(self):
        """Test: _last_command ist initial None"""
        self.assertIsNone(self.Move._last_command)

    def test_last_speed_sign_initial_zero(self):
        """Test: _last_speed_sign ist initial 0"""
        self.assertEqual(self.Move._last_speed_sign, 0)

    # NOTE: These variables are currently NOT used in the code!
    # This test documents the CURRENT state before Phase 2 implementation


class TestMoveIntegration(unittest.TestCase):
    """Integration Tests für komplexere Bewegungsabläufe"""

    def setUp(self):
        """Setup vor jedem Test"""
        self.patcher_pwm = patch('Move.pwm', MagicMock())
        self.patcher_time = patch('Move.time', MagicMock())

        self.mock_pwm = self.patcher_pwm.start()
        self.mock_time = self.patcher_time.start()

        import Move
        self.Move = Move

        # Reset state
        self.Move.gait_phase = 0.0
        self.Move._leg_positions = {
            'L1': 0, 'L2': 0, 'L3': 0,
            'R1': 0, 'R2': 0, 'R3': 0
        }
        self.Move.direction_command = self.Move.MOVE_NO
        self.Move.turn_command = self.Move.MOVE_NO
        self.Move.steadyMode = 0
        self.Move.abort_current_movement = False

    def tearDown(self):
        """Cleanup nach jedem Test"""
        self.patcher_pwm.stop()
        self.patcher_time.stop()

    def test_continuous_forward_movement(self):
        """Test: Kontinuierliche Forward-Bewegung über mehrere Zyklen"""
        self.Move.direction_command = self.Move.CMD_FORWARD
        self.Move.turn_command = self.Move.MOVE_NO
        self.Move.movement_speed = 35

        phases = []

        with patch('Move.apply_leg_position'):
            # Simulate 100 steps (more than one full cycle)
            for _ in range(100):
                phases.append(self.Move.gait_phase)
                self.Move.move_thread()

        # Phase should have cycled through multiple times
        self.assertGreater(max(phases), 0.9)
        self.assertLess(phases[-1], phases[-2] or phases[-1] < 0.1)  # Wrapped at least once

    def test_leg_positions_track_movement(self):
        """Test: _leg_positions tracken die Bewegung über Zeit"""
        self.Move.direction_command = self.Move.CMD_FORWARD
        self.Move.turn_command = self.Move.MOVE_NO
        self.Move.movement_speed = 35

        initial_positions = self.Move._leg_positions.copy()

        with patch('Move.apply_leg_position'):
            # Run several steps
            for _ in range(10):
                self.Move.move_thread()

        # At least some positions should have changed
        changed = sum(
            1 for leg in ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']
            if self.Move._leg_positions[leg] != initial_positions[leg]
        )
        self.assertGreater(changed, 0, "Some leg positions should have changed")


if __name__ == '__main__':
    # Run tests with verbose output
    unittest.main(verbosity=2)
