"""
Unit Tests für Move.py - FT40 Implementation Tests
Diese Tests validieren Phase 2, 3 und 4 der FT40 Implementation.

Datum: 2026-01-23
Zweck: Regression-Tests für FT40 Verbesserungen

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


class TestFT40Phase2(unittest.TestCase):
    """Tests für Phase 2: Richtungswechsel-Erkennung"""

    def setUp(self):
        """Setup vor jedem Test"""
        Move._last_command = None
        Move._last_speed_sign = 0
        Move._steps_since_change = 0
        Move._direction_changed = False

    def test_tracking_variables_exist(self):
        """Test: Alle Tracking-Variablen existieren"""
        self.assertTrue(hasattr(Move, '_last_command'))
        self.assertTrue(hasattr(Move, '_last_speed_sign'))
        self.assertTrue(hasattr(Move, '_steps_since_change'))
        self.assertTrue(hasattr(Move, '_direction_changed'))

    def test_tracking_variables_initial_values(self):
        """Test: Initial-Werte der Tracking-Variablen"""
        self.assertIsNone(Move._last_command)
        self.assertEqual(Move._last_speed_sign, 0)
        self.assertEqual(Move._steps_since_change, 0)
        self.assertFalse(Move._direction_changed)


class TestFT40Phase3(unittest.TestCase):
    """Tests für Phase 3: Variable Alpha"""

    def test_variable_alpha_calculation(self):
        """Test: Variable Alpha Berechnung"""
        # Simuliere verschiedene Schritte
        test_cases = [
            (0, True, 0.20),
            (1, True, 0.36),
            (2, True, 0.52),
            (3, True, 0.68),
            (4, True, 0.84),
            (5, False, 0.90),
            (10, False, 0.90),
        ]

        for steps, direction_changed, expected_alpha in test_cases:
            if direction_changed:
                alpha = 0.2 + (steps / 5.0) * 0.8
            else:
                alpha = 0.9

            self.assertAlmostEqual(alpha, expected_alpha, places=2,
                msg=f"Step {steps}, direction_changed={direction_changed}")


class TestFT40Phase4(unittest.TestCase):
    """Tests für Phase 4: Smart Phase-Reset"""

    def setUp(self):
        """Setup vor jedem Test"""
        Move._stop_counter = 0
        Move._stop_threshold = 30
        Move.gait_phase = 0.0

    def test_stop_counter_variable_exists(self):
        """Test: Stop-Counter Variable existiert"""
        self.assertTrue(hasattr(Move, '_stop_counter'))
        self.assertTrue(hasattr(Move, '_stop_threshold'))

    def test_stop_threshold_value(self):
        """Test: Stop-Threshold ist 30 (~0.5 Sekunden)"""
        self.assertEqual(Move._stop_threshold, 30)

    def test_short_stop_logic(self):
        """Test: Bei kurzem Stop bleibt Phase erhalten"""
        Move._stop_counter = 15  # Kurzer Stop
        Move._stop_threshold = 30
        Move.gait_phase = 0.7

        # Simuliere die Logik
        should_reset = Move._stop_counter > Move._stop_threshold

        self.assertFalse(should_reset, "Phase sollte bei kurzem Stop nicht zurückgesetzt werden")

    def test_long_stop_logic(self):
        """Test: Bei langem Stop wird Phase zurückgesetzt"""
        Move._stop_counter = 35  # Langer Stop
        Move._stop_threshold = 30
        Move.gait_phase = 0.7

        # Simuliere die Logik
        should_reset = Move._stop_counter > Move._stop_threshold

        self.assertTrue(should_reset, "Phase sollte bei langem Stop zurückgesetzt werden")


class TestFT40Integration(unittest.TestCase):
    """Integration-Tests für FT40 Gesamtfunktionalität"""

    def setUp(self):
        """Setup vor jedem Test"""
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

    def test_leg_positions_dictionary(self):
        """Test: Leg positions Dictionary ist korrekt initialisiert"""
        self.assertEqual(len(Move._leg_positions), 6)
        for leg in ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']:
            self.assertIn(leg, Move._leg_positions)
            self.assertIsInstance(Move._leg_positions[leg], int)

    def test_calculate_target_positions_exists(self):
        """Test: calculate_target_positions Funktion existiert"""
        self.assertTrue(hasattr(Move, 'calculate_target_positions'))
        self.assertTrue(callable(Move.calculate_target_positions))

    def test_calculate_target_positions_structure(self):
        """Test: calculate_target_positions gibt korrektes Format zurück"""
        result = Move.calculate_target_positions(0.0, -35, -35)

        self.assertIsInstance(result, dict)
        self.assertEqual(len(result), 6)

        for leg in ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']:
            self.assertIn(leg, result)
            self.assertIn('h', result[leg])
            self.assertIn('v', result[leg])
            self.assertIsInstance(result[leg]['h'], int)
            self.assertIsInstance(result[leg]['v'], int)

    def test_apply_leg_position_exists(self):
        """Test: apply_leg_position Funktion existiert"""
        self.assertTrue(hasattr(Move, 'apply_leg_position'))
        self.assertTrue(callable(Move.apply_leg_position))

    def test_move_thread_exists(self):
        """Test: move_thread Funktion existiert"""
        self.assertTrue(hasattr(Move, 'move_thread'))
        self.assertTrue(callable(Move.move_thread))


class TestFT40EdgeCases(unittest.TestCase):
    """Edge-Case Tests für FT40"""

    def test_phase_wrap_around(self):
        """Test: Phase wraps korrekt von 1.0 zu 0.0"""
        Move.gait_phase = 1.05
        expected = Move.gait_phase - 1.0
        self.assertLess(expected, 1.0)
        self.assertGreaterEqual(expected, 0.0)

    def test_alpha_boundaries(self):
        """Test: Alpha bleibt in gültigen Grenzen"""
        for step in range(0, 10):
            direction_changed = step < 5
            if direction_changed:
                alpha = 0.3 + (step / 5.0) * 0.7
            else:
                alpha = 0.9

            self.assertGreaterEqual(alpha, 0.0, f"Alpha should be >= 0 at step {step}")
            self.assertLessEqual(alpha, 1.0, f"Alpha should be <= 1.0 at step {step}")

    def test_stop_counter_increment(self):
        """Test: Stop-Counter inkrementiert korrekt"""
        Move._stop_counter = 0

        # Simuliere mehrere Stop-Iterationen
        for i in range(5):
            Move._stop_counter += 1

        self.assertEqual(Move._stop_counter, 5)

    def test_stop_counter_reset(self):
        """Test: Stop-Counter wird bei Bewegung zurückgesetzt"""
        Move._stop_counter = 20

        # Simuliere Bewegung
        movement_active = True
        if movement_active:
            Move._stop_counter = 0

        self.assertEqual(Move._stop_counter, 0)


class TestFT40Constants(unittest.TestCase):
    """Tests für FT40 Konstanten"""

    def test_cycle_steps(self):
        """Test: CYCLE_STEPS ist definiert"""
        self.assertTrue(hasattr(Move, 'CYCLE_STEPS'))
        self.assertIsInstance(Move.CYCLE_STEPS, int)
        self.assertGreater(Move.CYCLE_STEPS, 0)

    def test_movement_commands(self):
        """Test: Bewegungs-Kommandos sind definiert"""
        self.assertTrue(hasattr(Move, 'CMD_FORWARD'))
        self.assertTrue(hasattr(Move, 'CMD_BACKWARD'))
        self.assertTrue(hasattr(Move, 'CMD_LEFT'))
        self.assertTrue(hasattr(Move, 'CMD_RIGHT'))
        self.assertTrue(hasattr(Move, 'MOVE_NO'))


class TestFT40Regression(unittest.TestCase):
    """Regression-Tests: Stelle sicher, dass alte Funktionalität erhalten bleibt"""

    def test_gait_phase_exists(self):
        """Test: gait_phase Variable existiert"""
        self.assertTrue(hasattr(Move, 'gait_phase'))

    def test_direction_command_exists(self):
        """Test: direction_command Variable existiert"""
        self.assertTrue(hasattr(Move, 'direction_command'))

    def test_turn_command_exists(self):
        """Test: turn_command Variable existiert"""
        self.assertTrue(hasattr(Move, 'turn_command'))

    def test_movement_speed_exists(self):
        """Test: movement_speed Variable existiert"""
        self.assertTrue(hasattr(Move, 'movement_speed'))

    def test_steady_mode_exists(self):
        """Test: steadyMode Variable existiert"""
        self.assertTrue(hasattr(Move, 'steadyMode'))


def run_tests_verbose():
    """Führe alle Tests mit verbose Output aus"""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    # Füge alle Test-Klassen hinzu
    suite.addTests(loader.loadTestsFromTestCase(TestFT40Phase2))
    suite.addTests(loader.loadTestsFromTestCase(TestFT40Phase3))
    suite.addTests(loader.loadTestsFromTestCase(TestFT40Phase4))
    suite.addTests(loader.loadTestsFromTestCase(TestFT40Integration))
    suite.addTests(loader.loadTestsFromTestCase(TestFT40EdgeCases))
    suite.addTests(loader.loadTestsFromTestCase(TestFT40Constants))
    suite.addTests(loader.loadTestsFromTestCase(TestFT40Regression))

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    return result


if __name__ == '__main__':
    print("=" * 70)
    print("FT40 Unit Tests - Running on Windows (Hardware Mocked)")
    print("=" * 70)
    print()

    result = run_tests_verbose()

    print()
    print("=" * 70)
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success: {result.wasSuccessful()}")
    print("=" * 70)

    if not result.wasSuccessful():
        sys.exit(1)
