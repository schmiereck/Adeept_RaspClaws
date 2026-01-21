import unittest
from unittest import mock
import math
import sys
import os

# Add project root to path to allow imports from Server and root
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'Server'))

# Mock hardware modules before importing Move
sys.modules['Adafruit_PCA9685'] = mock.MagicMock()
sys.modules['mpu6050'] = mock.MagicMock()

from Server.Move import calculate_target_positions
from protocol import *

class TestCalculateTargetPositions(unittest.TestCase):

    def test_move_no_forward(self):
        """Test forward movement (MOVE_NO with negative speed)."""
        speed = -30  # Negative speed for forward

        # Phase 0.0: Group A (L1, R2, L3) starts forward swing
        pos_start = calculate_target_positions(0.0, speed, MOVE_NO)
        self.assertEqual(pos_start['L1']['h'], speed)
        self.assertAlmostEqual(pos_start['L1']['v'], 0)
        self.assertEqual(pos_start['R1']['h'], -speed)
        self.assertEqual(pos_start['R1']['v'], -10)

        # Phase 0.25: Group A is mid-air
        pos_mid_swing = calculate_target_positions(0.25, speed, MOVE_NO)
        self.assertAlmostEqual(pos_mid_swing['L1']['h'], 0)
        self.assertEqual(pos_mid_swing['L1']['v'], int(3 * abs(speed)))
        self.assertAlmostEqual(pos_mid_swing['R1']['h'], 0)
        self.assertEqual(pos_mid_swing['R1']['v'], -10)

        # Phase 0.5: Group B (R1, L2, R3) starts forward swing
        pos_half = calculate_target_positions(0.5, speed, MOVE_NO)
        self.assertEqual(pos_half['R1']['h'], speed)
        self.assertAlmostEqual(pos_half['R1']['v'], 0)
        self.assertEqual(pos_half['L1']['h'], -speed)
        self.assertEqual(pos_half['L1']['v'], -10)

    def test_turn_left(self):
        """Test left turn movement (CMD_LEFT)."""
        speed = 30

        # Phase 0.0: Group B starts swing
        pos_start = calculate_target_positions(0.0, speed, CMD_LEFT)
        self.assertAlmostEqual(pos_start['L2']['h'], abs(speed)) # Left leg starts swing from back
        self.assertAlmostEqual(pos_start['R1']['h'], -abs(speed)) # Right leg starts swing from front

        # Phase 0.25: Group B is mid-air, horizontal position is zero
        pos_mid = calculate_target_positions(0.25, speed, CMD_LEFT)
        self.assertAlmostEqual(pos_mid['L2']['h'], 0)
        self.assertAlmostEqual(pos_mid['R1']['h'], 0)
        self.assertGreater(pos_mid['L1']['v'], 0) # In air (Group A)
        self.assertEqual(pos_mid['L2']['v'], -10) # On ground (Group B)

    def test_turn_right(self):
        """Test right turn movement (CMD_RIGHT)."""
        speed = 30

        # Phase 0.0: Group B starts swing
        pos_start = calculate_target_positions(0.0, speed, CMD_RIGHT)
        self.assertAlmostEqual(pos_start['L2']['h'], -abs(speed)) # Left leg starts swing from front
        self.assertAlmostEqual(pos_start['R1']['h'], abs(speed)) # Right leg starts swing from back

        # Phase 0.25: Group B is mid-air, horizontal position is zero
        pos_mid = calculate_target_positions(0.25, speed, CMD_RIGHT)
        self.assertAlmostEqual(pos_mid['L2']['h'], 0)
        self.assertAlmostEqual(pos_mid['R1']['h'], 0)
        self.assertGreater(pos_mid['L1']['v'], 0) # In air (Group A)
        self.assertEqual(pos_mid['L2']['v'], -10) # On ground (Group B)

    def test_forward_left_arc(self):
        """Test forward-left arc movement."""
        speed = -30 # Forward arc
        
        # Phase 0.0
        pos = calculate_target_positions(0.0, speed, CMD_FORWARD_LEFT_ARC)
        # Check if left legs have smaller horizontal movement than right legs
        self.assertLess(abs(pos['L1']['h']), abs(pos['R2']['h']))
        self.assertLess(abs(pos['L2']['h']), abs(pos['R1']['h']))

    def test_forward_right_arc(self):
        """Test forward-right arc movement."""
        speed = -30 # Forward arc
        
        # Phase 0.0
        pos = calculate_target_positions(0.0, speed, CMD_FORWARD_RIGHT_ARC)
        # Check if right legs have smaller horizontal movement than left legs
        self.assertLess(abs(pos['R2']['h']), abs(pos['L1']['h']))
        self.assertLess(abs(pos['R1']['h']), abs(pos['L2']['h']))


if __name__ == '__main__':
    unittest.main()
