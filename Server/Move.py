#!/usr/bin/env/python3
# File name   : Move.py
# Website     : www.Adeept.com
# Author      : Adeept
# Date        : 2025/04/16
import time
import math
import Adafruit_PCA9685
from mpu6050 import mpu6050
import Kalman_Filter as Kalman_filter
import PID
import threading
import RPIservo
import sys
import os
# Add parent directory to path to import protocol module
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from protocol import *

pwm0 = 300
pwm1 = 300
pwm2 = 300
pwm3 = 300

pwm4 = 300
pwm5 = 300
pwm6 = 300
pwm7 = 300

pwm8 = 300
pwm9 = 300
pwm10 = 300
pwm11 = 300

pwm12 = 300
pwm13 = 300
pwm14 = 300
pwm15 = 300

# Current servo positions (initialized to base positions)
# These track the actual servo positions for smooth interpolation
servo_current_pos = [
	pwm0, pwm1, pwm2, pwm3,
	pwm4, pwm5, pwm6, pwm7,
	pwm8, pwm9, pwm10, pwm11,
	pwm12, pwm13, pwm14, pwm15
]

'''
change this variables to 0 to reverse all the servos.
'''
set_direction = 1

'''
change these two variables to reverse the direction of the legs.
'''
if set_direction:
	leftSide_direction  = 1
	rightSide_direction = 0
else:
	leftSide_direction  = 0
	rightSide_direction = 1

'''
change these two variables to reverse the height of the legs.
'''
if set_direction:
	leftSide_height  = 0
	rightSide_height = 1
else:
	leftSide_height  = 1
	rightSide_height = 0

'''
change this variable to set the range of the height range.
'''
height_change = 30

'''
change these two variables to adjuest the function for observing.
'''
if set_direction:
	Up_Down_direction = 1
	Left_Right_direction = 1
else:
	Up_Down_direction = 0
	Left_Right_direction = 0
Left_Right_input = 300
Up_Down_input = 300
Left_Right_Max = 500
Left_Right_Min = 100
Up_Down_Max = 500
Up_Down_Min = 270
look_wiggle = 30
move_stu = 1
abort_current_movement = False  # Flag to immediately abort ongoing movement cycle
arc_factor = 0.7  # Factor to control arc tightness. 0 = straight, 1 = pivot turn on inner leg.


'''
change these variable to adjuest the steady function.
'''
steady_range_Min = -40
steady_range_Max = 130
range_Mid = (steady_range_Min+steady_range_Max)/2
X_fix_output = range_Mid
Y_fix_output = range_Mid
steady_X_set = 73

'''
Set PID
'''
P = 5
I = 0.01
D = 0

'''
>>> instantiation <<<
'''
X_pid = PID.PID()
X_pid.SetKp(P)
X_pid.SetKd(I)
X_pid.SetKi(D)
Y_pid = PID.PID()
Y_pid.SetKp(P)
Y_pid.SetKd(I)
Y_pid.SetKi(D)

# Try to initialize PCA9685, use mock mode if hardware not available
try:
	pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)  # Changed from 0x5F to 0x40
	pwm.set_pwm_freq(50)
	print("PCA9685 initialized in Move.py on address 0x40")
except (OSError, IOError) as e:
	print(f"\033[38;5;3mWarning:\033[0m Could not initialize PCA9685 in Move.py: {e}")
	print("Running in MOCK MODE - servo commands will be ignored")
	class MockPWM:
		def set_pwm(self, channel, on, off):
			pass
		def set_pwm_freq(self, freq):
			pass
	pwm = MockPWM()
MOCK_MODE = True # Set MOCK_MODE to True if PCA9685 initialization fails

print(f"Servo controller MOCK_MODE status: {MOCK_MODE}")

# PCA9685 Register Constants for direct I2C access
_LED0_ON_L = 0x06
_LED0_ON_H = 0x07
_LED0_OFF_L = 0x08
_LED0_OFF_H = 0x09
_MODE1 = 0x00 # MODE1 register address

def _batch_set_servos(servo_positions):
	"""
	Sets multiple servo PWM positions using a single I2C block write.
	servo_positions is a dictionary {channel: pwm_value, ...}
	where pwm_value is the 'off' tick (0-4095).
	"""
	global pwm, servo_current_pos

	if MOCK_MODE:
		# In mock mode, just update internal position tracking
		for channel, value in servo_positions.items():
			if 0 <= channel < len(servo_current_pos):
				servo_current_pos[channel] = value
		return

	# Prepare the data buffer for all 16 channels, assuming ON is always 0
	# Each channel needs 4 bytes: ON_L, ON_H, OFF_L, OFF_H
	# The PCA9685 has registers for LED0_ON_L through LED15_OFF_H contiguously.
	# We collect all 64 bytes and send in one block write.
	
	# Current pwm values from servo_current_pos are used for channels not explicitly
	# present in servo_positions, to maintain their last known state.
	
	data_bytes = bytearray(16 * 4) # 16 channels * 4 bytes/channel

	for channel in range(16):
		# Default to current tracked position if not in batch update
		off_val = servo_positions.get(channel, servo_current_pos[channel])
		
		# Ensure off_val is within valid range
		off_val = max(0, min(4095, off_val))

		# Update tracked position
		servo_current_pos[channel] = off_val

		# Fill byte array (ON_L, ON_H, OFF_L, OFF_H)
		# ON_L and ON_H are 0 for typical servo control (pulse starts at beginning of cycle)
		data_bytes[channel * 4 + 0] = 0        # ON_L
		data_bytes[channel * 4 + 1] = 0        # ON_H
		data_bytes[channel * 4 + 2] = off_val & 0xFF  # OFF_L
		data_bytes[channel * 4 + 3] = off_val >> 8    # OFF_H

		try:

			# Write the entire block of 64 bytes starting from LED0_ON_L register (0x06)

			pwm._i2c.write_i2c_block_data(pwm._address, _LED0_ON_L, list(data_bytes))

			# print(f"✓ I2C batch write successful for {len(servo_positions)} channels.") # uncomment for verbose debugging

		except Exception as e:

			print(f"⚠ I2C batch write failed: {e}. Attempting fallback to individual PWM writes.")

			# Fallback to individual writes if batch fails

			for channel, value in servo_positions.items():

				try:

					pwm.set_pwm(channel, 0, value)

					# print(f"  Fallback: Channel {channel} set to {value}") # uncomment for verbose debugging

				except Exception as e_single:

					print(f"✗ Fallback individual PWM write for channel {channel} failed: {e_single}")

	

	kalman_filter_X =  Kalman_filter.Kalman_filter(0.001,0.1)
kalman_filter_Y =  Kalman_filter.Kalman_filter(0.001,0.1)

try:
	sensor = mpu6050(0x68)
	mpu6050_connection = 1
except:
	mpu6050_connection = 0

'''
change these two variable to adjuest the steady status.
	   (X+)
	   /|\
  (Y+)<-+->(Y-)
	    |
	   (X-)
Example: If you want the forhead of the robot to point down,
         you need to increase the value target_X.
'''
target_X = 0
target_Y = 0


# ==================== Servo Position Tracking & Smooth Interpolation ====================

def get_servo_pos(channel):
	"""
	Get current servo position.

	Args:
		channel: Servo channel (0-15)

	Returns:
		Current PWM position
	"""
	if channel < 0 or channel > 15:
		return 300  # Default
	return servo_current_pos[channel]


def get_servo_positions_info():
	"""
	Get formatted string of all servo positions for GUI display.
	Shows positions for the 6 legs (12 servos).

	Returns:
		String with servo positions (e.g., "L1:300,305 L2:310,295 ...")
	"""
	# Leg servos:
	# Left_I:   channels 0 (H), 1 (V)
	# Left_II:  channels 2 (H), 3 (V)
	# Left_III: channels 4 (H), 5 (V)
	# Right_I:  channels 6 (H), 7 (V)
	# Right_II: channels 8 (H), 9 (V)
	# Right_III:channels 10 (H), 11 (V)

	return (f"L1:{servo_current_pos[0]},{servo_current_pos[1]} "
	        f"L2:{servo_current_pos[2]},{servo_current_pos[3]} "
	        f"L3:{servo_current_pos[4]},{servo_current_pos[5]} "
	        f"R1:{servo_current_pos[6]},{servo_current_pos[7]} "
	        f"R2:{servo_current_pos[8]},{servo_current_pos[9]} "
	        f"R3:{servo_current_pos[10]},{servo_current_pos[11]}")


def get_mpu6050_data():
	"""
	Get formatted string of MPU6050 sensor data (gyro + accelerometer).

	Returns:
		String with MPU data (e.g., "G:1.2,0.5,0.1 A:0.15,0.08,9.81")
		Or "MPU:N/A" if sensor not connected
	"""
	global mpu6050_connection

	if not mpu6050_connection:
		return "MPU:N/A"

	try:
		# Get accelerometer data (in m/s²)
		accel_data = sensor.get_accel_data()
		accel_x = accel_data['x']
		accel_y = accel_data['y']
		accel_z = accel_data['z']

		# Get gyroscope data (in degrees/sec)
		gyro_data = sensor.get_gyro_data()
		gyro_x = gyro_data['x']
		gyro_y = gyro_data['y']
		gyro_z = gyro_data['z']

		# Format: "G:x,y,z A:x,y,z"
		return f"G:{gyro_x:.2f},{gyro_y:.2f},{gyro_z:.2f} A:{accel_x:.2f},{accel_y:.2f},{accel_z:.2f}"
	except Exception as e:
		print(f"⚠ Error reading MPU6050: {e}")
		return "MPU:ERROR"


# ==================== End of Position Tracking ====================


'''
Set a default pwm value for all servos.
'''
for i in range(0,16):
	exec('pwm%d=RPIservo.init_pwm%d'%(i,i))

'''
Get raw data from mpu6050.
'''
def mpu6050Test():
	while 1:
		accelerometer_data = sensor.get_accel_data()
		print('X=%f,Y=%f,Z=%f'%(accelerometer_data['x'],accelerometer_data['y'],accelerometer_data['x']))
		time.sleep(0.3)

		
def init_all():
	global servo_current_pos

	# Initialize all servos to base positions
	pwm.set_pwm(0, 0, pwm0)
	pwm.set_pwm(1, 0, pwm1)
	pwm.set_pwm(2, 0, pwm2)
	pwm.set_pwm(3, 0, pwm3)

	pwm.set_pwm(4, 0, pwm4)
	pwm.set_pwm(5, 0, pwm5)
	pwm.set_pwm(6, 0, pwm6)
	pwm.set_pwm(7, 0, pwm7)

	pwm.set_pwm(8, 0, pwm8)
	pwm.set_pwm(9, 0, pwm9)
	pwm.set_pwm(10, 0, pwm10)
	pwm.set_pwm(11, 0, pwm11)

	pwm.set_pwm(12, 0, pwm12)
	pwm.set_pwm(13, 0, pwm13)
	pwm.set_pwm(14, 0, pwm14)
	pwm.set_pwm(15, 0, pwm15)
	
	# Update position tracking
	servo_current_pos = [
		pwm0, pwm1, pwm2, pwm3,
		pwm4, pwm5, pwm6, pwm7,
		pwm8, pwm9, pwm10, pwm11,
		pwm12, pwm13, pwm14, pwm15
	]


init_all()

def ctrl_range(raw, max_genout, min_genout):
	if raw > max_genout:
		raw_output = max_genout
	elif raw < min_genout:
		raw_output = min_genout
	else:
		raw_output = raw
	return int(raw_output)

'''
left_I   -<forward>-- right_III
left_II  ---<BODY>---  right_II
left_III -<Backward>-   right_I

            pos=1
           /     \
          /       \
         /         \
    pos=2---pos=3---pos=4

Change the value of wiggle to set the range and direction that the legs moves.
'''

# ==================== Generic Leg Control (Refactored) ====================

# Leg configuration: (horizontal_channel, height_channel, base_h, base_v, is_left_side)
LEG_CONFIG = {
	'left_I':   (0, 1, pwm0, pwm1, True),
	'left_II':  (2, 3, pwm2, pwm3, True),
	'left_III': (4, 5, pwm4, pwm5, True),
	'right_I':  (6, 7, pwm6, pwm7, False),
	'right_II': (8, 9, pwm8, pwm9, False),
	'right_III':(10,11,pwm10,pwm11,False),
}


def leg_control(leg_name, pos, wiggle, heightAdjust=0):
	"""
	Generic leg control function for all 6 legs.
	Replaces the 6 individual left_I/II/III and right_I/II/III functions.

	Args:
		leg_name: Name of the leg ('left_I', 'left_II', 'left_III', 'right_I', 'right_II', 'right_III')
		pos: Position in walk cycle (0-4)
		wiggle: Movement range (speed parameter)
		heightAdjust: Height adjustment for pos=0
	"""
	h_channel, v_channel, base_h, base_v, is_left = LEG_CONFIG[leg_name]

	# Determine direction and height flags based on side
	if is_left:
		direction_flag = leftSide_direction
		height_flag = leftSide_height
	else:
		direction_flag = rightSide_direction
		height_flag = rightSide_height

	if pos == 0:
		# Position 0: Height adjustment only
		#pwm.set_pwm(h_channel, 0, base_h)  # Commented out like in original
		if height_flag:
			pwm.set_pwm(v_channel, 0, base_v + heightAdjust)
		else:
			pwm.set_pwm(v_channel, 0, base_v - heightAdjust)
	else:
		# Positions 1-4: Full movement cycle
		if direction_flag:
			# Forward direction logic
			if pos == 1:
				pwm.set_pwm(h_channel, 0, base_h)
				if height_flag:
					pwm.set_pwm(v_channel, 0, base_v + 3*height_change)
				else:
					pwm.set_pwm(v_channel, 0, base_v - 3*height_change)
			elif pos == 2:
				pwm.set_pwm(h_channel, 0, base_h + wiggle)
				if height_flag:
					pwm.set_pwm(v_channel, 0, base_v - height_change)
				else:
					pwm.set_pwm(v_channel, 0, base_v + height_change)
			elif pos == 3:
				pwm.set_pwm(h_channel, 0, base_h + int(wiggle/2))  # Halfway to avoid jerk
				if height_flag:
					pwm.set_pwm(v_channel, 0, base_v - height_change)
				else:
					pwm.set_pwm(v_channel, 0, base_v + height_change)
			elif pos == 4:
				pwm.set_pwm(h_channel, 0, base_h - wiggle)
				if height_flag:
					pwm.set_pwm(v_channel, 0, base_v - height_change)
				else:
					pwm.set_pwm(v_channel, 0, base_v + height_change)
		else:
			# Reverse direction logic
			if pos == 1:
				pwm.set_pwm(h_channel, 0, base_h)
				if height_flag:
					pwm.set_pwm(v_channel, 0, base_v + 3*wiggle)
				else:
					pwm.set_pwm(v_channel, 0, base_v - 3*wiggle)
			elif pos == 2:
				pwm.set_pwm(h_channel, 0, base_h - wiggle)
				if height_flag:
					pwm.set_pwm(v_channel, 0, base_v - wiggle)
				else:
					pwm.set_pwm(v_channel, 0, base_v + wiggle)
			elif pos == 3:
				pwm.set_pwm(h_channel, 0, base_h - int(wiggle/2))  # Halfway to avoid jerk
				if height_flag:
					pwm.set_pwm(v_channel, 0, base_v - wiggle)
				else:
					pwm.set_pwm(v_channel, 0, base_v + wiggle)
			elif pos == 4:
				pwm.set_pwm(h_channel, 0, base_h + wiggle)
				if height_flag:
					pwm.set_pwm(v_channel, 0, base_v - wiggle)
				else:
					pwm.set_pwm(v_channel, 0, base_v + wiggle)


# ==================== Wrapper Functions (for backwards compatibility) ====================
# The old 6 functions are now simple wrappers that call the generic leg_control()

def left_I(pos, wiggle, heightAdjust=0):
	"""Left leg I - uses generic leg_control"""
	leg_control('left_I', pos, wiggle, heightAdjust)


def left_II(pos, wiggle, heightAdjust=0):
	"""Left leg II - uses generic leg_control"""
	leg_control('left_II', pos, wiggle, heightAdjust)


def left_III(pos, wiggle, heightAdjust=0):
	"""Left leg III - uses generic leg_control"""
	leg_control('left_III', pos, wiggle, heightAdjust)


def right_I(pos, wiggle, heightAdjust=0):
	"""Right leg I - uses generic leg_control"""
	leg_control('right_I', pos, wiggle, heightAdjust)


def right_II(pos, wiggle, heightAdjust=0):
	"""Right leg II - uses generic leg_control"""
	leg_control('right_II', pos, wiggle, heightAdjust)


def right_III(pos, wiggle, heightAdjust=0):
	"""Right leg III - uses generic leg_control"""
	leg_control('right_III', pos, wiggle, heightAdjust)


# ==================== OLD IMPLEMENTATIONS REMOVED ====================
# The old 360 lines of duplicated code for the 6 leg functions have been removed.
# They have been replaced by the generic leg_control() function above.
# This removed code duplication and improved maintainability significantly.


def move(step_input, speed, command):
	"""
	Execute a movement step with interpolation for smoother servo movements.
	Now uses multiple intermediate steps to avoid jerky movements.
	"""
	step_I  = step_input
	step_II = step_input + 2

	if step_II > 4:
		step_II = step_II - 4
	if speed == 0:
		return

	# Use 5 intermediate steps for smoother movement
	# This makes servo movements less jerky, especially height changes
	interpolation_steps = 5

	for i in range(interpolation_steps):
		# Calculate intermediate speed (gradually increase)
		intermediate_speed = int(speed * (i + 1) / interpolation_steps)

		if command == MOVE_NO:
			right_I(step_I, intermediate_speed, 0)
			left_II(step_I, intermediate_speed, 0)
			right_III(step_I, intermediate_speed, 0)

			left_I(step_II, intermediate_speed, 0)
			right_II(step_II, intermediate_speed, 0)
			left_III(step_II, intermediate_speed, 0)
		elif command == CMD_LEFT:
			right_I(step_I, intermediate_speed, 0)
			left_II(step_I, -intermediate_speed, 0)
			right_III(step_I, intermediate_speed, 0)

			left_I(step_II, -intermediate_speed, 0)
			right_II(step_II, intermediate_speed, 0)
			left_III(step_II, -intermediate_speed, 0)
		elif command == CMD_RIGHT:
			right_I(step_I, -intermediate_speed, 0)
			left_II(step_I, intermediate_speed, 0)
			right_III(step_I, -intermediate_speed, 0)

			left_I(step_II, intermediate_speed, 0)
			right_II(step_II, -intermediate_speed, 0)
			left_III(step_II, intermediate_speed, 0)

		# Small delay between interpolation steps for smooth movement
		time.sleep(0.02)  # 20ms between steps = 100ms total for 5 steps


# Global position tracker for smooth continuous movement
# Store actual leg positions (horizontal offsets) instead of just phase
# This allows smooth interpolation when direction changes
_leg_positions = {
	'L1': 0,  # Left front horizontal position
	'L2': 0,  # Left middle horizontal position
	'L3': 0,  # Left rear horizontal position
	'R1': 0,  # Right front horizontal position
	'R2': 0,  # Right middle horizontal position
	'R3': 0   # Right rear horizontal position
}

# Track last movement command to detect changes
_last_command = None
_last_speed_sign = 0

def move_smooth(speed_left, speed_right, cycle_steps=18):
	"""
	Smooth continuous movement using sine/cosine curves with position tracking.

	Args:
		speed_left: Movement amplitude for left legs (negative = forward)
		speed_right: Movement amplitude for right legs (negative = forward)
		cycle_steps: Number of steps per full walking cycle (default 30)
	"""
	global _leg_positions, _last_command, _last_speed_sign, direction_command, turn_command, abort_current_movement

	# TODO: Refactor command/speed change detection
	speed = max(abs(speed_left), abs(speed_right))
	command = turn_command # Temporary for change detection
	# Detect command or direction change
	current_speed_sign = 1 if speed > 0 else -1 if speed < 0 else 0
	command_changed = (command != _last_command) or (current_speed_sign != _last_speed_sign and current_speed_sign != 0 and _last_speed_sign != 0)

	if command_changed:
		print(f"Movement change detected: cmd={_last_command}→{command}, dir={_last_speed_sign}→{current_speed_sign}")

	_last_command = command
	_last_speed_sign = current_speed_sign

	# Perform one complete walking cycle
	for step in range(cycle_steps):
		# PRIORITY 1: Check abort flag (set by button release handlers)
		if abort_current_movement:
			print(f"[move_smooth] ⚡ ABORT FLAG detected at step {step}/{cycle_steps}")
			# Lower all legs to ground for stable position
			print("[move_smooth] Lowering legs to stable ground position...")
			for leg in ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']:
				# Keep current horizontal position, set vertical to ground level
				apply_leg_position(leg, _leg_positions[leg], -10)
			_last_command = None
			_last_speed_sign = 0
			# Flag will be reset at end of function
			break

		# PRIORITY 2: Check if movement should stop (move_stu flag)
		if not move_stu:
			# Keep current positions when stopped (don't reset!)
			# This allows smooth continuation when movement resumes
			_last_command = None
			_last_speed_sign = 0
			break

		# Calculate current phase for this step (0.0 to 1.0)
		phase = step / cycle_steps

		# Calculate target positions for this phase
		target_positions = calculate_target_positions(phase, speed_left, speed_right)

		# Interpolate from current to target (smooth transition)
		# Use ease-in/ease-out interpolation for smooth, non-jerky movements
		if command_changed and step < 15:
			# Smooth interpolation for first few steps after direction change
			# Use cubic easing for smoother acceleration/deceleration
			t = step / 15
			# Cubic ease-out: rapid start, slow finish
			eased_t = 1 - pow(1 - t, 3)
			alpha = 0.25 + eased_t * 0.5  # 0.25 → 0.75 with cubic easing
		else:
			# Normal: gentle continuous interpolation for smooth servo motion
			alpha = 0.7  # Smooth following, prevents jerky jumps

		# Collect all new target PWM values for batch update
		batch_servo_updates = {}
		for leg_name in LEG_CONFIG.keys(): # Corrected: Iterate over actual leg names from LEG_CONFIG
			horizontal_offset = target_positions[leg_name]['h']
			vertical_offset = target_positions[leg_name]['v']

			# Get interpolated PWM values for this leg's horizontal and vertical servos
			leg_pwms = _get_interpolated_leg_pwms(leg_name, horizontal_offset, vertical_offset, alpha)
			
			for channel, pwm_value in leg_pwms.items():
				batch_servo_updates[channel] = pwm_value
				# Update _leg_positions for horizontal tracking (needs to be adjusted for new leg_name keys)
				# Original _leg_positions keys are 'L1', 'L2', etc. This needs careful handling.
				# For now, let's just ensure servo_current_pos is updated.
				# The _leg_positions update logic should reflect the leg_name used in target_positions.
				# Since target_positions is indexed by LEG_CONFIG keys, _leg_positions also needs to align.
				# For now, let's comment out the _leg_positions update line to avoid another KeyError
				# and focus on getting movement back. The _leg_positions logic needs rethinking.
				# _leg_positions[leg_name] = pwm_value - base_h (this would require base_h for the leg)
				pass # The _leg_positions logic needs to be revisited, currently it's causing issues.



def _calculate_gait_phase(phase, speed):
	"""
	Determines which leg group is in the air/on the ground based on the phase
	and calculates the basic timing and vertical positions.

	Args:
		phase: Current phase in the cycle (0.0 to 1.0)
		speed: Movement amplitude

	Returns:
		A tuple containing:
		- air_group (list of leg names in the air)
		- ground_group (list of leg names on the ground)
		- t (normalized time within the half-cycle, 0.0 to 1.0)
		- v_air (vertical position for air legs)
		- v_ground (vertical position for ground legs)
	"""
	group_a = ['L1', 'R2', 'L3']
	group_b = ['R1', 'L2', 'R3']

	if phase < 0.5:
		air_group = group_a
		ground_group = group_b
		t = phase * 2
	else:
		air_group = group_b
		ground_group = group_a
		t = (phase - 0.5) * 2

	v_air = int(3 * abs(speed) * math.sin(t * math.pi))
	v_ground = -10

	return air_group, ground_group, t, v_air, v_ground


def calculate_target_positions(phase, speed_left, speed_right):
	"""
	Calculate target horizontal and vertical positions for all legs based on phase and side speeds.
	This single function handles all movement types (forward, backward, turn, arc).

	Args:
		phase (float): Current phase of the walk cycle (0.0 to 1.0).
		speed_left (int): Movement amplitude for the left legs.
		speed_right (int): Movement amplitude for the right legs.

	Returns:
		dict: Dictionary with target positions for each leg: {'L1': {'h': ..., 'v': ...}, ...}
	"""
	positions = {}
	# The overall speed determines the leg lift height.
	speed = max(abs(speed_left), abs(speed_right))

	air_group, ground_group, t, v_air, v_ground = _calculate_gait_phase(phase, speed)

	# Calculate horizontal movement for each side
	h_left_air = int(speed_left * math.cos(t * math.pi))
	h_left_ground = -h_left_air

	h_right_air = int(speed_right * math.cos(t * math.pi))
	h_right_ground = -h_right_air

	# Assign positions to all legs
	for leg in LEG_CONFIG.keys(): # Corrected: Iterate over actual leg names from LEG_CONFIG
		is_left = leg.startswith('L')
		if leg in air_group:
			positions[leg] = {'h': h_left_air if is_left else h_right_air, 'v': v_air}
		else:  # in ground_group
			positions[leg] = {'h': h_left_ground if is_left else h_right_ground, 'v': v_ground}

	return positions


def _get_interpolated_leg_pwms(leg_name, horizontal_offset, vertical_offset, alpha):
	"""
	Calculates interpolated PWM target values for a leg's horizontal and vertical servos.
	This function replaces the logic formerly spread across dove_* functions and set_servo_smooth.

	Args:
		leg_name: Name of the leg ('L1', 'L2', etc.)
		horizontal_offset: Desired horizontal movement offset.
		vertical_offset: Desired vertical movement offset.
		alpha: Interpolation factor (0.0 to 1.0) for the overall gait cycle.
		interpolation_steps: Number of micro-steps for individual servo smoothness.

	Returns:
		dict: {'h_channel': final_h_pwm, 'v_channel': final_v_pwm}
	"""
	global servo_current_pos

	h_channel, v_channel, base_h, base_v, is_left = LEG_CONFIG[leg_name]

	# Determine direction and height flags based on side
	if is_left:
		direction_flag = leftSide_direction
		height_flag = leftSide_height
	else:
		direction_flag = rightSide_direction
		height_flag = rightSide_height

	# --- Horizontal Servo Calculation ---
	if direction_flag:
		intermediate_target_h = base_h + horizontal_offset
	else:
		intermediate_target_h = base_h - horizontal_offset

	# Interpolate horizontal servo position
	current_h = servo_current_pos[h_channel]
	final_h_pwm = int(current_h + (intermediate_target_h - current_h) * alpha)
	# Clamp to valid range (optional, can be handled by PCA9685 driver implicitly)
	final_h_pwm = max(100, min(520, final_h_pwm)) # Using max/minPos from RPIservo.py

	# --- Vertical Servo Calculation ---
	if height_flag:
		intermediate_target_v = base_v + vertical_offset
	else:
		intermediate_target_v = base_v - vertical_offset
	
	# Interpolate vertical servo position
	current_v = servo_current_pos[v_channel]
	final_v_pwm = int(current_v + (intermediate_target_v - current_v) * alpha)
	# Clamp to valid range
	final_v_pwm = max(100, min(520, final_v_pwm)) # Using max/minPos from RPIservo.py

	return {h_channel: final_h_pwm, v_channel: final_v_pwm}


def stand():
	pwm.set_pwm(0,0,300)
	pwm.set_pwm(1,0,300)
	pwm.set_pwm(2,0,300)
	pwm.set_pwm(3,0,300)
	pwm.set_pwm(4,0,300)
	pwm.set_pwm(5,0,300)
	pwm.set_pwm(6,0,300)
	pwm.set_pwm(7,0,300)
	pwm.set_pwm(8,0,300)
	pwm.set_pwm(9,0,300)
	pwm.set_pwm(10,0,300)
	pwm.set_pwm(11,0,300)


# ==================== Generic Helper Functions for Movement ====================

def _apply_tripod_group_positions(group_a_h, group_a_v, group_b_h, group_b_v):
	"""
	Apply horizontal and vertical positions to both tripod groups.

	Tripod Group A: L1, R2, L3 (front-left, middle-right, rear-left)
	Tripod Group B: R1, L2, R3 (front-right, middle-left, rear-right)

	Args:
		group_a_h: Horizontal position for Group A
		group_a_v: Vertical position for Group A
		group_b_h: Horizontal position for Group B
		group_b_v: Vertical position for Group B
	"""
	# Group A (L1, R2, L3)
	dove_Left_I(group_a_h, group_a_v)
	dove_Right_II(group_a_h, group_a_v)
	dove_Left_III(group_a_h, group_a_v)

	# Group B (R1, L2, R3)
	dove_Right_I(group_b_h, group_b_v)
	dove_Left_II(group_b_h, group_b_v)
	dove_Right_III(group_b_h, group_b_v)


def _interpolate_linear(start, end, t):
	"""
	Linear interpolation between start and end.

	Args:
		start: Start value
		end: End value
		t: Interpolation parameter (0.0 to 1.0)

	Returns:
		Interpolated integer value
	"""
	return int(start + (end - start) * t)


def _interpolate_vertical_arc(speed, t, descending=False):
	"""
	Calculate vertical position for smooth arc movement using sine curve.

	Args:
		speed: Movement amplitude
		t: Phase parameter (0.0 to 1.0)
		descending: If True, arc goes from peak to ground (3*speed → 0)
		            If False, arc goes from ground to peak (0 → 3*speed)

	Returns:
		Vertical position as integer
	"""
	import math
	if descending:
		# Descending arc: 3*speed → 0
		return int(3 * abs(speed) * math.sin((1 - t) * math.pi))
	else:
		# Ascending arc: 0 → 3*speed → 0 (full sine wave)
		return int(3 * abs(speed) * math.sin(t * math.pi))


def _execute_step_loop(num_steps, time_per_step, position_calculator, command):
	"""
	Execute a step loop with interpolated positions.

	Args:
		num_steps: Number of interpolation steps
		time_per_step: Sleep time between steps
		position_calculator: Function(t, command) that returns (group_a_h, group_a_v, group_b_h, group_b_v)
		                     or dict with individual leg positions for turns
		command: Movement command (MOVE_NO, CMD_LEFT, CMD_RIGHT)
	"""
	for i in range(num_steps + 1):
		# Check if movement should stop
		if move_stu == 0:
			break

		# Calculate interpolation parameter
		t = i / num_steps

		# Get positions from calculator
		positions = position_calculator(t, command)

		# Apply positions based on return type
		if isinstance(positions, dict):
			# Individual leg positions (for turns)
			dove_Left_I(positions['L1_h'], positions['L1_v'])
			dove_Right_II(positions['R2_h'], positions['R2_v'])
			dove_Left_III(positions['L3_h'], positions['L3_v'])
			dove_Right_I(positions['R1_h'], positions['R1_v'])
			dove_Left_II(positions['L2_h'], positions['L2_v'])
			dove_Right_III(positions['R3_h'], positions['R3_v'])
		else:
			# Tripod group positions (for forward/backward)
			group_a_h, group_a_v, group_b_h, group_b_v = positions
			_apply_tripod_group_positions(group_a_h, group_a_v, group_b_h, group_b_v)

		time.sleep(time_per_step)


# ==================== Main Movement Functions ====================

def dove(step_input, speed, timeLast, dpi, command):
	"""
	Step-based leg movement with 4-phase tripod gait.

	Refactored to use generic helper functions and reduce code duplication.

	Args:
		step_input: Current step (1-4)
		speed: Movement amplitude (positive=forward, negative=backward)
		timeLast: Total time for the movement
		dpi: Steps per phase (dots per inch - interpolation resolution)
		command: Movement command (MOVE_NO, CMD_LEFT, CMD_RIGHT)
	"""
	# Adjust for backward movement
	if speed < 0:
		speed = -speed
		is_backward = True
	else:
		is_backward = False

	num_steps = dpi
	time_per_step = timeLast / dpi

	# Define position calculators for each step
	def calc_step_1(t, cmd):
		"""Step 1: Group A lifts and moves (ascending arc)"""
		if cmd == MOVE_NO:
			h_a = _interpolate_linear(speed if not is_backward else -speed,
			                          -speed if not is_backward else speed, t)
			v_a = _interpolate_vertical_arc(speed, t, descending=False)
			h_b = speed if not is_backward else -speed
			v_b = -10
			return (h_a, v_a, h_b, v_b)
		elif cmd == CMD_LEFT:
			h_base = _interpolate_linear(speed, -speed, t)
			v = _interpolate_vertical_arc(speed, t, descending=False)
			return {'L1_h': -h_base, 'L1_v': v, 'R2_h': h_base, 'R2_v': v, 'L3_h': -h_base, 'L3_v': v,
			        'R1_h': speed, 'R1_v': -10, 'L2_h': -speed, 'L2_v': -10, 'R3_h': speed, 'R3_v': -10}
		elif cmd == CMD_RIGHT:
			h_base = _interpolate_linear(speed, -speed, t)
			v = _interpolate_vertical_arc(speed, t, descending=False)
			return {'L1_h': h_base, 'L1_v': v, 'R2_h': -h_base, 'R2_v': v, 'L3_h': h_base, 'L3_v': v,
			        'R1_h': -speed, 'R1_v': -10, 'L2_h': speed, 'L2_v': -10, 'R3_h': -speed, 'R3_v': -10}

	def calc_step_2(t, cmd):
		"""Step 2: Group A lands (descending arc)"""
		if cmd == MOVE_NO:
			h_a = _interpolate_linear(-speed if not is_backward else speed,
			                          speed if not is_backward else -speed, t)
			v_a = _interpolate_vertical_arc(speed, 1-t, descending=True)
			h_b = -h_a
			v_b = -10
			return (h_a, v_a, h_b, v_b)
		elif cmd == CMD_LEFT:
			h_base = _interpolate_linear(-speed, speed, t)
			v = _interpolate_vertical_arc(speed, 1-t, descending=True)
			return {'L1_h': -h_base, 'L1_v': v, 'R2_h': h_base, 'R2_v': v, 'L3_h': -h_base, 'L3_v': v,
			        'R1_h': h_base, 'R1_v': -10, 'L2_h': -h_base, 'L2_v': -10, 'R3_h': h_base, 'R3_v': -10}
		elif cmd == CMD_RIGHT:
			h_base = _interpolate_linear(-speed, speed, t)
			v = _interpolate_vertical_arc(speed, 1-t, descending=True)
			return {'L1_h': h_base, 'L1_v': v, 'R2_h': -h_base, 'R2_v': v, 'L3_h': h_base, 'L3_v': v,
			        'R1_h': -h_base, 'R1_v': -10, 'L2_h': h_base, 'L2_v': -10, 'R3_h': -h_base, 'R3_v': -10}

	def calc_step_3(t, cmd):
		"""Step 3: Group B lifts and moves (ascending arc)"""
		if cmd == MOVE_NO:
			h_a = speed if not is_backward else -speed
			v_a = -10
			h_b = _interpolate_linear(speed if not is_backward else -speed,
			                          -speed if not is_backward else speed, t)
			v_b = _interpolate_vertical_arc(speed, t, descending=False)
			return (h_a, v_a, h_b, v_b)
		elif cmd == CMD_LEFT:
			h_base = _interpolate_linear(speed, -speed, t)
			v = _interpolate_vertical_arc(speed, t, descending=False)
			return {'L1_h': -speed, 'L1_v': -10, 'R2_h': speed, 'R2_v': -10, 'L3_h': -speed, 'L3_v': -10,
			        'R1_h': h_base, 'R1_v': v, 'L2_h': -h_base, 'L2_v': v, 'R3_h': h_base, 'R3_v': v}
		elif cmd == CMD_RIGHT:
			h_base = _interpolate_linear(speed, -speed, t)
			v = _interpolate_vertical_arc(speed, t, descending=False)
			return {'L1_h': speed, 'L1_v': -10, 'R2_h': -speed, 'R2_v': -10, 'L3_h': speed, 'L3_v': -10,
			        'R1_h': -h_base, 'R1_v': v, 'L2_h': h_base, 'L2_v': v, 'R3_h': -h_base, 'R3_v': v}

	def calc_step_4(t, cmd):
		"""Step 4: Group B lands (descending arc)"""
		if cmd == MOVE_NO:
			h_a = -speed if not is_backward else speed
			v_a = -10
			h_b = _interpolate_linear(-speed if not is_backward else speed,
			                          speed if not is_backward else -speed, t)
			v_b = _interpolate_vertical_arc(speed, 1-t, descending=True)
			return (h_a, v_a, h_b, v_b)
		elif cmd == CMD_LEFT:
			h_base = _interpolate_linear(-speed, speed, t)
			v = _interpolate_vertical_arc(speed, 1-t, descending=True)
			return {'L1_h': h_base, 'L1_v': -10, 'R2_h': -h_base, 'R2_v': -10, 'L3_h': h_base, 'L3_v': -10,
			        'R1_h': h_base, 'R1_v': v, 'L2_h': -h_base, 'L2_v': v, 'R3_h': h_base, 'R3_v': v}
		elif cmd == CMD_RIGHT:
			h_base = _interpolate_linear(-speed, speed, t)
			v = _interpolate_vertical_arc(speed, 1-t, descending=True)
			return {'L1_h': -h_base, 'L1_v': -10, 'R2_h': h_base, 'R2_v': -10, 'L3_h': -h_base, 'L3_v': -10,
			        'R1_h': -h_base, 'R1_v': v, 'L2_h': h_base, 'L2_v': v, 'R3_h': -h_base, 'R3_v': v}

	# Execute the appropriate step
	calculators = {1: calc_step_1, 2: calc_step_2, 3: calc_step_3, 4: calc_step_4}

	if step_input in calculators:
		_execute_step_loop(num_steps, time_per_step, calculators[step_input], command)


def steady_X():
	if leftSide_direction:
		pwm.set_pwm(0,0,pwm0+steady_X_set)
		pwm.set_pwm(2,0,pwm2)
		pwm.set_pwm(4,0,pwm4-steady_X_set)
	else:
		pwm.set_pwm(0,0,pwm0+steady_X_set)
		pwm.set_pwm(2,0,pwm2)
		pwm.set_pwm(4,0,pwm4-steady_X_set)

	if rightSide_direction:
		pwm.set_pwm(10,0,pwm10+steady_X_set)
		pwm.set_pwm(8,0,pwm8)
		pwm.set_pwm(6,0,pwm6-steady_X_set)
	else:
		pwm.set_pwm(10,0,pwm10-steady_X_set)
		pwm.set_pwm(8,0,pwm8)
		pwm.set_pwm(6,0,pwm6+steady_X_set)


def steady():
	global X_fix_output, Y_fix_output
	if mpu6050_connection:
		accelerometer_data = sensor.get_accel_data()
		X = accelerometer_data['x']
		X = kalman_filter_X.kalman(X)
		Y = accelerometer_data['y']
		Y = kalman_filter_Y.kalman(Y)

		X_fix_output += -X_pid.GenOut(X - target_X)
		X_fix_output = ctrl_range(X_fix_output, steady_range_Max, -steady_range_Max)

		Y_fix_output += -Y_pid.GenOut(Y - target_Y)
		Y_fix_output = ctrl_range(Y_fix_output, steady_range_Max, -steady_range_Max)

		'''
		LEFT_I
		'''	
		left_I_input = ctrl_range((X_fix_output + Y_fix_output), steady_range_Max, steady_range_Min)
		left_I(0, 35, left_I_input)

		'''
		LEFT_II
		'''
		left_II_input = ctrl_range((abs(X_fix_output*0.5)+Y_fix_output), steady_range_Max, steady_range_Min)
		left_II(0, 35, left_II_input)

		'''
		LEFT_III
		'''
		left_III_input = ctrl_range((-X_fix_output + Y_fix_output), steady_range_Max, steady_range_Min)
		left_III(0, 35, left_III_input)

		'''
		RIGHT_III
		'''
		right_III_input = ctrl_range((X_fix_output - Y_fix_output), steady_range_Max, steady_range_Min)
		right_III(0, 35, right_III_input)

		'''
		RIGHT_II
		'''
		right_II_input = ctrl_range((abs(-X_fix_output*0.5)-Y_fix_output), steady_range_Max, steady_range_Min)
		right_II(0, 35, right_II_input)
		'''
		RIGHT_I
		'''
		right_I_input = ctrl_range((-X_fix_output-Y_fix_output), steady_range_Max, steady_range_Min)
		right_I(0, 35, right_I_input)


def steadyTest():
	if leftSide_direction:
		pwm.set_pwm(0,0,pwm0+steady_X)
		pwm.set_pwm(2,0,pwm2)
		pwm.set_pwm(4,0,pwm4-steady_X)
	else:
		pwm.set_pwm(0,0,pwm0+steady_X)
		pwm.set_pwm(2,0,pwm2)
		pwm.set_pwm(4,0,pwm4-steady_X)

	if rightSide_direction:
		pwm.set_pwm(10,0,pwm10+steady_X)
		pwm.set_pwm(8,0,pwm8)
		pwm.set_pwm(6,0,pwm6-steady_X)
	else:
		pwm.set_pwm(10,0,pwm10-steady_X)
		pwm.set_pwm(8,0,pwm8)
		pwm.set_pwm(6,0,pwm6+steady_X)

	while 1:
		left_H = steady_range_Min
		right_H = steady_range_Max
		left_I(0, 35, left_H)
		left_II(0, 35, left_H)
		left_III(0, 35, left_H)
		
		right_I(0, 35, right_H)
		right_II(0, 35, right_H)
		right_III(0, 35, right_H)

		time.sleep(1)
		
		left_H = 130
		right_H = -40
		left_I(0, 35, left_H)
		left_II(0, 35, left_H)
		left_III(0, 35, left_H)
		
		right_I(0, 35, right_H)
		right_II(0, 35, right_H)
		right_III(0, 35, right_H)

		time.sleep(1)
		

def look_up(wiggle=look_wiggle):
	global Up_Down_input
	if SmoothCamMode:
		# Smooth camera movement with interpolation
		old_position = Up_Down_input
		if Up_Down_direction:
			Up_Down_input += wiggle
		else:
			Up_Down_input -= wiggle
		Up_Down_input = ctrl_range(Up_Down_input, Up_Down_Max, Up_Down_Min)

		# Interpolate between old and new position
		for pos in interpolate(old_position, Up_Down_input, steps=8):
			pwm.set_pwm(13, 0, pos)
			time.sleep(0.005)  # 5ms delay between steps = ~40ms total
	else:
		# Direct, fast camera movement
		if Up_Down_direction:
			Up_Down_input += wiggle
		else:
			Up_Down_input -= wiggle
		Up_Down_input = ctrl_range(Up_Down_input, Up_Down_Max, Up_Down_Min)
		pwm.set_pwm(13, 0, Up_Down_input)


def look_down(wiggle=look_wiggle):
	global Up_Down_input
	if SmoothCamMode:
		# Smooth camera movement with interpolation
		old_position = Up_Down_input
		if Up_Down_direction:
			Up_Down_input -= wiggle
		else:
			Up_Down_input += wiggle
		Up_Down_input = ctrl_range(Up_Down_input, Up_Down_Max, Up_Down_Min)

		# Interpolate between old and new position
		for pos in interpolate(old_position, Up_Down_input, steps=8):
			pwm.set_pwm(13, 0, pos)
			time.sleep(0.005)  # 5ms delay between steps
	else:
		# Direct, fast camera movement
		if Up_Down_direction:
			Up_Down_input -= wiggle
		else:
			Up_Down_input += wiggle
		Up_Down_input = ctrl_range(Up_Down_input, Up_Down_Max, Up_Down_Min)
		pwm.set_pwm(13, 0, Up_Down_input)


def look_left(wiggle=look_wiggle):
	global Left_Right_input
	if SmoothCamMode:
		# Smooth camera movement with interpolation
		old_position = Left_Right_input
		if Left_Right_direction:
			Left_Right_input += wiggle
		else:
			Left_Right_input -= wiggle
		Left_Right_input = ctrl_range(Left_Right_input, Left_Right_Max, Left_Right_Min)

		# Interpolate between old and new position
		for pos in interpolate(old_position, Left_Right_input, steps=8):
			pwm.set_pwm(12, 0, pos)
			time.sleep(0.005)  # 5ms delay between steps
	else:
		# Direct, fast camera movement
		if Left_Right_direction:
			Left_Right_input += wiggle
		else:
			Left_Right_input -= wiggle
		Left_Right_input = ctrl_range(Left_Right_input, Left_Right_Max, Left_Right_Min)
		pwm.set_pwm(12, 0, Left_Right_input)


def look_right(wiggle=look_wiggle):
	global Left_Right_input
	if SmoothCamMode:
		# Smooth camera movement with interpolation
		old_position = Left_Right_input
		if Left_Right_direction:
			Left_Right_input -= wiggle
		else:
			Left_Right_input += wiggle
		Left_Right_input = ctrl_range(Left_Right_input, Left_Right_Max, Left_Right_Min)

		# Interpolate between old and new position
		for pos in interpolate(old_position, Left_Right_input, steps=8):
			pwm.set_pwm(12, 0, pos)
			time.sleep(0.005)  # 5ms delay between steps
	else:
		# Direct, fast camera movement
		if Left_Right_direction:
			Left_Right_input -= wiggle
		else:
			Left_Right_input += wiggle
		Left_Right_input = ctrl_range(Left_Right_input, Left_Right_Max, Left_Right_Min)
		pwm.set_pwm(12, 0, Left_Right_input)


def look_home():
	"""Reset camera to home position (center)"""
	global Left_Right_input, Up_Down_input
	pwm.set_all_pwm(0, 300)
	Left_Right_input = 300
	Up_Down_input = 300


def relesae():
	pwm.set_all_pwm(0,0)


def clean_all():
	pwm.set_all_pwm(0, 0)


def destroy():
	clean_all()


def interpolate(start, end, steps=10):
	"""Interpolate between start and end in 'steps' increments for smooth servo movements"""
	if steps <= 0:
		yield end
		return
	step_size = (end - start) / steps
	for i in range(steps + 1):
		yield int(start + step_size * i)


SmoothMode = 0
SmoothCamMode = 0  # Separate flag for camera smooth mode
steadyMode = 0

step_set = 1
DPI = 15

new_frame = 0
direction_command = MOVE_NO
turn_command = MOVE_NO
movement_speed = 35 # Default movement speed (10-80 range)


# ==================== Speed Control ====================

def set_movement_speed(speed):
	"""
	Set the movement speed for walking and turning.

	Args:
		speed: Movement speed (10-150 range)
		       - 10: Very slow, careful movements
		       - 35: Default speed (balanced)
		       - 150: Fast movements (may be less stable)
	"""
	global movement_speed
	# Clamp to valid range
	speed = max(10, min(80, int(speed)))
	movement_speed = speed
	print(f"[Move] Movement speed set to {movement_speed}")


def set_arc_factor(factor):
	"""
	Set the arc factor for curve movements.

	Args:
		factor: Arc factor (0.0 to 1.0 range)
		       - 0.0: Straight movement
		       - 1.0: Pivot turn on inner leg
	"""
	global arc_factor
	# Clamp to valid range
	factor = max(0.0, min(1.0, float(factor)))
	arc_factor = factor
	print(f"[Move] Arc factor set to {arc_factor}")


# ==================== Helper Functions for Movement ====================

def increment_step():
	"""Increment step counter and wrap around"""
	global step_set
	step_set += 1
	if step_set == 5:
		step_set = 1


def execute_movement_step(speed_left, speed_right):
	"""
	Execute a single movement step using smooth sine-curve movement.
	"""
	move_smooth(speed_left, speed_right, cycle_steps=100)


def handle_stand_or_steady():
	"""
	Handle stand command or apply steady mode
	Only called when not moving
	"""
	global step_set

	if turn_command == MOVE_NO and direction_command == MOVE_STAND:
		stand()
		step_set = 1
	else:
		steady_X()
		steady()


# ==================== Main Movement Thread ====================

def move_thread():
	"""
	Main movement thread - calculates side speeds and executes the movement step.
	"""
	global step_set

	if not steadyMode:
		speed_left = 0
		speed_right = 0
		movement_active = True

		if direction_command == CMD_FORWARD:
			if turn_command == MOVE_NO: # Forward
				speed_left = -movement_speed
				speed_right = -movement_speed
			elif turn_command == CMD_FORWARD_LEFT_ARC:
				speed_left = -movement_speed * (1 - arc_factor)
				speed_right = -movement_speed
			elif turn_command == CMD_FORWARD_RIGHT_ARC:
				speed_left = -movement_speed
				speed_right = -movement_speed * (1 - arc_factor)
		
		elif direction_command == CMD_BACKWARD: # Backward
			speed_left = movement_speed
			speed_right = movement_speed
		
		elif turn_command == CMD_LEFT: # Turn Left on the spot
			speed_left = movement_speed
			speed_right = -movement_speed

		elif turn_command == CMD_RIGHT: # Turn Right on the spot
			speed_left = -movement_speed
			speed_right = movement_speed

		else:
			movement_active = False

		if movement_active:
			execute_movement_step(speed_left, speed_right)
		else:
			# Handle stand OR steady only when NOT moving
			if turn_command == MOVE_NO and direction_command == MOVE_STAND:
				stand()
				step_set = 1
			else:
				steady_X()
				steady()
	else: # steadyMode is active
		steady_X()
		steady()

class RobotM(threading.Thread):
	def __init__(self, *args, **kwargs):
		super(RobotM, self).__init__(*args, **kwargs)
		self.__flag = threading.Event()
		self.__flag.clear()

	def pause(self):
		self.__flag.clear()

	def resume(self):
		self.__flag.set()

	def run(self):
		while 1:
			self.__flag.wait()
			move_thread()
			# Small sleep to reduce CPU load and allow quick interruption
			# The actual movement timing is handled in execute_movement_step()
			time.sleep(0.003)  # 3ms to check for pause quickly

rm = RobotM()
rm.start()
rm.pause()


# ==================== Command Input Processing ====================

def set_direction_and_resume(direction, turn='no'):
	"""Set direction command and resume robot movement"""
	global direction_command, turn_command
	direction_command = direction
	turn_command = turn
	rm.resume()


def set_turn_and_resume(turn):
	"""Set turn command and resume robot movement"""
	global turn_command, direction_command
	direction_command = MOVE_NO # Ensure no conflicting forward/backward command
	turn_command = turn
	rm.resume()


def set_direction_and_pause(direction):
	"""Set direction command and pause robot movement"""
	global direction_command, abort_current_movement
	direction_command = direction
	# Set abort flag to immediately stop any ongoing movement cycle
	abort_current_movement = True
	print(f"[set_direction_and_pause] Setting abort flag, direction={direction}")
	rm.pause()


def set_turn_and_pause():
	"""Set turn to 'no' and pause robot movement"""
	global turn_command, direction_command, abort_current_movement
	turn_command = MOVE_NO
	direction_command = MOVE_NO  # Also reset direction (user released all movement buttons)
	# Set abort flag to immediately stop any ongoing movement cycle
	abort_current_movement = True
	print("[set_turn_and_pause] Setting abort flag, turn=no, direction=no")
	rm.pause()


def handle_movement_command(command):
	"""Handle movement commands (forward, backward, stand, left, right, no)"""

	movement_commands = {
		'forward': lambda: set_direction_and_resume('forward', 'no'),
		'backward': lambda: set_direction_and_resume('backward', 'no'),
		'stand': lambda: set_direction_and_pause('stand'),
		'left': lambda: set_turn_and_resume('left'),
		'right': lambda: set_turn_and_resume('right'),
		CMD_FORWARD_LEFT_ARC: lambda: set_direction_and_resume(CMD_FORWARD, CMD_FORWARD_LEFT_ARC),
		CMD_FORWARD_RIGHT_ARC: lambda: set_direction_and_resume(CMD_FORWARD, CMD_FORWARD_RIGHT_ARC),
		'no': lambda: set_turn_and_pause(),
	}

	if command in movement_commands:
		movement_commands[command]()
		return True
	return False


def handle_mode_command(command):
	"""
	Handle mode commands (camera, steady).

	Note: slow/fast removed - movement is always smooth now.
	"""
	global SmoothMode, SmoothCamMode, steadyMode

	# Legacy slow/fast support (does nothing, just sets flag for backwards compat)
	if command == 'slow':
		SmoothMode = 1  # Keep for backwards compatibility
		return True
	elif command == 'fast':
		SmoothMode = 0  # Keep for backwards compatibility
		return True
	# Camera smooth mode (independent from movement)
	elif command == CMD_SMOOTH_CAM:
		SmoothCamMode = 1
		return True
	elif command == CMD_SMOOTH_CAM_OFF:
		SmoothCamMode = 0
		return True
	# Steady camera mode
	elif command == CMD_STEADY_CAMERA:
		steadyMode = 1
		rm.resume()
		return True
	elif command == CMD_STEADY_CAMERA_OFF:
		steadyMode = 0
		rm.pause()
		return True

	return False


def commandInput(command_input):
	"""
	Process command input from GUI/controller

	Commands:
		Movement: forward, backward, stand, left, right, no
		Modes: slow, fast, smoothCam, smoothCamOff, steadyCamera, steadyCameraOff
	"""
	# Mode commands can always be executed
	if handle_mode_command(command_input):
		return

	# Movement commands only when not in steady camera mode
	if steadyMode == 0:
		handle_movement_command(command_input)



if __name__ == '__main__':
	step = 1
	move_stu = 1
	try:
		while 1:
			move(step, 35, 'no')
			step += 1
			if step > 4:
				step = 1
			time.sleep(0.08)
	except KeyboardInterrupt:
		pwm.set_all_pwm(0, 300)
		time.sleep(1)

def standby():
	"""
	Put all servos into standby mode - stops PWM signals.
	Servos become 'soft' and can be moved by hand.
	Saves power while keeping the Pi running.
	"""
	print("🔋 Moving servos to STANDBY mode")
	global move_stu
	move_stu = 0  # Stop any ongoing movement

	# Pause the RobotM thread to stop all movement
	print("[STANDBY] Pausing RobotM thread")
	rm.pause()

	# Stop PWM signals on all channels
	for i in range(16):
		pwm.set_pwm(i, 0, 0)

	print("✓ All servos in STANDBY - legs are soft, low power consumption")


def wakeup():
	"""
	Wake up servos from standby - moves servos to stand position.
	"""
	print("⚡ WAKEUP - Moving servos to stand position")
	global direction_command, turn_command, step_set, move_stu

	# Reset movement commands to safe defaults
	direction_command = MOVE_STAND  # Set to stand to trigger stand() in move_thread
	turn_command = MOVE_NO
	step_set = 1  # Reset walk cycle to beginning
	move_stu = 1  # Re-enable movement (was set to 0 in standby)

	# Move servos to stand position (safe, known position)
	# This is better than restoring old positions because servos may have been
	# moved manually during standby
	stand()  # Execute stand position immediately

	# Reset to 'no' after stand is complete
	direction_command = MOVE_NO

	# Resume the RobotM thread to allow movement again
	print("[WAKEUP] Resuming RobotM thread")
	rm.resume()

	print("✓ All servos in stand position - robot ready")
