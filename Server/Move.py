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

def set_servo_smooth(channel, target_pos, steps=0):
	"""
	Move servo from current position to target position.
	Automatically updates servo_current_pos[channel].

	Args:
		channel: Servo channel (0-15)
		target_pos: Target PWM position
		steps: Number of interpolation steps (0 = direct move, no interpolation)
	"""
	global servo_current_pos

	if channel < 0 or channel > 15:
		return

	current = servo_current_pos[channel]

	# If already at target, just set it
	if abs(current - target_pos) < 2:
		pwm.set_pwm(channel, 0, target_pos)
		servo_current_pos[channel] = target_pos
		return

	# Direct move (no interpolation) for speed
	if steps == 0:
		pwm.set_pwm(channel, 0, target_pos)
		servo_current_pos[channel] = target_pos
	else:
		# Optional interpolation (not recommended in dove functions)
		for i in range(steps + 1):
			t = i / steps
			pos = int(current + (target_pos - current) * t)
			pwm.set_pwm(channel, 0, pos)
		# Update tracked position
		servo_current_pos[channel] = target_pos


def set_servo_immediate(channel, target_pos):
	"""
	Immediately set servo position without interpolation.
	Updates servo_current_pos[channel].

	Args:
		channel: Servo channel (0-15)
		target_pos: Target PWM position
	"""
	global servo_current_pos

	if channel < 0 or channel > 15:
		return

	pwm.set_pwm(channel, 0, target_pos)
	servo_current_pos[channel] = target_pos


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

		if command == 'no':
			right_I(step_I, intermediate_speed, 0)
			left_II(step_I, intermediate_speed, 0)
			right_III(step_I, intermediate_speed, 0)

			left_I(step_II, intermediate_speed, 0)
			right_II(step_II, intermediate_speed, 0)
			left_III(step_II, intermediate_speed, 0)
		elif command == 'left':
			right_I(step_I, intermediate_speed, 0)
			left_II(step_I, -intermediate_speed, 0)
			right_III(step_I, intermediate_speed, 0)

			left_I(step_II, -intermediate_speed, 0)
			right_II(step_II, intermediate_speed, 0)
			left_III(step_II, -intermediate_speed, 0)
		elif command == 'right':
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

def move_smooth(speed, command, cycle_steps=30):
	"""
	Smooth continuous movement using sine/cosine curves with position tracking.

	Args:
		speed: Movement amplitude (negative = forward, positive = backward)
		command: Movement command ('no', 'left', 'right')
		cycle_steps: Number of steps per full walking cycle (default 30)

	Uses actual leg position tracking to ensure smooth transitions even when
	direction changes. Interpolates from current position to target position.
	"""
	global _leg_positions, _last_command, _last_speed_sign

	# Detect command or direction change
	current_speed_sign = 1 if speed > 0 else -1 if speed < 0 else 0
	command_changed = (command != _last_command) or (current_speed_sign != _last_speed_sign and current_speed_sign != 0 and _last_speed_sign != 0)

	if command_changed:
		print(f"Movement change detected: cmd={_last_command}→{command}, dir={_last_speed_sign}→{current_speed_sign}")

	_last_command = command
	_last_speed_sign = current_speed_sign

	# Perform one complete walking cycle
	for step in range(cycle_steps):
		# Check if movement should stop
		if not move_stu:
			# Keep current positions when stopped (don't reset!)
			# This allows smooth continuation when movement resumes
			_last_command = None
			_last_speed_sign = 0
			break

		# Calculate current phase for this step (0.0 to 1.0)
		phase = step / cycle_steps

		# Calculate target positions for this phase
		target_positions = calculate_target_positions(phase, speed, command)

		# Interpolate from current to target (smooth transition)
		# Use stronger interpolation at the beginning of a cycle
		if command_changed and step < 5:
			# Stronger interpolation for first few steps after direction change
			alpha = 0.3 + (step / 5) * 0.7  # 0.3 → 1.0 over first 5 steps
		else:
			# Normal: move directly to target
			alpha = 1.0

		# Update and apply positions for each leg
		for leg in ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']:
			current = _leg_positions[leg]
			target = target_positions[leg]['h']
			vertical = target_positions[leg]['v']

			# Interpolate: new = current + alpha * (target - current)
			new_horizontal = int(current + alpha * (target - current))
			_leg_positions[leg] = new_horizontal

			# Apply to servos
			apply_leg_position(leg, new_horizontal, vertical)

		time.sleep(1.5 / cycle_steps)  # ~50ms per step = 1.5s per cycle


def calculate_target_positions(phase, speed, command):
	"""
	Calculate target horizontal and vertical positions for all legs at given phase.

	Args:
		phase: Current phase in cycle (0.0 to 1.0)
		speed: Movement amplitude (negative = forward, positive = backward)
		command: Movement command ('no', 'left', 'right')

	Returns:
		Dictionary with target positions for each leg: {'L1': {'h': ..., 'v': ...}, ...}
	"""
	positions = {}

	if command == 'no':
		# Forward/backward movement
		if phase < 0.5:
			# Group 1 (L1, R2, L3) in air
			t = phase * 2  # 0.0 to 1.0
			h1 = int(speed * math.cos(t * math.pi))
			v1 = int(3 * abs(speed) * math.sin(t * math.pi))

			# Group 2 (R1, L2, R3) on ground
			h2 = -h1
			v2 = -10

			positions['L1'] = {'h': h1, 'v': v1}
			positions['R2'] = {'h': h1, 'v': v1}
			positions['L3'] = {'h': h1, 'v': v1}
			positions['R1'] = {'h': h2, 'v': v2}
			positions['L2'] = {'h': h2, 'v': v2}
			positions['R3'] = {'h': h2, 'v': v2}
		else:
			# Group 2 (R1, L2, R3) in air
			t = (phase - 0.5) * 2  # 0.0 to 1.0
			h2 = int(speed * math.cos(t * math.pi))
			v2 = int(3 * abs(speed) * math.sin(t * math.pi))

			# Group 1 (L1, R2, L3) on ground
			h1 = -h2
			v1 = -10

			positions['L1'] = {'h': h1, 'v': v1}
			positions['R2'] = {'h': h1, 'v': v1}
			positions['L3'] = {'h': h1, 'v': v1}
			positions['R1'] = {'h': h2, 'v': v2}
			positions['L2'] = {'h': h2, 'v': v2}
			positions['R3'] = {'h': h2, 'v': v2}

	elif command == 'left':
		# Turn left: Right side walks forward (normal forward movement)
		#            Left side lifts/lowers synchronously but stays at h=0
		# This prevents left side from blocking the turn!
		if phase < 0.5:
			# Group 1 (L1, R2, L3) in air
			t = phase * 2
			h = int(abs(speed) * math.cos(t * math.pi))  # Full speed like forward!
			v = int(3 * abs(speed) * math.sin(t * math.pi))

			# Left legs (L1, L3) lift synchronously with right leg BUT stay at h=0
			positions['L1'] = {'h': 0, 'v': v}       # Left: lifts but doesn't move horizontally
			positions['L3'] = {'h': 0, 'v': v}       # Left: lifts but doesn't move horizontally

			# Right leg (R2) moves forward like normal
			positions['R2'] = {'h': h, 'v': v}       # Right: forward movement in air

			# Group 2 (R1, L2, R3) on ground
			positions['R1'] = {'h': h, 'v': -10}     # Right: pushes forward on ground
			positions['L2'] = {'h': 0, 'v': -10}     # Left: on ground at center
			positions['R3'] = {'h': h, 'v': -10}     # Right: pushes forward on ground
		else:
			# Group 2 (R1, L2, R3) in air
			t = (phase - 0.5) * 2
			h = int(abs(speed) * math.cos(t * math.pi))
			v = int(3 * abs(speed) * math.sin(t * math.pi))

			# Right legs (R1, R3) move forward in air
			positions['R1'] = {'h': h, 'v': v}       # Right: forward in air
			positions['R3'] = {'h': h, 'v': v}       # Right: forward in air

			# Left leg (L2) lifts synchronously BUT stays at h=0
			positions['L2'] = {'h': 0, 'v': v}       # Left: lifts but doesn't move horizontally

			# Group 1 (L1, R2, L3) on ground
			positions['L1'] = {'h': 0, 'v': -10}     # Left: on ground at center
			positions['R2'] = {'h': h, 'v': -10}     # Right: pushes forward on ground
			positions['L3'] = {'h': 0, 'v': -10}     # Left: on ground at center

	elif command == 'right':
		# Turn right: Left side walks forward, right side lifts/lowers synchronously
		if phase < 0.5:
			# Group 1 (L1, R2, L3) in air
			t = phase * 2
			h = int(abs(speed) * math.cos(t * math.pi))
			v = int(3 * abs(speed) * math.sin(t * math.pi))

			# Left legs (L1, L3) move forward
			positions['L1'] = {'h': h, 'v': v}       # Left: forward in air
			positions['L3'] = {'h': h, 'v': v}       # Left: forward in air

			# Right leg (R2) lifts synchronously BUT stays at h=0
			positions['R2'] = {'h': 0, 'v': v}       # Right: lifts but doesn't move horizontally

			# Group 2 on ground
			positions['R1'] = {'h': 0, 'v': -10}     # Right: on ground at center
			positions['L2'] = {'h': h, 'v': -10}     # Left: pushes forward on ground
			positions['R3'] = {'h': 0, 'v': -10}     # Right: on ground at center
		else:
			# Group 2 (R1, L2, R3) in air
			t = (phase - 0.5) * 2
			h = int(abs(speed) * math.cos(t * math.pi))
			v = int(3 * abs(speed) * math.sin(t * math.pi))

			# Left leg (L2) moves forward in air
			positions['L2'] = {'h': h, 'v': v}       # Left: forward in air

			# Right legs (R1, R3) lift synchronously BUT stay at h=0
			positions['R1'] = {'h': 0, 'v': v}       # Right: lifts but doesn't move horizontally
			positions['R3'] = {'h': 0, 'v': v}       # Right: lifts but doesn't move horizontally

			# Group 1 on ground
			positions['L1'] = {'h': h, 'v': -10}     # Left: pushes forward on ground
			positions['R2'] = {'h': 0, 'v': -10}     # Right: on ground at center
			positions['L3'] = {'h': h, 'v': -10}     # Left: pushes forward on ground

	return positions


def apply_leg_position(leg, horizontal, vertical):
	"""
	Apply horizontal and vertical position to a specific leg.

	Args:
		leg: Leg identifier ('L1', 'L2', 'L3', 'R1', 'R2', 'R3')
		horizontal: Horizontal offset (-speed to +speed)
		vertical: Vertical offset (positive = up, negative = down)
	"""
	if leg == 'L1':
		dove_Left_I(horizontal, vertical)
	elif leg == 'L2':
		dove_Left_II(horizontal, vertical)
	elif leg == 'L3':
		dove_Left_III(horizontal, vertical)
	elif leg == 'R1':
		dove_Right_I(horizontal, vertical)
	elif leg == 'R2':
		dove_Right_II(horizontal, vertical)
	elif leg == 'R3':
		dove_Right_III(horizontal, vertical)






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


'''
---Dove---
making the servo moves smooth.
'''
def dove_Left_I(horizontal, vertical):
	# Horizontal servo (channel 0)
	if leftSide_direction:
		target_h = pwm0 + horizontal
	else:
		target_h = pwm0 - horizontal
	set_servo_smooth(0, target_h, steps=0)

	# Vertical servo (channel 1)
	if leftSide_height:
		target_v = pwm1 + vertical
	else:
		target_v = pwm1 - vertical
	set_servo_smooth(1, target_v, steps=0)


def dove_Left_II(horizontal, vertical):
	# Horizontal servo (channel 2)
	if leftSide_direction:
		target_h = pwm2 + horizontal
	else:
		target_h = pwm2 - horizontal
	set_servo_smooth(2, target_h, steps=0)

	# Vertical servo (channel 3)
	if leftSide_height:
		target_v = pwm3 + vertical
	else:
		target_v = pwm3 - vertical
	set_servo_smooth(3, target_v, steps=0)


def dove_Left_III(horizontal, vertical):
	# Horizontal servo (channel 4)
	if leftSide_direction:
		target_h = pwm4 + horizontal
	else:
		target_h = pwm4 - horizontal
	set_servo_smooth(4, target_h, steps=0)

	# Vertical servo (channel 5)
	if leftSide_height:
		target_v = pwm5 + vertical
	else:
		target_v = pwm5 - vertical
	set_servo_smooth(5, target_v, steps=0)


def dove_Right_I(horizontal, vertical):
	# Horizontal servo (channel 6)
	if rightSide_direction:
		target_h = pwm6 + horizontal
	else:
		target_h = pwm6 - horizontal
	set_servo_smooth(6, target_h, steps=0)

	# Vertical servo (channel 7)
	if rightSide_height:
		target_v = pwm7 + vertical
	else:
		target_v = pwm7 - vertical
	set_servo_smooth(7, target_v, steps=0)


def dove_Right_II(horizontal, vertical):
	# Horizontal servo (channel 8)
	if rightSide_direction:
		target_h = pwm8 + horizontal
	else:
		target_h = pwm8 - horizontal
	set_servo_smooth(8, target_h, steps=0)

	# Vertical servo (channel 9)
	if rightSide_height:
		target_v = pwm9 + vertical
	else:
		target_v = pwm9 - vertical
	set_servo_smooth(9, target_v, steps=0)


def dove_Right_III(horizontal, vertical):
	# Horizontal servo (channel 10)
	if rightSide_direction:
		target_h = pwm10 + horizontal
	else:
		target_h = pwm10 - horizontal
	set_servo_smooth(10, target_h, steps=0)

	# Vertical servo (channel 11)
	if rightSide_height:
		target_v = pwm11 + vertical
	else:
		target_v = pwm11 - vertical
	set_servo_smooth(11, target_v, steps=0)


def dove_smooth(phase, speed, timeLast, command):
	"""
	Smooth continuous leg movement using sine/cosine curves.

	Args:
		phase: Current phase (0.0 to 1.0) of the walking cycle
		speed: Movement amplitude (horizontal range)
		timeLast: Time per cycle
		command: Movement command ('no', 'left', 'right')

	Phase mapping:
		0.0 - 0.5: Group 1 (L1, R2, L3) in air, Group 2 (R1, L2, R3) on ground
		0.5 - 1.0: Group 2 in air, Group 1 on ground
	"""
	import math

	# Calculate positions using sine/cosine for smooth transitions
	# Phase 0.0-0.5: Group 1 moves
	# Phase 0.5-1.0: Group 2 moves

	if command == 'no':
		# Forward/backward movement
		if phase < 0.5:
			# Group 1 (L1, R2, L3) in air, moving from +speed to -speed
			t = phase * 2  # 0.0 to 1.0

			# Horizontal: smooth transition from +speed to -speed using cosine
			h1 = int(speed * math.cos(t * math.pi))  # +speed → -speed

			# Vertical: smooth arc using sine (lift up and down)
			# Always use positive amplitude for vertical movement
			v1 = int(3 * abs(speed) * math.sin(t * math.pi))  # 0 → 3*speed → 0

			# Group 2 (R1, L2, R3) on ground, moving from -speed to +speed
			h2 = -h1  # Opposite horizontal position
			v2 = -10  # Stay on ground

			# Apply positions
			dove_Left_I(h1, v1)
			dove_Right_II(h1, v1)
			dove_Left_III(h1, v1)

			dove_Right_I(h2, v2)
			dove_Left_II(h2, v2)
			dove_Right_III(h2, v2)
		else:
			# Group 2 (R1, L2, R3) in air, moving from +speed to -speed
			t = (phase - 0.5) * 2  # 0.0 to 1.0

			# Horizontal: smooth transition
			h2 = int(speed * math.cos(t * math.pi))  # +speed → -speed

			# Vertical: smooth arc
			# Always use positive amplitude for vertical movement
			v2 = int(3 * abs(speed) * math.sin(t * math.pi))  # 0 → 3*speed → 0

			# Group 1 on ground
			h1 = -h2  # Opposite horizontal position
			v1 = -10  # Stay on ground

			# Apply positions
			dove_Left_I(h1, v1)
			dove_Right_II(h1, v1)
			dove_Left_III(h1, v1)

			dove_Right_I(h2, v2)
			dove_Left_II(h2, v2)
			dove_Right_III(h2, v2)

	elif command == 'left':
		# Turn left: left legs move backward, right legs move forward
		if phase < 0.5:
			t = phase * 2
			h = int(speed * math.cos(t * math.pi))
			v = int(3 * speed * math.sin(t * math.pi))

			dove_Left_I(-h, v)      # Left legs backward
			dove_Right_II(h, v)     # Right legs forward
			dove_Left_III(-h, v)

			dove_Right_I(h, -10)
			dove_Left_II(-h, -10)
			dove_Right_III(h, -10)
		else:
			t = (phase - 0.5) * 2
			h = int(speed * math.cos(t * math.pi))
			v = int(3 * speed * math.sin(t * math.pi))

			dove_Left_I(-h, -10)
			dove_Right_II(h, -10)
			dove_Left_III(-h, -10)

			dove_Right_I(h, v)
			dove_Left_II(-h, v)
			dove_Right_III(h, v)

	elif command == 'right':
		# Turn right: left legs move forward, right legs move backward
		if phase < 0.5:
			t = phase * 2
			h = int(speed * math.cos(t * math.pi))
			v = int(3 * speed * math.sin(t * math.pi))

			dove_Left_I(h, v)       # Left legs forward
			dove_Right_II(-h, v)    # Right legs backward
			dove_Left_III(h, v)

			dove_Right_I(-h, -10)
			dove_Left_II(h, -10)
			dove_Right_III(-h, -10)
		else:
			t = (phase - 0.5) * 2
			h = int(speed * math.cos(t * math.pi))
			v = int(3 * speed * math.sin(t * math.pi))

			dove_Left_I(h, -10)
			dove_Right_II(-h, -10)
			dove_Left_III(h, -10)

			dove_Right_I(-h, v)
			dove_Left_II(h, v)
			dove_Right_III(-h, v)


def dove(step_input, speed, timeLast, dpi, command):
	step_I  = step_input
	step_II = step_input + 2

	if step_II > 4:
		step_II = step_II - 4
	
	if speed > 0:
		if step_input == 1:
			# Ensure we interpolate exactly from +speed to -speed
			num_steps = dpi
			for i in range(num_steps + 1):
				if move_stu and command == 'no':
					# Gradual lift and forward motion for first group
					# Interpolate from +speed to -speed over num_steps
					t = i / num_steps  # 0.0 to 1.0
					horizontal_pos = int(speed - (2 * speed * t))  # +speed to -speed
					vertical_pos = int(3 * speed * t)              # 0 to 3*speed

					dove_Left_I(horizontal_pos, vertical_pos)
					dove_Right_II(horizontal_pos, vertical_pos)
					dove_Left_III(horizontal_pos, vertical_pos)

					dove_Right_I(speed, -10)  # Stay at +speed
					dove_Left_II(speed, -10)
					dove_Right_III(speed, -10)
					time.sleep(timeLast/dpi)
				else:
					pass

				if command == 'left':
					# Same logic for left turn: interpolate from +speed to -speed
					t = i / num_steps
					horizontal_pos = int(speed - (2 * speed * t))
					vertical_pos = int(3 * speed * t)

					dove_Left_I(-horizontal_pos, vertical_pos)
					dove_Right_II(horizontal_pos, vertical_pos)
					dove_Left_III(-horizontal_pos, vertical_pos)

					dove_Right_I(speed, -10)
					dove_Left_II(-speed, -10)
					dove_Right_III(speed, -10)
					time.sleep(timeLast/dpi)
				elif command == 'right':
					t = i / num_steps
					horizontal_pos = int(speed - (2 * speed * t))
					vertical_pos = int(3 * speed * t)

					dove_Left_I(horizontal_pos, vertical_pos)
					dove_Right_II(-horizontal_pos, vertical_pos)
					dove_Left_III(horizontal_pos, vertical_pos)

					dove_Right_I(-speed, -10)
					dove_Left_II(speed, -10)
					dove_Right_III(-speed, -10)
					time.sleep(timeLast/dpi)
				else:
					pass

				if move_stu == 0 and command == 'no':
					break

		elif step_input == 2:
			# Continue smoothly from step 1: interpolate from -speed to +speed
			num_steps = dpi
			for i in range(num_steps + 1):
				if move_stu and command == 'no':
					# Interpolate from -speed to +speed
					t = i / num_steps
					horizontal_pos = int(-speed + (2 * speed * t))  # -speed to +speed
					vertical_pos = int(3 * speed * (1 - t))        # 3*speed to 0

					dove_Left_I(horizontal_pos, vertical_pos)
					dove_Right_II(horizontal_pos, vertical_pos)
					dove_Left_III(horizontal_pos, vertical_pos)

					dove_Right_I(-horizontal_pos, -10)
					dove_Left_II(-horizontal_pos, -10)
					dove_Right_III(-horizontal_pos, -10)
					time.sleep(timeLast/dpi)
				else:
					pass

				if command == 'left':
					t = i / num_steps
					horizontal_pos = int(-speed + (2 * speed * t))
					vertical_pos = int(3 * speed * (1 - t))

					dove_Left_I(-horizontal_pos, vertical_pos)
					dove_Right_II(horizontal_pos, vertical_pos)
					dove_Left_III(-horizontal_pos, vertical_pos)

					dove_Right_I(-horizontal_pos, -10)
					dove_Left_II(horizontal_pos, -10)
					dove_Right_III(-horizontal_pos, -10)
					time.sleep(timeLast/dpi)
				elif command == 'right':
					t = i / num_steps
					horizontal_pos = int(-speed + (2 * speed * t))
					vertical_pos = int(3 * speed * (1 - t))

					dove_Left_I(horizontal_pos, vertical_pos)
					dove_Right_II(-horizontal_pos, vertical_pos)
					dove_Left_III(horizontal_pos, vertical_pos)

					dove_Right_I(horizontal_pos, -10)
					dove_Left_II(-horizontal_pos, -10)
					dove_Right_III(horizontal_pos, -10)
					time.sleep(timeLast/dpi)
				else:
					pass

				if move_stu == 0 and command == 'no':
					break
		elif step_input == 3:
			# Interpolate second group from +speed to -speed
			num_steps = dpi
			for i in range(num_steps + 1):
				if move_stu and command == 'no':
					# First group stays at +speed
					# Second group interpolates from +speed to -speed
					dove_Left_I(speed, -10)
					dove_Right_II(speed, -10)
					dove_Left_III(speed, -10)

					t = i / num_steps
					horizontal_pos = int(speed - (2 * speed * t))  # +speed to -speed
					vertical_pos = int(3 * speed * t)              # 0 to 3*speed

					dove_Right_I(horizontal_pos, vertical_pos)
					dove_Left_II(horizontal_pos, vertical_pos)
					dove_Right_III(horizontal_pos, vertical_pos)
					time.sleep(timeLast/dpi)
				else:
					pass

				if command == 'left':
					t = i / num_steps
					horizontal_pos = int(speed - (2 * speed * t))
					vertical_pos = int(3 * speed * t)

					dove_Left_I(-speed, -10)
					dove_Right_II(speed, -10)
					dove_Left_III(-speed, -10)

					dove_Right_I(horizontal_pos, vertical_pos)
					dove_Left_II(-horizontal_pos, vertical_pos)
					dove_Right_III(horizontal_pos, vertical_pos)
					time.sleep(timeLast/dpi)
				elif command == 'right':
					t = i / num_steps
					horizontal_pos = int(speed - (2 * speed * t))
					vertical_pos = int(3 * speed * t)

					dove_Left_I(speed, -10)
					dove_Right_II(-speed, -10)
					dove_Left_III(speed, -10)

					dove_Right_I(-horizontal_pos, vertical_pos)
					dove_Left_II(horizontal_pos, vertical_pos)
					dove_Right_III(-horizontal_pos, vertical_pos)
					time.sleep(timeLast/dpi)
				else:
					pass

				if move_stu == 0 and command == 'no':
					break
		elif step_input == 4:
			# Continue smoothly from step 3: interpolate from -speed to +speed
			num_steps = dpi
			for i in range(num_steps + 1):
				if move_stu and command == 'no':
					t = i / num_steps
					horizontal_pos = int(-speed + (2 * speed * t))  # -speed to +speed
					vertical_pos = int(3 * speed * (1 - t))        # 3*speed to 0

					dove_Left_I(-horizontal_pos, -10)
					dove_Right_II(-horizontal_pos, -10)
					dove_Left_III(-horizontal_pos, -10)

					dove_Right_I(horizontal_pos, vertical_pos)
					dove_Left_II(horizontal_pos, vertical_pos)
					dove_Right_III(horizontal_pos, vertical_pos)
					time.sleep(timeLast/dpi)
				else:
					pass

				if command == 'left':
					t = i / num_steps
					horizontal_pos = int(-speed + (2 * speed * t))
					vertical_pos = int(3 * speed * (1 - t))

					dove_Left_I(horizontal_pos, -10)
					dove_Right_II(-horizontal_pos, -10)
					dove_Left_III(horizontal_pos, -10)

					dove_Right_I(horizontal_pos, vertical_pos)
					dove_Left_II(-horizontal_pos, vertical_pos)
					dove_Right_III(horizontal_pos, vertical_pos)
					time.sleep(timeLast/dpi)
				elif command == 'right':
					t = i / num_steps
					horizontal_pos = int(-speed + (2 * speed * t))
					vertical_pos = int(3 * speed * (1 - t))

					dove_Left_I(-horizontal_pos, -10)
					dove_Right_II(horizontal_pos, -10)
					dove_Left_III(-horizontal_pos, -10)

					dove_Right_I(-horizontal_pos, vertical_pos)
					dove_Left_II(horizontal_pos, vertical_pos)
					dove_Right_III(-horizontal_pos, vertical_pos)
					time.sleep(timeLast/dpi)
				else:
					pass

				if move_stu == 0 and command == 'no':
					break
	else:
		speed = -speed
		if step_input == 1:
			# Backward movement: interpolate from -speed to +speed
			num_steps = dpi
			for i in range(num_steps + 1):
				if move_stu and command == 'no':
					t = i / num_steps
					horizontal_pos = int(-speed + (2 * speed * t))  # -speed to +speed
					vertical_pos = int(3 * speed * t)               # 0 to 3*speed

					dove_Left_I(horizontal_pos, vertical_pos)
					dove_Right_II(horizontal_pos, vertical_pos)
					dove_Left_III(horizontal_pos, vertical_pos)

					dove_Right_I(-speed, -10)  # Stay at -speed
					dove_Left_II(-speed, -10)
					dove_Right_III(-speed, -10)
					time.sleep(timeLast/dpi)
				else:
					pass
		elif step_input == 2:
			# Continue smoothly from step 1 for backward movement
			num_steps = dpi
			for i in range(num_steps + 1):
				if move_stu and command == 'no':
					t = i / num_steps
					horizontal_pos = int(speed - (2 * speed * t))  # +speed to -speed
					vertical_pos = int(3 * speed * (1 - t))       # 3*speed to 0

					dove_Left_I(-horizontal_pos, vertical_pos)
					dove_Right_II(-horizontal_pos, vertical_pos)
					dove_Left_III(-horizontal_pos, vertical_pos)

					dove_Right_I(horizontal_pos, -10)
					dove_Left_II(horizontal_pos, -10)
					dove_Right_III(horizontal_pos, -10)
					time.sleep(timeLast/dpi)
				else:
					pass
		elif step_input == 3:
			# Backward movement second group: interpolate from -speed to +speed
			num_steps = dpi
			for i in range(num_steps + 1):
				if move_stu and command == 'no':
					dove_Left_I(-speed, -10)  # Stay at -speed
					dove_Right_II(-speed, -10)
					dove_Left_III(-speed, -10)

					t = i / num_steps
					horizontal_pos = int(-speed + (2 * speed * t))  # -speed to +speed
					vertical_pos = int(3 * speed * t)               # 0 to 3*speed

					dove_Right_I(horizontal_pos, vertical_pos)
					dove_Left_II(horizontal_pos, vertical_pos)
					dove_Right_III(horizontal_pos, vertical_pos)
					time.sleep(timeLast/dpi)
				else:
					pass
		elif step_input == 4:
			# Continue smoothly from step 3 for backward movement
			num_steps = dpi
			for i in range(num_steps + 1):
				if move_stu and command == 'no':
					t = i / num_steps
					horizontal_pos = int(speed - (2 * speed * t))  # +speed to -speed
					vertical_pos = int(3 * speed * (1 - t))       # 3*speed to 0

					dove_Left_I(horizontal_pos, -10)
					dove_Right_II(horizontal_pos, -10)
					dove_Left_III(horizontal_pos, -10)

					dove_Right_I(-horizontal_pos, vertical_pos)
					dove_Left_II(-horizontal_pos, vertical_pos)
					dove_Right_III(-horizontal_pos, vertical_pos)
					time.sleep(timeLast/dpi)
				else:
					pass


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


def home():
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
direction_command = 'no'
turn_command = 'no'


# ==================== Helper Functions for Movement ====================

def increment_step():
	"""Increment step counter and wrap around"""
	global step_set
	step_set += 1
	if step_set == 5:
		step_set = 1


def execute_movement_step(speed, turn='no'):
	"""
	Execute a single movement step using smooth sine-curve movement.

	Args:
		speed: Movement speed (negative for forward, positive for backward)
		turn: Turn command ('left', 'right', or 'no')

	Note: Always uses smooth movement with sine curves (old normal mode removed).
	      Speed parameter can be adjusted in the future for variable speeds.
	"""
	# Always use smooth movement with sine curves
	# Old step-based move() function is deprecated
	# Pass speed with sign - negative = forward, positive = backward
	move_smooth(speed, turn, cycle_steps=30)



def handle_direction_movement():
	"""
	Handle forward/backward movement
	Returns True if movement was executed, False otherwise
	"""
	if direction_command == 'forward' and turn_command == 'no':
		execute_movement_step(35, 'no')
		return True
	elif direction_command == 'backward' and turn_command == 'no':
		execute_movement_step(-35, 'no')
		return True
	return False


def handle_turn_movement():
	"""
	Handle left/right turning
	Returns True if turn was executed, False otherwise
	"""
	if turn_command != 'no':
		execute_movement_step(20, turn_command)
		return True
	return False


def handle_stand_or_steady():
	"""
	Handle stand command or apply steady mode
	Only called when not moving
	"""
	global step_set

	if turn_command == 'no' and direction_command == 'stand':
		stand()
		step_set = 1
	else:
		steady_X()
		steady()


# ==================== Main Movement Thread ====================

def move_thread():
	"""
	Main movement thread - coordinates all movement commands

	This function is called repeatedly by the RobotM thread.
	Original logic:
	1. Check and execute directional movement (forward OR backward)
	2. Check and execute turn movement (if turn_command set)
	3. Handle stand OR steady only when NOT moving
	"""
	global step_set

	if not steadyMode:
		# Track if any movement was executed
		movement_executed = False

		# Step 1: Handle directional movement (forward/backward)
		# Only one of these will execute per cycle
		if direction_command == 'forward' and turn_command == 'no':
			execute_movement_step(-35, 'no')  # Negative = forward (legs pull forward)
			movement_executed = True
		elif direction_command == 'backward' and turn_command == 'no':
			execute_movement_step(35, 'no')  # Positive = backward (legs push back)
			movement_executed = True

		# Step 2: Handle turn movement (independent of directional movement)
		if turn_command != 'no':
			execute_movement_step(20, turn_command)
			movement_executed = True

		# Step 3: ONLY apply stand/steady when NO movement is happening
		# This prevents the jerky return to center position between cycles
		if not movement_executed:
			if turn_command == 'no' and direction_command == 'stand':
				stand()
				step_set = 1
			else:
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
			time.sleep(0.01)  # 10ms to check for pause quickly

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
	global turn_command
	turn_command = turn
	rm.resume()


def set_direction_and_pause(direction):
	"""Set direction command and pause robot movement"""
	global direction_command
	direction_command = direction
	rm.pause()


def set_turn_and_pause():
	"""Set turn to 'no' and pause robot movement"""
	global turn_command
	turn_command = 'no'
	rm.pause()


def handle_movement_command(command):
	"""Handle movement commands (forward, backward, stand, left, right, no)"""

	movement_commands = {
		'forward': lambda: set_direction_and_resume('forward', 'no'),
		'backward': lambda: set_direction_and_resume('backward', 'no'),
		'stand': lambda: set_direction_and_pause('stand'),
		'left': lambda: set_turn_and_resume('left'),
		'right': lambda: set_turn_and_resume('right'),
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
	elif command == 'smoothCam':
		SmoothCamMode = 1
		return True
	elif command == 'smoothCamOff':
		SmoothCamMode = 0
		return True
	# Steady camera mode
	elif command == 'steadyCamera':
		steadyMode = 1
		rm.resume()
		return True
	elif command == 'steadyCameraOff':
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
