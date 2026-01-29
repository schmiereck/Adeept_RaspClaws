#!/usr/bin/env/python3
# File name   : Move.py
# Website	 : www.Adeept.com
# Author	  : Adeept
# Date		: 2025/04/16
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

# Global state for continuous, non-blocking movement
gait_phase = 0.0  # Current phase of the walk cycle (0.0 to 1.0)
CYCLE_STEPS = 60  # Number of increments for a full walk cycle

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
_steps_since_change = 0  # Counter for steps since last direction/command change
_direction_changed = False  # Flag indicating if direction recently changed

# FT40 Phase 4: Track stop duration to decide when to reset phase
_stop_counter = 0  # Counter for how many iterations we've been stopped
_stop_threshold = 30  # Reset phase after ~0.5 seconds of stop (30 iterations at ~60Hz)


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
pwm = None  # Will be initialized by init_all() or explicit call


def initialize_pwm():
	"""
	Initialisiert den PCA9685 Servo-Controller.
	Wird von init_all() automatisch aufgerufen oder kann manuell aufgerufen werden.

	Returns:
		pwm object (Adafruit_PCA9685 or MockPWM)
	"""
	global pwm

	if pwm is not None:
		print("⚠️  PCA9685 in Move.py bereits initialisiert - überspringe")
		return pwm

	# Erst RPIservo initialisieren (falls noch nicht geschehen)
	RPIservo.initialize_pwm()

	# Dann eigene pwm-Referenz setzen
	pwm = RPIservo.pwm
	print("✓ Move.py verwendet PCA9685 von RPIservo.py")
	return pwm

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
		step_delay = 0.01 # Small delay between interpolation steps (10ms)
		print(f"    [set_servo_smooth] Channel {channel}: Interpolating from {current} to {target_pos} in {steps} steps with delay {step_delay}s")
		for i in range(steps + 1):
			t = i / steps
			pos = int(current + (target_pos - current) * t)
			pwm.set_pwm(channel, 0, pos)
			time.sleep(step_delay) # Added sleep here
		# Update tracked position
		servo_current_pos[channel] = target_pos
		print(f"    [set_servo_smooth] Channel {channel}: Interpolation finished at {servo_current_pos[channel]}")


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
	global servo_current_pos, pwm

	# Initialize PWM if not done yet (lazy initialization)
	if pwm is None:
		initialize_pwm()

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


# KEIN automatisches init_all() beim Import!
# Die Servos werden erst initialisiert, wenn init_all() explizit aufgerufen wird
# (z.B. von GUIServer.py oder ROSServer.py)
print("⏸️  Move.py geladen - Servos NICHT initialisiert")
print("   📌 Rufe Move.init_all() auf, um Servos zu aktivieren")


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
		   /	 \
		  /	   \
		 /		 \
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


def leg_control(leg_name, pos, wiggle, height_adjust=0):
	"""
	Generic leg control function for all 6 legs.
	Replaces the 6 individual left_I/II/III and right_I/II/III functions.

	Args:
		leg_name: Name of the leg ('left_I', 'left_II', 'left_III', 'right_I', 'right_II', 'right_III')
		pos: Position in walk cycle (0-4)
		wiggle: Movement range (speed parameter)
		height_adjust: Height adjustment for pos=0
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
			pwm.set_pwm(v_channel, 0, base_v + height_adjust)
		else:
			pwm.set_pwm(v_channel, 0, base_v - height_adjust)
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

def left_I(pos, wiggle, height_adjust=0):
	"""Left leg I - uses generic leg_control"""
	leg_control('left_I', pos, wiggle, height_adjust)


def left_II(pos, wiggle, height_adjust=0):
	"""Left leg II - uses generic leg_control"""
	leg_control('left_II', pos, wiggle, height_adjust)


def left_III(pos, wiggle, height_adjust=0):
	"""Left leg III - uses generic leg_control"""
	leg_control('left_III', pos, wiggle, height_adjust)


def right_I(pos, wiggle, height_adjust=0):
	"""Right leg I - uses generic leg_control"""
	leg_control('right_I', pos, wiggle, height_adjust)


def right_II(pos, wiggle, height_adjust=0):
	"""Right leg II - uses generic leg_control"""
	leg_control('right_II', pos, wiggle, height_adjust)


def right_III(pos, wiggle, height_adjust=0):
	"""Right leg III - uses generic leg_control"""
	leg_control('right_III', pos, wiggle, height_adjust)


# ==================== OLD IMPLEMENTATIONS REMOVED ====================
# The old 360 lines of duplicated code for the 6 leg functions have been removed.
# They have been replaced by the generic leg_control() function above.
# This removed code duplication and improved maintainability significantly.

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


def lower_legs_smoothly(target_vertical_offset=-10, interpolation_steps=10):
	"""
	Smoothly lowers all legs to a stable ground position.
	Horizontal servos are also reset to their base position smoothly.

	Args:
		target_vertical_offset: The target vertical offset (e.g., -10)
		interpolation_steps: Number of steps for smooth interpolation.
	"""
	print(f"[lower_legs_smoothly] Starting smooth lowering with {interpolation_steps} steps.")
	for leg_name_long in LEG_CONFIG.keys():
		h_channel, v_channel, base_h, base_v, is_left = LEG_CONFIG[leg_name_long]

		# Determine target vertical PWM for this leg
		if is_left:
			height_flag = leftSide_height
		else:
			height_flag = rightSide_height

		if height_flag:
			target_v_pwm = base_v + target_vertical_offset
		else:
			target_v_pwm = base_v - target_vertical_offset
		
		# Move vertical servo smoothly
		print(f"  [lower_legs_smoothly] Leg {leg_name_long} V_channel {v_channel} to {target_v_pwm}")
		set_servo_smooth(v_channel, target_v_pwm, steps=interpolation_steps)
		
		# Also move horizontal servos to base (300) for stability
		#print(f"  [lower_legs_smoothly] Leg {leg_name_long} H_channel {h_channel} to {base_h}")
		#set_servo_smooth(h_channel, base_h, steps=interpolation_steps)
	print("[lower_legs_smoothly] Finished smooth lowering.")


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
	for step in range(steps + 1):
		yield int(start + step_size * step)


SmoothMode = 0
SmoothCamMode = 0  # Separate flag for camera smooth mode
steadyMode = 0

step_set = 1
DPI = 15

new_frame = 0
direction_command = MOVE_NO
turn_command = MOVE_NO
movement_speed_min = 10
movement_speed_max = 80
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
	Main movement thread. Is called repeatedly by the RobotM thread.
	Performs one small increment of the walk cycle per call, creating smooth,
	continuous movement.
	"""
	global gait_phase, direction_command, turn_command, movement_speed_max, movement_speed, steadyMode, abort_current_movement, _last_command, _last_speed_sign, _steps_since_change, _direction_changed, _stop_counter

	if steadyMode:
		steady_X()
		steady()
		return

	# --- Handle abort logic first (moved to the top level) ---
	if abort_current_movement:
		print("[move_thread] ⚡ ABORT FLAG detected. Lowering legs to stable ground position.")
		# Calculate interpolation steps based on movement speed for adaptive smoothness
		# Faster movement speed -> quicker lowering (fewer steps)
		interpolation_steps = min(movement_speed_max, int(movement_speed * 1.5))
		lower_legs_smoothly(target_vertical_offset=-10, interpolation_steps=interpolation_steps)

		# Reset flags and phase after smooth lowering is complete
		abort_current_movement = False  # Reset flag
		gait_phase = 0.0  # Reset phase
		steadyMode = 0 # Ensure steady mode is off after stopping movement
		#rm.pause() # Ensure RobotM thread is paused after smooth lowering is complete

		return # IMPORTANT: return here to prevent further movement

	# Determine speed and direction for each side
	speed_left = 0
	speed_right = 0
	movement_active = True

	if direction_command == CMD_FORWARD:
		if turn_command == MOVE_NO:  # Forward
			speed_left = -movement_speed
			speed_right = -movement_speed
		elif turn_command == CMD_FORWARD_LEFT_ARC:
			speed_left = -movement_speed * (1 - arc_factor)
			speed_right = -movement_speed
		elif turn_command == CMD_FORWARD_RIGHT_ARC:
			speed_left = -movement_speed
			speed_right = -movement_speed * (1 - arc_factor)
	elif direction_command == CMD_BACKWARD:  # Backward
		speed_left = movement_speed
		speed_right = movement_speed
	elif turn_command == CMD_LEFT:  # Turn Left on the spot
		speed_left = movement_speed
		speed_right = -movement_speed
	elif turn_command == CMD_RIGHT:  # Turn Right on the spot
		speed_left = -movement_speed
		speed_right = movement_speed
	else:
		movement_active = False

		# Handle abort logic first
		if abort_current_movement:
			print("[move_thread] ⚡ ABORT FLAG detected. Lowering legs to stable ground position.")
			# Calculate interpolation steps based on movement speed for adaptive smoothness
			# Faster movement speed -> quicker lowering (fewer steps)
			interpolation_steps = max(5, int(20 - movement_speed / 4))
			lower_legs_smoothly(target_vertical_offset=-10, interpolation_steps=interpolation_steps)
			
			# Reset flags and phase after smooth lowering is complete
			abort_current_movement = False  # Reset flag
			gait_phase = 0.0  # Reset phase
			steadyMode = 0 # Ensure steady mode is off after stopping movement
			# rm.pause() # Pausing the thread is now handled by the calling function.

			return # IMPORTANT: return here to prevent further movement or stand() from overriding	# If no movement command is active, stand still and reset the gait phase

	if not movement_active:
		handle_stand_or_steady()

		# FT40 Phase 4: Don't immediately reset phase - allow smooth restart after short stops
		_stop_counter += 1
		if _stop_counter > _stop_threshold:
			# Only reset phase after prolonged stop (~0.5 seconds)
			# This allows smooth restart after brief button releases
			if gait_phase != 0.0:
				print(f"[FT40] Phase reset after long stop (counter={_stop_counter})")
				gait_phase = 0.0
		# If stop is brief, phase is preserved for smooth continuation
		return

	# Movement is active - reset stop counter
	_stop_counter = 0

	# --- FT40 Phase 2: Direction Change Detection ---
	# Detect command change (forward/backward/left/right)
	current_command = (direction_command, turn_command)
	command_changed = (current_command != _last_command) and (_last_command is not None)

	# Detect direction change (forward ↔ backward flip based on speed sign)
	current_direction_value = (speed_left + speed_right) / 2.0
	current_sign = 1 if current_direction_value < 0 else (-1 if current_direction_value > 0 else 0)

	direction_changed = (current_sign != 0 and
						_last_speed_sign != 0 and
						current_sign != _last_speed_sign)

	# Update tracking variables
	if command_changed or direction_changed:
		_steps_since_change = 0
		_direction_changed = True
		if direction_changed:
			print(f"[FT40] Direction change detected: {_last_speed_sign} → {current_sign}")
		if command_changed:
			print(f"[FT40] Command change detected: {_last_command} → {current_command}")
	else:
		_steps_since_change += 1
		if _steps_since_change > 10:
			_direction_changed = False

	_last_command = current_command
	_last_speed_sign = current_sign
	# --- End FT40 Phase 2 ---

	# --- Continuous Movement Core Logic ---

	# 1. Advance the gait phase for the next small step
	gait_phase += 1.0 / CYCLE_STEPS
	if gait_phase > 1.0:
		gait_phase -= 1.0

	# 2. Calculate target positions for all legs based on the current phase
	target_positions = calculate_target_positions(gait_phase, speed_left, speed_right)

	# 3. Smoothly move servos towards the calculated target positions
	# A high alpha means the servos will react quickly to target changes.

	# FT40 Phase 3: Variable alpha for smooth transitions during direction changes
	if _direction_changed and _steps_since_change < 5:
		# Gradual transition: Start with gentle interpolation (0.2) and increase to full (1.0) over 5 steps
		# This prevents jerky movements when changing direction
		# Extra gentle first step (0.2) reduces "backward jerk" when switching forward/backward
		alpha = 0.2 + (_steps_since_change / 5.0) * 0.8
		print(f"[FT40] Smooth transition: step={_steps_since_change}, alpha={alpha:.2f}")
	else:
		# Normal continuous movement: high responsiveness
		alpha = 0.9

	for leg in ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']:
		# Get the leg's current tracked horizontal position
		current_h_pos = _leg_positions.get(leg, 0)
		target_h_pos = target_positions[leg]['h']
		vertical_pos = target_positions[leg]['v']

		# Interpolate between current and target for smooth transitions
		new_horizontal_pos = int(current_h_pos + alpha * (target_h_pos - current_h_pos))
		
		# Update the tracked position for the next iteration
		_leg_positions[leg] = new_horizontal_pos

		# Apply the new calculated position to the leg's servos
		apply_leg_position(leg, new_horizontal_pos, vertical_pos)


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
	for leg in ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']:
		is_left = leg.startswith('L')
		if leg in air_group:
			positions[leg] = {'h': h_left_air if is_left else h_right_air, 'v': v_air}
		else:  # in ground_group
			positions[leg] = {'h': h_left_ground if is_left else h_right_ground, 'v': v_ground}

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
			# This sleep controls the overall speed of the gait.
			# 0.01 (10ms) -> 100 steps/sec. With 60 steps/cycle -> ~1.6 cycles/sec
			time.sleep(0.01)

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
	global direction_command, abort_current_movement, gait_phase, steadyMode, movement_speed
	direction_command = direction
	# Set abort flag to immediately stop any ongoing movement cycle
	# The abort_current_movement flag is no longer directly used in move_thread for pausing,
	# but is still set for consistency if needed elsewhere.
	abort_current_movement = True
	print(f"[set_direction_and_pause] Setting abort flag, direction={direction}")
	print(f"[set_direction_and_pause] Initiating smooth lowering...")
	
	# Perform smooth lowering immediately. lower_legs_smoothly is now blocking.
	interpolation_steps = max(5, int(20 - movement_speed / 4))
	lower_legs_smoothly(target_vertical_offset=-10, interpolation_steps=interpolation_steps)

	print("[set_direction_and_pause] Smooth lowering completed. Pausing thread.")
	# After smooth lowering is complete, pause the thread and reset flags
	abort_current_movement = False # Reset abort flag
	gait_phase = 0.0 # Reset gait phase
	steadyMode = 0 # Ensure steady mode is off
	rm.pause() # Pause the RobotM thread now that lowering is complete



def set_turn_and_pause():
	"""Set turn to 'no' and pause robot movement"""
	global turn_command, direction_command, abort_current_movement, gait_phase, steadyMode, movement_speed
	turn_command = MOVE_NO
	direction_command = MOVE_NO  # Also reset direction (user released all movement buttons)
	# Set abort flag to immediately stop any ongoing movement cycle
	# The abort_current_movement flag is no longer directly used in move_thread for pausing,
	# but is still set for consistency if needed elsewhere.
	abort_current_movement = True
	print("[set_turn_and_pause] Setting abort flag, turn=no, direction=no")
	print(f"[set_turn_and_pause] Initiating smooth lowering...")

	# Perform smooth lowering immediately. lower_legs_smoothly is now blocking.
	interpolation_steps = max(5, int(20 - movement_speed / 4))
	lower_legs_smoothly(target_vertical_offset=-10, interpolation_steps=interpolation_steps)

	print("[set_turn_and_pause] Smooth lowering completed. Pausing thread.")
	# After smooth lowering is complete, pause the thread and reset flags
	abort_current_movement = False # Reset abort flag
	gait_phase = 0.0 # Reset gait phase
	steadyMode = 0 # Ensure steady mode is off
	rm.pause() # Pause the RobotM thread now that lowering is complete



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
	for channel in range(16):
		pwm.set_pwm(channel, 0, 0)

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
