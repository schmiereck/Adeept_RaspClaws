#!/usr/bin/env/python3
# File name   : Move.py
# Website     : www.Adeept.com
# Author      : Adeept
# Date        : 2025/04/16
import time
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
         /         

    pos=2---pos=3---pos=4

Change the value of wiggle to set the range and direction that the legs moves.
'''
def left_I(pos,wiggle,heightAdjust=0):
	if pos == 0:
		#pwm.set_pwm(0,0,pwm0)
		if leftSide_height:
			pwm.set_pwm(1,0,pwm1+heightAdjust)
		else:
			pwm.set_pwm(1,0,pwm1-heightAdjust)
	else:
		if leftSide_direction:
			if pos == 1:
				pwm.set_pwm(0,0,pwm0)
				if leftSide_height:
					pwm.set_pwm(1,0,pwm1+3*height_change)
				else:
					pwm.set_pwm(1,0,pwm1-3*height_change)
			elif pos == 2:
				pwm.set_pwm(0,0,pwm0+wiggle)
				if leftSide_height:
					pwm.set_pwm(1,0,pwm1-height_change)
				else:
					pwm.set_pwm(1,0,pwm1+height_change)
			elif pos == 3:
				pwm.set_pwm(0,0,pwm0)
				if leftSide_height:
					pwm.set_pwm(1,0,pwm1-height_change)
				else:
					pwm.set_pwm(1,0,pwm1+height_change)
			elif pos == 4:
				pwm.set_pwm(0,0,pwm0-wiggle)
				if leftSide_height:
					pwm.set_pwm(1,0,pwm1-height_change)
				else:
					pwm.set_pwm(1,0,pwm1+height_change)
		else:
			if pos == 1:
				pwm.set_pwm(0,0,pwm0)
				if leftSide_height:
					pwm.set_pwm(1,0,pwm1+3*wiggle)
				else:
					pwm.set_pwm(1,0,pwm1-3*wiggle)
			elif pos == 2:
				pwm.set_pwm(0,0,pwm0-wiggle)
				if leftSide_height:
					pwm.set_pwm(1,0,pwm1-wiggle)
				else:
					pwm.set_pwm(1,0,pwm1+wiggle)
			elif pos == 3:
				pwm.set_pwm(0,0,pwm0)
				if leftSide_height:
					pwm.set_pwm(1,0,pwm1-wiggle)
				else:
					pwm.set_pwm(1,0,pwm1+wiggle)
			elif pos == 4:
				pwm.set_pwm(0,0,pwm0+wiggle)
				if leftSide_height:
					pwm.set_pwm(1,0,pwm1-wiggle)
				else:
					pwm.set_pwm(1,0,pwm1+wiggle)


def left_II(pos,wiggle,heightAdjust=0):
	if pos == 0:
		if leftSide_height:
			pwm.set_pwm(3,0,pwm3+heightAdjust)
		else:
			pwm.set_pwm(3,0,pwm3-heightAdjust)
	else:
		if leftSide_direction:
			if pos == 1:
				pwm.set_pwm(2,0,pwm2)
				if leftSide_height:
					pwm.set_pwm(3,0,pwm3+3*height_change)
				else:
					pwm.set_pwm(3,0,pwm3-3*height_change)
			elif pos == 2:
				pwm.set_pwm(2,0,pwm2+wiggle)
				if leftSide_height:
					pwm.set_pwm(3,0,pwm3-height_change)
				else:
					pwm.set_pwm(3,0,pwm3+height_change)
			elif pos == 3:
				pwm.set_pwm(2,0,pwm2)
				if leftSide_height:
					pwm.set_pwm(3,0,pwm3-height_change)
				else:
					pwm.set_pwm(3,0,pwm3+height_change)
			elif pos == 4:
				pwm.set_pwm(2,0,pwm2-wiggle)
				if leftSide_height:
					pwm.set_pwm(3,0,pwm3-height_change)
				else:
					pwm.set_pwm(3,0,pwm3+height_change)
		else:
			if pos == 1:
				pwm.set_pwm(2,0,pwm2)
				if leftSide_height:
					pwm.set_pwm(3,0,pwm3+3*wiggle)
				else:
					pwm.set_pwm(3,0,pwm3-3*wiggle)
			elif pos == 2:
				pwm.set_pwm(2,0,pwm2-wiggle)
				if leftSide_height:
					pwm.set_pwm(3,0,pwm3-wiggle)
				else:
					pwm.set_pwm(3,0,pwm3+wiggle)
			elif pos == 3:
				pwm.set_pwm(2,0,pwm2)
				if leftSide_height:
					pwm.set_pwm(3,0,pwm3-wiggle)
				else:
					pwm.set_pwm(3,0,pwm3+wiggle)
			elif pos == 4:
				pwm.set_pwm(2,0,pwm2+wiggle)
				if leftSide_height:
					pwm.set_pwm(3,0,pwm3-wiggle)
				else:
					pwm.set_pwm(3,0,pwm3+wiggle)


def left_III(pos,wiggle,heightAdjust=0):
	if pos == 0:
		if leftSide_height:
			pwm.set_pwm(5,0,pwm5+heightAdjust)
		else:
			pwm.set_pwm(5,0,pwm5-heightAdjust)
	else:
		if leftSide_direction:
			if pos == 1:
				pwm.set_pwm(4,0,pwm4)
				if leftSide_height:
					pwm.set_pwm(5,0,pwm5+3*height_change)
				else:
					pwm.set_pwm(5,0,pwm5-3*height_change)
			elif pos == 2:
				pwm.set_pwm(4,0,pwm4+wiggle)
				if leftSide_height:
					pwm.set_pwm(5,0,pwm5-height_change)
				else:
					pwm.set_pwm(5,0,pwm5+height_change)
			elif pos == 3:
				pwm.set_pwm(4,0,pwm4)
				if leftSide_height:
					pwm.set_pwm(5,0,pwm5-height_change)
				else:
					pwm.set_pwm(5,0,pwm5+height_change)
			elif pos == 4:
				pwm.set_pwm(4,0,pwm4-wiggle)
				if leftSide_height:
					pwm.set_pwm(5,0,pwm5-height_change)
				else:
					pwm.set_pwm(5,0,pwm5+height_change)
		else:
			if pos == 1:
				pwm.set_pwm(4,0,pwm4)
				if leftSide_height:
					pwm.set_pwm(5,0,pwm5+3*wiggle)
				else:
					pwm.set_pwm(5,0,pwm5-3*wiggle)
			elif pos == 2:
				pwm.set_pwm(4,0,pwm4-wiggle)
				if leftSide_height:
					pwm.set_pwm(5,0,pwm5-wiggle)
				else:
					pwm.set_pwm(5,0,pwm5+wiggle)
			elif pos == 3:
				pwm.set_pwm(4,0,pwm4)
				if leftSide_height:
					pwm.set_pwm(5,0,pwm5-wiggle)
				else:
					pwm.set_pwm(5,0,pwm5+wiggle)
			elif pos == 4:
				pwm.set_pwm(4,0,pwm4+wiggle)
				if leftSide_height:
					pwm.set_pwm(5,0,pwm5-wiggle)
				else:
					pwm.set_pwm(5,0,pwm5+wiggle)


def right_I(pos,wiggle,heightAdjust=0):
	if pos == 0:
		if rightSide_height:
			pwm.set_pwm(7,0,pwm7+heightAdjust)
		else:
			pwm.set_pwm(7,0,pwm7-heightAdjust)
	else:
		if rightSide_direction:
			if pos == 1:
				pwm.set_pwm(6,0,pwm6)
				if rightSide_height:
					pwm.set_pwm(7,0,pwm7+3*height_change)
				else:
					pwm.set_pwm(7,0,pwm7-3*height_change)
			elif pos == 2:
				pwm.set_pwm(6,0,pwm6+wiggle)
				if rightSide_height:
					pwm.set_pwm(7,0,pwm7-height_change)
				else:
					pwm.set_pwm(7,0,pwm7+height_change)
			elif pos == 3:
				pwm.set_pwm(6,0,pwm6)
				if rightSide_height:
					pwm.set_pwm(7,0,pwm7-height_change)
				else:
					pwm.set_pwm(7,0,pwm7+height_change)
			elif pos == 4:
				pwm.set_pwm(6,0,pwm6-wiggle)
				if rightSide_height:
					pwm.set_pwm(7,0,pwm7-height_change)
				else:
					pwm.set_pwm(7,0,pwm7+height_change)
		else:
			if pos == 1:
				pwm.set_pwm(6,0,pwm6)
				if rightSide_height:
					pwm.set_pwm(7,0,pwm7+3*height_change)
				else:
					pwm.set_pwm(7,0,pwm7-3*height_change)
			elif pos == 2:
				pwm.set_pwm(6,0,pwm6-wiggle)
				if rightSide_height:
					pwm.set_pwm(7,0,pwm7-height_change)
				else:
					pwm.set_pwm(7,0,pwm7+height_change)
			elif pos == 3:
				pwm.set_pwm(6,0,pwm6)
				if rightSide_height:
					pwm.set_pwm(7,0,pwm7-height_change)
				else:
					pwm.set_pwm(7,0,pwm7+height_change)
			elif pos == 4:
				pwm.set_pwm(6,0,pwm6+wiggle)
				if rightSide_height:
					pwm.set_pwm(7,0,pwm7-height_change)
				else:
					pwm.set_pwm(7,0,pwm7+height_change)


def right_II(pos,wiggle,heightAdjust=0):
	if pos == 0:
		if rightSide_height:
			pwm.set_pwm(9,0,pwm9+heightAdjust)
		else:
			pwm.set_pwm(9,0,pwm9-heightAdjust)
	else:
		if rightSide_direction:
			if pos == 1:
				pwm.set_pwm(8,0,pwm8)
				if rightSide_height:
					pwm.set_pwm(9,0,pwm9+3*height_change)
				else:
					pwm.set_pwm(9,0,pwm9-3*height_change)
			elif pos == 2:
				pwm.set_pwm(8,0,pwm8+wiggle)
				if rightSide_height:
					pwm.set_pwm(9,0,pwm9-height_change)
				else:
					pwm.set_pwm(9,0,pwm9+height_change)
			elif pos == 3:
				pwm.set_pwm(8,0,pwm8)
				if rightSide_height:
					pwm.set_pwm(9,0,pwm9-height_change)
				else:
					pwm.set_pwm(9,0,pwm9+height_change)
			elif pos == 4:
				pwm.set_pwm(8,0,pwm8-wiggle)
				if rightSide_height:
					pwm.set_pwm(9,0,pwm9-height_change)
				else:
					pwm.set_pwm(9,0,pwm9+height_change)
		else:
			if pos == 1:
				pwm.set_pwm(8,0,pwm8)
				if rightSide_height:
					pwm.set_pwm(9,0,pwm9+3*height_change)
				else:
					pwm.set_pwm(9,0,pwm9-3*height_change)
			elif pos == 2:
				pwm.set_pwm(8,0,pwm8-wiggle)
				if rightSide_height:
					pwm.set_pwm(9,0,pwm9-height_change)
				else:
					pwm.set_pwm(9,0,pwm9+height_change)
			elif pos == 3:
				pwm.set_pwm(8,0,pwm8)
				if rightSide_height:
					pwm.set_pwm(9,0,pwm9-height_change)
				else:
					pwm.set_pwm(9,0,pwm9+height_change)
			elif pos == 4:
				pwm.set_pwm(8,0,pwm8+wiggle)
				if rightSide_height:
					pwm.set_pwm(9,0,pwm9-height_change)
				else:
					pwm.set_pwm(9,0,pwm9+height_change)


def right_III(pos,wiggle,heightAdjust=0):
	if pos == 0:
		if rightSide_height:
			pwm.set_pwm(11,0,pwm11+heightAdjust)
		else:
			pwm.set_pwm(11,0,pwm11-heightAdjust)
	else:
		if rightSide_direction:
			if pos == 1:
				pwm.set_pwm(10,0,pwm10)
				if rightSide_height:
					pwm.set_pwm(11,0,pwm11+3*height_change)
				else:

					pwm.set_pwm(11,0,pwm11-3*height_change)
			elif pos == 2:
				pwm.set_pwm(10,0,pwm10+wiggle)
				if rightSide_height:
					pwm.set_pwm(11,0,pwm11-height_change)
				else:
					pwm.set_pwm(11,0,pwm11+height_change)
			elif pos == 3:
				pwm.set_pwm(10,0,pwm10)
				if rightSide_height:
					pwm.set_pwm(11,0,pwm11-height_change)
				else:
					pwm.set_pwm(11,0,pwm11+height_change)
			elif pos == 4:
				pwm.set_pwm(10,0,pwm10-wiggle)
				if rightSide_height:
					pwm.set_pwm(11,0,pwm11-height_change)
				else:
					pwm.set_pwm(11,0,pwm11+height_change)
		else:
			if pos == 1:
				pwm.set_pwm(10,0,pwm10)
				if rightSide_height:
					pwm.set_pwm(11,0,pwm11+3*height_change)
				else:
					pwm.set_pwm(11,0,pwm11-3*height_change)
			elif pos == 2:
				pwm.set_pwm(10,0,pwm10-wiggle)
				if rightSide_height:
					pwm.set_pwm(11,0,pwm11-height_change)
				else:
					pwm.set_pwm(11,0,pwm11+height_change)
			elif pos == 3:
				pwm.set_pwm(10,0,pwm10)
				if rightSide_height:
					pwm.set_pwm(11,0,pwm11-height_change)
				else:
					pwm.set_pwm(11,0,pwm11+height_change)
			elif pos == 4:
				pwm.set_pwm(10,0,pwm10+wiggle)
				if rightSide_height:
					pwm.set_pwm(11,0,pwm11-height_change)
				else:
					pwm.set_pwm(11,0,pwm11+height_change)


def move(step_input, speed, command):
	step_I  = step_input
	step_II = step_input + 2

	if step_II > 4:
		step_II = step_II - 4
	if speed == 0:
		return

	if command == 'no':
		right_I(step_I, speed, 0)
		left_II(step_I, speed, 0)
		right_III(step_I, speed, 0)

		left_I(step_II, speed, 0)
		right_II(step_II, speed, 0)
		left_III(step_II, speed, 0)
	elif command == 'left':
		right_I(step_I, speed, 0)
		left_II(step_I, -speed, 0)
		right_III(step_I, speed, 0)

		left_I(step_II, -speed, 0)
		right_II(step_II, speed, 0)
		left_III(step_II, -speed, 0)
	elif command == 'right':
		right_I(step_I, -speed, 0)
		left_II(step_I, speed, 0)
		right_III(step_I, -speed, 0)

		left_I(step_II, speed, 0)
		right_II(step_II, -speed, 0)
		left_III(step_II, speed, 0)


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
	if leftSide_direction:
		pwm.set_pwm(0,0,pwm0 + horizontal)
	else:
		pwm.set_pwm(0,0,pwm0 - horizontal)

	if leftSide_height:
		pwm.set_pwm(1,0,pwm1+vertical)
	else:
		pwm.set_pwm(1,0,pwm1-vertical)


def dove_Left_II(horizontal, vertical):
	if leftSide_direction:
		pwm.set_pwm(2,0,pwm2 + horizontal)
	else:
		pwm.set_pwm(2,0,pwm2 - horizontal)

	if leftSide_height:
		pwm.set_pwm(3,0,pwm3+vertical)
	else:
		pwm.set_pwm(3,0,pwm3-vertical)


def dove_Left_III(horizontal, vertical):
	if leftSide_direction:
		pwm.set_pwm(4,0,pwm4 + horizontal)
	else:
		pwm.set_pwm(4,0,pwm4 - horizontal)

	if leftSide_height:
		pwm.set_pwm(5,0,pwm5+vertical)
	else:
		pwm.set_pwm(5,0,pwm5-vertical)


def dove_Right_I(horizontal, vertical):
	if rightSide_direction:
		pwm.set_pwm(6,0,pwm6 + horizontal)
	else:
		pwm.set_pwm(6,0,pwm6 - horizontal)

	if rightSide_height:
		pwm.set_pwm(7,0,pwm7+vertical)
	else:
		pwm.set_pwm(7,0,pwm7-vertical)


def dove_Right_II(horizontal, vertical):
	if rightSide_direction:
		pwm.set_pwm(8,0,pwm8 + horizontal)
	else:
		pwm.set_pwm(8,0,pwm8 - horizontal)

	if rightSide_height:
		pwm.set_pwm(9,0,pwm9+vertical)
	else:
		pwm.set_pwm(9,0,pwm9-vertical)


def dove_Right_III(horizontal, vertical):
	if rightSide_direction:
		pwm.set_pwm(10,0,pwm10 + horizontal)
	else:
		pwm.set_pwm(10,0,pwm10 - horizontal)

	if rightSide_height:
		pwm.set_pwm(11,0,pwm11+vertical)
	else:
		pwm.set_pwm(11,0,pwm11-vertical)


def dove(step_input, speed, timeLast, dpi, command):
	step_I  = step_input
	step_II = step_input + 2

	if step_II > 4:
		step_II = step_II - 4
	
	if speed > 0:
		if step_input == 1:
			for speed_I in range(0, (speed+int(speed/dpi)), int(speed/dpi)):
				if move_stu and command == 'no':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(-speed_I, 3*speed_II)
					dove_Right_II(-speed_I, 3*speed_II)
					dove_Left_III(-speed_I, 3*speed_II)

					dove_Right_I(speed_I, -10)
					dove_Left_II(speed_I, -10)
					dove_Right_III(speed_I, -10)
					time.sleep(timeLast/dpi)
				else:
					pass

				if command == 'left':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(speed_I, 3*speed_II)
					dove_Right_II(-speed_I, 3*speed_II)
					dove_Left_III(speed_I, 3*speed_II)

					dove_Right_I(speed_I, -10)
					dove_Left_II(-speed_I, -10)
					dove_Right_III(speed_I, -10)
					time.sleep(timeLast/dpi)
				elif command == 'right':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(-speed_I, 3*speed_II)
					dove_Right_II(speed_I, 3*speed_II)
					dove_Left_III(-speed_I, 3*speed_II)

					dove_Right_I(-speed_I, -10)
					dove_Left_II(speed_I, -10)
					dove_Right_III(-speed_I, -10)
					time.sleep(timeLast/dpi)
				else:
					pass

				if move_stu == 0 and command == 'no':
					break

		elif step_input == 2:
			for speed_I in range(0, (speed+int(speed/dpi)), int(speed/dpi)):
				if move_stu and command == 'no':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(speed_II, 3*(speed-speed_II))
					dove_Right_II(speed_II, 3*(speed-speed_II))
					dove_Left_III(speed_II, 3*(speed-speed_II))

					dove_Right_I(-speed_II, -10)
					dove_Left_II(-speed_II, -10)
					dove_Right_III(-speed_II, -10)
					time.sleep(timeLast/dpi)
				else:
					pass

				if command == 'left':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(-speed_II, 3*(speed-speed_II))
					dove_Right_II(speed_II, 3*(speed-speed_II))
					dove_Left_III(-speed_II, 3*(speed-speed_II))

					dove_Right_I(-speed_II, -10)
					dove_Left_II(speed_II, -10)
					dove_Right_III(-speed_II, -10)
					time.sleep(timeLast/dpi)
				elif command == 'right':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(speed_II, 3*(speed-speed_II))
					dove_Right_II(-speed_II, 3*(speed-speed_II))
					dove_Left_III(speed_II, 3*(speed-speed_II))

					dove_Right_I(speed_II, -10)
					dove_Left_II(-speed_II, -10)
					dove_Right_III(speed_II, -10)
					time.sleep(timeLast/dpi)
				else:
					pass

				if move_stu == 0 and command == 'no':
					break
		elif step_input == 3:
			for speed_I in range(0, (speed+int(speed/dpi)), int(speed/dpi)):
				if move_stu and command == 'no':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(speed_I, -10)
					dove_Right_II(speed_I, -10)
					dove_Left_III(speed_I, -10)

					dove_Right_I(-speed_I, 3*speed_II)
					dove_Left_II(-speed_I, 3*speed_II)
					dove_Right_III(-speed_I, 3*speed_II)
					time.sleep(timeLast/dpi)
				else:
					pass

				if command == 'left':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(-speed_I, -10)
					dove_Right_II(speed_I, -10)
					dove_Left_III(-speed_I, -10)

					dove_Right_I(-speed_I, 3*speed_II)
					dove_Left_II(speed_I, 3*speed_II)
					dove_Right_III(-speed_I, 3*speed_II)
					time.sleep(timeLast/dpi)
				elif command == 'right':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(speed_I, -10)
					dove_Right_II(-speed_I, -10)
					dove_Left_III(speed_I, -10)

					dove_Right_I(speed_I, 3*speed_II)
					dove_Left_II(-speed_I, 3*speed_II)
					dove_Right_III(speed_I, 3*speed_II)
					time.sleep(timeLast/dpi)
				else:
					pass

				if move_stu == 0 and command == 'no':
					break
		elif step_input == 4:
			for speed_I in range(0, (speed+int(speed/dpi)), int(speed/dpi)):
				if move_stu and command == 'no':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(-speed_II, -10)
					dove_Right_II(-speed_II, -10)
					dove_Left_III(-speed_II, -10)

					dove_Right_I(speed_II, 3*(speed-speed_II))
					dove_Left_II(speed_II, 3*(speed-speed_II))
					dove_Right_III(speed_II, 3*(speed-speed_II))
					time.sleep(timeLast/dpi)
				else:
					pass

				if command == 'left':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(speed_II, -10)
					dove_Right_II(-speed_II, -10)
					dove_Left_III(speed_II, -10)

					dove_Right_I(speed_II, 3*(speed-speed_II))
					dove_Left_II(-speed_II, 3*(speed-speed_II))
					dove_Right_III(speed_II, 3*(speed-speed_II))
					time.sleep(timeLast/dpi)
				elif command == 'right':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(-speed_II, -10)
					dove_Right_II(speed_II, -10)
					dove_Left_III(-speed_II, -10)

					dove_Right_I(-speed_II, 3*(speed-speed_II))
					dove_Left_II(speed_II, 3*(speed-speed_II))
					dove_Right_III(-speed_II, 3*(speed-speed_II))
					time.sleep(timeLast/dpi)
				else:
					pass

				if move_stu == 0 and command == 'no':
					break
	else:
		speed = -speed
		if step_input == 1:
			for speed_I in range(0, (speed+int(speed/dpi)), int(speed/dpi)):
				if move_stu and command == 'no':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(speed_I, 3*speed_II)
					dove_Right_II(speed_I, 3*speed_II)
					dove_Left_III(speed_I, 3*speed_II)

					dove_Right_I(-speed_I, -10)
					dove_Left_II(-speed_I, -10)
					dove_Right_III(-speed_I, -10)
					time.sleep(timeLast/dpi)
				else:
					pass
		elif step_input == 2:
			for speed_I in range(0, (speed+int(speed/dpi)), int(speed/dpi)):
				if move_stu and command == 'no':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(-speed_II, 3*(speed-speed_II))
					dove_Right_II(-speed_II, 3*(speed-speed_II))
					dove_Left_III(-speed_II, 3*(speed-speed_II))

					dove_Right_I(speed_II, -10)
					dove_Left_II(speed_II, -10)
					dove_Right_III(speed_II, -10)
					time.sleep(timeLast/dpi)
				else:
					pass
		elif step_input == 3:
			for speed_I in range(0, (speed+int(speed/dpi)), int(speed/dpi)):
				if move_stu and command == 'no':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(-speed_I, -10)
					dove_Right_II(-speed_I, -10)
					dove_Left_III(-speed_I, -10)

					dove_Right_I(speed_I, 3*speed_II)
					dove_Left_II(speed_I, 3*speed_II)
					dove_Right_III(speed_I, 3*speed_II)
					time.sleep(timeLast/dpi)
				else:
					pass
		elif step_input == 4:
			for speed_I in range(0, (speed+int(speed/dpi)), int(speed/dpi)):
				if move_stu and command == 'no':
					speed_II = speed_I
					speed_I = speed - speed_I
					dove_Left_I(speed_II, -10)
					dove_Right_II(speed_II, -10)
					dove_Left_III(speed_II, -10)

					dove_Right_I(-speed_II, 3*(speed-speed_II))
					dove_Left_II(-speed_II, 3*(speed-speed_II))
					dove_Right_III(-speed_II, 3*(speed-speed_II))
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
	"""Interpoliert zwischen start und end in 'steps' Schritten für sanfte Servo-Bewegungen"""
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
	Execute a single movement step with smooth or normal mode

	Args:
		speed: Movement speed (positive for forward, negative for backward)
		turn: Turn command ('left', 'right', or 'no')
	"""
	global step_set

	if SmoothMode:
		dove(step_set, speed, 0.001, DPI, turn)
		increment_step()
	else:
		move(step_set, speed, turn)
		increment_step()


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
	3. Always handle stand OR steady at the end
	"""
	global step_set

	if not steadyMode:
		# Step 1: Handle directional movement (forward/backward)
		# Only one of these will execute per cycle
		if direction_command == 'forward' and turn_command == 'no':
			execute_movement_step(35, 'no')
		elif direction_command == 'backward' and turn_command == 'no':
			execute_movement_step(-35, 'no')

		# Step 2: Handle turn movement (independent of directional movement)
		if turn_command != 'no':
			execute_movement_step(20, turn_command)

		# Step 3: Always handle stand or steady at the end
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
			# Sleep time controls movement speed
			# We use shorter sleep intervals to respond quickly to pause()
			sleep_time = 0.05 if SmoothMode else 0.1
			sleep_start = time.time()
			while (time.time() - sleep_start) < sleep_time:
				if not self.__flag.is_set():
					break  # Exit sleep early if paused
				time.sleep(0.01)  # Check every 10ms

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


def handle_movement_command(command):
	"""Handle movement commands (forward, backward, left, right, stand, no)"""
	global direction_command

	movement_commands = {
		'forward': lambda: set_direction_and_resume('forward', 'no'),
		'backward': lambda: set_direction_and_resume('backward', 'no'),
		'stand': lambda: (setattr(globals(), 'direction_command', 'stand'), rm.pause()),
		'left': lambda: set_turn_and_resume('left'),
		'right': lambda: set_turn_and_resume('right'),
		'no': lambda: (setattr(globals(), 'turn_command', 'no'), rm.pause()),
	}

	if command in movement_commands:
		movement_commands[command]()
		return True
	return False


def handle_mode_command(command):
	"""Handle mode commands (smooth, camera, steady)"""
	global SmoothMode, SmoothCamMode, steadyMode

	mode_commands = {
		'slow': lambda: setattr(globals(), 'SmoothMode', 1),
		'fast': lambda: setattr(globals(), 'SmoothMode', 0),
		'smoothCam': lambda: setattr(globals(), 'SmoothCamMode', 1),
		'smoothCamOff': lambda: setattr(globals(), 'SmoothCamMode', 0),
		'steadyCamera': lambda: (setattr(globals(), 'steadyMode', 1), rm.resume()),
		'steadyCameraOff': lambda: (setattr(globals(), 'steadyMode', 0), rm.pause()),
	}

	if command in mode_commands:
		mode_commands[command]()
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
