#!/usr/bin/python
# -*- coding: UTF-8 -*-
# File name   : client.py
# Description : client  
# Website	 : www.adeept.com
# E-mail	  : support@adeept.com
# Author	  : Devin
# Date		: 2023/06/14

from socket import *
import time
import threading as thread
import tkinter as tk
import subprocess
from ip_utils import num_import, replace_num
import sys
import os
# Add parent directory to path to import protocol module
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from protocol import *
try:
	import cv2
except Exception as e:
	print("Couldn't import OpenCV:", e)

try:
	import zmq
except Exception as e:
	print("Couldn't import zmq:", e)

try:
	import base64
except Exception as e:
	print("Couldn't import base64:", e)

try:
	import numpy as np
except Exception as e:
	print("Couldn't import numpy:", e)

# ==================== Global Variables ====================
BUFSIZ = 1024  # Buffer size for socket communication

ip_stu=1		#Shows connection status
c_f_stu = 0
c_b_stu = 0
c_l_stu = 0
c_r_stu = 0
c_ls_stu= 0
c_rs_stu= 0
funcMode= 0
steadyMode = 0
TS_stu  = 0
DS_stu  = 0
tcpClicSock = ''
root = ''
stat = 0
ip_adr = ''
footage_socket = None
speed_slider_widget = None
speed_value_label_widget = None
arc_factor_slider_widget = None
arc_factor_value_label_widget = None
current_speed = 35  # Default movement speed

Switch_3 = 0
Switch_2 = 0
Switch_1 = 0
SmoothMode = 0
SmoothCamMode = 0

# MPU6050 sensor data
GYRO_X, GYRO_Y, GYRO_Z = 0.0, 0.0, 0.0
ACCEL_X, ACCEL_Y, ACCEL_Z = 0.0, 0.0, 0.0
MPU_AVAILABLE = False

video_thread_started = False  # Track if video thread was started (global)
shutdown_requested = False  # Flag to signal application shutdown

########>>>>>VIDEO<<<<<########

def RGB_to_Hex(r, g, b):
	return ('#'+str(hex(r))[-2:]+str(hex(g))[-2:]+str(hex(b))[-2:]).replace('x','0').upper()

# Global variable to track the Footage-GUI process
footage_process = None

def run_open():
	global footage_process
	script_path = 'Footage-GUI.py'
	print(f"Starting {script_path} as background process...")
	try:
		# Start as non-blocking background process
		footage_process = subprocess.Popen(
			['python', script_path],
			stdout=subprocess.PIPE,
			stderr=subprocess.PIPE,
			text=True,
			bufsize=1
		)
		print(f"✓ {script_path} started (PID: {footage_process.pid})")

		# Start a thread to monitor the process output
		def monitor_footage():
			if footage_process:
				for line in footage_process.stdout:
					print(f"[Footage] {line.strip()}")
				# Also print stderr if any
				for line in footage_process.stderr:
					print(f"[Footage ERROR] {line.strip()}")

		monitor_thread = thread.Thread(target=monitor_footage, daemon=True)
		monitor_thread.start()
	except Exception as e:
		print(f"✗ Failed to start {script_path}: {e}")
def video_thread():
	global footage_socket, font, frame_num, fps
	context = zmq.Context()
	footage_socket = context.socket(zmq.SUB)

	# Set high-water mark to prevent buffering old frames
	footage_socket.setsockopt(zmq.RCVHWM, 1)

	footage_socket.connect('tcp://%s:5555'%ip_adr)
	footage_socket.setsockopt_string(zmq.SUBSCRIBE, np.unicode(''))

	# Give ZMQ time to establish subscription (avoid "slow joiner" problem)
	time.sleep(0.2)
	print("Video socket connected and subscribed")

	font = cv2.FONT_HERSHEY_SIMPLEX


	frame_num = 0
	fps = 0

def get_FPS():
	global frame_num, fps
	while 1:
		try:
			time.sleep(1)
			fps = frame_num
			frame_num = 0
		except:
			time.sleep(1)



fps_threading=thread.Thread(target=get_FPS, daemon=True)		 #Define a thread for FPV and OpenCV
fps_threading.start()									 #Thread starts


########>>>>>VIDEO<<<<<########


# ==================== Command Helper Functions ====================

def send_command(command):
	"""Send a command to the server"""
	try:
		if tcpClicSock:
			tcpClicSock.send(command.encode())
	except Exception as e:
		print(f"Error sending command '{command}': {e}")
		# Socket is probably closed, ignore the error


def send_movement_command(command, state_var_name, state_value=1):
	"""Send movement command and update state variable"""
	global c_f_stu, c_b_stu, c_l_stu, c_r_stu, c_ls_stu, c_rs_stu

	state_dict = {
		'c_f_stu': lambda: globals().update({'c_f_stu': state_value}),
		'c_b_stu': lambda: globals().update({'c_b_stu': state_value}),
		'c_l_stu': lambda: globals().update({'c_l_stu': state_value}),
		'c_r_stu': lambda: globals().update({'c_r_stu': state_value}),
		'c_ls_stu': lambda: globals().update({'c_ls_stu': state_value}),
		'c_rs_stu': lambda: globals().update({'c_rs_stu': state_value}),
	}

	current_state = globals().get(state_var_name, 0)
	if current_state == 0:
		send_command(command)
		state_dict[state_var_name]()


def on_speed_change(value):
	"""Handle speed slider value change"""
	global current_speed, speed_value_label_widget

	speed = int(float(value))
	current_speed = speed

	# Update value label
	if speed_value_label_widget:
		speed_value_label_widget.config(text=str(speed))

	# Send speed command to server
	send_command(f'{CMD_SET_SPEED}{speed}')
	print(f"[Speed] Set to {speed}")


def on_arc_factor_change(value):
	"""Handle arc factor slider value change"""
	global arc_factor_value_label_widget

	arc_factor = float(value)

	# Update value label
	if arc_factor_value_label_widget:
		arc_factor_value_label_widget.config(text=f'{arc_factor:.2f}')

	# Send arc factor command to server
	send_command(f'{CMD_SET_ARC_FACTOR}{arc_factor}')
	print(f"[ArcFactor] Set to {arc_factor:.2f}")


# ==================== Movement Callbacks ====================

def call_forward(event):
	"""Command car to move forward"""
	send_movement_command(CMD_FORWARD, 'c_f_stu')


def call_back(event):
	"""Command car to move backward"""
	send_movement_command(CMD_BACKWARD, 'c_b_stu')


def call_Left(event):
	"""Command car to turn left"""
	send_movement_command(CMD_LEFT, 'c_l_stu')


def call_Right(event):
	"""Command car to turn right"""
	send_movement_command(CMD_RIGHT, 'c_r_stu')


def call_LeftSide(event):
	"""Command car to move in a forward-left arc"""
	send_movement_command(CMD_FORWARD_LEFT_ARC, 'c_ls_stu')


def call_RightSide(event):
	"""Command car to move in a forward-right arc"""
	send_movement_command(CMD_FORWARD_RIGHT_ARC, 'c_rs_stu')


def call_DS(event):
	"""Direction Stop"""
	global DS_stu
	send_command(CMD_DIRECTION_STOP)
	DS_stu = 0


def call_TS(event):
	"""Turn Stop"""
	global TS_stu
	send_command(CMD_TURN_STOP)
	TS_stu = 0


def call_FB_stop(event):
	"""Stop forward/backward movement"""
	global c_f_stu, c_b_stu
	c_f_stu = 0
	c_b_stu = 0
	send_command(CMD_DIRECTION_STOP)


def call_Turn_stop(event):
	"""Stop turning movement"""
	global c_l_stu, c_r_stu, c_ls_stu, c_rs_stu
	c_l_stu = 0
	c_r_stu = 0
	c_ls_stu = 0
	c_rs_stu = 0
	send_command(CMD_TURN_STOP)


# ==================== Camera Callbacks ====================

def call_headup(event):
	send_command(CMD_LOOK_UP)


def call_headdown(event):
	send_command(CMD_LOOK_DOWN)


def call_headleft(event):
	send_command(CMD_LOOK_LEFT)


def call_headright(event):
	send_command(CMD_LOOK_RIGHT)


def call_LRstop(event):
	send_command(CMD_LR_STOP)


def call_UDstop(event):
	send_command(CMD_UD_STOP)


def call_headhome(event):
	send_command(CMD_LOOK_HOME)


# ==================== Mode Toggle Callbacks ====================

def call_steady(event):
	"""Toggle steady camera mode"""
	global steadyMode
	if steadyMode == 0:
		send_command(CMD_STEADY_CAMERA)
		steadyMode = 1
	else:
		send_command(CMD_STEADY_CAMERA_OFF)
		steadyMode = 0


def call_SmoothCam(event):
	"""Toggle smooth camera movement mode"""
	global SmoothCamMode
	if SmoothCamMode == 0:
		send_command(CMD_SMOOTH_CAM)
		SmoothCamMode = 1
	else:
		send_command(CMD_SMOOTH_CAM_OFF)
		SmoothCamMode = 0



def call_Switch_1(event):
	"""Toggle GPIO Switch 1"""
	if Switch_1 == 0:
		send_command(CMD_SWITCH_1_ON)
	else:
		send_command(CMD_SWITCH_1_OFF)


def call_Switch_2(event):
	"""Toggle GPIO Switch 2"""
	if Switch_2 == 0:
		send_command(CMD_SWITCH_2_ON)
	else:
		send_command(CMD_SWITCH_2_OFF)


def call_Switch_3(event):
	"""Toggle GPIO Switch 3"""
	if Switch_3 == 0:
		send_command(CMD_SWITCH_3_ON)
	else:
		send_command(CMD_SWITCH_3_OFF)



# ==================== Power Management Callbacks ====================

servo_standby_state = False
camera_pause_state = False


def call_servo_standby(event):
	"""Toggle servo standby mode"""
	global servo_standby_state
	try:
		if not servo_standby_state:
			send_command(CMD_SERVO_STANDBY)
			Btn_ServoStandby.config(bg='#FF6D00', fg='#000000', text='Servo Wake [M]')
			servo_standby_state = True
			print("🔋 Servos in STANDBY mode - low power")
		else:
			send_command(CMD_SERVO_WAKEUP)
			Btn_ServoStandby.config(bg=color_btn, fg=color_text, text='Servo Standby [M]')
			servo_standby_state = False
			print("⚡ Servos AWAKE - ready to move")
	except Exception as e:
		print(f"Error: {e}")


def call_camera_pause(event):
	"""Toggle camera pause mode"""
	global camera_pause_state
	try:
		if not camera_pause_state:
			send_command(CMD_CAMERA_PAUSE)
			Btn_CameraPause.config(bg='#FF6D00', fg='#000000', text='Camera Resume [,]')
			camera_pause_state = True
			print("📷 Camera PAUSED - saving power")
		else:
			send_command(CMD_CAMERA_RESUME)
			Btn_CameraPause.config(bg=color_btn, fg=color_text, text='Camera Pause [,]')
			camera_pause_state = False
			print("📷 Camera RESUMED")
	except Exception as e:
		print(f"Error: {e}")


# ==================== End Power Management ====================


# ==================== MPU6050 Parsing and Visualization ====================

def parse_mpu_data(mpu_string):
	"""
	Parse MPU6050 data string and update global variables + canvas display.

	Format: "G:x,y,z A:x,y,z" or "MPU:N/A" or "MPU:ERROR"

	Args:
		mpu_string: String from server with MPU data
	"""
	global GYRO_X, GYRO_Y, GYRO_Z, ACCEL_X, ACCEL_Y, ACCEL_Z, MPU_AVAILABLE

	if 'MPU:N/A' in mpu_string or 'MPU:ERROR' in mpu_string:
		MPU_AVAILABLE = False
		return

	try:
		# Split "G:x,y,z A:x,y,z"
		parts = mpu_string.split()

		# Parse Gyro: "G:x,y,z"
		if len(parts) >= 1 and parts[0].startswith('G:'):
			gyro_str = parts[0][2:]  # Remove "G:"
			gyro_vals = gyro_str.split(',')
			if len(gyro_vals) == 3:
				GYRO_X, GYRO_Y, GYRO_Z = float(gyro_vals[0]), float(gyro_vals[1]), float(gyro_vals[2])

		# Parse Accel: "A:x,y,z"
		if len(parts) >= 2 and parts[1].startswith('A:'):
			accel_str = parts[1][2:]  # Remove "A:"
			accel_vals = accel_str.split(',')
			if len(accel_vals) == 3:
				ACCEL_X, ACCEL_Y, ACCEL_Z = float(accel_vals[0]), float(accel_vals[1]), float(accel_vals[2])

		MPU_AVAILABLE = True

		# Update canvas display
		update_mpu_canvas()

	except Exception as e:
		print(f"⚠ Error parsing MPU data: {e}")
		MPU_AVAILABLE = False


def update_mpu_canvas():
	"""Update the 2D cross display with current gyro/accel data"""
	global canvas_mpu, GYRO_X, GYRO_Y, ACCEL_X, ACCEL_Y, ACCEL_Z

	try:
		# Clear canvas
		canvas_mpu.delete("all")

		# Canvas dimensions
		width = 160
		height = 160
		center_x = width // 2
		center_y = height // 2

		# Draw cross (X and Y axes)
		canvas_mpu.create_line(0, center_y, width, center_y, fill='#424242', width=1)  # X-axis
		canvas_mpu.create_line(center_x, 0, center_x, height, fill='#424242', width=1)  # Y-axis

		# Draw concentric circles for scale (every 25% of max range)
		max_range = 70  # Maximum pixel distance from center
		for r in [25, 50, 75]:
			radius = (r / 100) * max_range
			canvas_mpu.create_oval(center_x - radius, center_y - radius,
			                       center_x + radius, center_y + radius,
			                       outline='#616161', width=1)

		if not MPU_AVAILABLE:
			# Display "N/A" if sensor not available
			canvas_mpu.create_text(center_x, center_y, text="MPU: N/A",
			                       fill='#757575', font=('Arial', 10))
			return

		# Scale gyro values to canvas coordinates
		# Gyro range: typically ±250°/s, we map to ±max_range pixels
		gyro_scale = max_range / 250.0
		gyro_x_pos = int(GYRO_Y * gyro_scale)  # Gyro Y → Canvas X (pitch)
		gyro_y_pos = int(-GYRO_X * gyro_scale)  # Gyro X → Canvas Y (roll, inverted)

		# Clamp to canvas bounds
		gyro_x_pos = max(-max_range, min(max_range, gyro_x_pos))
		gyro_y_pos = max(-max_range, min(max_range, gyro_y_pos))

		# Draw gyro point (BLUE)
		gyro_dot_x = center_x + gyro_x_pos
		gyro_dot_y = center_y + gyro_y_pos
		canvas_mpu.create_oval(gyro_dot_x - 6, gyro_dot_y - 6,
		                       gyro_dot_x + 6, gyro_dot_y + 6,
		                       fill='#2196F3', outline='#1976D2', width=2)

		# Scale accel values to canvas coordinates
		# Accel range: ±1g (±9.81 m/s²) for tilt indication, we map to ±max_range pixels
		# This gives better sensitivity for tilt display (45° = ~50% deflection)
		accel_scale = max_range / 9.81
		accel_x_pos = int(ACCEL_Y * accel_scale)  # Accel Y → Canvas X
		accel_y_pos = int(-ACCEL_X * accel_scale)  # Accel X → Canvas Y (inverted)

		# Clamp to canvas bounds
		accel_x_pos = max(-max_range, min(max_range, accel_x_pos))
		accel_y_pos = max(-max_range, min(max_range, accel_y_pos))

		# Draw accel point (ORANGE) with size based on Z-accel
		accel_dot_x = center_x + accel_x_pos
		accel_dot_y = center_y + accel_y_pos

		# Z-acceleration affects dot size (9.81 m/s² = normal gravity)
		# Larger Z = larger dot (more vertical acceleration)
		z_factor = abs(ACCEL_Z) / 9.81  # Normalize to 1.0 at 1g
		dot_radius = int(4 + 4 * z_factor)  # 4-8 pixels radius
		dot_radius = max(3, min(10, dot_radius))  # Clamp to 3-10

		canvas_mpu.create_oval(accel_dot_x - dot_radius, accel_dot_y - dot_radius,
		                       accel_dot_x + dot_radius, accel_dot_y + dot_radius,
		                       fill='#FF9800', outline='#F57C00', width=2)

		# Draw labels
		canvas_mpu.create_text(10, 10, text="Gyro (°/s)", fill='#2196F3', anchor='nw', font=('Arial', 8, 'bold'))
		canvas_mpu.create_text(10, 25, text="Accel (tilt)", fill='#FF9800', anchor='nw', font=('Arial', 8, 'bold'))

		# Draw axis labels
		canvas_mpu.create_text(width - 5, center_y - 5, text="Y+", fill='#616161', anchor='se', font=('Arial', 7))
		canvas_mpu.create_text(5, center_y - 5, text="Y-", fill='#616161', anchor='sw', font=('Arial', 7))
		canvas_mpu.create_text(center_x + 5, 5, text="X+", fill='#616161', anchor='nw', font=('Arial', 7))
		canvas_mpu.create_text(center_x + 5, height - 5, text="X-", fill='#616161', anchor='sw', font=('Arial', 7))

	except Exception as e:
		print(f"⚠ Error updating MPU canvas: {e}")


# ==================== End MPU6050 ====================


# ==================== Message Handlers ====================

def handle_video_ready():
	"""Handle VIDEO_READY signal from server"""
	global video_thread_started
	if not video_thread_started:
		print("✅ Server signals: Video stream is ready")
		print("   Waiting 3s for SSH tunnel to establish video port...")
		time.sleep(3.0)  # Give SSH tunnel time to forward port 5555
		video_threading = thread.Thread(target=run_open, daemon=True)
		video_threading.start()
		video_thread_started = True
		print("✓ Video thread started")


def handle_video_timeout():
	"""Handle VIDEO_TIMEOUT signal from server"""
	print("⚠ Video server failed to start - no video stream available")


def update_battery_display(battery_volt):
	"""Update battery display with color coding"""
	if battery_volt == 0.0:
		# Battery monitoring not available
		BATTERY_lab.config(text='Battery: N/A', bg='#757575', fg=color_text)
	else:
		# Calculate percentage (6.0V = 0%, 8.4V = 100%)
		Vref = 8.4
		WarningThreshold = 6.0
		battery_percent = ((battery_volt - WarningThreshold) / (Vref - WarningThreshold)) * 100
		battery_percent = max(0, min(100, battery_percent))  # Clamp to 0-100%

		# Color coding: green > 60%, orange 30-60%, red < 30%
		if battery_percent >= 60:
			bg_color = '#4CAF50'  # Green
		elif battery_percent >= 30:
			bg_color = '#FF9800'  # Orange
		else:
			bg_color = '#F44336'  # Red

		BATTERY_lab.config(text='Battery: %.1fV (%.0f%%)'%(battery_volt, battery_percent),
		                   bg=bg_color, fg='#FFFFFF')


def update_system_info(system_info):
	"""Update CPU, RAM, and Battery information"""
	global CPU_TEP, CPU_USE, RAM_USE, BATTERY_VOLTAGE

	info_get = system_info.split()
	if len(info_get) >= 4:
		# New format: CPU_TEMP CPU_USE RAM_USE BATTERY_VOLTAGE
		CPU_TEP, CPU_USE, RAM_USE, BATTERY_VOLTAGE = info_get[0], info_get[1], info_get[2], info_get[3]
		CPU_TEP_lab.config(text='CPU Temp: %s℃'%CPU_TEP)
		CPU_USE_lab.config(text='CPU Usage: %s'%CPU_USE)
		RAM_lab.config(text='RAM Usage: %s'%RAM_USE)

		# Update battery display
		battery_volt = float(BATTERY_VOLTAGE)
		update_battery_display(battery_volt)

	elif len(info_get) >= 3:
		# Old format without battery (backwards compatibility)
		CPU_TEP, CPU_USE, RAM_USE = info_get[0], info_get[1], info_get[2]
		CPU_TEP_lab.config(text='CPU Temp: %s℃'%CPU_TEP)
		CPU_USE_lab.config(text='CPU Usage: %s'%CPU_USE)
		RAM_lab.config(text='RAM Usage: %s'%RAM_USE)
		BATTERY_lab.config(text='Battery: N/A', bg='#757575', fg=color_text)


def handle_info_message(car_info):
	"""Process INFO message from server (CPU/RAM/Battery/MPU/Servo data)"""
	try:
		info_data = car_info[len(STATUS_INFO_PREFIX):].strip()  # Remove 'INFO:' prefix

		# Split by '|' to separate system info, servo positions, and MPU data
		parts = info_data.split('|')
		system_info = parts[0].strip()
		servo_info = parts[1].strip() if len(parts) > 1 else None
		mpu_info = parts[2].strip() if len(parts) > 2 else None

		# Update system information
		update_system_info(system_info)

		# Parse MPU6050 data (format: "G:x,y,z A:x,y,z" or "MPU:N/A")
		if mpu_info:
			parse_mpu_data(mpu_info)

		# Log servo positions to terminal with timestamp
		if servo_info:
			timestamp = int(time.time() * 1000)  # Milliseconds since epoch
			print(f"[{timestamp}] [SERVOS] {servo_info}")

	except Exception as e:
		print(f"⚠ Error processing INFO: {e}")


def handle_steady_camera(enabled):
	"""Handle steady camera mode toggle"""
	global steadyMode, SmoothMode
	if enabled:
		steadyMode = 1
		SmoothMode = 1
		Btn_Steady.config(bg='#FF6D00', fg='#000000')
	else:
		steadyMode = 0
		SmoothMode = 0
		Btn_Steady.config(bg=color_btn, fg=color_text)


def handle_smooth_cam(enabled):
	"""Handle smooth camera mode toggle"""
	global SmoothCamMode
	if enabled:
		SmoothCamMode = 1
		Btn_SmoothCam.config(bg='#FF6D00', fg='#000000')
	else:
		SmoothCamMode = 0
		Btn_SmoothCam.config(bg=color_btn, fg=color_text)


def handle_switch(switch_num, enabled):
	"""Handle switch on/off state"""
	global Switch_1, Switch_2, Switch_3

	if switch_num == 1:
		Switch_1 = 1 if enabled else 0
		Btn_Switch_1.config(bg='#4CAF50' if enabled else color_btn)
	elif switch_num == 2:
		Switch_2 = 1 if enabled else 0
		Btn_Switch_2.config(bg='#4CAF50' if enabled else color_btn)
	elif switch_num == 3:
		Switch_3 = 1 if enabled else 0
		Btn_Switch_3.config(bg='#4CAF50' if enabled else color_btn)


def handle_servo_standby_status(active):
	"""Handle servo standby status update from server"""
	global servo_standby_state, Btn_ServoStandby, color_btn, color_text

	servo_standby_state = active

	# Check if button exists (might not be created yet during startup)
	try:
		if active:
			# Servo is in standby - show "Wake" button
			Btn_ServoStandby.config(bg='#FF6D00', fg='#000000', text='Servo Wake [M]')
			print("🔋 Servo Standby ACTIVE (synced from server)")
		else:
			# Servo is awake - show "Standby" button
			Btn_ServoStandby.config(bg=color_btn, fg=color_text, text='Servo Standby [M]')
			print("⚡ Servo Standby INACTIVE (synced from server)")
	except (NameError, AttributeError) as e:
		# Button not created yet, state will be applied when button is created
		print(f"Servo standby state received: {'ACTIVE' if active else 'INACTIVE'} (button not ready yet) - {e}")


def handle_camera_pause_status(paused):
	"""Handle camera pause status update from server"""
	global camera_pause_state, Btn_CameraPause, color_btn, color_text

	camera_pause_state = paused

	# Check if button exists (might not be created yet during startup)
	try:
		if paused:
			# Camera is paused - show "Resume" button
			Btn_CameraPause.config(bg='#FF6D00', fg='#000000', text='Camera Resume [,]')
			print("📷 Camera Pause ACTIVE (synced from server)")
		else:
			# Camera is active - show "Pause" button
			Btn_CameraPause.config(bg=color_btn, fg=color_text, text='Camera Pause [,]')
			print("📷 Camera Pause INACTIVE (synced from server)")
	except (NameError, AttributeError) as e:
		# Button not created yet, state will be applied when button is created
		print(f"Camera pause state received: {'PAUSED' if paused else 'ACTIVE'} (button not ready yet) - {e}")


# ==================== Connection Thread ====================
def connection_thread():
	"""Main connection thread - receives and processes messages from server"""
	global funcMode, Switch_3, Switch_2, Switch_1, SmoothMode, SmoothCamMode, steadyMode
	global CPU_TEP, CPU_USE, RAM_USE, BATTERY_VOLTAGE
	global GYRO_X, GYRO_Y, GYRO_Z, ACCEL_X, ACCEL_Y, ACCEL_Z  # MPU6050 data
	global video_thread_started  # Use the global flag

	try:
		print("[connection_thread] Starting to receive messages...")
		while 1:
			car_info = (tcpClicSock.recv(BUFSIZ)).decode()
			if not car_info:
				print("[connection_thread] Connection closed by server (empty recv)")
				break  # Exit loop - connection is closed!

			# DEBUG: Log ALL received messages initially for debugging
			if not car_info.startswith(STATUS_INFO_PREFIX):
				print(f"[DEBUG] Received: {car_info[:100]}...")  # First 100 chars
			else:
				# Log first INFO message to confirm data flow
				if not hasattr(connection_thread, '_first_info_logged'):
					print(f"[DEBUG] First INFO received: {car_info[:50]}...")
					connection_thread._first_info_logged = True

			# Dispatch message to appropriate handler
			if STATUS_VIDEO_READY in car_info:
				handle_video_ready()

			elif 'VIDEO_TIMEOUT' in car_info:
				handle_video_timeout()

			elif car_info.startswith(STATUS_INFO_PREFIX):
				handle_info_message(car_info)

			elif car_info == CMD_STEADY_CAMERA:
				handle_steady_camera(enabled=True)

			elif car_info == CMD_STEADY_CAMERA_OFF:
				handle_steady_camera(enabled=False)

			elif CMD_SMOOTH_CAM in car_info:
				handle_smooth_cam(enabled=True)

			elif CMD_SMOOTH_CAM_OFF in car_info:
				handle_smooth_cam(enabled=False)

			# Note: Use 'if' instead of 'elif' for status messages
			# because multiple status updates can arrive in one TCP message
			if STATUS_SERVO_STANDBY in car_info:
				handle_servo_standby_status(active=True)

			if STATUS_SERVO_WAKEUP in car_info:
				handle_servo_standby_status(active=False)

			if STATUS_CAMERA_PAUSED in car_info:
				handle_camera_pause_status(paused=True)

			if STATUS_CAMERA_RESUMED in car_info:
				handle_camera_pause_status(paused=False)

			elif CMD_SWITCH_1_ON in car_info:
				handle_switch(1, enabled=True)

			elif CMD_SWITCH_1_OFF in car_info:
				handle_switch(1, enabled=False)

			elif CMD_SWITCH_2_ON in car_info:
				handle_switch(2, enabled=True)

			elif CMD_SWITCH_2_OFF in car_info:
				handle_switch(2, enabled=False)

			elif CMD_SWITCH_3_ON in car_info:
				handle_switch(3, enabled=True)

			elif CMD_SWITCH_3_OFF in car_info:
				handle_switch(3, enabled=False)


	except Exception as e:
		# Connection closed or socket error - exit thread gracefully
		print(f"[connection_thread] Connection thread stopped: {e}")
		import traceback
		traceback.print_exc()

	# Signal that connection was lost
	print("[connection_thread] Connection lost - will attempt reconnect")


def connection_monitor_thread():
	"""Monitor connection and auto-reconnect if lost"""
	global ip_stu, video_thread_started

	max_reconnect_attempts = 5
	reconnect_delay = 5  # seconds

	while not shutdown_requested:
		# Start connection thread
		conn_thread = thread.Thread(target=connection_thread, daemon=True)
		conn_thread.start()

		# Wait for connection thread to finish (means connection was lost)
		conn_thread.join()

		if shutdown_requested:
			break

		# Connection was lost - attempt reconnect
		for attempt in range(1, max_reconnect_attempts + 1):
			if shutdown_requested:
				break

			print(f"\n[Auto-Reconnect] Attempt {attempt}/{max_reconnect_attempts} in {reconnect_delay}s...")
			update_connection_status('Reconnecting', '#FF9800', f'Retry {attempt}/{max_reconnect_attempts}')

			time.sleep(reconnect_delay)

			if shutdown_requested:
				break

			# Try to reconnect
			print(f"[Auto-Reconnect] Attempting to reconnect to {ip_adr}:10223...")
			try:
				# Reset video thread flag
				video_thread_started = False

				# Create new socket
				global tcpClicSock
				tcpClicSock = socket(AF_INET, SOCK_STREAM)
				tcpClicSock.connect((ip_adr, 10223))

				print("✓ Reconnected to server!")
				update_connection_status('Connected', '#558B2F', f'Reconnected')
				ip_stu = 0

				# Break retry loop - connection successful, outer while will start new connection_thread
				break

			except Exception as e:
				print(f"✗ Reconnect attempt {attempt} failed: {e}")
				if attempt < max_reconnect_attempts:
					continue
				else:
					print(f"✗ Failed to reconnect after {max_reconnect_attempts} attempts")
					update_connection_status('Disconnected', '#F44336', 'Reconnect failed')
					ip_stu = 1
					return  # Exit monitor thread

		if ip_stu == 1:  # All reconnect attempts failed
			break


# ==================== Connection Functions ====================

def update_connection_status(status, color, message=''):
	"""Update connection status display"""
	l_ip_4.config(text=status, bg=color)
	if message:
		l_ip_5.config(text=message)


def get_server_address():
	"""Get server IP address from Entry or default config"""
	ip_adr = E1.get()

	if ip_adr == '':
		ip_adr = num_import('IP:')
		update_connection_status('Connecting', '#FF8F00', f'Default:{ip_adr}')

	return ip_adr


def establish_connection(server_ip, server_port):
	"""Try to establish connection with server with retry and exponential backoff"""
	global tcpClicSock, ip_stu

	ADDR = (server_ip, server_port)
	tcpClicSock = socket(AF_INET, SOCK_STREAM)

	max_retries = 8
	retry_delays = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0]  # Progressive delays

	for attempt in range(max_retries):
		if ip_stu == 1:
			print(f"Connecting to server @ {server_ip}:{server_port} (attempt {attempt + 1}/{max_retries})...")
			update_connection_status(f'Connecting {attempt + 1}/{max_retries}...', '#2196F3')

			try:
				tcpClicSock.connect(ADDR)
				print("✓ Connected to server")
				update_connection_status('Connected', '#4CAF50')
				return True
			except Exception as e:
				print(f"✗ Connection attempt {attempt + 1} failed: {e}")
				if attempt < max_retries - 1:
					delay = retry_delays[attempt]
					print(f"   Retrying in {delay}s...")
					update_connection_status(f'Retry in {delay}s...', '#EF6C00')
					time.sleep(delay)
				else:
					print(f"✗ Failed to connect after {max_retries} attempts")
					update_connection_status('Connection failed', '#F44336')
		else:
			break

	return False


def video_ready_timeout_watchdog():
	"""Watchdog that warns if VIDEO_READY signal is not received within timeout"""
	global video_thread_started

	timeout_seconds = 15  # Should receive VIDEO_READY within 15 seconds
	time.sleep(timeout_seconds)

	if not video_thread_started:
		print("⚠️ WARNING: Video server did not respond within 15s")
		print("   This is usually caused by:")
		print("   1. SSH tunnel not forwarding video port (check for 'connection refused')")
		print("   2. Video server failed to start on Raspberry Pi")
		print("   3. Network issues between client and server")
		print("   → GUI will continue to work, but video stream is unavailable")
		print("   → Try reconnecting or check server logs")


def start_connection_threads():
	"""Start connection and video threads with auto-reconnect"""
	global video_thread_started

	# Start connection monitor (manages connection_thread and reconnects)
	monitor_threading = thread.Thread(target=connection_monitor_thread, daemon=True)
	monitor_threading.start()

	print("Connection monitor started with auto-reconnect...")
	print("Waiting for video server to initialize...")

	# Start timeout watchdog for VIDEO_READY signal
	timeout_thread = thread.Thread(target=video_ready_timeout_watchdog, daemon=True)
	timeout_thread.start()


def socket_connect():
	"""Call this function to connect with the server"""
	global ADDR, tcpClicSock, ip_stu, ipaddr, ip_adr, video_thread_started

	# Reset video thread flag for new connection
	video_thread_started = False

	# Get server address
	ip_adr = get_server_address()

	SERVER_IP = ip_adr
	SERVER_PORT = 10223

	# Try to establish connection
	if establish_connection(SERVER_IP, SERVER_PORT):
		# Connection successful
		update_connection_status('Connected', '#558B2F', f'IP:{ip_adr}')

		replace_num('IP:', ip_adr)
		E1.config(state='disabled')
		Btn14.config(state='disabled')

		ip_stu = 0  # '0' means connected

		# Start connection threads
		start_connection_threads()
	else:
		# Connection failed
		if ip_stu == 1:
			update_connection_status('Disconnected', '#F44336')


def connect(event):	   #Call this function to connect with the server
	if ip_stu == 1:
		sc=thread.Thread(target=socket_connect, daemon=True) #Define a thread for connection
		sc.start()							  #Thread starts


def connect_click():	   #Call this function to connect with the server
	if ip_stu == 1:
		sc=thread.Thread(target=socket_connect, daemon=True) #Define a thread for connection
		sc.start()							  #Thread starts


# ==================== GUI Helper Functions ====================

def setup_window_closing_handler(root):
	"""Setup the window closing handler for clean shutdown"""
	def on_closing():
		"""Clean shutdown when window is closed"""
		global tcpClicSock, footage_socket, footage_process, shutdown_requested

		shutdown_requested = True  # Signal that shutdown was requested
		print("Shutting down GUI...")

		try:
			# Close TCP socket
			if tcpClicSock and tcpClicSock != '':
				print("Closing TCP socket...")
				tcpClicSock.close()
		except Exception as e:
			print(f"Error closing TCP socket: {e}")

		try:
			# Close footage socket
			if footage_socket is not None:
				print("Closing footage socket...")
				footage_socket.close()
		except Exception as e:
			print(f"Error closing footage socket: {e}")

		try:
			# Terminate the Footage-GUI process if running
			if footage_process is not None and footage_process.poll() is None:
				print("Terminating Footage-GUI process...")
				footage_process.terminate()
				try:
					footage_process.wait(timeout=2)
					print("✓ Footage-GUI process terminated")
				except:
					print("Force killing Footage-GUI process...")
					footage_process.kill()
		except Exception as e:
			print(f"Error terminating footage process: {e}")

		try:
			# Close all OpenCV windows
			cv2.destroyAllWindows()
		except Exception as e:
			print(f"Error closing OpenCV windows: {e}")

		# Destroy the root window
		print("Closing GUI window...")
		root.destroy()

		# Exit the application completely
		print("✓ Application shutdown complete")
		import sys
		sys.exit(0)

	# Register the cleanup function
	root.protocol("WM_DELETE_WINDOW", on_closing)


def create_status_labels(root, color_text, color_bg, color_btn):
	"""Create status labels (CPU, RAM, Battery, Connection)"""
	global CPU_TEP_lab, CPU_USE_lab, RAM_lab, BATTERY_lab, l_ip_4, l_ip_5

	# Logo
	try:
		logo = tk.PhotoImage(file='logo2.png')
		l_logo = tk.Label(root, image=logo, bg=color_bg)
		l_logo.image = logo  # Keep a reference!
		l_logo.place(x=30, y=13)
	except:
		pass

	# System status labels
	CPU_TEP_lab = tk.Label(root, width=18, text='CPU Temp:', fg=color_text, bg='#212121')
	CPU_TEP_lab.place(x=400, y=15)

	CPU_USE_lab = tk.Label(root, width=18, text='CPU Usage:', fg=color_text, bg='#212121')
	CPU_USE_lab.place(x=400, y=45)

	RAM_lab = tk.Label(root, width=18, text='RAM Usage:', fg=color_text, bg='#212121')
	RAM_lab.place(x=400, y=75)

	BATTERY_lab = tk.Label(root, width=18, text='Battery: N/A', fg=color_text, bg='#757575')
	BATTERY_lab.place(x=400, y=105)

	# Connection status labels
	l_ip_4 = tk.Label(root, width=18, text='Disconnected', fg=color_text, bg='#F44336')
	l_ip_4.place(x=400, y=140)

	l_ip_5 = tk.Label(root, width=18, text='Use default IP', fg=color_text, bg=color_btn)
	l_ip_5.place(x=400, y=175)


def create_ip_entry(root, color_text):
	"""Create IP address entry field"""
	global E1, l_ip_3, Btn14

	E1 = tk.Entry(root, show=None, width=16, bg="#37474F", fg='#eceff1')
	E1.place(x=180, y=40)

	l_ip_3 = tk.Label(root, width=10, text='IP Address:', fg=color_text, bg='#000000')
	l_ip_3.place(x=175, y=15)

	Btn14 = tk.Button(root, width=8, height=2, text='Connect', fg=color_text, bg=color_btn,
	                  command=connect_click, relief='ridge')
	Btn14.place(x=315, y=15)

	root.bind('<Return>', connect)


def create_movement_buttons(root, color_text, color_btn):
	"""Create movement control buttons"""
	# Switch buttons (Port 1, 2, 3)
	global Btn_Switch_1, Btn_Switch_2, Btn_Switch_3
	global speed_slider_widget, speed_value_label_widget
	global arc_factor_slider_widget, arc_factor_value_label_widget

	# Speed control slider
	speed_label = tk.Label(root, text='Speed:', fg=color_text, bg='#212121', width=6)
	speed_label.place(x=30, y=115)

	speed_slider = tk.Scale(root, from_=10, to=80, orient=tk.HORIZONTAL,
							length=200, fg=color_text, bg='#37474F',
							troughcolor='#263238', highlightthickness=0,
							command=lambda val: on_speed_change(val))
	speed_slider.set(35)  # Default speed
	speed_slider.place(x=30, y=140)

	speed_value_label = tk.Label(root, text='35', fg=color_text, bg='#212121', width=3)
	speed_value_label.place(x=240, y=145)

	speed_slider_widget = speed_slider
	speed_value_label_widget = speed_value_label

	# Arc Factor control slider
	arc_factor_label = tk.Label(root, text='Arc Factor:', fg=color_text, bg='#212121', width=10)
	arc_factor_label.place(x=30, y=180)

	arc_factor_slider = tk.Scale(root, from_=0.0, to=1.0, resolution=0.05, orient=tk.HORIZONTAL,
								 length=200, fg=color_text, bg='#37474F',
								 troughcolor='#263238', highlightthickness=0,
								 command=lambda val: on_arc_factor_change(val))
	arc_factor_slider.set(0.7)  # Default arc factor
	arc_factor_slider.place(x=30, y=205)

	arc_factor_value_label = tk.Label(root, text='0.70', fg=color_text, bg='#212121', width=4)
	arc_factor_value_label.place(x=240, y=210)

	arc_factor_slider_widget = arc_factor_slider
	arc_factor_value_label_widget = arc_factor_value_label

	# Movement buttons (Forward, Backward, Left, Right) - Repositioned
	Btn0 = tk.Button(root, width=8, text='Forward', fg=color_text, bg=color_btn, relief='ridge')
	Btn1 = tk.Button(root, width=8, text='Backward', fg=color_text, bg=color_btn, relief='ridge')
	Btn_LeftSide = tk.Button(root, width=8, text='Arc Left', fg=color_text, bg=color_btn, relief='ridge')
	Btn2 = tk.Button(root, width=8, text='Left', fg=color_text, bg=color_btn, relief='ridge')
	Btn3 = tk.Button(root, width=8, text='Right', fg=color_text, bg=color_btn, relief='ridge')
	Btn_RightSide = tk.Button(root, width=8, text='Arc Right', fg=color_text, bg=color_btn, relief='ridge')

	Btn0.place(x=100, y=265)    # Forward (center top)
	Btn_LeftSide.place(x=30, y=265) # Left Side (left top, above Left)
	Btn_RightSide.place(x=170, y=265) # Right Side (right top, above Right)
	Btn1.place(x=100, y=300)    # Backward (center middle)
	Btn2.place(x=30, y=300)     # Left (left middle)
	Btn3.place(x=170, y=300)    # Right (right middle)

	# Switch buttons (Port 1, 2, 3) - Repositioned
	Btn_Switch_1 = tk.Button(root, width=8, text='Port 1', fg=color_text, bg=color_btn, relief='ridge')
	Btn_Switch_2 = tk.Button(root, width=8, text='Port 2', fg=color_text, bg=color_btn, relief='ridge')
	Btn_Switch_3 = tk.Button(root, width=8, text='Port 3', fg=color_text, bg=color_btn, relief='ridge')

	Btn_Switch_1.place(x=30, y=345)
	Btn_Switch_2.place(x=100, y=345)
	Btn_Switch_3.place(x=170, y=345)

	Btn_Switch_1.bind('<ButtonPress-1>', call_Switch_1)
	Btn_Switch_2.bind('<ButtonPress-1>', call_Switch_2)
	Btn_Switch_3.bind('<ButtonPress-1>', call_Switch_3)

	# Bind keyboard events
	root.bind('<KeyPress-w>', call_forward)
	root.bind('<KeyRelease-w>', call_DS)
	root.bind('<KeyPress-s>', call_back)
	root.bind('<KeyRelease-s>', call_DS)
	root.bind('<KeyPress-a>', call_Left)
	root.bind('<KeyRelease-a>', call_TS)
	root.bind('<KeyPress-d>', call_Right)
	root.bind('<KeyRelease-d>', call_TS)

	root.bind('<KeyRelease-w>', call_FB_stop)
	root.bind('<KeyRelease-s>', call_FB_stop)
	root.bind('<KeyRelease-a>', call_Turn_stop)
	root.bind('<KeyRelease-d>', call_Turn_stop)

	# Additional keyboard shortcuts for sideways movement
	root.bind('<KeyPress-q>', call_LeftSide)
	root.bind('<KeyPress-e>', call_RightSide)
	root.bind('<KeyRelease-q>', call_Turn_stop)
	root.bind('<KeyRelease-e>', call_Turn_stop)

	root.bind('<KeyPress-h>', call_headhome)

	# Bind mouse events
	Btn0.bind('<ButtonPress-1>', call_forward)
	Btn0.bind('<ButtonRelease-1>', call_DS)
	Btn1.bind('<ButtonPress-1>', call_back)
	Btn1.bind('<ButtonRelease-1>', call_DS)
	Btn2.bind('<ButtonPress-1>', call_Left)
	Btn2.bind('<ButtonRelease-1>', call_TS)
	Btn3.bind('<ButtonPress-1>', call_Right)
	Btn3.bind('<ButtonRelease-1>', call_TS)
	Btn_LeftSide.bind('<ButtonPress-1>', call_LeftSide)
	Btn_LeftSide.bind('<ButtonRelease-1>', call_Turn_stop)
	Btn_RightSide.bind('<ButtonPress-1>', call_RightSide)
	Btn_RightSide.bind('<ButtonRelease-1>', call_Turn_stop)

	Btn0.bind('<ButtonRelease-1>', call_FB_stop)
	Btn1.bind('<ButtonRelease-1>', call_FB_stop)
	Btn2.bind('<ButtonRelease-1>', call_Turn_stop)
	Btn3.bind('<ButtonRelease-1>', call_Turn_stop)


def create_camera_control_buttons(root, color_text, color_btn):
	"""Create camera control buttons (pan/tilt)"""
	Btn_up = tk.Button(root, width=8, text='Up', fg=color_text, bg=color_btn, relief='ridge')
	Btn_down = tk.Button(root, width=8, text='Down', fg=color_text, bg=color_btn, relief='ridge')
	Btn_left = tk.Button(root, width=8, text='Left', fg=color_text, bg=color_btn, relief='ridge')
	Btn_right = tk.Button(root, width=8, text='Right', fg=color_text, bg=color_btn, relief='ridge')
	Btn_home = tk.Button(root, width=8, text='Home', fg=color_text, bg=color_btn, relief='ridge')
	
	# Position camera buttons on the RIGHT side
	Btn_up.place(x=400, y=195 + 70)      # Up
	Btn_down.place(x=400, y=265 + 70)    # Down
	Btn_left.place(x=330, y=230 + 70)    # Left
	Btn_right.place(x=470, y=230 + 70)   # Right
	Btn_home.place(x=400, y=230 + 70)    # Home

	# Keyboard bindings
	root.bind('<KeyPress-i>', call_headup)
	root.bind('<KeyRelease-i>', call_UDstop)
	root.bind('<KeyPress-k>', call_headdown)
	root.bind('<KeyRelease-k>', call_UDstop)
	root.bind('<KeyPress-j>', call_headleft)
	root.bind('<KeyRelease-j>', call_LRstop)
	root.bind('<KeyPress-l>', call_headright)
	root.bind('<KeyRelease-l>', call_LRstop)

	# Mouse bindings
	Btn_up.bind('<ButtonPress-1>', call_headup)
	Btn_up.bind('<ButtonRelease-1>', call_UDstop)
	Btn_down.bind('<ButtonPress-1>', call_headdown)
	Btn_down.bind('<ButtonRelease-1>', call_UDstop)
	Btn_left.bind('<ButtonPress-1>', call_headleft)
	Btn_left.bind('<ButtonRelease-1>', call_LRstop)
	Btn_right.bind('<ButtonPress-1>', call_headright)
	Btn_right.bind('<ButtonRelease-1>', call_LRstop)
	Btn_home.bind('<ButtonPress-1>', call_headhome)


def create_feature_buttons(root, color_text, color_btn):
	"""Create feature buttons (Steady, SmoothCam, Power Management)"""
	global Btn_Steady, Btn_SmoothCam, Btn_ServoStandby, Btn_CameraPause
	global servo_standby_state, camera_pause_state

	# Steady mode button
	Btn_Steady = tk.Button(root, width=10, text='Steady [Z]', fg=color_text, bg=color_btn, relief='ridge')
	Btn_Steady.place(x=30, y=410)
	root.bind('<KeyPress-z>', call_steady)
	Btn_Steady.bind('<ButtonPress-1>', call_steady)

	# SmoothCam button
	Btn_SmoothCam = tk.Button(root, width=10, text='Smooth-Cam [N]', fg=color_text, bg=color_btn, relief='ridge')
	Btn_SmoothCam.place(x=115, y=410)
	root.bind('<KeyPress-n>', call_SmoothCam)
	Btn_SmoothCam.bind('<ButtonPress-1>', call_SmoothCam)

	# Power Management buttons - initialize with current state from server
	# Servo Standby button
	if servo_standby_state:
		# Servo is in standby - show "Wake" button (orange)
		Btn_ServoStandby = tk.Button(root, width=21, text='Servo Wake [M]',
		                              fg='#000000', bg='#FF6D00', relief='ridge')
	else:
		# Servo is awake - show "Standby" button (blue)
		Btn_ServoStandby = tk.Button(root, width=21, text='Servo Standby [M]',
		                              fg=color_text, bg=color_btn, relief='ridge')
	Btn_ServoStandby.place(x=30, y=480)
	root.bind('<KeyPress-m>', call_servo_standby)
	Btn_ServoStandby.bind('<ButtonPress-1>', call_servo_standby)

	# Camera Pause button
	if camera_pause_state:
		# Camera is paused - show "Resume" button (orange)
		Btn_CameraPause = tk.Button(root, width=21, text='Camera Resume [,]',
		                             fg='#000000', bg='#FF6D00', relief='ridge')
	else:
		# Camera is active - show "Pause" button (blue)
		Btn_CameraPause = tk.Button(root, width=21, text='Camera Pause [,]',
		                             fg=color_text, bg=color_btn, relief='ridge')
	Btn_CameraPause.place(x=200, y=480)
	root.bind('<KeyPress-comma>', call_camera_pause)
	Btn_CameraPause.bind('<ButtonPress-1>', call_camera_pause)


def create_mpu_canvas(root):
	"""Create MPU6050 visualization canvas"""
	global canvas_mpu

	canvas_mpu = tk.Canvas(root, bg='#1a1a1a', width=160, height=160,
	                       highlightthickness=1, highlightbackground='#424242')
	canvas_mpu.place(x=385, y=480)

	# Initial display (will show "N/A" until data arrives)
	update_mpu_canvas()


# ==================== Main GUI Loop ====================

def loop():
	"""Main GUI loop - creates and manages the GUI window"""
	global tcpClicSock, root, E1, connect, l_ip_4, l_ip_5, color_btn, color_text, Btn14
	global CPU_TEP_lab, CPU_USE_lab, RAM_lab, BATTERY_lab
	global Btn_Steady, Btn_Switch_1, Btn_Switch_2, Btn_Switch_3, Btn_SmoothCam
	global canvas_mpu, color_bg, shutdown_requested, stat

	while True:
		# Define color scheme
		color_bg = '#000000'      # Background color
		color_text = '#E1F5FE'    # Text color
		color_btn = '#0277BD'     # Button color
		color_line = '#01579B'    # Line color
		color_can = '#212121'     # Canvas color
		color_oval = '#2196F3'    # Oval color
		target_color = '#FF6D00'  # Target color

		# Create main window
		root = tk.Tk()
		root.title('RaspClaws (schmiereck)')
		root.geometry('565x680')
		root.config(bg=color_bg)

		# Setup window closing handler
		setup_window_closing_handler(root)

		# Create GUI components
		create_status_labels(root, color_text, color_bg, color_btn)
		create_ip_entry(root, color_text)
		create_movement_buttons(root, color_text, color_btn)
		create_camera_control_buttons(root, color_text, color_btn)
		create_feature_buttons(root, color_text, color_btn)
		create_mpu_canvas(root)

		# Run the main loop
		if stat == 0:  # Ensure the mainloop runs only once
			root.mainloop()
			stat = 1   # Change the value to '1' so the mainloop() would not run again.

		# If shutdown was requested, break the while loop
		if shutdown_requested:
			print("Exiting application loop...")
			break



if __name__ == '__main__':
	try:
		loop()				   # Load GUI
	except Exception as e:
		print("Error: ", e)
		import traceback
		traceback.print_exc()
		try:
			tcpClicSock.close() # Close socket or it may not connect with the server again
		except:
			pass
		try:
			footage_socket.close()
		except:
			pass
		try:
			# Terminate the Footage-GUI process if running
			if footage_process and footage_process.poll() is None:
				print("Terminating Footage-GUI process...")
				footage_process.terminate()
				footage_process.wait(timeout=2)
				print("✓ Footage-GUI process terminated")
		except Exception as e:
			print(f"Error terminating footage process: {e}")
			try:
				footage_process.kill()  # Force kill if terminate didn't work
			except:
				pass
		try:
			cv2.destroyAllWindows()
		except:
			pass
