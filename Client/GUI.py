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

Switch_3 = 0
Switch_2 = 0
Switch_1 = 0
SmoothMode = 0
SmoothCamMode = 0
FV_Line = 0
FV_Start = 0
video_thread_started = False  # Track if video thread was started (global)

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
	tcpClicSock.send(command.encode())


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


# ==================== Movement Callbacks ====================

def call_forward(event):
	"""Command car to move forward"""
	send_movement_command('forward', 'c_f_stu')


def call_back(event):
	"""Command car to move backward"""
	send_movement_command('backward', 'c_b_stu')


def call_Left(event):
	"""Command car to turn left"""
	send_movement_command('left', 'c_l_stu')


def call_Right(event):
	"""Command car to turn right"""
	send_movement_command('right', 'c_r_stu')


def call_LeftSide(event):
	"""Command car to strafe left"""
	send_movement_command('leftside', 'c_ls_stu')


def call_RightSide(event):
	"""Command car to strafe right"""
	send_movement_command('rightside', 'c_rs_stu')


def call_DS(event):
	"""Direction Stop"""
	global DS_stu
	send_command('DS')
	DS_stu = 0


def call_TS(event):
	"""Turn Stop"""
	global TS_stu
	send_command('TS')
	TS_stu = 0


def call_FB_stop(event):
	"""Stop forward/backward movement"""
	global c_f_stu, c_b_stu
	c_f_stu = 0
	c_b_stu = 0
	send_command('DS')


def call_Turn_stop(event):
	"""Stop turning movement"""
	global c_l_stu, c_r_stu, c_ls_stu, c_rs_stu
	c_l_stu = 0
	c_r_stu = 0
	c_ls_stu = 0
	c_rs_stu = 0
	send_command('TS')


# ==================== Camera Callbacks ====================

def call_headup(event):
	send_command('up')


def call_headdown(event):
	send_command('down')


def call_headleft(event):
	send_command('lookleft')


def call_headright(event):
	send_command('lookright')


def call_LRstop(event):
	send_command('LRstop')


def call_UDstop(event):
	send_command('UDstop')


def call_headhome(event):
	send_command('home')


# ==================== Mode Toggle Callbacks ====================

def call_steady(event):
	"""Toggle steady camera mode"""
	global steadyMode
	if steadyMode == 0:
		send_command('steadyCamera')
		steadyMode = 1
	else:
		send_command('steadyCameraOff')
		steadyMode = 0


def call_FindColor(event):
	"""Toggle find color mode"""
	global funcMode
	if funcMode == 0:
		send_command('findColor')
		funcMode = 1
	else:
		send_command('stopCV')
		funcMode = 0


def call_WatchDog(event):
	"""Toggle watchdog/motion detection mode"""
	global funcMode
	if funcMode == 0:
		send_command('motionGet')
		funcMode = 1
	else:
		send_command('stopCV')
		funcMode = 0


def call_Smooth(event):
	"""Toggle smooth movement mode"""
	global SmoothMode
	if SmoothMode == 0:
		send_command('slow')
		SmoothMode = 1
	else:
		send_command('fast')
		SmoothMode = 0


def call_SmoothCam(event):
	"""Toggle smooth camera movement mode"""
	global SmoothCamMode
	if SmoothCamMode == 0:
		send_command('smoothCam')
		SmoothCamMode = 1
	else:
		send_command('smoothCamOff')
		SmoothCamMode = 0


def call_Police(event):
	"""Toggle police LED mode"""
	global funcMode
	if funcMode == 0:
		send_command('police')
		funcMode = 1
	else:
		send_command('policeOff')
		funcMode = 0


def call_Switch_1(event):
	"""Toggle GPIO Switch 1"""
	if Switch_1 == 0:
		send_command('Switch_1_on')
	else:
		send_command('Switch_1_off')


def call_Switch_2(event):
	"""Toggle GPIO Switch 2"""
	if Switch_2 == 0:
		send_command('Switch_2_on')
	else:
		send_command('Switch_2_off')


def call_Switch_3(event):
	"""Toggle GPIO Switch 3"""
	if Switch_3 == 0:
		send_command('Switch_3_on')
	else:
		send_command('Switch_3_off')


def all_btn_red():
	"""Set all mode buttons to active (red) color"""
	for btn in [Btn_Steady, Btn_FindColor, Btn_WatchDog, Btn_Smooth, Btn_Police]:
		btn.config(bg='#FF6D00', fg='#000000')


def all_btn_normal():
	"""Set all mode buttons to normal color"""
	for btn in [Btn_Steady, Btn_FindColor, Btn_WatchDog, Btn_Smooth, Btn_Police]:
		btn.config(bg=color_btn, fg=color_text)


# ==================== Connection Thread ====================
def connection_thread():
	global funcMode, Switch_3, Switch_2, Switch_1, SmoothMode, SmoothCamMode, steadyMode
	global CPU_TEP, CPU_USE, RAM_USE, BATTERY_VOLTAGE
	global video_thread_started  # Use the global flag

	while 1:
		car_info = (tcpClicSock.recv(BUFSIZ)).decode()
		if not car_info:
			continue

		# Handle VIDEO_READY signal from server
		elif 'VIDEO_READY' in car_info:
			if not video_thread_started:
				print("✅ Server signals: Video stream is ready")
				video_threading = thread.Thread(target=run_open, daemon=True)
				video_threading.start()
				video_thread_started = True
				print("✓ Video thread started")
			continue

		elif 'VIDEO_TIMEOUT' in car_info:
			print("⚠ Video server failed to start - no video stream available")
			continue

		elif car_info.startswith('INFO:'):
			# Process CPU/RAM/Battery info from server
			try:
				info_data = car_info[5:].strip()  # Remove 'INFO:' prefix

				# Split by '|' to separate system info from servo positions
				parts = info_data.split('|')
				system_info = parts[0].strip()
				servo_info = parts[1].strip() if len(parts) > 1 else None

				info_get = system_info.split()
				if len(info_get) >= 4:
					# New format: CPU_TEMP CPU_USE RAM_USE BATTERY_VOLTAGE
					CPU_TEP, CPU_USE, RAM_USE, BATTERY_VOLTAGE = info_get[0], info_get[1], info_get[2], info_get[3]
					CPU_TEP_lab.config(text='CPU Temp: %s℃'%CPU_TEP)
					CPU_USE_lab.config(text='CPU Usage: %s'%CPU_USE)
					RAM_lab.config(text='RAM Usage: %s'%RAM_USE)

					# Update battery display with color coding
					battery_volt = float(BATTERY_VOLTAGE)
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

					# Log servo positions to terminal with timestamp
					if servo_info:
						timestamp = int(time.time() * 1000)  # Milliseconds since epoch
						print(f"[{timestamp}] [SERVOS] {servo_info}")

				elif len(info_get) >= 3:
					# Old format without battery (backwards compatibility)
					CPU_TEP, CPU_USE, RAM_USE = info_get[0], info_get[1], info_get[2]
					CPU_TEP_lab.config(text='CPU Temp: %s℃'%CPU_TEP)
					CPU_USE_lab.config(text='CPU Usage: %s'%CPU_USE)
					RAM_lab.config(text='RAM Usage: %s'%RAM_USE)
					BATTERY_lab.config(text='Battery: N/A', bg='#757575', fg=color_text)

					# Log servo positions to terminal with timestamp
					if servo_info:
						timestamp = int(time.time() * 1000)  # Milliseconds since epoch
						print(f"[{timestamp}] [SERVOS] {servo_info}")

			except Exception as e:
				print(f"⚠ Error processing INFO: {e}")
				pass
		elif 'findColor' in car_info:
			funcMode = 1
			SmoothMode = 1
			Btn_FindColor.config(bg='#FF6D00', fg='#000000')

		elif 'steadyCamera' == car_info:
			steadyMode = 1
			SmoothMode = 1
			Btn_Steady.config(bg='#FF6D00', fg='#000000')

		elif 'steadyCameraOff' == car_info:
			steadyMode = 0
			SmoothMode = 0
			Btn_Steady.config(bg=color_btn, fg=color_text)

		elif 'motionGet' in car_info:
			funcMode = 1
			SmoothMode = 1
			Btn_WatchDog.config(bg='#FF6D00', fg='#000000')

		elif 'slow' in car_info:
			funcMode = 1
			SmoothMode = 1
			Btn_Smooth.config(bg='#FF6D00', fg='#000000')

		elif 'fast' in car_info:
			funcMode = 0
			SmoothMode = 0
			Btn_Smooth.config(bg=color_btn, fg=color_text)

		elif 'smoothCam' in car_info:
			SmoothCamMode = 1
			Btn_SmoothCam.config(bg='#FF6D00', fg='#000000')

		elif 'smoothCamOff' in car_info:
			SmoothCamMode = 0
			Btn_SmoothCam.config(bg=color_btn, fg=color_text)

		elif 'police' == car_info:
			funcMode = 1
			SmoothMode = 1
			Btn_Police.config(bg='#FF6D00', fg='#000000')

		elif 'policeOff' == car_info:
			funcMode = 0
			SmoothMode = 0
			Btn_Police.config(bg=color_btn, fg=color_text)

		elif 'Switch_3_on' in car_info:
			Switch_3 = 1
			Btn_Switch_3.config(bg='#4CAF50')

		elif 'Switch_2_on' in car_info:
			Switch_2 = 1
			Btn_Switch_2.config(bg='#4CAF50')

		elif 'Switch_1_on' in car_info:
			Switch_1 = 1
			Btn_Switch_1.config(bg='#4CAF50')

		elif 'Switch_3_off' in car_info:
			Switch_3 = 0
			Btn_Switch_3.config(bg=color_btn)

		elif 'Switch_2_off' in car_info:
			Switch_2 = 0
			Btn_Switch_2.config(bg=color_btn)

		elif 'Switch_1_off' in car_info:
			Switch_1 = 0
			Btn_Switch_1.config(bg=color_btn)

		elif 'CVFL' in car_info:
			FV_Start = 1


		elif 'stopCV' in car_info:
			SmoothMode = 0
			funcMode = 0
			Btn_FindColor.config(bg=color_btn, fg=color_text)
			Btn_WatchDog.config(bg=color_btn, fg=color_text)

		print(car_info)


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
	"""Try to establish connection with server"""
	global tcpClicSock, ip_stu

	ADDR = (server_ip, server_port)
	tcpClicSock = socket(AF_INET, SOCK_STREAM)

	for i in range(1, 6):  # Try 5 times if disconnected
		if ip_stu == 1:
			print(f"Connecting to server @ {server_ip}:{server_port}...")
			print("Connecting")

			try:
				tcpClicSock.connect(ADDR)
				print("Connected")
				return True
			except:
				print(f"Cannot connect to server, try {i}/5 time(s)")
				update_connection_status(f'Try {i}/5 time(s)', '#EF6C00')
				time.sleep(1)
				continue
		else:
			break

	return False


def start_connection_threads():
	"""Start connection and video threads"""
	global video_thread_started

	connection_threading = thread.Thread(target=connection_thread, daemon=True)
	connection_threading.start()

	print("Waiting for video server to initialize...")


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



def EC_send(event):#z
		tcpClicSock.send(('setEC %s'%var_ec.get()).encode())
		time.sleep(0.03)


def EC_default(event):#z
	var_ec.set(0)
	tcpClicSock.send(('defEC').encode())


def scale_FL(x,y,w):#1
	global Btn_CVFL,FV_Line,FV_Start
	def lip1_send(event):
		time.sleep(0.03)
		tcpClicSock.send(('CVFLL1 %s'%var_lip1.get()).encode())

	def lip2_send(event):
		time.sleep(0.03)
		tcpClicSock.send(('CVFLL2 %s'%var_lip2.get()).encode())

	def err_send(event):
		time.sleep(0.03)
		tcpClicSock.send(('err %s'%var_err.get()).encode())



	def call_CVFL(event):
		global FV_Start
		if not FV_Start:
			tcpClicSock.send(('CVFL').encode())
			FV_Start = 1
		else:
			tcpClicSock.send(('stopCV').encode())
			FV_Start = 0
    

	def call_WB(event):
		global FV_Line
		if not FV_Line:
			tcpClicSock.send(('CVFLColorSet 0').encode())
			FV_Line = 1
		else:
			tcpClicSock.send(('CVFLColorSet 255').encode())
			FV_Line = 0

	Scale_lip1 = tk.Scale(root,label=None,
	from_=0,to=480,orient=tk.HORIZONTAL,length=w,
	showvalue=1,tickinterval=None,resolution=1,variable=var_lip1,troughcolor='#212121',command=lip1_send,fg=color_text,bg=color_bg,highlightthickness=0)
	Scale_lip1.place(x=x,y=y)							#Define a Scale and put it in position

	Scale_lip2 = tk.Scale(root,label=None,
	from_=0,to=480,orient=tk.HORIZONTAL,length=w,
	showvalue=1,tickinterval=None,resolution=1,variable=var_lip2,troughcolor='#212121',command=lip2_send,fg=color_text,bg=color_bg,highlightthickness=0)
	Scale_lip2.place(x=x,y=y+30)						#Define a Scale and put it in position

	Scale_err = tk.Scale(root,label=None,
	from_=0,to=200,orient=tk.HORIZONTAL,length=w,
	showvalue=1,tickinterval=None,resolution=1,variable=var_err,troughcolor='#212121',command=err_send,fg=color_text,bg=color_bg,highlightthickness=0)
	Scale_err.place(x=x,y=y+60)							#Define a Scale and put it in position

	canvas_cover=tk.Canvas(root,bg=color_bg,height=30,width=510,highlightthickness=0)
	canvas_cover.place(x=x,y=y+90)



	Btn_CVFL = tk.Button(root, width=23, text='CV FL',fg=color_text,bg='#212121',relief='ridge')
	Btn_CVFL.place(x=x+w+21,y=y+20)
	Btn_CVFL.bind('<ButtonPress-1>', call_CVFL)

	Btn_WB = tk.Button(root, width=23, text='LineColorSwitch',fg=color_text,bg='#212121',relief='ridge')
	Btn_WB.place(x=x+w+21,y=y+60)
	Btn_WB.bind('<ButtonPress-1>', call_WB)


def loop():					  #GUI
	global tcpClicSock,root,E1,connect,l_ip_4,l_ip_5,color_btn,color_text,Btn14,CPU_TEP_lab,CPU_USE_lab,RAM_lab,BATTERY_lab,canvas_ultra,color_text,var_lip1,var_lip2,var_err,var_R,var_B,var_G,var_ec,Btn_Police,Btn_Steady,Btn_FindColor,Btn_WatchDog,Btn_Fun5,Btn_Fun6,Btn_Switch_1,Btn_Switch_2,Btn_Switch_3,Btn_Smooth,Btn_SmoothCam,color_bg   #1 The value of tcpClicSock changes in the function loop(),would also changes in global so the other functions could use it.
	while True:
		color_bg='#000000'		#Set background color
		color_text='#E1F5FE'	  #Set text color
		color_btn='#0277BD'	   #Set button color
		color_line='#01579B'	  #Set line color
		color_can='#212121'	   #Set canvas color
		color_oval='#2196F3'	  #Set oval color
		target_color='#FF6D00'

		root = tk.Tk()			#Define a window named root
		root.title('Adeept RaspClaws (schmiereck)')	  #Main window title
		root.geometry('565x680')  #1 Main window size, middle of the English letter x.
		root.config(bg=color_bg)  #Set the background color of root window
		try:
			logo =tk.PhotoImage(file = 'logo.png')		 #Define the picture of logo,but only supports '.png' and '.gif'
			l_logo=tk.Label(root,image = logo,bg=color_bg) #Set a label to show the logo picture
			l_logo.place(x=30,y=13)						#Place the Label in a right position
		except:
			pass

		CPU_TEP_lab=tk.Label(root,width=18,text='CPU Temp:',fg=color_text,bg='#212121')
		CPU_TEP_lab.place(x=400,y=15)						 #Define a Label and put it in position

		CPU_USE_lab=tk.Label(root,width=18,text='CPU Usage:',fg=color_text,bg='#212121')
		CPU_USE_lab.place(x=400,y=45)						 #Define a Label and put it in position

		RAM_lab=tk.Label(root,width=18,text='RAM Usage:',fg=color_text,bg='#212121')
		RAM_lab.place(x=400,y=75)						 #Define a Label and put it in position

		BATTERY_lab=tk.Label(root,width=18,text='Battery: N/A',fg=color_text,bg='#757575')
		BATTERY_lab.place(x=400,y=105)					 #Define a Label and put it in position


		l_ip_4=tk.Label(root,width=18,text='Disconnected',fg=color_text,bg='#F44336')
		l_ip_4.place(x=400,y=140)						 #Define a Label and put it in position

		l_ip_5=tk.Label(root,width=18,text='Use default IP',fg=color_text,bg=color_btn)
		l_ip_5.place(x=400,y=175)						 #Define a Label and put it in position

		E1 = tk.Entry(root,show=None,width=16,bg="#37474F",fg='#eceff1')
		E1.place(x=180,y=40)							 #Define a Entry and put it in position

		l_ip_3=tk.Label(root,width=10,text='IP Address:',fg=color_text,bg='#000000')
		l_ip_3.place(x=175,y=15)						 #Define a Label and put it in position


		################################
		#canvas_rec=canvas_ultra.create_rectangle(0,0,340,30,fill = '#FFFFFF',width=0)
		#canvas_text=canvas_ultra.create_text((90,11),text='Ultrasonic Output: 0.75m',fill=color_text)
		################################
		Btn_Switch_1 = tk.Button(root, width=8, text='Port 1',fg=color_text,bg=color_btn,relief='ridge')
		Btn_Switch_2 = tk.Button(root, width=8, text='Port 2',fg=color_text,bg=color_btn,relief='ridge')
		Btn_Switch_3 = tk.Button(root, width=8, text='Port 3',fg=color_text,bg=color_btn,relief='ridge')

		Btn_Switch_1.place(x=30,y=265)
		Btn_Switch_2.place(x=100,y=265)
		Btn_Switch_3.place(x=170,y=265)

		Btn_Switch_1.bind('<ButtonPress-1>', call_Switch_1)
		Btn_Switch_2.bind('<ButtonPress-1>', call_Switch_2)
		Btn_Switch_3.bind('<ButtonPress-1>', call_Switch_3)

		Btn0 = tk.Button(root, width=8, text='Forward',fg=color_text,bg=color_btn,relief='ridge')
		Btn1 = tk.Button(root, width=8, text='Backward',fg=color_text,bg=color_btn,relief='ridge')
		Btn2 = tk.Button(root, width=8, text='Left',fg=color_text,bg=color_btn,relief='ridge')
		Btn3 = tk.Button(root, width=8, text='Right',fg=color_text,bg=color_btn,relief='ridge')



		Btn0.place(x=100,y=195)
		Btn1.place(x=100,y=230)
		Btn2.place(x=30,y=230)
		Btn3.place(x=170,y=230)

		Btn0.bind('<ButtonPress-1>', call_forward)
		Btn0.bind('<ButtonRelease-1>', call_DS)
		Btn1.bind('<ButtonPress-1>', call_back)
		Btn1.bind('<ButtonRelease-1>', call_DS)
		Btn2.bind('<ButtonPress-1>', call_Left)
		Btn2.bind('<ButtonRelease-1>', call_TS)
		Btn3.bind('<ButtonPress-1>', call_Right)
		Btn3.bind('<ButtonRelease-1>', call_TS)

		Btn0.bind('<ButtonRelease-1>', call_FB_stop)
		Btn1.bind('<ButtonRelease-1>', call_FB_stop)
		Btn2.bind('<ButtonRelease-1>', call_Turn_stop)
		Btn3.bind('<ButtonRelease-1>', call_Turn_stop)

		root.bind('<KeyPress-w>', call_forward) 
		root.bind('<KeyRelease-w>', call_DS) 
		root.bind('<KeyPress-a>', call_Left)
		root.bind('<KeyRelease-a>', call_TS)
		root.bind('<KeyPress-d>', call_Right)
		root.bind('<KeyRelease-d>', call_TS)
		root.bind('<KeyPress-s>', call_back)
		root.bind('<KeyRelease-s>', call_DS)

		root.bind('<KeyPress-q>', call_LeftSide)
		root.bind('<KeyPress-e>', call_RightSide)
		root.bind('<KeyRelease-q>', call_Turn_stop)
		root.bind('<KeyRelease-e>', call_Turn_stop)

		root.bind('<KeyRelease-w>', call_FB_stop)
		root.bind('<KeyRelease-a>', call_Turn_stop)
		root.bind('<KeyRelease-d>', call_Turn_stop)
		root.bind('<KeyRelease-s>', call_FB_stop)

		Btn_up = tk.Button(root, width=8, text='Up',fg=color_text,bg=color_btn,relief='ridge')
		Btn_down = tk.Button(root, width=8, text='Down',fg=color_text,bg=color_btn,relief='ridge')
		Btn_left = tk.Button(root, width=8, text='Left',fg=color_text,bg=color_btn,relief='ridge')
		Btn_right = tk.Button(root, width=8, text='Right',fg=color_text,bg=color_btn,relief='ridge')
		Btn_home = tk.Button(root, width=8, text='Home',fg=color_text,bg=color_btn,relief='ridge')
		Btn_up.place(x=400,y=195)
		Btn_down.place(x=400,y=265)
		Btn_left.place(x=330,y=230)
		Btn_right.place(x=470,y=230)
		Btn_home.place(x=400,y=230)
		root.bind('<KeyPress-i>', call_headup) 
		root.bind('<KeyRelease-i>', call_UDstop)
		root.bind('<KeyPress-k>', call_headdown)
		root.bind('<KeyRelease-k>', call_UDstop) 
		root.bind('<KeyPress-j>', call_headleft)
		root.bind('<KeyPress-l>', call_headright)
		root.bind('<KeyRelease-l>', call_LRstop)
		root.bind('<KeyRelease-j>', call_LRstop)
		Btn_up.bind('<ButtonPress-1>', call_headup)
		Btn_up.bind('<ButtonRelease-1>', call_UDstop)
		Btn_down.bind('<ButtonPress-1>', call_headdown)
		Btn_down.bind('<ButtonRelease-1>', call_UDstop)
		Btn_left.bind('<ButtonPress-1>', call_headleft)
		Btn_left.bind('<ButtonRelease-1>', call_LRstop)
		Btn_right.bind('<ButtonPress-1>', call_headright)
		Btn_right.bind('<ButtonRelease-1>', call_LRstop)
		Btn_home.bind('<ButtonPress-1>', call_headhome)

		Btn14= tk.Button(root, width=8,height=2, text='Connect',fg=color_text,bg=color_btn,command=connect_click,relief='ridge')
		Btn14.place(x=315,y=15)						  #Define a Button and put it in position

		root.bind('<Return>', connect)

		var_lip1 = tk.StringVar()#1
		var_lip1.set(440)
		var_lip2 = tk.StringVar()
		var_lip2.set(380)
		var_err = tk.StringVar()
		var_err.set(20)
  
		global canvas_show
		def R_send(event):
			canvas_show.config(bg = RGB_to_Hex(int(var_R.get()), int(var_G.get()), int(var_B.get())))
			time.sleep(0.03)


		def G_send(event):
			canvas_show.config(bg = RGB_to_Hex(int(var_R.get()), int(var_G.get()), int(var_B.get())))
			time.sleep(0.03)

		def B_send(event):
			canvas_show.config(bg = RGB_to_Hex(int(var_R.get()), int(var_G.get()), int(var_B.get())))
			time.sleep(0.03)  

		def call_SET(event):
			r = int(var_R.get())
			g = int(var_G.get())
			b = int(var_B.get())

			data_str = f"{r}, {g}, {b}"
			message = f"{{'title': 'findColorSet', 'data': [{data_str}]}}"
			tcpClicSock.send(message.encode())
		var_R = tk.StringVar()
		var_R.set(80)

		Scale_R = tk.Scale(root,label=None,
		from_=0,to=255,orient=tk.HORIZONTAL,length=238,
		showvalue=1,tickinterval=None,resolution=1,variable=var_R,troughcolor='#F44336',command=R_send,fg=color_text,bg=color_bg,highlightthickness=0)
		Scale_R.place(x=30,y=330)							#Define a Scale and put it in position

		var_G = tk.StringVar()
		var_G.set(80)

		Scale_G = tk.Scale(root,label=None,
		from_=0,to=255,orient=tk.HORIZONTAL,length=238,
		showvalue=1,tickinterval=None,resolution=1,variable=var_G,troughcolor='#00E676',command=G_send,fg=color_text,bg=color_bg,highlightthickness=0)
		Scale_G.place(x=30,y=360)							#Define a Scale and put it in position

		var_B = tk.StringVar()
		var_B.set(80)

		Scale_B = tk.Scale(root,label=None,
		from_=0,to=255,orient=tk.HORIZONTAL,length=238,
		showvalue=1,tickinterval=None,resolution=1,variable=var_B,troughcolor='#448AFF',command=B_send,fg=color_text,bg=color_bg,highlightthickness=0)
		Scale_B.place(x=30,y=390)							#Define a Scale and put it in position
  
		canvas_cover=tk.Canvas(root,bg=color_bg,height=30,width=510,highlightthickness=0)
		canvas_cover.place(x=30,y=330+90)
		canvas_show=tk.Canvas(root,bg=RGB_to_Hex(int(var_R.get()), int(var_G.get()), int(var_B.get())),height=35,width=170,highlightthickness=0)
		canvas_show.place(x=238+30+21,y=330+15)


		Btn_WB = tk.Button(root, width=23, text='Color Set',fg=color_text,bg='#212121',relief='ridge')
		Btn_WB.place(x=30+238+21,y=330+60)
		Btn_WB.bind('<ButtonPress-1>', call_SET)
  
		var_ec = tk.StringVar() #Z start
		var_ec.set(0)			


		Btn_Steady = tk.Button(root, width=10, text='Steady [Z]',fg=color_text,bg=color_btn,relief='ridge')
		Btn_Steady.place(x=30,y=445)
		root.bind('<KeyPress-z>', call_steady)
		Btn_Steady.bind('<ButtonPress-1>', call_steady)

		Btn_FindColor = tk.Button(root, width=10, text='FindColor [X]',fg=color_text,bg=color_btn,relief='ridge')
		Btn_FindColor.place(x=115,y=445)
		root.bind('<KeyPress-x>', call_FindColor)
		Btn_FindColor.bind('<ButtonPress-1>', call_FindColor)

		Btn_WatchDog = tk.Button(root, width=10, text='WatchDog [C]',fg=color_text,bg=color_btn,relief='ridge')
		Btn_WatchDog.place(x=200,y=445)
		root.bind('<KeyPress-c>', call_WatchDog)
		Btn_WatchDog.bind('<ButtonPress-1>', call_WatchDog)

		Btn_Smooth = tk.Button(root, width=10, text='Slow [V]',fg=color_text,bg=color_btn,relief='ridge')
		Btn_Smooth.place(x=285,y=445)
		root.bind('<KeyPress-v>', call_Smooth)
		Btn_Smooth.bind('<ButtonPress-1>', call_Smooth)

		Btn_Police = tk.Button(root, width=10, text='Police [B]',fg=color_text,bg=color_btn,relief='ridge')
		Btn_Police.place(x=370,y=445)
		root.bind('<KeyPress-b>', call_Police)
		Btn_Police.bind('<ButtonPress-1>', call_Police)

		# Second row - SmoothCam button
		Btn_SmoothCam = tk.Button(root, width=10, text='Smooth-Cam [N]',fg=color_text,bg=color_btn,relief='ridge')
		Btn_SmoothCam.place(x=455,y=445)
		root.bind('<KeyPress-n>', call_SmoothCam)
		Btn_SmoothCam.bind('<ButtonPress-1>', call_SmoothCam)

		scale_FL(30,490,315)#1

		global stat
		if stat==0:			  # Ensure the mainloop runs only once
			root.mainloop()  # Run the mainloop()
			stat=1		   # Change the value to '1' so the mainloop() would not run again.


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