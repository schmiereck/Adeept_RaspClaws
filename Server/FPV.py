#!/usr/bin/env/python
# File name   : FPV.py
# Website     : www.Adeept.com
# Author      : Adeept
# Date		  : 2025/04/16

import time
import threading
import cv2
import zmq
import base64
import picamera2
import libcamera
from picamera2 import Picamera2, Preview
import io
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput
import argparse
import imutils
from collections import deque
import psutil
import os
import PID
import Kalman_Filter as Kalman_filter
import datetime
import Move as move
import Switch as switch
import numpy as np
import RPIservo

# ==================== Power Management ====================
# Global flag for camera stream pause/resume
camera_paused = False


def pause_stream():
	"""Pause the camera video stream to save power"""
	global camera_paused
	camera_paused = True
	print("📷 Camera stream PAUSED - saving power")


def resume_stream():
	"""Resume the camera video stream"""
	global camera_paused
	camera_paused = False
	print("📷 Camera stream RESUMED")


# ==================== End Power Management ====================

pid = PID.PID()
pid.SetKp(0.5)
pid.SetKd(0)
pid.SetKi(0)
Y_lock = 0
X_lock = 0
tor	= 17

# Note: FindColor, WatchDog, LineFollow features removed - not needed

hflip = 0  # Video flip horizontally: 0 or 1
vflip = 0  # Video vertical flip: 0/1
picam2 = Picamera2()
preview_config = picam2.preview_configuration
preview_config.size = (640, 480)
preview_config.format = 'RGB888'  # 'XRGB8888', 'XBGR8888', 'RGB888', 'BGR888', 'YUV420'
preview_config.transform = libcamera.Transform(hflip=hflip, vflip=vflip)
preview_config.colour_space = libcamera.ColorSpace.Sycc()
preview_config.buffer_count = 4
preview_config.queue = True

if not picam2.is_open:
    raise RuntimeError('Could not start camera.')
try:
    picam2.start()
except Exception as e:
    print(f"\033[38;5;1mError:\033[0m\n{e}")
    print("\nPlease check whether the camera is connected well, and disable the \"legacy camera driver\" on raspi-config")


modeText = 'Select your mode, ARM or PT?'

scGear = RPIservo.ServoCtrl()
scGear.moveInit()
scGear.start()

Dv = -1 #Directional variable
CVRun = 1
turn_speed = 15

# Note: tracking_servo, map, findLineCtrl, and cvFindLine functions removed - not needed


class FPV: 
	kalman_filter_X =  Kalman_filter.Kalman_filter(0.01,0.1)
	kalman_filter_Y =  Kalman_filter.Kalman_filter(0.01,0.1)
	P_direction = -1
	T_direction = -1
	P_servo = 12 
	T_servo = 13 
	P_anglePos = 0
	T_anglePos = 0
	cameraDiagonalW = 64
	cameraDiagonalH = 48
	videoW = 640
	videoH = 480
	Y_lock = 0
	X_lock = 0
	tor = 17
	def __init__(self):
		self.frame_num = 0
		self.fps = 0

	def SetIP(self,invar):
		self.IP = invar

	# Note: FindColor, WatchDog, FindLineMode, colorFindSet, servoMove, changeMode methods removed - not needed

	def capture_thread(self,IPinver):
		global frame_image, camera#Z
		ap = argparse.ArgumentParser()			#OpenCV initialization
		ap.add_argument("-b", "--buffer", type=int, default=64,
			help="max buffer size")
		args = vars(ap.parse_args())
		pts = deque(maxlen=args["buffer"])

		font = cv2.FONT_HERSHEY_SIMPLEX

		context = zmq.Context()
		footage_socket = context.socket(zmq.PUB)  # Changed from PAIR to PUB

		# Set high-water mark to 1 to prevent buffering old frames
		# This ensures clients always get the latest frame, not buffered old ones
		footage_socket.setsockopt(zmq.SNDHWM, 1)

		print(f"Video server binding to port 5555 (PUB socket, allows multiple clients)")
		footage_socket.bind('tcp://*:5555')  # Server binds, clients subscribe

		# Give ZMQ time to establish the socket
		time.sleep(0.5)
		print("✅ Video server ready for client connections")

		# Write ready marker file for GUIServer
		try:
			with open('/tmp/video_ready', 'w') as f:
				f.write('1')
			print("✓ Video ready marker written")
		except Exception as e:
			print(f"⚠ Could not write video ready marker: {e}")

		while True:
			frame_image = picam2.capture_array()
			timestamp = datetime.datetime.now()
			cv2.line(frame_image, (300, 240), (340, 240), (128, 255, 128), 1)
			cv2.line(frame_image, (320, 220), (320, 260), (128, 255, 128), 1)

			# Share frame with ROS2 (if available, optional)
			try:
				from FPV_ROS2_simple import set_latest_frame
				set_latest_frame(frame_image)
			except:
				pass  # ROS2 not available or not needed, continue normal operation

			# Check if camera is paused - skip frame sending if paused
			if not camera_paused:
				# Frame encoding and sending (only when NOT paused)
				encoded, buffer = cv2.imencode('.jpg', frame_image)
				jpg_as_text = base64.b64encode(buffer)
				footage_socket.send(jpg_as_text)

			# Limit frame rate to reduce CPU load (~30 FPS = 33ms per frame)
			time.sleep(0.033)  # 33ms = ~30 FPS, reduces CPU load significantly


if __name__ == '__main__':
	fpv=FPV()
	while 1:
		fpv.capture_thread('192.168.3.199')
		pass
