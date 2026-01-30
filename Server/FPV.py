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

# Camera imports - make optional for systems without camera
try:
    import picamera2
    import libcamera
    from picamera2 import Picamera2, Preview
    from picamera2.encoders import MJPEGEncoder
    from picamera2.outputs import FileOutput
    CAMERA_AVAILABLE = True
except ImportError as e:
    print(f"⚠ Camera module not available: {e}")
    print("  (FPV.py will not work without camera)")
    CAMERA_AVAILABLE = False
    # Create dummy classes to prevent NameError
    class Picamera2:
        pass

import io
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

# Note: RPIservo import removed - not needed in FPV.py
# Servo control is handled by Move.py and GUIServer.py

# ROS2 integration - optional
try:
    from FPV_ROS2_simple import set_latest_frame
    ROS2_FRAME_SHARING_AVAILABLE = True
except ImportError:
    # ROS2 not available - that's OK, FPV.py works without it
    ROS2_FRAME_SHARING_AVAILABLE = False
    def set_latest_frame(frame):
        pass  # Dummy function to avoid None checks

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

# Initialize camera only if available
picam2 = None
if CAMERA_AVAILABLE:
    try:
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
        picam2.start()
        print("✓ Picamera2 initialized successfully")
    except Exception as e:
        print(f"\033[38;5;1mCamera initialization error:\033[0m\n{e}")
        CAMERA_AVAILABLE = False
        picam2 = None
else:
    print("⚠ Camera not available - FPV.py will not capture frames")
    print("\nPlease check whether the camera is connected well, and disable the \"legacy camera driver\" on raspi-config")


modeText = 'Select your mode, ARM or PT?'

# Note: Servo initialization removed from FPV.py
# Servos are now managed by Move.py and GUIServer.py
# This allows FPV.py to work in Lazy-Mode without initializing hardware

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
		try:
			footage_socket.bind('tcp://*:5555')  # Server binds, clients subscribe
		except zmq.error.ZMQError as e:
			if 'Address already in use' in str(e):
				print("\n" + "="*60)
				print("❌ ERROR: Video port 5555 already in use")
				print("="*60)
				print("Another FPV/GUIServer process is already using port 5555.")
				print("\nTo fix this, run:")
				print("  bash Server/stop_guiserver.sh")
				print("\nOr manually find and kill the process:")
				print("  sudo lsof -i :5555")
				print("  sudo kill -9 <PID>")
				print("="*60 + "\n")
			raise  # Re-raise to stop the thread

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

		# Check if camera is available
		if not CAMERA_AVAILABLE or picam2 is None:
			print("❌ ERROR: Camera not available - cannot start capture loop")
			print("   FPV.py needs camera hardware to work")
			return

		while True:
			frame_image = picam2.capture_array()
			timestamp = datetime.datetime.now()
			cv2.line(frame_image, (300, 240), (340, 240), (128, 255, 128), 1)
			cv2.line(frame_image, (320, 220), (320, 260), (128, 255, 128), 1)

			# Share frame with ROS2 (if available, already imported at top of file)
			if ROS2_FRAME_SHARING_AVAILABLE:
				set_latest_frame(frame_image)

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
