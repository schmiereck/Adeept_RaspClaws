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
import sys

print("[FPV] Starting FPV.py imports...")

# Camera imports - make optional for systems without camera
try:
    print("[FPV] Attempting to import picamera2 and libcamera...")
    import picamera2
    import libcamera
    from picamera2 import Picamera2, Preview
    from picamera2.encoders import MJPEGEncoder
    from picamera2.outputs import FileOutput
    CAMERA_AVAILABLE = True
    print("[FPV] ✓ picamera2 and libcamera imported successfully.")
except ImportError as e:
    print(f"[FPV] ⚠ Camera module not available: {e}")
    print("[FPV]   (FPV.py will not work without camera)")
    CAMERA_AVAILABLE = False
    # Create dummy classes to prevent NameError
    class Picamera2:
        pass
    print("[FPV] ⚠ Falling back to dummy Picamera2 class.")

print("[FPV] Continuing FPV.py imports...")
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

print("[FPV] Finished FPV.py imports.")

# Note: RPIservo import removed - not needed in FPV.py
# Servo control is handled by Move.py and GUIServer.py

# ROS2 integration - optional
try:
    print("[FPV] Attempting to import FPV_ROS2_simple...")
    from FPV_ROS2_simple import set_latest_frame
    ROS2_FRAME_SHARING_AVAILABLE = True
    print("[FPV] ✓ FPV_ROS2_simple imported successfully.")
except ImportError:
    print("[FPV] ⚠ FPV_ROS2_simple not available - FPV.py works without it.")
    ROS2_FRAME_SHARING_AVAILABLE = False
    def set_latest_frame(frame):
        pass  # Dummy function to avoid None checks

print("[FPV] Initializing global variables and power management...")
# ==================== Power Management ====================
# Global flag for camera stream pause/resume
camera_paused = False


def pause_stream():
	"""Pause the camera video stream to save power"""
	global camera_paused
	camera_paused = True
	print("[FPV.pause_stream] 📷 Camera stream PAUSED - saving power")


def resume_stream():
	"""Resume the camera video stream"""
	global camera_paused
	camera_paused = False
	print("[FPV.resume_stream] 📷 Camera stream RESUMED")


# ==================== End Power Management ====================

print("[FPV] Initializing PID for FPV...")
pid = PID.PID()
pid.SetKp(0.5)
pid.SetKd(0)
pid.SetKi(0)
Y_lock = 0
X_lock = 0
tor	= 17
print("[FPV] PID for FPV initialized.")

# Note: FindColor, WatchDog, LineFollow features removed - not needed

hflip = 0  # Video flip horizontally: 0 or 1
vflip = 0  # Video vertical flip: 0/1

print("[FPV] Camera initialization deferred until capture_thread starts", flush=True)
print("[FPV] This avoids blocking server startup if camera hangs (Debian Trixie bug)", flush=True)
# Don't initialize camera at module level - defer until capture_thread()
# This allows GUIServer to start even if camera hangs
picam2 = None

modeText = 'Select your mode, ARM or PT?'

# Note: Servo initialization removed from FPV.py
# Servos are now managed by Move.py and GUIServer.py
# This allows FPV.py to work in Lazy-Mode without initializing hardware

Dv = -1 #Directional variable
CVRun = 1
turn_speed = 15

# Note: tracking_servo, map, findLineCtrl, and cvFindLine functions removed - not needed

print("[FPV] Initializing FPV class...")
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
		print("[FPV.__init__] FPV instance created.")
		self.frame_num = 0
		self.fps = 0

	def SetIP(self,invar):
		self.IP = invar

	# Note: FindColor, WatchDog, FindLineMode, colorFindSet, servoMove, changeMode methods removed - not needed

	def capture_thread(self,IPinver):
		print(f"[FPV.capture_thread] Starting capture thread for IP {IPinver}...")
		global frame_image, camera#Z
		ap = argparse.ArgumentParser()			#OpenCV initialization
		ap.add_argument("-b", "--buffer", type=int, default=64,
			help="max buffer size")
		args = vars(ap.parse_args())
		pts = deque(maxlen=args["buffer"])

		font = cv2.FONT_HERSHEY_SIMPLEX

		print("[FPV.capture_thread] Initializing ZMQ context and socket...")
		context = zmq.Context()
		footage_socket = context.socket(zmq.PUB)  # Changed from PAIR to PUB

		# Set high-water mark to 1 to prevent buffering old frames
		# This ensures clients always get the latest frame, not buffered old ones
		footage_socket.setsockopt(zmq.SNDHWM, 1)

		print(f"[FPV.capture_thread] Video server binding to port 5555 (PUB socket)...")
		try:
			footage_socket.bind('tcp://*:5555')  # Server binds, clients subscribe
			print("[FPV.capture_thread] ✅ Video server bound successfully.")
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
		print("[FPV.capture_thread] ✅ Video server ready for client connections")

		# Write ready marker file for GUIServer
		try:
			with open('/tmp/video_ready', 'w') as f:
				f.write('1')
			print("[FPV.capture_thread] ✓ Video ready marker written", flush=True)
		except Exception as e:
			print(f"[FPV.capture_thread] ⚠ Could not write video ready marker: {e}", flush=True)

		# Initialize camera with timeout workaround for Debian Trixie bug
		global picam2, CAMERA_AVAILABLE
		if not CAMERA_AVAILABLE:
			print("[FPV.capture_thread] ❌ Camera module not available - cannot start", flush=True)
			return

		print("[FPV.capture_thread] Initializing camera...", flush=True)
		try:
			print("[FPV.capture_thread]   Creating Picamera2 object...", flush=True)
			picam2 = Picamera2()
			print("[FPV.capture_thread]   Configuring camera...", flush=True)
			# OPTIMIZATION: Set resolution to 320x240 directly at source
			# This avoids resizing downstream and saves massive CPU/Bandwidth
			preview_config = picam2.create_preview_configuration(
				main={"size": (320, 240), "format": "RGB888"},
				transform=libcamera.Transform(hflip=hflip, vflip=vflip),
				buffer_count=4,
				queue=True
			)
			picam2.configure(preview_config)
			print("[FPV.capture_thread]   Starting camera with 5-second timeout...", flush=True)
			sys.stdout.flush()  # Force flush before potentially blocking call

			# Timeout workaround for Debian Trixie bug
			start_success = [False]
			start_error = [None]

			def start_camera():
				try:
					picam2.start()
					start_success[0] = True
				except Exception as e:
					start_error[0] = e

			start_thread = threading.Thread(target=start_camera, daemon=True)
			start_thread.start()
			start_thread.join(timeout=5.0)

			if start_thread.is_alive():
				print("[FPV.capture_thread] ❌ TIMEOUT: Camera start hung (Debian Trixie bug)", flush=True)
				print("[FPV.capture_thread]    picam2.start() blocked for >5 seconds", flush=True)
				print("[FPV.capture_thread]    This is a known Debian Trixie issue after reboot", flush=True)
				return
			elif start_error[0]:
				raise start_error[0]
			elif start_success[0]:
				print("[FPV.capture_thread] ✓ Camera started successfully", flush=True)
		except Exception as e:
			print(f"[FPV.capture_thread] ❌ Camera init error: {e}", flush=True)
			return

		print("[FPV.capture_thread] Starting video capture loop...")
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
	print("[FPV] Running FPV.py directly for testing.")
	fpv=FPV()
	while 1:
		print("[FPV] Calling capture_thread from main...")
		fpv.capture_thread('192.168.3.199')
		print("[FPV] capture_thread returned. Looping...")
		pass
