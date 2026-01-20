#!/usr/bin/env/python
# File name   : server.py
# Description : main programe for RaspClaws
# Website     : www.adeept.com
# E-mail      : support@adeept.com
# Author      : William
# Date        : 2018/08/22

import socket
import time
import threading
import sys
import Move as move
import Adafruit_PCA9685
import argparse
import os
import FPV
import psutil
import Switch as switch
import RobotLight as robotLight
import ast
step_set = 1
speed_set = 100

new_frame = 0
direction_command = 'no'
turn_command = 'no'

# Try to initialize PCA9685, use mock mode if hardware not available
try:
	pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)  # Changed from 0x5F to 0x40
	pwm.set_pwm_freq(50)
	print("PCA9685 initialized in GUIServer on address 0x40")
except (OSError, IOError) as e:
	print(f"\033[38;5;3mWarning:\033[0m Could not initialize PCA9685 in GUIServer: {e}")
	print("Running in MOCK MODE - servo commands will be ignored")
	class MockPWM:
		def set_pwm(self, channel, on, off):
			pass
		def set_pwm_freq(self, freq):
			pass
	pwm = MockPWM()

rm = move.RobotM()
rm.start()
rm.pause()

SmoothMode = 0
steadyMode = 0

# Battery monitoring using ADS7830
battery_available = False
adc = None

# Try method 1: smbus (original method)
try:
	import smbus

	class ADS7830(object):
		def __init__(self):
			self.cmd = 0x84
			self.bus = smbus.SMBus(1)
			self.address = 0x48  # ADS7830 default i2c address

		def analogRead(self, chn):  # ADS7830 has 8 ADC input pins, chn:0-7
			value = self.bus.read_byte_data(self.address, self.cmd|(((chn<<2 | chn>>1)&0x07)<<4))
			return value

	# Try to initialize ADS7830
	adc = ADS7830()
	# Test read to verify it works
	test_value = adc.analogRead(0)
	battery_available = True
	print("✓ ADS7830 battery monitor initialized successfully (using smbus)")
	print(f"  Battery ADC test read: {test_value}")
except Exception as e:
	print(f"⚠ smbus method failed: {e}")
	# Try method 2: adafruit_bus_device (fallback, like in BatteryLevelMonitoring.py example)
	try:
		# Try to import adafruit libraries (may not be installed)
		try:
			import board
			import busio
			from adafruit_bus_device.i2c_device import I2CDevice  # type: ignore
		except ImportError as import_error:
			raise Exception(f"Adafruit libraries not installed: {import_error}")

		class ADS7830_Adafruit(object):
			def __init__(self):
				self.cmd = 0x84
				i2c = busio.I2C(board.SCL, board.SDA)
				self.device = I2CDevice(i2c, 0x48)  # ADS7830 address

			def analogRead(self, chn):
				control_byte = self.cmd | (((chn << 2 | chn >> 1) & 0x07) << 4)
				buffer = bytearray(1)
				self.device.write_then_readinto(bytes([control_byte]), buffer)
				return buffer[0]

		# Try to initialize ADS7830 with Adafruit library
		adc = ADS7830_Adafruit()
		# Test read to verify it works
		test_value = adc.analogRead(0)
		battery_available = True
		print("✓ ADS7830 battery monitor initialized successfully (using adafruit_bus_device)")
		print(f"  Battery ADC test read: {test_value}")
	except Exception as e2:
		print(f"⚠ adafruit_bus_device method also failed: {e2}")
		print("  Battery monitoring not available - no suitable I2C library found.")
		print("  This is normal if ADS7830 ADC hardware is not connected.")
		print("  Battery display will show 'N/A' in GUI.")
		battery_available = False

# Battery constants (from Voltage.py and BatteryLevelMonitoring.py)
ADCVref = 4.93  # Can be adjusted based on actual reference voltage
battery_channel = 0
R15 = 3000
R17 = 1000
DivisionRatio = R17 / (R15 + R17)


def  ap_thread():
    os.system("sudo create_ap wlan0 eth0 AdeeptCar 12345678")


def get_cpu_tempfunc():
    """ Return CPU temperature """
    result = 0
    mypath = "/sys/class/thermal/thermal_zone0/temp"
    with open(mypath, 'r') as mytmpfile:
        for line in mytmpfile:
            result = line

    result = float(result)/1000
    result = round(result, 1)
    return str(result)


def get_gpu_tempfunc():
    """ Return GPU temperature as a character string"""
    res = os.popen('/opt/vc/bin/vcgencmd measure_temp').readline()
    return res.replace("temp=", "")


def get_cpu_use():
    """ Return CPU usage using psutil"""
    cpu_cent = psutil.cpu_percent()
    return str(cpu_cent)


def get_ram_info():
    """ Return RAM usage using psutil """
    ram_cent = psutil.virtual_memory()[2]
    return str(ram_cent)


def get_swap_info():
    """ Return swap memory  usage using psutil """
    swap_cent = psutil.swap_memory()[3]
    return str(swap_cent)


def get_battery_voltage():
    """ Return battery voltage in Volts """
    global battery_available, adc

    if not battery_available or adc is None:
        return "0.0"  # Return 0.0 if battery monitoring not available

    try:
        ADCValue = adc.analogRead(battery_channel)
        A0Voltage = ADCValue / 255.0 * ADCVref
        actual_battery_voltage = A0Voltage / DivisionRatio
        return str(round(actual_battery_voltage, 2))
    except Exception as e:
        print(f"⚠ Error reading battery voltage: {e}")
        return "0.0"


def info_get():
    global cpu_t,cpu_u,gpu_t,ram_info
    while 1:
        cpu_t = get_cpu_tempfunc()
        cpu_u = get_cpu_use()
        ram_info = get_ram_info()
        time.sleep(3)




def info_send_client():
    # Use the existing tcpCliSock connection instead of creating a new one

    # Send VIDEO_READY signal multiple times at the start
    # This ensures the client receives it even if it's not ready immediately
    print("INFO_SEND_CLIENT: Starting to send VIDEO_READY signals...")
    sys.stdout.flush()
    success_count = 0
    for i in range(10):  # Try 10 times over 10 seconds (increased from 5)
        try:
            tcpCliSock.send('VIDEO_READY\n'.encode())
            success_count += 1
            print(f"✅ Sent VIDEO_READY signal (attempt {i+1}/10, success #{success_count})")
            sys.stdout.flush()
            time.sleep(1)
        except Exception as e:
            print(f"⚠ Failed to send VIDEO_READY (attempt {i+1}/10): {e}")
            sys.stdout.flush()
            # Don't break - continue trying!
            time.sleep(1)

    print(f"INFO_SEND_CLIENT: Finished VIDEO_READY phase ({success_count}/10 successful)")
    sys.stdout.flush()

    # Then continue with regular info sending
    while 1:
        try:
            battery_voltage = get_battery_voltage()
            servo_positions = move.get_servo_positions_info()

            # Get MPU6050 data (gyro/accelerometer)
            mpu_data = move.get_mpu6050_data()

            info_data = 'INFO:' + get_cpu_tempfunc() + ' ' + get_cpu_use() + ' ' + get_ram_info() + ' ' + battery_voltage + ' | ' + servo_positions + ' | ' + mpu_data + '\n'
            tcpCliSock.send(info_data.encode())
            time.sleep(0.2)  # 200ms update interval for better servo analysis
        except Exception as e:
            print(f"⚠ Failed to send INFO: {e}")
            sys.stdout.flush()
            break  # Exit thread on persistent error


def FPV_thread():
    global fpv
    print("Starting FPV thread...")
    fpv=FPV.FPV()
    # Note: IP is only used for logging in capture_thread, not for connection
    # The actual binding is to tcp://*:5555 (all interfaces)
    fpv.capture_thread("0.0.0.0")  # Dummy IP, actual binding is to all interfaces
    print("FPV thread ended")


# Global variable to track FPV thread
fps_threading = None


# ==================== Command Handlers ====================

def handle_movement_command(data):
	"""Handle movement commands (forward, backward, left, right, etc.)"""
	global direction_command, turn_command

	# Map GUI commands to Move.py commands
	command_mapping = {
		'forward': 'forward',
		'backward': 'backward',
		'DS': 'stand',
		'left': 'left',
		'right': 'right',
		'TS': 'no',
		'leftside': 'left',    # Strafe left -> turn left
		'rightside': 'right',  # Strafe right -> turn right
	}

	# Find matching command
	move_command = None
	if data in command_mapping:
		move_command = command_mapping[data]
	else:
		# Check for partial matches (e.g., 'DS' in data)
		for key, value in command_mapping.items():
			if key in data:
				move_command = value
				break

	if move_command:
		print(f"[GUIServer] Movement command: '{data}' -> '{move_command}'")
		# Use Move.py's handle_movement_command which properly resumes the robot thread
		result = move.handle_movement_command(move_command)
		if result:
			# Update local state variables for GUI sync
			if move_command in ['forward', 'backward', 'stand']:
				direction_command = move_command
			elif move_command in ['left', 'right', 'no']:
				turn_command = move_command
		return result
	return False  # Command not handled


def handle_camera_command(data):
	"""Handle camera movement commands (up, down, left, right, home)"""
	if data == 'up':
		move.look_up()
	elif data == 'down':
		move.look_down()
	elif data == 'home':
		move.home()
	elif data == 'lookleft':
		move.look_left()
	elif data == 'lookright':
		move.look_right()
	elif data == 'steadyCamera':
		move.commandInput(data)
		tcpCliSock.send('steadyCamera'.encode())
	elif data == 'steadyCameraOff':
		move.commandInput(data)
		tcpCliSock.send('steadyCameraOff'.encode())
	elif data == 'smoothCam':
		move.commandInput(data)
		tcpCliSock.send('smoothCam'.encode())
	elif data == 'smoothCamOff':
		move.commandInput(data)
		tcpCliSock.send('smoothCamOff'.encode())
	else:
		return False  # Command not handled
	return True  # Command was handled


# Note: handle_computer_vision_command removed - CV features (FindColor, WatchDog, LineFollow) not needed


def handle_speed_command(data):
	"""
	Handle speed control commands.

	Note: fast/slow modes removed - always uses smooth movement now.
	      This function kept for backwards compatibility but does nothing.
	      Speed adjustment will be implemented in the future via speed parameter.
	"""
	if data == 'fast' or data == 'slow':
		# Deprecated: Movement is always smooth now
		# Just acknowledge the command for backwards compatibility
		tcpCliSock.send(data.encode())
		return True
	return False  # Command not handled


def handle_led_command(data):
	"""Handle LED commands (police, policeOff)"""
	global ws2812

	if data == 'police':
		if ws2812:
			ws2812.police()
		tcpCliSock.send('police'.encode())
	elif data == 'policeOff':
		if ws2812:
			ws2812.breath(70, 70, 255)
		tcpCliSock.send('policeOff'.encode())
	else:
		return False  # Command not handled
	return True  # Command was handled


def handle_switch_command(data):
	"""Handle GPIO switch commands (Switch_1/2/3 on/off)"""
	if 'Switch_1_on' in data:
		switch.switch(1, 1)
		tcpCliSock.send('Switch_1_on'.encode())
	elif 'Switch_1_off' in data:
		switch.switch(1, 0)
		tcpCliSock.send('Switch_1_off'.encode())
	elif 'Switch_2_on' in data:
		switch.switch(2, 1)
		tcpCliSock.send('Switch_2_on'.encode())
	elif 'Switch_2_off' in data:
		switch.switch(2, 0)
		tcpCliSock.send('Switch_2_off'.encode())
	elif 'Switch_3_on' in data:
		switch.switch(3, 1)
		tcpCliSock.send('Switch_3_on'.encode())
	elif 'Switch_3_off' in data:
		switch.switch(3, 0)
		tcpCliSock.send('Switch_3_off'.encode())
	else:
		return False  # Command not handled
	return True  # Command was handled


# Note: handle_line_tracking_command removed - Line Following feature not needed


def process_client_command(data):
	"""Main command dispatcher - routes commands to appropriate handlers"""
	# Try each handler in order
	if handle_movement_command(data):
		return
	if handle_camera_command(data):
		return
	# Note: handle_computer_vision_command removed - CV features not needed
	if handle_speed_command(data):
		return
	if handle_led_command(data):
		return
	if handle_switch_command(data):
		return
	# Note: handle_line_tracking_command removed - Line Following not needed
	if handle_power_management_command(data):
		return

	# Command not recognized - this is OK, just ignore
	pass


# ==================== Power Management Commands ====================

def handle_power_management_command(data):
	"""Handle servo standby/wakeup and camera pause/resume commands"""
	global fps_threading

	if data == 'servo_standby':
		print("🔋 SERVO STANDBY - Stopping PWM signals")
		move.standby()  # Call standby in Move module
		return True

	elif data == 'servo_wakeup':
		print("⚡ SERVO WAKEUP - Restoring servo positions")
		move.wakeup()  # Call wakeup in Move module
		return True

	elif data == 'camera_pause':
		print("📷 CAMERA PAUSE - Stopping video stream")
		FPV.pause_stream()
		return True

	elif data == 'camera_resume':
		print("📷 CAMERA RESUME - Restarting video stream")
		FPV.resume_stream()
		return True

	return False


# ==================== Main Server Loop ====================

def run():
	global direction_command, turn_command, SmoothMode, steadyMode, ws2812
	info_threading=threading.Thread(target=info_send_client)   #Define a thread for communication
	info_threading.setDaemon(True)                             #'True' means it is a front thread,it would close when the mainloop() closes
	info_threading.start()                                     #Thread starts

	Y_pitch = 300
	Y_pitch_MAX = 600
	Y_pitch_MIN = 100

	while True:
		try:
			data = ''
			data = str(tcpCliSock.recv(BUFSIZ).decode())
			if not data:
				# Empty string means socket was closed by client
				print("Client disconnected (empty recv)")
				sys.stdout.flush()
				break  # Exit loop to wait for new connection

			# Process the command using the dispatcher
			process_client_command(data)

			print(data)
			time.sleep(0.01)  # 10ms sleep to reduce CPU load (100 updates/sec is sufficient)

		except (ConnectionResetError, BrokenPipeError, ConnectionAbortedError, OSError) as e:
			print(f"\n\033[38;5;3mWarning:\033[0m Client connection lost: {e}")
			print("Waiting for new client connection...")
			break  # Exit the loop to wait for a new connection
		except Exception as e:
			print(f"\n\033[38;5;1mError:\033[0m Unexpected error in main loop: {e}")
			import traceback
			traceback.print_exc()
			break  # Exit the loop on unexpected errors


def destory():
	move.clean_all()


# ==================== Initialization Functions ====================

def initialize_leds():
	"""Initialize WS2812 LED strip"""
	ws2812 = None
	try:
		print("Initializing WS2812 LEDs...")
		ws2812 = robotLight.Adeept_SPI_LedPixel(16, 255)
		if ws2812.check_spi_state() != 0:
			print("WS2812 initialized successfully")
			ws2812.start()
			ws2812.breath(70, 70, 255)
		else:
			print("\033[38;5;3mWarning:\033[0m SPI not available for WS2812 LEDs")
			ws2812.led_close()
			ws2812 = None
	except KeyboardInterrupt:
		if ws2812:
			ws2812.led_close()
		raise
	except Exception as e:
		print(f"\033[38;5;3mWarning:\033[0m Could not initialize WS2812 LEDs: {e}")
		ws2812 = None
	return ws2812


def start_video_thread():
	"""Start FPV video streaming thread"""
	global fps_threading
	print("Starting FPV video stream thread (runs continuously)...")
	fps_threading = threading.Thread(target=FPV_thread)
	fps_threading.setDaemon(True)
	fps_threading.start()
	print("FPV thread started. Clients can connect to video stream.")


def check_network_and_start_ap(ws2812):
	"""Check network connectivity and start Access Point if needed"""
	try:
		s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		s.connect(("1.1.1.1", 80))
		ipaddr_check = s.getsockname()[0]
		s.close()
		print(ipaddr_check)
	except:
		ap_threading = threading.Thread(target=ap_thread)
		ap_threading.setDaemon(True)
		ap_threading.start()

		if ws2812:
			# LED animation for AP mode
			for brightness in [50, 100, 150, 200, 255]:
				ws2812.set_all_led_color_data(0, 16, brightness)
				ws2812.show()
				time.sleep(1)
			ws2812.set_all_led_color_data(35, 255, 35)
			ws2812.show()


def setup_server_socket(addr):
	"""Create and configure server socket"""
	tcpSerSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	tcpSerSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	tcpSerSock.bind(addr)
	tcpSerSock.listen(5)
	return tcpSerSock


def set_led_connected_state(ws2812):
	"""Set LED to connected state (blue)"""
	try:
		if ws2812:
			ws2812.breath_status_set(0)
			ws2812.set_all_led_color_data(64, 128, 255)
			ws2812.show()
			print("WS2812 LEDs set to blue (connected state)")
		else:
			print("WS2812 LEDs not available - skipping LED setup")
	except Exception as e:
		print(f"\033[38;5;3mWarning:\033[0m Could not set WS2812 LED color: {e}")


def cleanup_after_disconnect(tcpCliSock, tcpSerSock, ws2812):
	"""Clean up sockets and LEDs after client disconnect"""
	print("\n" + "="*50)
	print("Client disconnected. Cleaning up...")
	print("="*50)

	# Close sockets
	try:
		tcpCliSock.close()
		print("✓ Client socket closed")
	except Exception as e:
		print(f"! Error closing client socket: {e}")

	try:
		tcpSerSock.close()
		print("✓ Server socket closed")
	except Exception as e:
		print(f"! Error closing server socket: {e}")

	# Reset LED color to indicate waiting state
	try:
		if ws2812:
			ws2812.set_all_led_color_data(70, 70, 255)
			ws2812.breath_status_set(1)
			ws2812.show()
			print("✓ LED reset to waiting state (breathing)")
	except Exception as e:
		print(f"! Error resetting LED: {e}")

	# Give some time for cleanup and port release
	print("\nWaiting 2 seconds before accepting new connection...")
	time.sleep(2)

	print("\n" + "="*50)
	print("Ready for new connection...")
	print("="*50 + "\n")


# ==================== Main Entry Point ====================

if __name__ == '__main__':
	switch.switchSetup()
	switch.set_all_switch_off()
	move.init_all()

	HOST = ''
	PORT = 10223
	BUFSIZ = 1024
	ADDR = (HOST, PORT)

	# Initialize WS2812 LEDs
	ws2812 = initialize_leds()

	# Clean up old video ready marker
	try:
		if os.path.exists('/tmp/video_ready'):
			os.remove('/tmp/video_ready')
			print("✓ Cleaned up old video ready marker")
	except:
		pass

	# Start FPV thread ONCE at server startup
	start_video_thread()

	# Main connection loop
	while 1:
		check_network_and_start_ap(ws2812)

		# Setup server socket and wait for client
		try:
			tcpSerSock = setup_server_socket(ADDR)
			print('waiting for connection...')
			tcpCliSock, addr = tcpSerSock.accept()
			print('...connected from :', addr)
		except Exception as e:
			print(f"Error setting up connection: {e}")
			time.sleep(1)
			continue

		# Set LED to connected state
		set_led_connected_state(ws2812)

		# Run the command processing loop
		run()

		# Clean up after disconnect
		cleanup_after_disconnect(tcpCliSock, tcpSerSock, ws2812)

