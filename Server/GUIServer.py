import socket
import time
import threading
import sys
import signal
import Move as move
import argparse
import os
import FPV
import psutil
import Switch as switch
import RobotLight as robotLight
import ast
# Add parent directory to path to import protocol module
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from protocol import *
step_set = 1

# ==================== Shutdown Handler ====================

shutdown_requested = False

def signal_handler(sig, frame):
        """Handle Ctrl+C (SIGINT) and other termination signals"""
        global shutdown_requested

        if shutdown_requested:
                print("\n\n⚠️  Force shutdown - killing immediately\n")
                # Use os._exit() instead of sys.exit() to avoid traceback
                os._exit(1)

        shutdown_requested = True
        print("\n\n" + "="*60)
        print("🛑 Shutdown signal received (Ctrl+C)")
        print("="*60)
        print("Shutting down gracefully...")
        print("(Press Ctrl+C again to force immediate shutdown)")
        print("="*60 + "\n")

        # Close sockets if they exist
        try:
                global tcpCliSock, tcpSerSock
                if 'tcpCliSock' in globals() and tcpCliSock:
                        tcpCliSock.close()
                        print("✓ Client socket closed")
        except:
                pass

        try:
                if 'tcpSerSock' in globals() and tcpSerSock:
                        tcpSerSock.close()
                        print("✓ Server socket closed")
        except:
                pass

        # Stop video thread
        print("✓ Video thread will stop automatically (daemon)")

        # Clean up LEDs
        try:
                global ws2812
                if 'ws2812' in globals() and ws2812:
                        ws2812.led_close()
                        print("✓ LEDs turned off")
        except:
                pass

        # Clean up Move module
        try:
                move.clean_all()
                print("✓ Move module cleaned up")
        except:
                pass

        print("\n" + "="*60)
        print("✅ GUIServer shutdown complete")
        print("="*60 + "\n")

        # Use os._exit() instead of sys.exit() to avoid traceback
        os._exit(0)


def sigtstp_handler(sig, frame):
        """Handle Ctrl+Z (SIGTSTP) - warn user and perform cleanup instead of suspending"""
        print("\n\n" + "="*60)
        print("⚠️  WARNING: Ctrl+Z detected!")
        print("="*60)
        print("Ctrl+Z suspends the process and keeps ports blocked!")
        print("")
        print("❌ DO NOT USE Ctrl+Z")
        print("✅ USE Ctrl+C instead")
        print("")
        print("I will now perform a clean shutdown for you...")
        print("="*60 + "\n")

        # Call the normal signal handler to do cleanup
        signal_handler(sig, frame)


# Register signal handlers
signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C
signal.signal(signal.SIGTERM, signal_handler)  # kill command

# Intercept Ctrl+Z and redirect to clean shutdown
try:
        signal.signal(signal.SIGTSTP, sigtstp_handler)  # Ctrl+Z
        print("✓ Ctrl+Z handler registered (will perform clean shutdown instead of suspend)")
except AttributeError:
        # SIGTSTP might not be available on some systems (e.g., Windows)
        print("⚠️  SIGTSTP (Ctrl+Z) handler not available on this system")

# ==================== End Shutdown Handler ====================


new_frame = 0
direction_command = 'no'
turn_command = 'no'

# PCA9685 is now initialized via move.init_all() which calls RPIservo.initialize_pwm()
# We don't need a local pwm object here anymore.

rm = move.RobotM()
rm.start()
rm.pause()

SmoothMode = 0
steadyMode = 0
SmoothCamMode = 1 # Enable SmoothCam by default


# Power Management Status
servo_standby_active = False
camera_paused_active = False

# Battery monitoring using ADS7830
battery_available = False
adc = None

print("[GUIServer] Attempting to initialize ADS7830 battery monitor...")

# Try method 1: smbus (original method)
try:
        print("[GUIServer]   Attempting to import smbus...")
        import smbus
        print("[GUIServer]   smbus imported successfully.")

        class ADS7830(object):
                def __init__(self):
                        self.cmd = 0x84
                        self.bus = smbus.SMBus(1)
                        self.address = 0x48  # ADS7830 default i2c address

                def analogRead(self, chn):  # ADS7830 has 8 ADC input pins, chn:0-7
                        value = self.bus.read_byte_data(self.address, self.cmd|(((chn<<2 | chn>>1)&0x07)<<4))
                        return value

        print("[GUIServer]   Attempting to initialize ADS7830 with smbus...")
        adc = ADS7830()
        # Test read to verify it works
        test_value = adc.analogRead(0)
        battery_available = True
        print("✓ [GUIServer] ADS7830 battery monitor initialized successfully (using smbus)")
        print(f"  [GUIServer] Battery ADC test read: {test_value}")
except Exception as e:
        print(f"⚠ [GUIServer] smbus method failed: {e}")
        # Try method 2: adafruit_bus_device (fallback, like in BatteryLevelMonitoring.py example)
        print("[GUIServer]   Attempting to initialize ADS7830 with adafruit_bus_device...")
        try:
                # Try to import adafruit libraries (may not be installed)
                try:
                        print("[GUIServer]     Attempting to import board, busio, I2CDevice...")
                        import board
                        import busio
                        from adafruit_bus_device.i2c_device import I2CDevice  # type: ignore
                        print("[GUIServer]     board, busio, I2CDevice imported successfully.")
                except ImportError as import_error:
                        print(f"⚠ [GUIServer] Adafruit libraries not installed for ADS7830 fallback: {import_error}")
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

                print("[GUIServer]   Attempting to initialize ADS7830 with Adafruit library...")
                adc = ADS7830_Adafruit()
                # Test read to verify it works
                test_value = adc.analogRead(0)
                battery_available = True
                print("✓ [GUIServer] ADS7830 battery monitor initialized successfully (using adafruit_bus_device)")
                print(f"  [GUIServer] Battery ADC test read: {test_value}")
        except Exception as e2:
                print(f"⚠ [GUIServer] adafruit_bus_device method also failed: {e2}")
                print("  [GUIServer] Battery monitoring not available - no suitable I2C library found.")
                print("  [GUIServer] This is normal if ADS7830 ADC hardware is not connected.")
                print("  [GUIServer] Battery display will show 'N/A' in GUI.")
                battery_available = False

print("[GUIServer] Finished ADS7830 battery monitor initialization attempt.")

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

    # Send initial status immediately (don't wait for VIDEO_READY to finish!)
    print("INFO_SEND_CLIENT: Sending initial status...")
    sys.stdout.flush()

    try:
        # Send Power Management status first
        if servo_standby_active:
            tcpCliSock.send(f'{STATUS_SERVO_STANDBY}\n'.encode())
            print("✓ Sent servo standby status: ACTIVE")
        else:
            tcpCliSock.send(f'{STATUS_SERVO_WAKEUP}\n'.encode())
            print("✓ Sent servo standby status: INACTIVE")

        if camera_paused_active:
            tcpCliSock.send(f'{STATUS_CAMERA_PAUSED}\n'.encode())
            print("✓ Sent camera pause status: PAUSED")
        else:
            tcpCliSock.send(f'{STATUS_CAMERA_RESUMED}\n'.encode())
            print("✓ Sent camera pause status: ACTIVE")

        if SmoothCamMode:
            tcpCliSock.send(f'{CMD_SMOOTH_CAM}\n'.encode())
            print("✓ Sent SmoothCam status: ACTIVE")
        else:
            tcpCliSock.send(f'{CMD_SMOOTH_CAM_OFF}\n'.encode())
            print("✓ Sent SmoothCam status: INACTIVE")

        # Send first INFO message immediately
        battery_voltage = get_battery_voltage()
        servo_positions = move.get_servo_positions_info()
        mpu_data = move.get_mpu6050_data()
        info_data = STATUS_INFO_PREFIX + get_cpu_tempfunc() + ' ' + get_cpu_use() + ' ' + get_ram_info() + ' ' + battery_voltage + ' | ' + servo_positions + ' | ' + mpu_data + '\n'
        tcpCliSock.send(info_data.encode())
        print("✓ Sent initial INFO data")

    except Exception as e:
        # Silently exit if socket is closed (client disconnected)
        if "Bad file descriptor" not in str(e):
            print(f"⚠ Failed to send initial status: {e}")
        return

    sys.stdout.flush()

    # Wait for client to start connection_thread (avoid race condition)
    print("INFO_SEND_CLIENT: Waiting 2s for client to initialize...")
    sys.stdout.flush()
    time.sleep(2.0)

    # Send VIDEO_READY signals (10 times with 1s delays = 10s window)
    print("INFO_SEND_CLIENT: Starting VIDEO_READY signals...")
    sys.stdout.flush()
    success_count = 0
    for i in range(10):
        try:
            tcpCliSock.send(f'{STATUS_VIDEO_READY}\n'.encode())
            success_count += 1
            print(f"✅ VIDEO_READY {i+1}/10")
            sys.stdout.flush()
            time.sleep(1.0)
        except Exception as e:
            # Silently exit if socket is closed (client disconnected)
            if "Bad file descriptor" in str(e) or "Broken pipe" in str(e):
                print(f"INFO_SEND_CLIENT: Client disconnected during VIDEO_READY phase")
                sys.stdout.flush()
                return
            print(f"⚠ Failed to send VIDEO_READY {i+1}/10: {e}")
            sys.stdout.flush()
            time.sleep(1.0)

    print(f"INFO_SEND_CLIENT: Finished VIDEO_READY phase ({success_count}/10 successful)")
    sys.stdout.flush()

    # Then continue with regular info sending
    while 1:
        try:
            battery_voltage = get_battery_voltage()
            servo_positions = move.get_servo_positions_info()

            # Get MPU6050 data (gyro/accelerometer)
            mpu_data = move.get_mpu6050_data()

            info_data = STATUS_INFO_PREFIX + get_cpu_tempfunc() + ' ' + get_cpu_use() + ' ' + get_ram_info() + ' ' + battery_voltage + ' | ' + servo_positions + ' | ' + mpu_data + '\n'
            tcpCliSock.send(info_data.encode())
            time.sleep(0.2)  # 200ms update interval for better servo analysis
        except Exception as e:
            # Silently exit if socket is closed (client disconnected)
            if "Bad file descriptor" not in str(e) and "Broken pipe" not in str(e):
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

        # EXACT match only - no partial matching to avoid conflicts with lookleft/lookright
        move_command = GUI_TO_MOVE_COMMAND_MAP.get(data)

        if move_command:
                print(f"[GUIServer] Movement command: '{data}' -> '{move_command}'")
                # Use Move.py's handle_movement_command which properly resumes the robot thread
                result = move.handle_movement_command(move_command)
                if result:
                        # Update local state variables for GUI sync
                        if move_command in [CMD_FORWARD, CMD_BACKWARD, MOVE_STAND]:
                                direction_command = move_command
                        elif move_command in [CMD_LEFT, CMD_RIGHT, MOVE_NO]:
                                turn_command = move_command
                return result
        return False  # Command not handled


def handle_camera_command(data):
        """Handle camera movement commands (lookUp, lookDown, lookLeft, lookRight, home)"""
        if data == CMD_LOOK_UP:
                print(f"[GUIServer] Camera command: lookUp")
                move.look_up()
        elif data == CMD_LOOK_DOWN:
                print(f"[GUIServer] Camera command: lookDown")
                move.look_down()
        elif data == CMD_LOOK_HOME:
                print(f"[GUIServer] Camera command: lookHome - calling move.look_home()")
                move.look_home()
                print(f"[GUIServer] move.look_home() completed")
        elif data == CMD_LOOK_LEFT:
                print(f"[GUIServer] Camera command: lookLeft")
                move.look_left()
        elif data == CMD_LOOK_RIGHT:
                print(f"[GUIServer] Camera command: lookRight")
                move.look_right()
        elif data == CMD_LR_STOP:
                pass  # Camera servos don't need explicit stop (they move to position and stay)
        elif data == CMD_UD_STOP:
                pass  # Camera servos don't need explicit stop (they move to position and stay)
        elif data == CMD_STEADY_CAMERA:
                move.commandInput(data)
                tcpCliSock.send(CMD_STEADY_CAMERA.encode())
        elif data == CMD_STEADY_CAMERA_OFF:
                move.commandInput(data)
                tcpCliSock.send(CMD_STEADY_CAMERA_OFF.encode())
        elif data == CMD_SMOOTH_CAM:
                move.commandInput(data)
                tcpCliSock.send(CMD_SMOOTH_CAM.encode())
        elif data == CMD_SMOOTH_CAM_OFF:
                move.commandInput(data)
                tcpCliSock.send(CMD_SMOOTH_CAM_OFF.encode())
        else:
                return False  # Command not handled
        return True  # Command was handled


# Note: handle_computer_vision_command removed - CV features (FindColor, WatchDog, LineFollow) not needed


def handle_speed_command(data):
        """
        Handle speed control commands.

        Supports:
        - CMD_SET_SPEED: Set custom movement speed (format: "setSpeed:35")
        - CMD_FAST/CMD_SLOW: Legacy commands (deprecated, kept for backwards compatibility)
        """
        if data.startswith(CMD_SET_SPEED):
                # Extract speed value from "setSpeed:35"
                try:
                        speed_str = data[len(CMD_SET_SPEED):]
                        speed = int(speed_str)
                        # Clamping to valid range is handled in Move.py
                        print(f"[GUIServer] Setting movement speed to {speed}")
                        move.set_movement_speed(speed)
                        return True
                except ValueError:
                        print(f"[GUIServer] Invalid speed value: {data}")
                        return False
        return False  # Command not handled


def handle_arc_factor_command(data):
        """
        Handle arc factor control command.
        """
        if data.startswith(CMD_SET_ARC_FACTOR):
                try:
                        arc_factor_str = data[len(CMD_SET_ARC_FACTOR):]
                        # Handle potential concatenated commands or newlines by taking the first part
                        arc_factor_str = arc_factor_str.split()[0]
                        arc_factor = float(arc_factor_str)
                        print(f"[GUIServer] Setting arc factor to {arc_factor}")
                        move.set_arc_factor(arc_factor)
                        return True
                except (ValueError, IndexError):  # IndexError if split is empty
                        print(f"[GUIServer] Invalid arc factor value: {data}")
                        return False
        return False


def handle_led_command(data):
        """Handle LED commands (police, policeOff)"""
        global ws2812

        if data == CMD_POLICE:
                if ws2812:
                        ws2812.police()
                tcpCliSock.send(CMD_POLICE.encode())
        elif data == CMD_POLICE_OFF:
                if ws2812:
                        ws2812.breath(70, 70, 255)
                tcpCliSock.send(CMD_POLICE_OFF.encode())
        else:
                return False  # Command not handled
        return True  # Command was handled


def handle_switch_command(data):
        """Handle GPIO switch commands (Switch_1/2/3 on/off)"""
        if CMD_SWITCH_1_ON in data:
                switch.switch(1, 1)
                tcpCliSock.send(CMD_SWITCH_1_ON.encode())
        elif CMD_SWITCH_1_OFF in data:
                switch.switch(1, 0)
                tcpCliSock.send(CMD_SWITCH_1_OFF.encode())
        elif CMD_SWITCH_2_ON in data:
                switch.switch(2, 1)
                tcpCliSock.send(CMD_SWITCH_2_ON.encode())
        elif CMD_SWITCH_2_OFF in data:
                switch.switch(2, 0)
                tcpCliSock.send(CMD_SWITCH_2_OFF.encode())
        elif CMD_SWITCH_3_ON in data:
                switch.switch(3, 1)
                tcpCliSock.send(CMD_SWITCH_3_ON.encode())
        elif CMD_SWITCH_3_OFF in data:
                switch.switch(3, 0)
                tcpCliSock.send(CMD_SWITCH_3_OFF.encode())
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
        if handle_arc_factor_command(data):
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
        global fps_threading, servo_standby_active, camera_paused_active

        if data == CMD_SERVO_STANDBY:
                print("🔋 SERVO STANDBY - Stopping PWM signals")
                move.standby()  # Call standby in Move module
                servo_standby_active = True
                # Send status update to client
                tcpCliSock.send(f'{STATUS_SERVO_STANDBY}\n'.encode())
                return True

        elif data == CMD_SERVO_WAKEUP:
                print("⚡ SERVO WAKEUP - Restoring servo positions")
                move.wakeup()  # Call wakeup in Move module
                servo_standby_active = False
                # Send status update to client
                tcpCliSock.send(f'{STATUS_SERVO_WAKEUP}\n'.encode())
                return True

        elif data == CMD_CAMERA_PAUSE:
                print("📷 CAMERA PAUSE - Stopping video stream")
                FPV.pause_stream()
                camera_paused_active = True
                # Send status update to client
                tcpCliSock.send(f'{STATUS_CAMERA_PAUSED}\n'.encode())
                return True

        elif data == CMD_CAMERA_RESUME:
                print("📷 CAMERA RESUME - Restarting video stream")
                FPV.resume_stream()
                camera_paused_active = False
                # Send status update to client
                tcpCliSock.send(f'{STATUS_CAMERA_RESUMED}\n'.encode())
                return True

        return False


# ==================== Main Server Loop ====================

def run():
        global direction_command, turn_command, SmoothMode, steadyMode, ws2812
        info_threading=threading.Thread(target=info_send_client)   #Define a thread for communication
        info_threading.daemon = True                               #'True' means it is a front thread,it would close when the mainloop() closes
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
        print("[GUIServer] Initializing WS2812 LEDs...")
        try:
                print("[GUIServer] Attempting to create Adeept_SPI_LedPixel object...")
                ws2812 = robotLight.Adeept_SPI_LedPixel(16, 255)
                print("[GUIServer] Adeept_SPI_LedPixel object created. Checking SPI state...")
                if ws2812.check_spi_state() != 0:
                        print("[GUIServer] WS2812 initialized successfully. Starting LED animations...")
                        ws2812.start()
                        ws2812.breath(70, 70, 255)
                        print("[GUIServer] WS2812 LEDs started breathing animation.")
                else:
                        print("\033[38;5;3mWarning:[GUIServer]\033[0m SPI not available for WS2812 LEDs")
                        ws2812.led_close()
                        ws2812 = None
        except KeyboardInterrupt:
                if ws2812:
                        ws2812.led_close()
                raise
        except Exception as e:
                print(f"\033[38;5;3mWarning:[GUIServer]\033[0m Could not initialize WS2812 LEDs: {e}")
                ws2812 = None
        print("[GUIServer] Finished WS2812 LED initialization.")
        return ws2812


def start_video_thread():
        """Start FPV video streaming thread"""
        global fps_threading
        print("[GUIServer] Starting FPV video stream thread (runs continuously)...")
        fps_threading = threading.Thread(target=FPV_thread)
        fps_threading.daemon = True
        fps_threading.start()
        print("[GUIServer] FPV thread started. Clients can connect to video stream.")


def check_network_and_start_ap(ws2812):
        """Check network connectivity and start Access Point if needed"""
        print("[GUIServer] Checking network connectivity and starting AP if needed...")
        try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(("1.1.1.1", 80))
                ipaddr_check = s.getsockname()[0]
                s.close()
                print(f"[GUIServer] Network detected, IP: {ipaddr_check}")
        except:
                print("[GUIServer] No network detected, attempting to start Access Point...")
                ap_threading = threading.Thread(target=ap_thread)
                ap_threading.daemon = True
                ap_threading.start()

                if ws2812:
                        # LED animation for AP mode
                        print("[GUIServer] Setting LED animation for AP mode...")
                        for brightness in [50, 100, 150, 200, 255]:
                                ws2812.set_all_led_color_data(0, 16, brightness)
                                ws2812.show()
                                time.sleep(0.1) # Shorter sleep for faster animation
                        ws2812.set_all_led_color_data(35, 255, 35) # Green
                        ws2812.show()
                        print("[GUIServer] LED animation for AP mode set.")
        print("[GUIServer] Finished network check and AP setup.")


def setup_server_socket(addr):
        """Create and configure server socket"""
        try:
                tcpSerSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                tcpSerSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                tcpSerSock.bind(addr)
                tcpSerSock.listen(5)
                return tcpSerSock
        except OSError as e:
                if e.errno == 98:  # Address already in use
                        print("\n" + "="*60)
                        print("❌ ERROR: Address already in use")
                        print("="*60)
                        print(f"Port {addr[1]} is already in use by another process.")
                        print("\nThis usually means:")
                        print("  1. Another GUIServer is already running")
                        print("  2. A crashed GUIServer didn't release the port")
                        print("\nTo fix this, run:")
                        print("  bash Server/stop_guiserver.sh")
                        print("\nOr manually find and kill the process:")
                        print(f"  sudo lsof -i :{addr[1]}")
                        print(f"  sudo kill -9 <PID>")
                        print("="*60 + "\n")
                        raise  # Re-raise to exit the program
                else:
                        raise  # Re-raise other errors


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
                check_network_and_start_ap(.venv)

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