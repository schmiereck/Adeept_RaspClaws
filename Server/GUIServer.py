import socket
import time
import threading
import sys
import signal
import logging
import traceback

# ==================== Logging Setup ====================
log_file = '/tmp/guiserver_debug.log'
# Clear previous log file
try:
    with open(log_file, 'w'):
        pass
except IOError:
    # Handle cases where the agent might not have write permission to /tmp on the remote machine
    # Fallback to a local log file in that case.
    log_file = 'guiserver_debug.log'
    with open(log_file, 'w'):
        pass


logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s.%(msecs)03d - %(levelname)s - [%(funcName)s:%(lineno)d] - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler(sys.stdout)
    ])

logging.info("=================================================")
logging.info("              GUIServer Starting Up              ")
logging.info("=================================================")

def handle_exception(exc_type, exc_value, exc_traceback):
    """Log any unhandled exceptions"""
    if issubclass(exc_type, KeyboardInterrupt):
        sys.__excepthook__(exc_type, exc_value, exc_traceback)
        return
    logging.error("Unhandled exception", exc_info=(exc_type, exc_value, exc_traceback))

sys.excepthook = handle_exception

try:
    logging.info("Importing modules...")
    import Move as move
    logging.info("Imported Move")
    import argparse
    logging.info("Imported argparse")
    import os
    logging.info("Imported os")
    # FPV and RobotLight are disabled for debugging
    # import FPV
    # import RobotLight as robotLight
    import psutil
    logging.info("Imported psutil")
    import Switch as switch
    logging.info("Imported Switch")
    import ast
    logging.info("Imported ast")

    logging.info("Adding parent directory to path...")
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from protocol import *
    logging.info("Imported protocol")

    step_set = 1
    logging.info("Initial variable setup complete.")

except Exception as e:
    logging.critical(f"Error during initial imports: {e}", exc_info=True)
    sys.exit(1)

# ==================== Shutdown Handler ====================
logging.info("Setting up shutdown handlers...")
shutdown_requested = False

def signal_handler(sig, frame):
    """Handle Ctrl+C (SIGINT) and other termination signals"""
    global shutdown_requested
    logging.info(f"Signal {sig} received.")

    if shutdown_requested:
        logging.warning("Force shutdown requested. Killing immediately.")
        os._exit(1)

    shutdown_requested = True
    logging.info("="*60)
    logging.info("🛑 Shutdown signal received. Shutting down gracefully...")
    logging.info("(Press Ctrl+C again to force immediate shutdown)")
    logging.info("="*60)

    # The rest of the cleanup logic
    try:
        global tcpCliSock
        if 'tcpCliSock' in globals() and tcpCliSock:
            tcpCliSock.close()
            logging.info("✓ Client socket closed")
    except Exception as e:
        logging.error(f"Error closing client socket: {e}", exc_info=True)

    try:
        global tcpSerSock
        if 'tcpSerSock' in globals() and tcpSerSock:
            tcpSerSock.close()
            logging.info("✓ Server socket closed")
    except Exception as e:
        logging.error(f"Error closing server socket: {e}", exc_info=True)

    try:
        # if 'ws2812' in globals() and ws2812:
        #     ws2812.led_close()
        #     logging.info("✓ LEDs turned off")
        pass # Disabled
    except Exception as e:
        logging.error(f"Error cleaning up LEDs: {e}", exc_info=True)

    try:
        logging.info("Cleaning up Move module...")
        move.clean_all()
        logging.info("✓ Move module cleaned up")
    except Exception as e:
        logging.error(f"Error cleaning up Move module: {e}", exc_info=True)

    logging.info("="*60)
    logging.info("✅ GUIServer shutdown complete.")
    logging.info("="*60)
    os._exit(0)

def sigtstp_handler(sig, frame):
    logging.warning("="*60)
    logging.warning("⚠️ WARNING: Ctrl+Z detected! This can leave ports blocked.")
    logging.warning("✅ USE Ctrl+C instead. Initiating clean shutdown...")
    logging.warning("="*60)
    signal_handler(sig, frame)

try:
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGTSTP, sigtstp_handler)
    logging.info("✓ Signal handlers registered.")
except Exception as e:
    logging.error("Could not register signal handlers.", exc_info=True)

# ==================== Main Code ====================

# Global state variables
direction_command = 'no'
turn_command = 'no'
SmoothMode = 0
steadyMode = 0
SmoothCamMode = 1
servo_standby_active = False
camera_paused_active = False
battery_available = False
adc = None
ws2812 = None # Disabled for now

def initialize_components():
    """Initialize hardware components like RobotM and ADS7830."""
    global rm, adc, battery_available
    logging.info("Initializing Move.RobotM...")
    try:
        rm = move.RobotM()
        rm.start()
        rm.pause()
        logging.info("✓ RobotM initialized, started, and paused.")
    except Exception as e:
        logging.critical("Failed to initialize RobotM.", exc_info=True)
        sys.exit(1)

    logging.info("Attempting to initialize ADS7830 battery monitor...")
    try:
        import smbus
        class ADS7830:
            def __init__(self, busnum=1, address=0x48):
                self.bus = smbus.SMBus(busnum)
                self.address = address
                self.cmd = 0x84
            def analogRead(self, chn):
                value = self.bus.read_byte_data(self.address, self.cmd | (((chn << 2 | chn >> 1) & 0x07) << 4))
                return value
        adc = ADS7830()
        adc.analogRead(0) # Test read
        battery_available = True
        logging.info("✓ ADS7830 initialized successfully (smbus).")
    except Exception as e:
        logging.warning(f"smbus method for ADS7830 failed: {e}. Trying adafruit fallback.")
        try:
            import board
            import busio
            from adafruit_bus_device.i2c_device import I2CDevice
            class ADS7830_Adafruit:
                def __init__(self, address=0x48):
                    i2c = busio.I2C(board.SCL, board.SDA)
                    self.device = I2CDevice(i2c, address)
                    self.cmd = 0x84
                def analogRead(self, chn):
                    control_byte = self.cmd | (((chn << 2 | chn >> 1) & 0x07) << 4)
                    buffer = bytearray(1)
                    with self.device:
                         self.device.write_then_readinto(bytes([control_byte]), buffer)
                    return buffer[0]
            adc = ADS7830_Adafruit()
            adc.analogRead(0) # Test read
            battery_available = True
            logging.info("✓ ADS7830 initialized successfully (adafruit).")
        except Exception as e2:
            logging.error(f"adafruit_bus_device method also failed: {e2}")
            logging.warning("Battery monitoring not available.")
            battery_available = False

# System Info Functions
def get_cpu_tempfunc():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", 'r') as f:
            return str(round(float(f.read()) / 1000, 1))
    except Exception: return "N/A"

def get_cpu_use(): return str(psutil.cpu_percent())
def get_ram_info(): return str(psutil.virtual_memory()[2])

def get_battery_voltage():
    if not battery_available: return "0.0"
    try:
        ADCVref = 4.93
        R15, R17 = 3000, 1000
        DivisionRatio = R17 / (R15 + R17)
        ADCValue = adc.analogRead(0)
        A0Voltage = ADCValue / 255.0 * ADCVref
        return str(round(A0Voltage / DivisionRatio, 2))
    except Exception as e:
        logging.error(f"Error reading battery voltage: {e}")
        return "0.0"

# ... (Rest of the functions like info_send_client, command handlers, etc.)
# Note: FPV and RobotLight calls are commented out or disabled.
def info_send_client():
    logging.info("INFO_SEND_CLIENT thread started.")
    try:
        # Simplified initial status sending
        initial_info = f"{STATUS_INFO_PREFIX}{get_cpu_tempfunc()} {get_cpu_use()} {get_ram_info()} {get_battery_voltage()} | {move.get_servo_positions_info()} | {move.get_mpu6050_data()}\n"
        tcpCliSock.send(initial_info.encode())
        logging.info("Sent initial INFO data.")
    except Exception as e:
        logging.error(f"Failed to send initial status: {e}", exc_info=True)
        return

    # Video ready signals (even if video is off, client expects this)
    logging.info("Starting VIDEO_READY signals...")
    for i in range(5): # Reduced for faster startup
        try:
            tcpCliSock.send(f'{STATUS_VIDEO_READY}\n'.encode())
            time.sleep(0.5)
        except Exception as e:
            logging.warning(f"Client disconnected during VIDEO_READY. {e}")
            return
    logging.info("Finished VIDEO_READY signals.")

    while not shutdown_requested:
        try:
            info_data = f"{STATUS_INFO_PREFIX}{get_cpu_tempfunc()} {get_cpu_use()} {get_ram_info()} {get_battery_voltage()} | {move.get_servo_positions_info()} | {move.get_mpu6050_data()}\n"
            tcpCliSock.send(info_data.encode())
            time.sleep(0.2)
        except (ConnectionResetError, BrokenPipeError, OSError):
            break # Normal disconnect
        except Exception as e:
            logging.error(f"Error in info_send_client loop: {e}", exc_info=True)
            break
    logging.info("INFO_SEND_CLIENT thread finished.")


def process_client_command(data):
    """Main command dispatcher"""
    logging.debug(f"Processing command: {data}")
    # Using a simple if/elif structure for clarity
    if GUI_TO_MOVE_COMMAND_MAP.get(data):
        move.handle_movement_command(GUI_TO_MOVE_COMMAND_MAP.get(data))
    elif data in [CMD_LOOK_UP, CMD_LOOK_DOWN, CMD_LOOK_HOME, CMD_LOOK_LEFT, CMD_LOOK_RIGHT]:
         move.commandInput(data) # Simplified
    elif data.startswith(CMD_SET_SPEED):
        try:
            speed = int(data[len(CMD_SET_SPEED):])
            move.set_movement_speed(speed)
        except ValueError:
            logging.warning(f"Invalid speed value: {data}")
    # Add other command handlers here...
    else:
        logging.debug(f"Command not handled: {data}")


def run_client_communication():
    """Main loop to handle a single client connection."""
    logging.info("Starting 'run' loop to process client commands.")
    info_thread = threading.Thread(target=info_send_client)
    info_thread.daemon = True
    info_thread.start()

    while not shutdown_requested:
        try:
            data = tcpCliSock.recv(1024).decode().strip()
            if not data:
                logging.warning("Client disconnected (empty recv).")
                break
            for command in data.split('\n'):
                if command:
                    process_client_command(command)
        except (ConnectionResetError, BrokenPipeError, OSError) as e:
            logging.warning(f"Client connection lost: {e}")
            break
        except Exception as e:
            logging.error("Unexpected error in 'run' loop.", exc_info=True)
            break
    logging.info("'run' loop finished.")


if __name__ == '__main__':
    try:
        logging.info("Executing main block.")
        logging.info("Setting up switches...")
        switch.switchSetup()
        switch.set_all_switch_off()
        logging.info("✓ Switches set up.")

        logging.info("Initializing move systems...")
        move.init_all()
        logging.info("✓ Move systems initialized.")

        initialize_components()

        HOST = ''
        PORT = 10223
        ADDR = (HOST, PORT)

        # FPV and LEDS are disabled
        # start_video_thread()
        # ws2812 = initialize_leds()

        while not shutdown_requested:
            try:
                tcpSerSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                tcpSerSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                tcpSerSock.bind(ADDR)
                tcpSerSock.listen(5)
                logging.info(f'Waiting for connection on port {PORT}...')
                tcpCliSock, addr = tcpSerSock.accept()
                logging.info(f'...connected from: {addr}')
            except Exception as e:
                logging.error(f"Error in connection accept loop: {e}", exc_info=True)
                time.sleep(2)
                continue

            run_client_communication() # Handle this client

            # Cleanup after disconnect
            try: tcpCliSock.close()
            except: pass
            try: tcpSerSock.close()
            except: pass
            logging.info("Sockets closed. Ready for new connection.")

    except Exception as e:
        logging.critical("Fatal error in main execution block.", exc_info=True)
    finally:
        logging.info("GUIServer is shutting down.")
        move.clean_all()