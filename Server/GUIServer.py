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
            info_data = 'INFO:' + get_cpu_tempfunc() + ' ' + get_cpu_use() + ' ' + get_ram_info() + '\n'
            tcpCliSock.send(info_data.encode())
            time.sleep(1)
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
                continue
            elif 'forward' == data:
                direction_command = 'forward'
                move.commandInput(direction_command)
            elif 'backward' == data:
                direction_command = 'backward'
                move.commandInput(direction_command)
            elif 'DS' in data:
                direction_command = 'stand'
                move.commandInput(direction_command)

            elif 'left' == data:
                direction_command = 'left'
                move.commandInput(direction_command)
            elif 'right' == data:
                direction_command = 'right'
                move.commandInput(direction_command)
            elif 'TS' in data:
                direction_command = 'no'
                move.commandInput(direction_command)

            elif 'up' == data:
                move.look_up()
            elif 'down' == data:
                move.look_down()
            elif 'home' == data:
                move.home()

            elif 'lookleft' == data:
                move.look_left()
            elif 'lookright' == data:
                move.look_right()

            elif 'findColor' ==  data:
                fpv.FindColor(1)
                tcpCliSock.send(('findColor').encode())

            elif 'motionGet' in data:
                fpv.WatchDog(1)
                tcpCliSock.send(('motionGet').encode())

            elif 'steadyCamera' == data:
                move.commandInput(data)
                tcpCliSock.send(('steadyCamera').encode())
            elif 'steadyCameraOff' == data:
                move.commandInput(data)
                tcpCliSock.send(('steadyCameraOff').encode())

            elif 'stopCV' in data:
                fpv.FindColor(0)
                fpv.WatchDog(0)
                fpv.FindLineMode(0)
                tcpCliSock.send(('stopCV').encode())
                direction_command = 'stand'
                turn_command = 'no'
                move.commandInput('stand')
                move.commandInput('no')

            elif 'fast' in data:
                move.commandInput(data)
                tcpCliSock.send(('fast').encode())

            elif 'slow' in data:
                move.commandInput(data)
                tcpCliSock.send(('slow').encode())

            elif 'smoothCam' == data:
                move.commandInput(data)
                tcpCliSock.send(('smoothCam').encode())

            elif 'smoothCamOff' == data:
                move.commandInput(data)
                tcpCliSock.send(('smoothCamOff').encode())

            elif 'police' == data:
                if ws2812:
                    ws2812.police()
                tcpCliSock.send(('police').encode())

            elif 'policeOff' == data:
                if ws2812:
                    ws2812.breath(70,70,255)
                tcpCliSock.send(('policeOff').encode())

            elif 'Switch_1_on' in data:
                switch.switch(1,1)
                tcpCliSock.send(('Switch_1_on').encode())

            elif 'Switch_1_off' in data:
                switch.switch(1,0)
                tcpCliSock.send(('Switch_1_off').encode())

            elif 'Switch_2_on' in data:
                switch.switch(2,1)
                tcpCliSock.send(('Switch_2_on').encode())

            elif 'Switch_2_off' in data:
                switch.switch(2,0)
                tcpCliSock.send(('Switch_2_off').encode())

            elif 'Switch_3_on' in data:
                switch.switch(3,1)
                tcpCliSock.send(('Switch_3_on').encode())

            elif 'Switch_3_off' in data:
                switch.switch(3,0)
                tcpCliSock.send(('Switch_3_off').encode())

            elif 'CVFL' == data and steadyMode == 0:
                if not FPV.FindLineMode:
                    FPV.FindLineMode = 1
                    tcpCliSock.send(('CVFL_on').encode())

            elif 'CVFLColorSet 0' ==  data:
                FPV.lineColorSet = 0

            elif 'CVFLColorSet 255' ==  data:
                FPV.lineColorSet = 255

            elif 'CVFLL1' in data:
                try:
                    set_lip1=data.split()
                    lip1_set = int(set_lip1[1])
                    FPV.linePos_1 = lip1_set
                except:
                    pass

            elif 'CVFLL2' in data:
                try:
                    set_lip2=data.split()
                    lip2_set = int(set_lip2[1])
                    FPV.linePos_2 = lip2_set
                except:
                    pass

            elif 'findColorSet' in data:
                try:
                    command_dict = ast.literal_eval(data)
                    if 'data' in command_dict and len(command_dict['data']) == 3:
                        r, g, b = command_dict['data']
                        fpv.colorFindSet(r, g, b)
                        print(f"color: r={r}, g={g}, b={b}")
                except (SyntaxError, ValueError):
                    print("The received string format is incorrect and cannot be parsed.")
            else:
                pass
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


if __name__ == '__main__':
    switch.switchSetup()
    switch.set_all_switch_off()
    move.init_all()

    HOST = ''
    PORT = 10223                              #Define port serial 
    BUFSIZ = 1024                             #Define buffer size
    ADDR = (HOST, PORT)

    # Initialize WS2812 LEDs
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

    # Clean up old video ready marker
    try:
        if os.path.exists('/tmp/video_ready'):
            os.remove('/tmp/video_ready')
            print("✓ Cleaned up old video ready marker")
    except:
        pass

    # Start FPV thread ONCE at server startup (not per client connection!)
    print("Starting FPV video stream thread (runs continuously)...")
    fps_threading = threading.Thread(target=FPV_thread)
    fps_threading.setDaemon(True)
    fps_threading.start()
    print("FPV thread started. Clients can connect to video stream.")

    while  1:
        try:
            s =socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            s.connect(("1.1.1.1",80))
            ipaddr_check=s.getsockname()[0]
            s.close()
            print(ipaddr_check)
        except:
            ap_threading=threading.Thread(target=ap_thread)   #Define a thread for data receiving
            ap_threading.setDaemon(True)                          #'True' means it is a front thread,it would close when the mainloop() closes
            ap_threading.start()                                  #Thread starts

            if ws2812:
                ws2812.set_all_led_color_data(0,16,50)
                ws2812.show()
                time.sleep(1)
                ws2812.set_all_led_color_data(0,16,100)
                ws2812.show()
                time.sleep(1)
                ws2812.set_all_led_color_data(0,16,150)
                ws2812.show()
                time.sleep(1)
                ws2812.set_all_led_color_data(0,16,200)
                ws2812.show()
                time.sleep(1)
                ws2812.set_all_led_color_data(0,16,255)
                ws2812.show()
                time.sleep(1)
                ws2812.set_all_led_color_data(35,255,35)
                ws2812.show()

        # MOVED INSIDE while loop - Repeat after each disconnect
        try:
            tcpSerSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            tcpSerSock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            tcpSerSock.bind(ADDR)
            tcpSerSock.listen(5)                      #Start server,waiting for client
            print('waiting for connection...')
            tcpCliSock, addr = tcpSerSock.accept()
            print('...connected from :', addr)

            # Note: VIDEO_READY signal is now sent in info_send_client thread
            # which starts automatically when run() is called

        except Exception as e:
            print(f"Error setting up connection: {e}")
            time.sleep(1)
            continue  # Try again

        try:
            if ws2812:
                ws2812.breath_status_set(0)
                ws2812.set_all_led_color_data(64,128,255)
                ws2812.show()
                print("WS2812 LEDs set to blue (connected state)")
            else:
                print("WS2812 LEDs not available - skipping LED setup")
        except Exception as e:
            print(f"\033[38;5;3mWarning:\033[0m Could not set WS2812 LED color: {e}")

        # Run the command processing loop
        run()

        # After run() exits (client disconnected), clean up and loop back
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
        # Loop back to accept new connection

